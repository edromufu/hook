#!/usr/bin/env python3
#coding=utf=8

import rospy
import numpy as np

import sys, os
hook_dir = '/home/'+os.getlogin()+'/hook/src/'

sys.path.append(hook_dir+'movement/humanoid_definition/src')
from setup_robot import Robot


from movement_utils.srv import *
from movement_utils.msg import *
from std_srvs.srv import *
from sensor_msgs.msg import JointState

QUEUE_TIME = rospy.get_param('/movement_core/queue_time') #Em segundos
PUB2VIS = rospy.get_param('/movement_core/pub2vis')
L = 0.024
r = 0.014

class Core:
    def __init__(self): 
        
        # Inicialização das variáveis do ROS
        rospy.init_node('movement_central')

        # Inicialização das variáveis do ROS para u2d2
        if rospy.get_param('/movement_core/wait4u2d2'):
            rospy.wait_for_service('u2d2_comm/feedbackAllArm')
            rospy.wait_for_service('u2d2_comm/enableTorque')

            #Estruturas para comunicação com U2D2
            self.motorsFeedback = rospy.ServiceProxy('u2d2_comm/feedbackAllArm', arm_feedback)
            self.enableTorque = rospy.ServiceProxy('u2d2_comm/enableTorque', enable_torque)

            self.pub2arm = rospy.Publisher('u2d2_comm/data2arm', arm_motors_data, queue_size=100)
            self.pub2armMsg = arm_motors_data()

            self.pub2gripper = rospy.Publisher('u2d2_comm/data2gripper', gripper_motor_data, queue_size=100)
            self.pub2gripperMsg = gripper_motor_data()

            self.queue = []  

        # Inicialização das variáveis do ROS de requisição na core
        rospy.Service('movement_central/request_base_rotation', base_rotation, self.movementManager)
        rospy.Service('movement_central/request_gripper_gap', gripper_gap, self.movementManager)
        rospy.Service('movement_central/request_endeffector_kinematics', kinematic_request, self.movementManager)
        rospy.Service('movement_central/request_first_pose', SetBool, self.movementManager)

        #Inicialização do objeto (modelo) da robô em código
        robot_name = rospy.get_param('/movement_core/name')
                
        self.robotInstance = Robot(robot_name)
        self.robotModel = self.robotInstance.robotJoints
        self.lastRequest = None
        self.lastIncrement = 0
        self.motorsPosition = [0]*6

        # Definições para visualizador
        if PUB2VIS:
            self.queuevis = []
            self.pub2vis = rospy.Publisher('/joint_states', JointState, queue_size=100)
            self.pub2vismsg = JointState()
            self.pub2vismsg.name = ['BASE_UZ_joint', 'SHOULDER_UY_joint', 'ELBOW_UY_joint', 
                                    'WRIST_UY_joint', 'WRIST_UZ_joint', 
                                    'LEFT_GRIPPER_FINGER_joint', 'RIGHT_GRIPPER_FINGER_joint']

        #Timer para fila de publicações
        rospy.Timer(rospy.Duration(QUEUE_TIME), self.sendFromQueue)

        self.enableTorque(True, [-1])
        self.callRobotModelUpdate()

    def callRobotModelUpdate(self):
        self.motorsCurrentPosition = list(self.motorsFeedback(True).pos_vector)
        
        positions2Update = []
        for element in self.motorsCurrentPosition:
            positions2Update.append(element)

        positions2Update = self.invertMotorsPosition(positions2Update)
        
        self.robotInstance.updateRobotModel([0] + positions2Update[1:4] + [0])
    
    def invertMotorsPosition(self, toInvert):
        for index, joint in enumerate(self.robotModel):
            if joint.is_inverted():
                toInvert[joint.get_id()] *= -1
        
        return toInvert

    def movementManager(self, req):
        
        self.callRobotModelUpdate()

        if 'base_rotation' in str(req.__class__):
            
            if self.lastRequest != 'base_rotation':

                currentPosition = []
                for index, lastPosition in enumerate(self.motorsPosition):
                    if abs(lastPosition-self.motorsCurrentPosition[index]) >= self.lastIncrement:
                        currentPosition.append(self.motorsCurrentPosition[index])
                    else:
                        currentPosition.append(lastPosition)
                
                self.motorsPosition = currentPosition

            baseUzNewPosition = req.base_rotation_increment + self.motorsCurrentPosition[0]
            allMotorsNewPosition = [baseUzNewPosition] + self.motorsPosition[1:]

            if rospy.get_param('/movement_core/wait4u2d2'):
                self.queue.append(allMotorsNewPosition)

            if PUB2VIS:
                a = 1
                b = -2*r*np.cos(allMotorsNewPosition[-1])
                c = r**2 - L**2

                delta = b**2 - 4*a*c

                angle2slide = (-b+np.sqrt(delta))/(2*a)

                pub2VisPos = allMotorsNewPosition[:-1] + [angle2slide,angle2slide]
                self.queuevis.append(pub2VisPos)

            response = base_rotationResponse()
            response.success = True

            self.lastRequest = 'base_rotation'
            self.lastIncrement = req.base_rotation_increment 

        elif 'gripper_gap' in str(req.__class__):
            
            if self.lastRequest != 'gripper_gap':
                currentPosition = []
                for index, lastPosition in enumerate(self.motorsPosition):
                    if abs(lastPosition-self.motorsCurrentPosition[index]) >= self.lastIncrement:
                        currentPosition.append(self.motorsCurrentPosition[index])
                    else:
                        currentPosition.append(lastPosition)
                
                self.motorsPosition = currentPosition

            gripperNewPosition = req.gripper_gap_increment + self.motorsCurrentPosition[-1]
            allMotorsNewPosition = self.motorsPosition[:-1] + [gripperNewPosition] 

            if rospy.get_param('/movement_core/wait4u2d2'):
                self.queue.append(allMotorsNewPosition)

            if PUB2VIS:
                a = 1
                b = -2*r*np.cos(allMotorsNewPosition[-1])
                c = r**2 - L**2

                delta = b**2 - 4*a*c

                angle2slide = (-b+np.sqrt(delta))/(2*a)

                pub2VisPos = allMotorsNewPosition[:-1] + [angle2slide,angle2slide]
                self.queuevis.append(pub2VisPos)

            response = gripper_gapResponse()
            response.success = True

            self.lastRequest = 'gripper_gap'
            self.lastIncrement = req.gripper_gap_increment
        
        elif 'SetBool' in str(req.__class__):
            self.motorsPosition = [0, -0.8, -2, -1, 0, 0]

            allMotorsNewPosition = self.motorsPosition

            if rospy.get_param('/movement_core/wait4u2d2'):
                self.queue.append(allMotorsNewPosition)

            if PUB2VIS:
                a = 1
                b = -2*r*np.cos(allMotorsNewPosition[-1])
                c = r**2 - L**2

                delta = b**2 - 4*a*c

                angle2slide = (-b+np.sqrt(delta))/(2*a)

                pub2VisPos = allMotorsNewPosition[:-1] + [angle2slide,angle2slide]
                self.queuevis.append(pub2VisPos)
            
            response = SetBoolResponse()
            response.success = True

            self.lastRequest = 'SetBool'
            self.lastIncrement = 0

        return response 
        
    def sendFromQueue(self, event):
        
        if rospy.get_param('/movement_core/wait4u2d2'):
            if self.queue:
                current_all_arm_pose = self.queue.pop(0)
                self.pub2armMsg.pos_vector = current_all_arm_pose[:-1]
                self.pub2arm.publish(self.pub2armMsg)

                self.pub2gripperMsg.gripper_position = current_all_arm_pose[-1]
                self.pub2gripper.publish(self.pub2gripperMsg)

        if PUB2VIS:
            if self.queuevis:
                self.pub2vismsg.position = self.queuevis.pop(0)
                self.pub2vismsg.header.stamp = rospy.Time.now()
                self.pub2vis.publish(self.pub2vismsg)


if __name__ == '__main__':
    np.set_printoptions(precision=3, suppress=True, linewidth=np.inf, threshold=sys.maxsize)
    movement = Core()
    rospy.spin()