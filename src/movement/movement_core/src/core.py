#!/usr/bin/env python3
#coding=utf=8

import rospy
import numpy as np

import sys, os
hook_dir = '/home/'+os.getlogin()+'/hook/src/'

sys.path.append(hook_dir+'movement/kinematic_functions/src')
from ik_analytical import callIK

from movement_utils.srv import *
from movement_utils.msg import *
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState

QUEUE_TIME = rospy.get_param('/movement_core/queue_time') #Em segundos
PUB2VIS = rospy.get_param('/movement_core/pub2vis')

#Constantes relacionadas à transmissão da rotação do motor do gripper para sua translação
L = 0.012 #Tamanho da manivela (m)
r = 0.014 #Raio do motor (m)

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
        rospy.Subscriber('movement_central/request_base_rotation', base_rotation, self.movementManager)
        rospy.Subscriber('movement_central/request_gripper_gap', gripper_gap, self.movementManager)
        rospy.Subscriber('movement_central/request_endeffector_kinematics', kinematic_request, self.movementManager)
        rospy.Subscriber('movement_central/request_first_pose', Bool, self.movementManager)

        #Inicialização do objeto (modelo) da robô em código
        robot_name = rospy.get_param('/movement_core/name')
                
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

    def updateCurrentMotorsPosition(self):
        self.motorsPosFeedback = list(self.motorsFeedback(True).pos_vector)

        currentPosition = []

        for index, lastPosition in enumerate(self.motorsPosition):
                if abs(lastPosition-self.motorsPosFeedback[index]) >= self.lastIncrement:
                    currentPosition.append(self.motorsPosFeedback[index])
                else:
                    currentPosition.append(lastPosition)
        
        return currentPosition

    def baseIncrementAlgorithm(self, base_increment):
        baseUzNewPosition = base_increment + self.motorsPosFeedback[0]

        return [baseUzNewPosition] + self.motorsPosition[1:]
    
    def gripperGapAlgorithm(self, gripper_increment):

        gripperNewPosition = gripper_increment + self.motorsPosFeedback[-1]

        return self.motorsPosition[:-1] + [gripperNewPosition] 

    def kinematicAlgorithm(self, ikMotorsPosition, deltaX, deltaZ, deltaPitch):
        ikOutput = callIK(ikMotorsPosition, deltaX, deltaZ, deltaPitch)

        self.motorsPosition = [self.motorsPosition[0]] + ikOutput + self.motorsPosition[4:]

        return self.motorsPosition

    def getSlideFromAngle(self, theta):
        b = -2*r*np.cos(theta)
        c = r**2 - L**2

        delta = b**2 - 4*c
        
        return (-b+np.sqrt(delta))/2

    def movementManager(self, msg):
        
        self.enableTorque(True, [-1])

        if 'base_rotation' in str(msg.__class__):
            
            if self.lastRequest != 'base_rotation':
                
                self.motorsPosition = self.updateCurrentMotorsPosition()

            allMotorsNewPosition = self.baseIncrementAlgorithm(msg.base_rotation_increment)

            if rospy.get_param('/movement_core/wait4u2d2'):
                self.queue.append(allMotorsNewPosition)

            if PUB2VIS:

                slide = self.getSlideFromAngle(allMotorsNewPosition[-1])

                pub2VisPos = allMotorsNewPosition[:-1] + [slide,slide]
                self.queuevis.append(pub2VisPos)

            self.lastRequest = 'base_rotation'
            self.lastIncrement = msg.base_rotation_increment 

        elif 'gripper_gap' in str(msg.__class__):
            
            if self.lastRequest != 'gripper_gap':

                self.motorsPosition = self.updateCurrentMotorsPosition()

            allMotorsNewPosition = self.gripperGapAlgorithm(msg.gripper_gap_increment)

            if rospy.get_param('/movement_core/wait4u2d2'):
                self.queue.append(allMotorsNewPosition)

            if PUB2VIS:
                slide = self.getSlideFromAngle(allMotorsNewPosition[-1])

                pub2VisPos = allMotorsNewPosition[:-1] + [slide,slide]
                self.queuevis.append(pub2VisPos)

            self.lastRequest = 'gripper_gap'
            self.lastIncrement = msg.gripper_gap_increment
        
        elif 'Bool' in str(msg.__class__):
            self.motorsPosition = [0, -0.8, 1.5, 1.5, 0, 0]

            allMotorsNewPosition = self.motorsPosition

            if rospy.get_param('/movement_core/wait4u2d2'):
                self.queue.append(allMotorsNewPosition)

            if PUB2VIS:
                slide = self.getSlideFromAngle(allMotorsNewPosition[-1])

                pub2VisPos = allMotorsNewPosition[:-1] + [slide,slide]
                self.queuevis.append(pub2VisPos)

            self.lastRequest = 'Bool'
            self.lastIncrement = 0
        
        elif 'kinematic_request' in str(msg.__class__):
            
            if self.lastRequest != 'kinematic_request':

                self.motorsPosition = self.updateCurrentMotorsPosition()

            allMotorsNewPosition = self.kinematicAlgorithm(self.motorsPosition[1:4], msg.x_increment, msg.z_increment, msg.pitch_increment) 

            if rospy.get_param('/movement_core/wait4u2d2'):
                self.queue.append(allMotorsNewPosition)

            if PUB2VIS:
                slide = self.getSlideFromAngle(allMotorsNewPosition[-1])

                pub2VisPos = allMotorsNewPosition[:-1] + [slide,slide]
                self.queuevis.append(pub2VisPos)
            
            self.lastRequest = 'kinematic_request'
            self.lastIncrement = 0

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