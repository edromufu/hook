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
from sensor_msgs.msg import JointState

QUEUE_TIME = rospy.get_param('/movement_core/queue_time') #Em segundos
PUB2VIS = rospy.get_param('/movement_core/pub2vis')

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

            self.pub2arm = rospy.Publisher('u2d2_comm/data2arm', arm_motors_data, queue_size=100)
            self.pub2armMsg = arm_motors_data()

            self.pub2gripper = rospy.Publisher('u2d2_comm/data2gripper', gripper_motor_data, queue_size=100)
            self.pub2gripperMsg = gripper_motor_data()

            self.queue = []  

        # Inicialização das variáveis do ROS de requisição na core
        rospy.Service('movement_central/request_base_rotation', base_rotation, self.movementManager)

        #Inicialização do objeto (modelo) da robô em código
        robot_name = rospy.get_param('/movement_core/name')
                
        self.robotInstance = Robot(robot_name)
        self.robotModel = self.robotInstance.robotJoints

        # Definições para visualizador
        if PUB2VIS:
            self.queuevis = []
            self.pub2vis = rospy.Publisher('/joint_states', JointState, queue_size=100)
            self.pub2vismsg = JointState()
            self.pub2vismsg.name = ['BASE_UZ_joint', 'SHOULDER_UY_joint', 'ELBOW_UY_joint', 
                                    'WRIST_UY_joint', 'WRIST_UZ_joint']

        #Timer para fila de publicações
        rospy.Timer(rospy.Duration(QUEUE_TIME), self.sendFromQueue)

        #! TESTE - EXCLUIR POSTERIORMENTE
        self.motorRotation = [0]*6

    def movementManager(self, req):
        
        if 'base_rotation' in str(req.__class__):

            self.motorRotation[0] += req.base_rotation_increment

            if rospy.get_param('/movement_core/wait4u2d2'):
                self.queue.append(self.motorRotation)

            if PUB2VIS:
                self.queuevis.append(self.motorRotation[:-1])

            response = base_rotationResponse()
            response.success = True

        return response 
        
    def sendFromQueue(self, event):
        
        if rospy.get_param('/movement_core/wait4u2d2'):
            if self.queue:
                current_all_arm_pose = self.queue.pop(0)
                self.pub2armMsg.pos_vector = current_all_arm_pose[:-1]
                self.pub2arm.publish(self.pub2armMsg)

                self.pub2gripperMsg.pos_vector = current_all_arm_pose[-1]
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