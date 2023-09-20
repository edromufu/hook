#!/usr/bin/env python3
#coding=utf-8

import rospy, serial
from movement_utils.msg import *
from std_msgs.msg import Bool

BASE_INCREMENT = 0.2/5
X_Z_INCREMENT = -0.02/5
PITCH_INCREMENT = 0.1/5
WRIST_INCREMENT = 0.5
TIME_PUB = 2e8

class controllerRead():

    def __init__(self):
        self.controller = serial.Serial('/dev/ttyUSB1', 115200)
        self.lastRead = {'LeftX': 0, 'LeftY': 0, 'Button': 0, 'RightX': 0, 'RightY': 0}
        
        self.pubCore = rospy.Publisher('movement_central/general_request', core, queue_size=1)
        self.pubCoreMsg = core()

    def publishAll(self):
        self.pubCoreMsg.base_rotation_increment = self.lastRead['LeftY']*BASE_INCREMENT

        self.pubCoreMsg.x_increment = self.lastRead['LeftX']*X_Z_INCREMENT
        self.pubCoreMsg.z_increment = self.lastRead['RightX']*X_Z_INCREMENT
        self.pubCoreMsg.pitch_increment = self.lastRead['RightY']*PITCH_INCREMENT

        self.pubCoreMsg.wrist_increment = self.lastRead['Button']*WRIST_INCREMENT

        self.pubCore.publish(self.pubCoreMsg)

    def run(self):
        lastTime = rospy.Time.now().nsecs
        while not rospy.is_shutdown():   
            try:
                value = 0
                if self.controller.inWaiting():
                    controller_output = self.controller.readline()
                    controller_output = controller_output.strip().split()
                    if 'B' not in controller_output[0].decode('utf-8'):
                        self.lastRead[controller_output[0].decode('utf-8')] = int(controller_output[1])

                    else:
                        if 'Right' in controller_output[0].decode('utf-8'):
                            value += int(controller_output[1])
                        else:
                            value -= int(controller_output[1])

                        self.lastRead['Button'] = value

                if abs(rospy.Time.now().nsecs - lastTime) >= TIME_PUB:
                    self.publishAll()
                    lastTime = rospy.Time.now().nsecs

            except Exception as e:
                print(e)
            
        self.controller.close()

if __name__ == '__main__':
    rospy.init_node('Controller_node', anonymous=False)
    
    controller = controllerRead()
    controller.run()

    rospy.spin()