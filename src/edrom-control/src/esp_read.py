#!/usr/bin/env python3
#coding=utf-8

import rospy, serial, os, sys
from sensor_msgs.msg import Joy
from your_custom_msgs.msg import RobotControl  # Import the custom message

edrom_dir = '/home/' + os.getlogin() + '/hook/src/'

class JoystickReader:

    def __init__(self):
        self.joySub = rospy.Subscriber('/joy', Joy, self.joyCallback)
        self.robotControlPub = rospy.Publisher('/robot_control', RobotControl, queue_size=10)
        self.esp = serial.Serial('/dev/ttyUSB0', 115200)
        self.rate = rospy.Rate(10)

    def joyCallback(self, joy_msg):
        try:
            esp_output = self.esp.readline().decode().strip().split()
            esp_output = [float(string) for string in esp_output]

            self.robotControlPub.publish(robot_control_msg)

        except Exception as e:
            rospy.logerr(str(e))

    def run(self):
        while not rospy.is_shutdown():
            #





            
            self.rate.sleep()

        self.esp.close()

if __name__ == '__main__':
    rospy.init_node('joystickReader_node', anonymous=False)

    joystick_reader = JoystickReader()
    joystick_reader.run()

    rospy.spin()