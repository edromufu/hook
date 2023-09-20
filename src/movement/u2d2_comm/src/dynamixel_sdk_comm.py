#!/usr/bin/env python3

import rospy
from dynamixel_sdk import *

from movement_utils.msg import *
from movement_utils.srv import *

BAUDRATE = 1000000

DEVICENAME = rospy.get_param('u2d2/port')
PROTOCOL_1_INFOS =  {'TORQUE_ADDR': 24, 'LED_ADDR': 25 , 'MOVING_SPEED_ADDR': 32, 'MOVING_SPEED_LEN': 2, 'PRES_POS_ADDR': 36, 'GOAL_POS_ADDR': 30, 'GOAL_POS_LEN': 2}
MOVING_SPEED = [200]*6

class u2d2Control():

    def __init__(self):

        rospy.init_node('u2d2')
        
        rospy.Subscriber('u2d2_comm/data2arm', arm_motors_data, self.data2arm)

        rospy.Subscriber('u2d2_comm/data2gripper', gripper_motor_data, self.data2gripper)

        rospy.Service('u2d2_comm/enableTorque', enable_torque, self.enableTorque)
        self.enableTorqueRes = enable_torqueResponse()

        rospy.Service('u2d2_comm/feedbackAllArm', arm_feedback, self.feedbackArmMotors)
        self.feedbackRes = arm_feedbackResponse()

        self.portHandler = PortHandler(DEVICENAME)

        self.packetHandler = PacketHandler(1.0)
        self.armGroup = GroupSyncWrite(self.portHandler, self.packetHandler, PROTOCOL_1_INFOS['GOAL_POS_ADDR'], PROTOCOL_1_INFOS['GOAL_POS_LEN'])
        self.armMovingSpeed = GroupSyncWrite(self.portHandler, self.packetHandler, PROTOCOL_1_INFOS['MOVING_SPEED_ADDR'], PROTOCOL_1_INFOS['MOVING_SPEED_LEN'])
        self.gripperGroup = GroupSyncWrite(self.portHandler, self.packetHandler, PROTOCOL_1_INFOS['GOAL_POS_ADDR'], PROTOCOL_1_INFOS['GOAL_POS_LEN'])
            
        for motor_id in range(5):
            value = MOVING_SPEED[motor_id]
            bytes_value = value.to_bytes(2, byteorder='little')

            self.armMovingSpeed.addParam(motor_id, bytes_value)

        self.startComm()
        
    def startComm(self):
        # Open port
        try:
            self.portHandler.openPort()
            print("Succeeded to open the port")
        except:
            print("Failed to open the port")
            quit()

        # Set port baudrate
        try:
            self.portHandler.setBaudRate(BAUDRATE)
            print("Succeeded to change the baudrate")
        except:
            print("Failed to change the baudrate")
            quit()
    
    def enableTorque(self, req):
        
        for _ in range(3):
            if req.motor_ids[0] == -1:
                motor_ids = range(6)
            else:
                motor_ids = req.motor_ids
            
            for motor_id in motor_ids:
                self.packetHandler.write1ByteTxOnly(self.portHandler, motor_id, PROTOCOL_1_INFOS['TORQUE_ADDR'], req.data)
                self.packetHandler.write1ByteTxOnly(self.portHandler, motor_id, PROTOCOL_1_INFOS['LED_ADDR'], req.data)

            self.enableTorqueRes.success = True
        
        return self.enableTorqueRes

    def feedbackArmMotors(self, req):

        try:

            self.feedbackRes.pos_vector = [0]*6

            for motor_id in range(6):
                motor_position, comm, hard = self.packetHandler.read2ByteTxRx(self.portHandler, motor_id, PROTOCOL_1_INFOS['PRES_POS_ADDR'])

                if comm != 0 or hard != 0:
                    raise Exception(f'Erro de comunicação ou hardware no motor {motor_id}, refazendo.')
                else:
                    self.feedbackRes.pos_vector[motor_id] = self.pos2rad(motor_position)
            return self.feedbackRes

        except Exception as e:
            #print(e)
            return self.feedbackArmMotors(req)

    def data2arm(self, msg):

        self.armGroup.clearParam()

        for motor_id in range(5):
            motor_position = msg.pos_vector[motor_id]

            value = self.rad2pos(motor_position)
            bytes_value = value.to_bytes(2, byteorder='little')

            self.armGroup.addParam(motor_id, bytes_value)

    
        self.armGroup.txPacket()
        self.armMovingSpeed.txPacket()

    def data2gripper(self, msg):
        
        self.gripperGroup.clearParam()

        motor_position = msg.gripper_position

        value = self.rad2pos(motor_position)
        bytes_value = value.to_bytes(2, byteorder='little')

        self.gripperGroup.addParam(5, bytes_value)

        
        self.gripperGroup.txPacket()
    
    def rad2pos(self, pos_in_rad):
        
        motor_position = int(195.379*pos_in_rad + 512)
        motor_position = min(motor_position, 1023)
        motor_position = max(motor_position, 0)
        
        return motor_position

    def pos2rad(self, motor_position):

        pos_in_rad = (motor_position-512)/195.379

        return pos_in_rad

    def run(self):
        rospy.spin()

if __name__ == '__main__':

    u2d2 = u2d2Control()
    u2d2.run()