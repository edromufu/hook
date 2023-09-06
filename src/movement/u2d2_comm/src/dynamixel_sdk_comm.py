#!/usr/bin/env python3

import os, json
import rospy
from dynamixel_sdk import *

from movement_utils.msg import *
from movement_utils.srv import *

BAUDRATE = 1000000

DEVICENAME = rospy.get_param('u2d2/port')
ROBOT_MOTORS = rospy.get_param('u2d2/robot_name')

PROTOCOL_1_INFOS =  {'TORQUE_ADDR': 24, 'LED_ADDR': 25 , 'PRES_POS_ADDR': 36, 'GOAL_POS_ADDR': 30, 'GOAL_POS_LEN': 2}
PROTOCOL_2_INFOS =  {'TORQUE_ADDR': 64, 'LED_ADDR': 65 , 'PRES_POS_ADDR': 132, 'GOAL_POS_ADDR': 116, 'GOAL_POS_LEN': 4}

class u2d2Control():

    def __init__(self):
        self.loadMotorsType()

        rospy.init_node('u2d2')
        
        rospy.Subscriber('u2d2_comm/data2body', body_motors_data, self.data2body)

        rospy.Subscriber('u2d2_comm/data2head', head_motors_data, self.data2head)

        rospy.Service('u2d2_comm/enableTorque', enable_torque, self.enableTorque)
        self.enableTorqueRes = enable_torqueResponse()

        rospy.Service('u2d2_comm/feedbackBody', body_feedback, self.feedbackBodyMotors)
        self.bodyFeedbackRes = body_feedbackResponse()

        rospy.Service('u2d2_comm/feedbackHead', head_feedback, self.feedbackHeadMotors)
        self.headFeedbackRes = head_feedbackResponse()

        self.portHandler = PortHandler(DEVICENAME)

        if self.AX_12_MOTORS:
            self.packetHandler1 = PacketHandler(1.0)
            self.bodyGroup1 = GroupSyncWrite(self.portHandler, self.packetHandler1, PROTOCOL_1_INFOS['GOAL_POS_ADDR'], PROTOCOL_1_INFOS['GOAL_POS_LEN'])
            self.headGroup1 = GroupSyncWrite(self.portHandler, self.packetHandler1, PROTOCOL_1_INFOS['GOAL_POS_ADDR'], PROTOCOL_1_INFOS['GOAL_POS_LEN'])

        if self.MX_106_MOTORS or self.MX_64_MOTORS:
            self.packetHandler2 = PacketHandler(2.0)   
            self.bodyGroup2 = GroupSyncWrite(self.portHandler, self.packetHandler2, PROTOCOL_2_INFOS['GOAL_POS_ADDR'], PROTOCOL_2_INFOS['GOAL_POS_LEN'])
            self.headGroup2 = GroupSyncWrite(self.portHandler, self.packetHandler2, PROTOCOL_2_INFOS['GOAL_POS_ADDR'], PROTOCOL_2_INFOS['GOAL_POS_LEN'])  

        self.startComm()
        
    def loadMotorsType(self):
        os.chdir('/home/'+os.getlogin()+'/edromufu/src/movement_bioloid/humanoid_definition/robots_jsons/')

        with open(ROBOT_MOTORS+'.json') as f:
            json_data = json.loads(f.read())
        
        self.AX_12_MOTORS = json_data["motor_type"]["AX-12"]
        self.MX_106_MOTORS = json_data["motor_type"]["MX-106"]
        self.MX_64_MOTORS = json_data["motor_type"]["MX-64"]

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
                motor_ids = range(20)
            else:
                motor_ids = req.motor_ids
            
            for motor_id in motor_ids:
                if motor_id in self.AX_12_MOTORS:
                    self.packetHandler1.write1ByteTxOnly(self.portHandler, motor_id, PROTOCOL_1_INFOS['TORQUE_ADDR'], req.data)
                    self.packetHandler1.write1ByteTxOnly(self.portHandler, motor_id, PROTOCOL_1_INFOS['LED_ADDR'], req.data)

                elif (motor_id in self.MX_106_MOTORS) or (motor_id in self.MX_64_MOTORS):
                    self.packetHandler2.write1ByteTxOnly(self.portHandler, motor_id, PROTOCOL_2_INFOS['TORQUE_ADDR'], req.data)
                    self.packetHandler2.write1ByteTxOnly(self.portHandler, motor_id, PROTOCOL_2_INFOS['LED_ADDR'], req.data)

            self.enableTorqueRes.success = True
        
        return self.enableTorqueRes

    def feedbackBodyMotors(self, req):

        try:

            self.bodyFeedbackRes.pos_vector = [0]*18

            for motor_id in range(18):
                if motor_id in self.AX_12_MOTORS:
                    motor_position, comm, hard = self.packetHandler1.read2ByteTxRx(self.portHandler, motor_id, PROTOCOL_1_INFOS['PRES_POS_ADDR'])

                    if comm !=0 or hard != 0:
                        self.bodyFeedbackRes.pos_vector[motor_id] = -1
                    else:
                        self.bodyFeedbackRes.pos_vector[motor_id] = self.pos2rad(motor_position, 1.0)

                elif (motor_id in self.MX_106_MOTORS) or (motor_id in self.MX_64_MOTORS):
                    motor_position, comm, hard = self.packetHandler2.read2ByteTxRx(self.portHandler, motor_id, PROTOCOL_2_INFOS['PRES_POS_ADDR'])

                    if comm !=0 or hard != 0:
                        self.bodyFeedbackRes.pos_vector[motor_id] = -1
                    else:
                        self.bodyFeedbackRes.pos_vector[motor_id] = self.pos2rad(motor_position, 2.0)

            return self.bodyFeedbackRes

        except:
            return self.feedbackBodyMotors(req)
    
    def feedbackHeadMotors(self, req):
        try:
            self.headFeedbackRes.pos_vector = [0]*2

            for motor_id in range(2):
                if motor_id in self.AX_12_MOTORS:
                    motor_position, comm, hard = self.packetHandler1.read2ByteTxRx(self.portHandler, motor_id+18, PROTOCOL_1_INFOS['PRES_POS_ADDR'])

                    if comm !=0 or hard != 0:
                        self.headFeedbackRes.pos_vector[motor_id] = -1
                    else:
                        self.headFeedbackRes.pos_vector[motor_id] = self.pos2rad(motor_position, 1.0)
                
                elif (motor_id in self.MX_106_MOTORS) or (motor_id in self.MX_64_MOTORS):
                    motor_position, comm, hard = self.packetHandler2.read2ByteTxRx(self.portHandler, motor_id+18, PROTOCOL_2_INFOS['PRES_POS_ADDR'])

                    if comm !=0 or hard != 0:
                        self.headFeedbackRes.pos_vector[motor_id] = -1
                    else:
                        self.headFeedbackRes.pos_vector[motor_id] = self.pos2rad(motor_position, 2.0)

            return self.headFeedbackRes

        except:
            return self.feedbackHeadMotors(req)

    def data2body(self, msg):
        if self.AX_12_MOTORS:
            self.bodyGroup1.clearParam()
        if self.MX_106_MOTORS or self.MX_64_MOTORS:
            self.bodyGroup2.clearParam()

        for motor_id in range(18):
            motor_position = msg.pos_vector[motor_id]

            if motor_id in self.AX_12_MOTORS:
                value = self.rad2pos(motor_position, 1.0)
                bytes_value = value.to_bytes(2, byteorder='little')

                self.bodyGroup1.addParam(motor_id, bytes_value)

            elif (motor_id in self.MX_106_MOTORS) or (motor_id in self.MX_64_MOTORS):
                value = self.rad2pos(motor_position, 2.0)
                bytes_value = value.to_bytes(4, byteorder='little')

                self.bodyGroup2.addParam(motor_id, bytes_value)

        if self.AX_12_MOTORS:
            self.bodyGroup1.txPacket()
        if self.MX_106_MOTORS or self.MX_64_MOTORS:
            self.bodyGroup2.txPacket()

    def data2head(self, msg):
        if self.AX_12_MOTORS:
            self.headGroup1.clearParam()
        if self.MX_106_MOTORS or self.MX_64_MOTORS:
            self.headGroup2.clearParam()

        for motor_id in range(2):
            motor_position = msg.pos_vector[motor_id]

            if motor_id in self.AX_12_MOTORS:
                value = self.rad2pos(motor_position, 1.0)
                bytes_value = value.to_bytes(2, byteorder='little')

                self.headGroup1.addParam(motor_id+18, bytes_value)

            elif (motor_id in self.MX_106_MOTORS) or (motor_id in self.MX_64_MOTORS):
                value = self.rad2pos(motor_position, 2.0)
                bytes_value = value.to_bytes(4, byteorder='little')

                self.headGroup2.addParam(motor_id+18, bytes_value)

        if self.AX_12_MOTORS:
            self.headGroup1.txPacket()
        if self.MX_106_MOTORS or self.MX_64_MOTORS:
            self.headGroup2.txPacket()
    
    def rad2pos(self, pos_in_rad, motor_protocol):
        
        if motor_protocol == 1.0:
            motor_position = int(195.379*pos_in_rad + 512)
            motor_position = min(motor_position, 1023)
            motor_position = max(motor_position, 0)

        elif motor_protocol == 2.0:
            motor_position = int(651.739*pos_in_rad + 2047.5)
            motor_position = min(motor_position, 4095)
            motor_position = max(motor_position, 0)
        
        return motor_position

    def pos2rad(self, motor_position, motor_protocol):

        if motor_protocol == 1.0:
            pos_in_rad = (motor_position-512)/195.379
        
        elif motor_protocol == 2.0:
            pos_in_rad = (motor_position-2047.5)/651.739

        return pos_in_rad

    def run(self):
        rospy.spin()

if __name__ == '__main__':

    u2d2 = u2d2Control()
    u2d2.run()