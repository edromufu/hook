#!/usr/bin/env python3
#coding=utf-8

import numpy as np
import copy

import sys, os
hook_dir = '/home/'+os.getlogin()+'/hook/src/'

sys.path.append(hook_dir+'movement/kinematic_functions/src')
from ik_numerical import InverseKinematics

def callIK(robot, newFootAbsPosition, newFootAbsPosture, currentFoot):
    robotIK = copy.deepcopy(robot)
        
    joint_angles = [0]*len(robot)
    try:
        joint_angles = InverseKinematics(newFootAbsPosition, newFootAbsPosture, currentFoot, robotIK)
    except Exception as e:
        print(e)

    return joint_angles

def moveGripper(robot, deltaX, deltaZ, deltaPitch):

    for motor in robot:
        if 'GRIPPER' in motor.get_name():
            gripperInitialPosture = motor.absolutePosture
            gripperInitialPosition = motor.absolutePosition

    newGripperAbsPosition = gripperInitialPosition + np.array([[deltaX, 0, deltaZ]]).T
    
    return callIK(robot, newGripperAbsPosition, gripperInitialPosture, -1)