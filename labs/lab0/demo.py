"""
Date: 08/26/2021

Purpose: This script creates an ArmController and uses it to command the arm's
joint positions and gripper.

Try changing the target position to see what the arm does!

"""

import sys
import rospy
import numpy as np
from math import pi

from core.interfaces import ArmController

rospy.init_node('demo')

arm = ArmController()
arm.set_arm_speed(0.2)

arm.close_gripper()

q = arm.neutral_position()
arm.safe_move_to_position(q)
arm.open_gripper()

q = np.array([0,-1 ,0,-2,0,1,1]) # TODO: try changing this!
arm.safe_move_to_position(q)

q = np.array([1.5,1 ,1,-0.5,0.5,2,0]) # TODO: 1
arm.safe_move_to_position(q)

q = np.array([1.5,-1 ,-1,-2,0,1,2]) # TODO: 2
arm.safe_move_to_position(q)

q = np.array([2,1 ,-1,-1,0,1,2.8]) # TODO: 3
arm.safe_move_to_position(q)

q = np.array([2,-1 ,-2,-2,0,2,2.8]) # TODO: 4
arm.safe_move_to_position(q)

arm.close_gripper()
