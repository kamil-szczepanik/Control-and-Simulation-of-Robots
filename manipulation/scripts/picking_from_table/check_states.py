#!/usr/bin/env python

import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
import rospy
import math
from velma_common import *
from rcprg_ros_utils import exitError
from rcprg_planner import *
from velma_kinematics.velma_ik_geom import KinematicsSolverVelma
import numpy as np

import math 
 

rospy.init_node('check_states')

rospy.sleep(0.5)
print("Running python interface for Velma...")
velma = VelmaInterface()
print("Waiting for VelmaInterface initialization...")
if not velma.waitForInit(timeout_s=10.0):
    exitError(1, msg="Could not initialize VelmaInterface")
print("Initialization ok!")
print("Motors must be enabled every time after the robot enters safe state.")
print("If the motors are already enabled, enabling them has no effect.")
print("Enabling motors...")
if velma.enableMotors() != 0:
    exitError(2, msg="Could not enable motors")
rospy.sleep(0.5)
js_init = velma.getLastJointState()
print(js_init[1])