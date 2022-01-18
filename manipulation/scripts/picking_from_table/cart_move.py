#!/usr/bin/env python
import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
import rospy
import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
import rospy
import math
from velma_common import *
from rcprg_ros_utils import exitError
from rcprg_planner import *
from velma_kinematics.velma_ik_geom import KinematicsSolverVelma
import numpy as np
from tf_conversions import posemath

import math

if __name__ == "__main__":
    rospy.init_node('mug_position')

    rospy.sleep(0.5)
    print("Running python interface for Velma...")
    velma = VelmaInterface()
    print("Waiting for VelmaInterface initialization...")
    if not velma.waitForInit(timeout_s=10.0):
        exitError(1, msg="Could not initialize VelmaInterface")
    print("Initialization ok!")

    T_B_Jar = velma.getTf("B", "jar_hollow")

    print(T_B_Jar)


    obj_pose = posemath.toMsg(T_B_Jar)
    print(obj_pose)

    approach_pose = posemath.Pose()
    approach_pose.position.x = obj_pose.position.x - 0.5
    approach_pose.position.y = obj_pose.position.y
    approach_pose.position.z = obj_pose.position.z + 0.1
    approach_pose.orientation = posemath.Quaternion(0.0, 0.0, 1, 0.0)
    print(approach_pose)

    approach_pose_frame = posemath.fromMsg(approach_pose)
   


    
    
   
    print "Moving left wrist to pose defined in world frame..."
    T_B_Trd = PyKDL.Frame(approach_pose_frame)
    if not velma.moveCartImpLeft([T_B_Trd], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(8)
    if velma.waitForEffectorLeft() != 0:
        exitError(9)
    rospy.sleep(0.5)
    print "calculating difference between desiread and reached pose..."
    T_B_T_diff = PyKDL.diff(T_B_Trd, velma.getTf("B", "Wl"), 1.0)
    print T_B_T_diff
    if T_B_T_diff.vel.Norm() > 0.05 or T_B_T_diff.rot.Norm() > 0.05:
        exitError(10)


    exitError(0)
