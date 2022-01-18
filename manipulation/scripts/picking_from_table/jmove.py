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
    
    T_B_Jar = velma.getTf("Wo", "frame_01")

    if T_B_Jar is None:
        exitError(997)

    flips = []
    for flip_shoulder in (True, False):
        for flip_elbow in (True, False):
            for flip_ee in (True, False):
                flips.append( (flip_shoulder, flip_elbow, flip_ee) )
    solv = KinematicsSolverVelma()
    torso_angle = 0.0
    arm_name = "left"


    if arm_name == 'right':
        central_point = PyKDL.Vector( 0.7, -0.7, 1.4 )
    else:
        central_point = PyKDL.Vector( 0.7, 0.7, 1.4 )

    torso_angle = 0.0


    js = velma.getLastJointState()[1]

    xyz = (0.43378, 0.4633, 0.97565)
    rpy = (0,0,0)

    rot = PyKDL.Rotation.Quaternion(0.039055, 0.016422, 0.9991, 0.00098)
    rot.GetRPY()
    
    # T_B_A7d = PyKDL.Frame(PyKDL.Rotation.RPY(*rot.GetRPY()), PyKDL.Vector(*xyz))
    print(T_B_Jar)
    arm_q = []
    for flip_shoulder, flip_elbow, flip_ee in flips:
        for elbow_circle_angle in np.linspace(-math.pi, math.pi, 20):
            q = solv.calculateIkArm(arm_name, T_B_Jar, torso_angle, elbow_circle_angle, flip_shoulder, flip_elbow, flip_ee)

            print(q[0])
            if not q[0] is None:
                q_dict = js.copy()
                for i in range(len(q)):
                    if i > 0:
                        q_dict['right_arm_{}_joint'.format(i)] = q[i]
                    else:
                        q_dict['torso_0_joint'] = q[i]
                arm_q.append(q_dict)
    
    # print(q_dict)
