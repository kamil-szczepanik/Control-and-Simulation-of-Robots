#!/usr/bin/env python
import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
import rospy
import math
from velma_common import *
from rcprg_ros_utils import exitError
from rcprg_planner import *
from velma_kinematics.velma_ik_geom import KinematicsSolverVelma
import numpy as np
from tf_conversions import posemath
import PyKDL

class Configer():
    def __init__(self, velma):
        self.velma = velma

    def get_cirle(self, x_offset,y_offset,r=0.3):
        X = []
        Y = []
        points = []
        for theta in np.linspace(-math.pi, math.pi, 20):
            x = r*math.cos(theta) + x_offset
            y = r*math.sin(theta) + y_offset
            X.append(x)
            Y.append(y)
            points.append((x,y))
        return points

    def get_frames(self):
        T_B_Jar = self.velma.getTf("B", "jar_hollow")
        print(T_B_Jar)
        obj_pose = posemath.toMsg(T_B_Jar)
        print(obj_pose)
        frames = []
        x_y_points = self.get_cirle(obj_pose.position.x, obj_pose.position.y)
        for x, y in x_y_points:
            vec = PyKDL.Vector(x,y,obj_pose.position.z+ 0.05)
            orient = PyKDL.Rotation.RPY(0,0, math.atan2((obj_pose.position.y-y),(obj_pose.position.x-x)))
            frame =  PyKDL.Frame(orient, vec)
            frames.append(frame)
        return frames


        
    def get_flips(self):
        flips = []
        for flip_shoulder in (True, False):
            for flip_elbow in (True, False):
                for flip_ee in (True, False):
                    flips.append( (flip_shoulder, flip_elbow, flip_ee))

        return flips
    def get_confs(self):
        solv = KinematicsSolverVelma()
        flips = self.get_flips()
        arm_q = []
        frames = self.get_frames()
        js = self.velma.getLastJointState()[1]
        for frame in frames:
            for flip_shoulder, flip_elbow, flip_ee in flips:
                for torso_angle in np.linspace(-math.pi/2,math.pi/2,5):
                    for elbow_circle_angle in np.linspace(-math.pi, math.pi, 20):
                        arm_conf = solv.calculateIkLeftArm(frame, torso_angle, elbow_circle_angle, flip_shoulder, flip_elbow, flip_ee)
                        if not arm_conf[0] is None:
                            print(arm_conf)

                            q_dict = js.copy()
                            for i in range(len(arm_conf)):
                                if i > 0:
                                    q_dict['left_arm_{}_joint'.format(i)] = arm_conf[i]
                                else:
                                    q_dict['torso_0_joint'] = arm_conf[i]
                            arm_q.append(q_dict)
        return arm_q



if __name__ == "__main__":
    rospy.init_node('mug_position')

    rospy.sleep(0.5)
    print("Running python interface for Velma...")
    velma = VelmaInterface()
    print("Waiting for VelmaInterface initialization...")
    if not velma.waitForInit(timeout_s=10.0):
        exitError(1, msg="Could not initialize VelmaInterface")
    print("Initialization ok!")

    conf = Configer(velma)
    confs = conf.get_confs()



    # T_B_Jar = velma.getTf("B", "jar_hollow")



    # if T_B_Jar is None:
    #     exitError(997)

    # # flips = []
    # # for flip_shoulder in (True, False):
    # #     for flip_elbow in (True, False):
    # #         for flip_ee in (True, False):
    # #             flips.append( (flip_shoulder, flip_elbow, flip_ee) )
    # solv = KinematicsSolverVelma()
    # # torso_angle = 0.0
    # # arm_name = "left"


    # # if arm_name == 'right':
    # #     central_point = PyKDL.Vector( 0.7, -0.7, 1.4 )
    # # else:
    # #     central_point = PyKDL.Vector( 0.7, 0.7, 1.4 )

    # # torso_angle = 0.0


    # js = velma.getLastJointState()[1]

    # xyz = (0.43378, 0.4633, 0.97565)
    # rpy = (0,0,0)

    # rot = PyKDL.Rotation.Quaternion(0.039055, 0.016422, 0.9991, 0.00098)
    # rot.GetRPY()
    
    # # T_B_A7d = PyKDL.Frame(PyKDL.Rotation.RPY(*rot.GetRPY()), PyKDL.Vector(*xyz))
    # print(T_B_Jar)
    # arm_q = []

    # for flip_shoulder, flip_elbow, flip_ee in flips:
    #     for elbow_circle_angle in np.linspace(-math.pi, math.pi, 20):
    #         q = solv.calculateIkArm(arm_name, T_B_Jar, torso_angle, elbow_circle_angle, flip_shoulder, flip_elbow, flip_ee)

    #         print(q[0])
    #         if not q[0] is None:
    #             q_dict = js.copy()
    #             for i in range(len(q)):
    #                 if i > 0:
    #                     q_dict['right_arm_{}_joint'.format(i)] = q[i]
    #                 else:
    #                     q_dict['torso_0_joint'] = q[i]
    #             arm_q.append(q_dict)
    
    # print(q_dict)

