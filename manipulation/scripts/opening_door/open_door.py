#!/usr/bin/env python
import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
import rospy
from velma_common.velma_interface import VelmaInterface,isConfigurationClose
from rcprg_ros_utils import exitError
from tf_conversions import posemath
import tf2_ros
import geometry_msgs.msg
import PyKDL
import numpy as np
from velma_kinematics.velma_ik_geom import KinematicsSolverLWR4, KinematicsSolverVelma
import math

def publish_tf(frame):
        obj_pose = posemath.toMsg(frame)
        print(obj_pose)

        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "abc"
        t.transform.translation.x = obj_pose.position.x - 0.2
        t.transform.translation.y = obj_pose.position.y
        t.transform.translation.z = obj_pose.position.z

        t.transform.rotation = obj_pose.orientation
        br.sendTransform(t)

        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            t.header.stamp = rospy.Time.now()
            print("sending transormation")
            br.sendTransform(t)
            rate.sleep()

def inverse_kinematics(T_B_Wr):
    my_Iks= []
    torso_angles_ik= []
    solver = KinematicsSolverVelma()

    for torso_angle in np.linspace(-1.55, 1.55, 6):
        for elbow_circle_angle in np.linspace(-1.5, 1.5, 30):
            # for T_B_Wr in T_B_Wrs:
            ik = solver.calculateIkLeftArm(T_B_Wr, torso_angle, elbow_circle_angle, False, False, False)
            if None not in ik:
                my_Iks.append(ik)
                torso_angles_ik.append(torso_angle)
    return my_Iks, torso_angles_ik

def choose_possible_goals(IK, torso_angle, velma):
        if IK != None or len(IK)!=0:
            q_map_goals= []
            # dict_joint_limits= velma.getBodyJointLimits()
            for i in range(0, len(IK)):
                q_map_goal = {'torso_0_joint':torso_angle[i], 'right_arm_0_joint':0, 'right_arm_1_joint':0,
                    'right_arm_2_joint':0, 'right_arm_3_joint':0, 'right_arm_4_joint':0, 'right_arm_5_joint':0,
                    'right_arm_6_joint':0,  'left_arm_0_joint':IK[i][0], 'left_arm_1_joint':IK[i][1], 'left_arm_2_joint':IK[i][2],
                'left_arm_3_joint':IK[i][3], 'left_arm_4_joint':IK[i][4], 'left_arm_5_joint':IK[i][5], 'left_arm_6_joint':IK[i][6] }
                
                # if (not self.isJointLimitClose(dict_joint_limits, q_map_goal)):
                q_map_goals.append(q_map_goal)
            
            return q_map_goals

class Conf:

    def __init__(self):
        rospy.init_node('cabinet_position')
        rospy.sleep(0.5)
        self.velma = self.setup_velma()
    
    def setup_velma(self):
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

        diag = velma.getCoreCsDiag()
        if not diag.motorsReady():
            exitError(1, msg="Motors must be homed and ready to use for this test.")

        return velma

    def switch_to_jnt_imp(self):
            print("Switch to jnt_imp mode (no trajectory)...")
            self.velma.moveJointImpToCurrentPos(start_time=0.5)
            error = self.velma.waitForJoint()
            if error != 0:
                exitError(3, msg="The action should have ended without error,"\
                            " but the error code is {}".format(error))


if __name__ == "__main__":
    conf = Conf()
    # res = conf.velma.getTf('Gr', 'Wr')
    # f = inverse_kinematics(res)
    # print(f)

    T_Wo_cabinet = conf.velma.getTf("Wo", "cabinet_handle")
    print(T_Wo_cabinet)
    # T_Wo_cabinet.M.DoRotZ(3.14)

    iks, torsos = inverse_kinematics(T_Wo_cabinet)
    print(len(iks))
    possible_goals = choose_possible_goals(iks,torsos, conf.velma)
    print(possible_goals)
    q_map_intermediate = possible_goals[0]
    fr = KinematicsSolverVelma().getLeftArmFk(torsos[1], iks[1])

    # publish_tf(T_Wo_cabinet)

    conf.velma.moveJoint(q_map_intermediate, 20, start_time=0.5, position_tol=15.0/180.0*math.pi)
    error = conf.velma.waitForJoint()
    if error != 0:
        exitError(10, msg="The action should have ended without error,"\
                        " but the error code is {}".format(error))

    rospy.sleep(0.5)
    js = conf.velma.getLastJointState()
    if not isConfigurationClose(q_map_intermediate, js[1], tolerance=0.1):
        # exitError(10)
        print("CONFIG NOT CLOSE")

    

    # def T_B_Wr_aroud_goal(self, point, velma):
    #     R=0.2
    #     angle_step=math.pi/8 # angle_step
    #     steps = int(round(2*math.pi/angle_step))
    #     T_B_Wr = []
    #     for step in range(steps):
    #         angle = angle_step * step
    #         x = point.x() + R * math.cos(angle)
    #         y = point.y() + R * math.sin(angle)
    #         z = point.z() 
    #         roll = 0
    #         pitch = 0
    #         yaw = self.correct_angle(math.pi + angle)
    #         rotation = PyKDL.Rotation.RPY(roll, pitch, yaw)
    #         vector = PyKDL.Vector(x, y, z)
    #         frame =  PyKDL.Frame(rotation, vector) * PyKDL.Frame(PyKDL.Rotation.RPY(0,math.pi/2,0))
    #         T_B_Wr.append(frame*velma.getTf('Gr', 'Wr'))
    #     return T_B_Wr


