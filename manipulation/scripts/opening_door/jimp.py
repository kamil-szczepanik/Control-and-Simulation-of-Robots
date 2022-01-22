#!/usr/bin/env python
import math
import PyKDL
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, PoseArray
from visualization_msgs.msg import Marker
import tf_conversions.posemath as pm
import rospy
from rcprg_ros_utils import exitError, MarkerPublisher
from velma_common import *
from rcprg_planner import *
from velma_kinematics.velma_ik_geom import KinematicsSolverVelma

import numpy as np


class Jimp:

    def __init__(self, velma):
        self.velma = velma

    def planAndMove(self, qs, planner, collision_object=None):
        qs = [qMapToConstraints(q, 0.01, group=self.velma.getJointGroup("impedance_joints")) for q in qs]
        for i in range(10):
            rospy.sleep(0.1)
            js = self.velma.getLastJointState()

            print("Planning (try", i, ")...")
            if collision_object is None:
                trajectory = planner.plan(js[1], qs, "impedance_joints", num_planning_attempts=10, max_velocity_scaling_factor=0.15, planner_id="RRTConnect")
            else:
                trajectory = planner.plan(js[1], qs, "impedance_joints", num_planning_attempts=10, max_velocity_scaling_factor=0.15, planner_id="RRTConnect", 
                                    attached_collision_objects=[collision_object])

            if trajectory == None:
                continue
            print("Executing trajectory...")
            if not self.velma.moveJointTraj(trajectory, start_time=0.5, position_tol=15.0/180.0 * math.pi, velocity_tol=15.0/180.0*math.pi):
                exitError(5)
            if self.velma.waitForJoint() == 0:
                return True
            else:
                print("The trajectory could not be completed, retrying...")
                continue
        return False


    def moveHandToHandle(self, planner, solver, hand):
        frames = self.getHandleGripPose(hand)
        frames = self.getGripperTransform(frames, hand)

        pose_arr = PoseArray()
        pose_arr.header.frame_id = 'world'
        poses = []
        IKs, torsos = self.generateInverseKinematics(hand, frames, solver)
        
        qs = self.InverseKinematicsToQs(IKs, torsos, hand)
        return self.planAndMove(qs, planner)


    def generateInverseKinematics(self, hand, frames, solver):
        iks = []
        torsos = []
        for torso in np.linspace(-1.55, 1.55, 6):
            for elbow_ang in np.linspace(-1.55, 1.55, 6):
                for frame in frames:
                    ik = solver.calculateIkArm(hand, frame, torso, elbow_ang, False, False, False)
                    if None not in ik:
                        iks.append(ik)
                        torsos.append(torso)
        return iks, torsos

    def InverseKinematicsToQs(self, iks, torsos, hand):
        qs = []
        for torso, IK in zip(torsos, iks):
            qs.append(self.InverseKinematicsToArmQ(IK, torso, hand))
        return qs

    def InverseKinematicsToArmQ(self, ik, torso, hand):
        q = {
            'torso_0_joint': torso,
            hand + '_arm_0_joint': ik[0],
            hand + '_arm_1_joint': ik[1],
            hand + '_arm_2_joint': ik[2],
            hand + '_arm_3_joint': ik[3],
            hand + '_arm_4_joint': ik[4],
            hand + '_arm_5_joint': ik[5],
            hand + '_arm_6_joint': ik[6]
            }
        return q


    def getGripperTransform(self, frames, hand):
        new_frames = []
        if hand == 'right':
            transformation = self.velma.getTf('Gr', 'Wr')
        elif hand == 'left':
            transformation = self.velma.getTf('Gl', 'Wl')
        for frame in frames:
            new_frames.append(frame * transformation)
        return new_frames


    def getHandleGripPose(self, hand):
        frames = []

        handle_frame = self.velma.getTf('B', 'cabinet_handle').p

        x = handle_frame.x() - 0.1
        y = handle_frame.y() + 0.01
        z = handle_frame.z() + 0.08

        roll = 0
        if hand == 'left':
            roll = 3.14
        pitch = 0
        yaw = 0
        rotation = PyKDL.Rotation.RPY(roll, pitch, yaw)
        vector = PyKDL.Vector(x, y, z)
        if hand == 'right':
            frame = PyKDL.Frame(rotation, vector) * PyKDL.Frame(PyKDL.Rotation.RPY(0,math.pi/2,0))
        elif hand == 'left':
            frame =  PyKDL.Frame(rotation, vector) * PyKDL.Frame(PyKDL.Rotation.RPY(0,math.pi/2,0)) * PyKDL.Frame(PyKDL.Rotation.RPY(0,0,math.pi))
        frames.append(frame)
        return frames


    def getToolPose(self, velma_pose, solver, hand):
        q = (
            velma_pose[hand + "_arm_0_joint"],
            velma_pose[hand + "_arm_1_joint"],
            velma_pose[hand + "_arm_2_joint"],
            velma_pose[hand + "_arm_3_joint"],
            velma_pose[hand + "_arm_4_joint"],
            velma_pose[hand + "_arm_5_joint"],
            velma_pose[hand + "_arm_6_joint"]
        )
        torso_angle = velma_pose["torso_0_joint"]
        return solver.getLeftArmFk(torso_angle, q).p
