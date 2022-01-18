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

    def switch_to_jimp(self):
        print ("Switch to jnt_imp mode (no trajectory)...")
        self.velma.moveJointImpToCurrentPos(start_time=0.5)
        error = self.velma.waitForJoint()
        if error != 0:
            print ("The action should have ended without error, but the error code is", error)
            exitError(3)

        rospy.sleep(0.5)

        diag = self.velma.getCoreCsDiag()
        if not diag.inStateJntImp():
            print ("The core_cs should be in jnt_imp state, but it is not")
            exitError(3)

    def move_to_init_pose(self, planner):
        init_pose = {
            'torso_0_joint':0,
            'right_arm_0_joint':-0.3,
            'right_arm_1_joint':-1.8,
            'right_arm_2_joint':1.25,
            'right_arm_3_joint':0.85,
            'right_arm_4_joint':0,
            'right_arm_5_joint':-0.5,
            'right_arm_6_joint':0,
            'left_arm_0_joint':0.3,
            'left_arm_1_joint':1.8,
            'left_arm_2_joint':-1.25,
            'left_arm_3_joint':-0.85,
            'left_arm_4_joint':0,
            'left_arm_5_joint':0.5,
            'left_arm_6_joint':0
            }
        return self.plan_and_move([init_pose], planner)

    def plan_and_move(self, qs, planner, collision_object=None):
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


    def move_to_handle(self, planner, solver, hand):
        frames = self.get_pose_in_front(hand)
        frames = self.gripper_transform(frames, hand)

        pose_arr = PoseArray()
        pose_arr.header.frame_id = 'world'
        poses = []
        IKs, torsos = self.generate_iks(hand, frames, solver)
        for IK, torso in zip(IKs, torsos):
            if hand == 'right':
                frame = solver.getRightArmFk(torso, IK)
            elif hand == 'left':
                frame = solver.getLeftArmFk(torso, IK)
            p = Pose()
            p.position.x = frame.p.x()
            p.position.y = frame.p.y()
            p.position.z = frame.p.z()
            x,y,z,w = frame.M.GetQuaternion()
            p.orientation.x = x
            p.orientation.y = y
            p.orientation.z = z
            p.orientation.w  = w
            poses.append(p)
            pose_arr.poses = poses

        pub = rospy.Publisher('ik_poses', PoseArray, queue_size=10)
        rate = rospy.Rate(10)
        for i in range(5):
            pub.publish(pose_arr)
            rate.sleep()
        
        qs = self.iks_to_qs(IKs, torsos, hand)
        return self.plan_and_move(qs, planner)


    def generate_iks(self, hand, frames, solver):
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

    def iks_to_qs(self, iks, torsos, hand):
        qs = []
        for torso, IK in zip(torsos, iks):
            qs.append(self.ik_to_arm_q(IK, torso, hand))
        return qs

    def ik_to_arm_q(self, ik, torso, hand):
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

    def normalize_angle(self, angle):
        angle =  angle % (2*math.pi)

        angle = (angle + 2*math.pi) % (2*math.pi)

        if angle > math.pi:  
            angle -= 2*math.pi
        return angle

    def dist(self, p1, p2):
        return math.sqrt((p1.x()-p2.x())**2 + (p1.y()-p2.y())**2 + (p1.z()-p2.z())**2)

    def gripper_transform(self, frames, hand):
        new_frames = []
        if hand == 'right':
            transformation = self.velma.getTf('Gr', 'Wr')
        elif hand == 'left':
            transformation = self.velma.getTf('Gl', 'Wl')
        for frame in frames:
            new_frames.append(frame * transformation)
        return new_frames

    def generate_poses_around(self, point, hand, R=0.15, angle_step=math.pi/8):
        steps = int(round(2*math.pi/angle_step))
        frames = []
        for step in range(steps):
            angle = angle_step * step
            x = point.x() + R * math.cos(angle)
            y = point.y() + R * math.sin(angle)
            z = point.z()
            roll = 0.0
            pitch = 0.0
            yaw = self.normalize_angle(math.pi + angle)
            rotation = PyKDL.Rotation.RPY(roll, pitch, yaw)
            vector = PyKDL.Vector(x, y, z)
            if hand == 'right':
                frame = PyKDL.Frame(rotation, vector) * PyKDL.Frame(PyKDL.Rotation.RPY(0,math.pi/2,0))
            elif hand == 'left':
                frame =  PyKDL.Frame(rotation, vector) * PyKDL.Frame(PyKDL.Rotation.RPY(0,math.pi/2,0)) * PyKDL.Frame(PyKDL.Rotation.RPY(0,0,math.pi))
            frames.append(frame)
        return frames

    def get_pose_in_front(self, hand):
        frames = []

        handle_frame = self.velma.getTf('B', 'right_handle').p

        x = handle_frame.x() - 0.1
        y = handle_frame.y()
        z = handle_frame.z() + 0.08

        roll = 0
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

    def get_tool_pose(self, velma_pose, solver, hand):
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

    def select_hand(self, jar_pose, velma_pose, solver):
        right_effector_jar_dist = self.dist(jar_pose.p, self.get_tool_pose(velma_pose, solver, 'right'))
        left_effector_jar_dist = self.dist(jar_pose.p, self.get_tool_pose(velma_pose, solver, 'left'))
        if left_effector_jar_dist < right_effector_jar_dist:
            return 'left'
        else:
            return 'right'

    def create_object(self, hand):
        object1 = AttachedCollisionObject()
        object1.link_name = hand + "_HandGripLink"
        object1.object.header.frame_id = hand + "_HandGripLink"
        object1.object.id = "object1"
        object1_prim = SolidPrimitive()
        object1_prim.type = SolidPrimitive.CYLINDER
        object1_prim.dimensions=[None, None]    # set initial size of the list to 2
        object1_prim.dimensions[SolidPrimitive.CYLINDER_HEIGHT] = rospy.get_param("/velma_mover/object/h") - 0.025
        object1_prim.dimensions[SolidPrimitive.CYLINDER_RADIUS] = rospy.get_param("/velma_mover/object/r")
        object1_pose = pm.toMsg(PyKDL.Frame(PyKDL.Rotation.RotY(math.pi/2)))
        object1.object.primitives.append(object1_prim)
        object1.object.primitive_poses.append(object1_pose)
        object1.object.operation = CollisionObject.ADD
        if hand == 'right':
            object1.touch_links = ['right_HandPalmLink',
                'right_HandFingerOneKnuckleOneLink',
                'right_HandFingerOneKnuckleTwoLink',
                'right_HandFingerOneKnuckleThreeLink',
                'right_HandFingerTwoKnuckleOneLink',
                'right_HandFingerTwoKnuckleTwoLink',
                'right_HandFingerTwoKnuckleThreeLink',
                'right_HandFingerThreeKnuckleTwoLink',
                'right_HandFingerThreeKnuckleThreeLink']
        elif hand == 'left':
            object1.touch_links = ['left_HandPalmLink',
                'left_HandFingerOneKnuckleOneLink',
                'left_HandFingerOneKnuckleTwoLink',
                'left_HandFingerOneKnuckleThreeLink',
                'left_HandFingerTwoKnuckleOneLink',
                'left_HandFingerTwoKnuckleTwoLink',
                'left_HandFingerTwoKnuckleThreeLink',
                'left_HandFingerThreeKnuckleTwoLink',
                'left_HandFingerThreeKnuckleThreeLink']

        return object1
