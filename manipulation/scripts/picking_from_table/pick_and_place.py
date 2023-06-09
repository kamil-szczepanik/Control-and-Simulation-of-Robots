#!/usr/bin/env python

import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
import rospy
import math
from velma_common import *
from rcprg_ros_utils import exitError
from rcprg_planner import *
from velma_kinematics.velma_ik_geom import KinematicsSolverVelma
from tf_conversions import posemath
import numpy as np

import math

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
                for torso_angle in np.linspace(-1,1,5):
                    for elbow_circle_angle in np.linspace(-2, 2, 20):
                        arm_conf = solv.calculateIkLeftArm(frame, torso_angle, elbow_circle_angle, flip_shoulder, flip_elbow, flip_ee)
                        if not arm_conf[0] is None:
                            print(arm_conf)

                            q_dict = js.copy()
                            q_dict['torso_0_joint'] = torso_angle
                            for i in range(len(arm_conf)):
                                # if i > 0:
                                q_dict['left_arm_{}_joint'.format(i)] = arm_conf[i]
                                # else:
                                    
                            arm_q.append(q_dict)
        return arm_q

class StateMachine:
    
    def __init__(self):
        self.handlers = {}
        self.startState = None
        self.endStates = []

    def add_state(self, name, handler, end_state=0):
        name = name.upper()
        self.handlers[name] = handler
        if end_state:
            self.endStates.append(name)

    def set_start(self, name):
        self.startState = name.upper()

    def run(self, cargo):
        try:
            handler = self.handlers[self.startState]
        except:
            print("Initialize Error")
    
        while True:
            (newState, cargo) = handler(cargo)
            if newState.upper() in self.endStates:
                print("reached ", newState)
                break 
            else:
                handler = self.handlers[newState.upper()]  


def initialization(velma):
    global p
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


    q_map_starting = {'torso_0_joint':0,
        'right_arm_0_joint':-0.3,   'left_arm_0_joint':0.3,
        'right_arm_1_joint':-1.8,   'left_arm_1_joint':1.8,
        'right_arm_2_joint':1.25,   'left_arm_2_joint':-1.25,
        'right_arm_3_joint':0.85,   'left_arm_3_joint':-0.85,
        'right_arm_4_joint':0,      'left_arm_4_joint':0,
        'right_arm_5_joint':-0.5,   'left_arm_5_joint':0.5,
        'right_arm_6_joint':0,      'left_arm_6_joint':0 }

    q_map_1 = {'torso_0_joint':0.0,
        'right_arm_0_joint':-0.3, 'left_arm_0_joint':0.3,
        'right_arm_1_joint':-1.8, 'left_arm_1_joint':1.57,
        'right_arm_2_joint':1.25, 'left_arm_2_joint':-1.57,
        'right_arm_3_joint':0.85, 'left_arm_3_joint':-1.7,
        'right_arm_4_joint':0,    'left_arm_4_joint':0.0,
        'right_arm_5_joint':-0.5, 'left_arm_5_joint':1.57,
        'right_arm_6_joint':0,    'left_arm_6_joint':0.0 }

    rospy.sleep(0.5)


    diag = velma.getCoreCsDiag()
    if not diag.motorsReady():
        print "Motors must be homed and ready to use for this test."
        exitError(1)



    p = Planner(velma.maxJointTrajLen())
    if not p.waitForInit():
        print "could not initialize PLanner"
        exitError(2)
    print "Planner init ok"
    oml = OctomapListener("/octomap_binary")
    rospy.sleep(1.0)
    octomap = oml.getOctomap(timeout_s=5.0)
    # p.processWorld(octomap)
    print("octomap connected successfully")


    print "Switch to jnt_imp mode (no trajectory)..."
    velma.moveJointImpToCurrentPos(start_time=0.2)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(3)

    rospy.sleep(0.5)
    diag = velma.getCoreCsDiag()
    if not diag.inStateJntImp():
        print "The core_cs should be in jnt_imp state, but it is not"
        exitError(3)

    rospy.sleep(1.0)

    print("Moving to the starting position...")

    planAndExecute(q_map_starting, p)


    print "Checking if the starting configuration is as expected..."
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_map_starting, js[1], tolerance=0.2):
        print "This test requires starting pose:"
        print q_map_starting
        exitError(10)

    # get initial configuration
    js_init = velma.getLastJointState()

    # planAndExecute(q_map_1, p)
    conf = Configer(velma)
    global cnt_err
    planAndExecute_list(conf.get_confs(), p)

    print "Switch to cart_imp mode (no trajectory)..."
    if not velma.moveCartImpRightCurrentPos(start_time=0.2):
        exitError(8)
    if velma.waitForEffectorRight() != 0:
        exitError(9)

    rospy.sleep(0.5)

    diag = velma.getCoreCsDiag()
    if not diag.inStateCartImp():
        print "The core_cs should be in cart_imp state, but it is not"
        exitError(3)

    print "Reset tools for both arms..."
    T_B_Wr = velma.getTf("B", "Wr")
    T_B_Wl = velma.getTf("B", "Wl")
    if not velma.moveCartImpRight([T_B_Wr], [0.1], [PyKDL.Frame()], [0.1], None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(8)
    if not velma.moveCartImpLeft([T_B_Wl], [0.1], [PyKDL.Frame()], [0.1], None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(8)
    if velma.waitForEffectorRight() != 0:
        exitError(9)
    if velma.waitForEffectorLeft() != 0:
        exitError(9)

    newState = "Approach_to_object"
    return (newState, velma)

def planAndExecute(q_dest, p):
    global cnt_err
    cnt_err = 0
    print "Planning motion to the goal position using set of all joints..."
    print "Moving to valid position, using planned trajectory."
    goal_constraint = qMapToConstraints(q_dest, 0.01, group=velma.getJointGroup("impedance_joints"))
    for i in range(5):
        rospy.sleep(0.5)
        js = velma.getLastJointState()
        print "Planning (try", i, ")..."
        traj = p.plan(js[1], [goal_constraint], "impedance_joints", max_velocity_scaling_factor=0.15, planner_id="RRTConnect")
        print("cnt increment")
        cnt_err+=1
        if traj == None:
            continue
        print "Executing trajectory..."
        if not velma.moveJointTraj(traj, start_time=0.5):
            exitError(5)
        if velma.waitForJoint() == 0:
            break
        else:
            print ("The trajectory could not be completed, retrying...")
            
            continue
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_dest, js[1]):

        exitError(6)


def planAndExecute_list(q_dest_list, p):
    for q_dest in q_dest_list:
        print "Planning motion to the goal position using set of all joints..."
        print "Moving to valid position, using planned trajectory."
        goal_constraint = qMapToConstraints(q_dest, 0.01, group=velma.getJointGroup("impedance_joints"))
        for i in range(5):
            rospy.sleep(0.5)
            js = velma.getLastJointState()
            print "Planning (try", i, ")..."
            traj = p.plan(js[1], [goal_constraint], "impedance_joints", max_velocity_scaling_factor=0.15, planner_id="RRTConnect")
            if traj == None:
                continue
            print "Executing trajectory..."
            if not velma.moveJointTraj(traj, start_time=0.5):
                exitError(5)
            if velma.waitForJoint() == 0:
                break
            else:
                print ("The trajectory could not be completed, retrying...")
                
                continue
        rospy.sleep(0.5)
        js = velma.getLastJointState()
        if not isConfigurationClose(q_dest, js[1]):
            print("nie udalo sie")
            # exitError(6)


def jmove(velma):
    global p
    
    conf = Configer(velma)

    planAndExecute(conf.get_confs()[cnt_err], p)

    newState = "Departure_from_object"
    return (newState, velma)



def grab_jar(velma):
    dest_q = [math.radians(180), math.radians(180), math.radians(180), math.radians(0)]
    velma.moveHandLeft(dest_q, [1, 1, 1, 1], [8000, 8000, 8000, 8000], 100000000, hold=False)
    if velma.waitForHandLeft() != 0:
        exitError(2)


    print("Jar grabbed succesfully")

    newState = "Departure_from_object"
    return (newState, velma)

def approach_to_object(velma):


    T_B_Jar = velma.getTf("B", "jar_hollow")

    obj_pose = posemath.toMsg(T_B_Jar)
    # print(obj_pose)

    approach_pose = posemath.Pose()
    approach_pose.position.x = obj_pose.position.x - 0.4
    approach_pose.position.y = obj_pose.position.y
    approach_pose.position.z = obj_pose.position.z + 0.1
    approach_pose.orientation = obj_pose.orientation

    grab_pose = posemath.Pose()
    grab_pose.position.x = obj_pose.position.x - 0.25
    grab_pose.position.y = obj_pose.position.y
    grab_pose.position.z = obj_pose.position.z + 0.1
    grab_pose.orientation = obj_pose.orientation

    print '\nObject pose:\n' , approach_pose


    print "Moving left wrist to pose defined in world frame..."
    T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.039055, 0.016422, 0.9991, 0.00098 ), PyKDL.Vector( approach_pose.position.x, approach_pose.position.y, approach_pose.position.z ))
    if not velma.moveCartImpLeft([T_B_Trd], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(8)
    if velma.waitForEffectorLeft() != 0:
        exitError(9)
    rospy.sleep(0.5)
    print "calculating difference between desiread and reached pose..."
    T_B_T_diff = PyKDL.diff(T_B_Trd, velma.getTf("B", "Wl"), 1.0)
    print T_B_T_diff
    print T_B_T_diff.vel.Norm()
    print T_B_T_diff.rot.Norm()
    if T_B_T_diff.vel.Norm() > 0.05 or T_B_T_diff.rot.Norm() > 0.15:
        exitError(10)

    print "Moving left wrist to pose defined in world frame..."
    T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.039, 0.016, 1, 0.0 ), PyKDL.Vector( grab_pose.position.x, grab_pose.position.y, grab_pose.position.z ))
    if not velma.moveCartImpLeft([T_B_Trd], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(8)
    if velma.waitForEffectorLeft() != 0:
        exitError(9)
    rospy.sleep(0.5)
    print "calculating difference between desiread and reached pose..."
    T_B_T_diff = PyKDL.diff(T_B_Trd, velma.getTf("B", "Wl"), 1.0)
    print T_B_T_diff
    print T_B_T_diff.vel.Norm()
    print T_B_T_diff.rot.Norm()
    if T_B_T_diff.vel.Norm() > 0.05 or T_B_T_diff.rot.Norm() > 0.15:
        exitError(10)

    newState = "Grab_object"
    return (newState, velma)

def departure_from_object(velma):
    global p
    q_map_2 = {'torso_0_joint':-0.6674,
    'right_arm_0_joint':0.4311,  'left_arm_0_joint':-0.2817,
    'right_arm_1_joint':-1.8279, 'left_arm_1_joint':1.768,
    'right_arm_2_joint':0.1529,  'left_arm_2_joint':-1.4637,
    'right_arm_3_joint':0.46215, 'left_arm_3_joint':-1.37117,
    'right_arm_4_joint':0.762,   'left_arm_4_joint':-0.4979,
    'right_arm_5_joint':-0.551,  'left_arm_5_joint':1.50468,
    'right_arm_6_joint':-0.0487, 'left_arm_6_joint':0.8559 }
    print("DEPARTURE START")

    print "Moving left wrist to pose defined in world frame..."
    T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.039055, 0.016422, 0.9991, 0.00098 ), PyKDL.Vector( 0.43, 0.46, 0.97+0.2 ))
    if not velma.moveCartImpLeft([T_B_Trd], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(8)
    if velma.waitForEffectorLeft() != 0:
        exitError(9)
    rospy.sleep(0.5)
    print "calculating difference between desiread and reached pose..."
    T_B_T_diff = PyKDL.diff(T_B_Trd, velma.getTf("B", "Wl"), 1.0)
    print T_B_T_diff
    print T_B_T_diff.vel.Norm()
    print T_B_T_diff.rot.Norm()
    if T_B_T_diff.vel.Norm() > 0.05 or T_B_T_diff.rot.Norm() > 0.15:
        exitError(10)

    print("Moving to second table...")

    planAndExecute(q_map_2, p)
   

    newState = "Approach_to_table2_drop"
    return (newState, velma)



def approach_to_table2_drop(velma):
    print "Moving left wrist to pose defined in world frame..."
    T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.039, 0.016, 1, 0.0 ), PyKDL.Vector( 0.83, -0.1, 0.97 ))
    if not velma.moveCartImpLeft([T_B_Trd], [4.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(8)
    if velma.waitForEffectorLeft() != 0:
        exitError(9)
    rospy.sleep(0.5)
    print "calculating difference between desiread and reached pose..."
    T_B_T_diff = PyKDL.diff(T_B_Trd, velma.getTf("B", "Wl"), 1.0)
    print T_B_T_diff
    print T_B_T_diff.vel.Norm()
    print T_B_T_diff.rot.Norm()
    if T_B_T_diff.vel.Norm() > 0.05 or T_B_T_diff.rot.Norm() > 0.2:
        exitError(10)

    newState = "Drop"
    return (newState, velma)

def drop(velma):
    print("Dropping jar")
    dest_q = [math.radians(0), math.radians(0), math.radians(0), math.radians(0)]
    velma.moveHandLeft(dest_q, [1, 1, 1, 1], [8000, 8000, 8000, 8000], 100000000, hold=False)
    if velma.waitForHandLeft() != 0:
        exitError(2)


    print("Jar dropped succesfully")

    newState = "Default_position"
    return (newState, velma)



def default_position(velma):

    print "Moving left wrist to pose defined in world frame..."
    T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.039, 0.016, 1, 0.0 ), PyKDL.Vector( 0.6, -0.1, 1.1 ))
    if not velma.moveCartImpLeft([T_B_Trd], [4.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(8)
    if velma.waitForEffectorLeft() != 0:
        exitError(9)
    rospy.sleep(0.5)
    print "calculating difference between desiread and reached pose..."
    T_B_T_diff = PyKDL.diff(T_B_Trd, velma.getTf("B", "Wl"), 1.0)
    print T_B_T_diff
    print T_B_T_diff.vel.Norm()
    print T_B_T_diff.rot.Norm()
    if T_B_T_diff.vel.Norm() > 0.05 or T_B_T_diff.rot.Norm() > 0.2:
        exitError(10)

    initialization(velma)


    newState = "Finish"
    return (newState, velma)


def finish(velma):
    print("Mission passed!\n  Respect +")







if __name__ == "__main__":

    rospy.init_node('pick_and_place')

    rospy.sleep(0.5)
    print("Running python interface for Velma...")
    velma = VelmaInterface()


    m = StateMachine()
    m.add_state("Initialization", initialization)
    m.add_state("Approach_to_object", approach_to_object)
    m.add_state("Grab_object", grab_jar)
    m.add_state("Departure_from_object", departure_from_object)
    m.add_state("Approach_to_table2_drop", approach_to_table2_drop)
    m.add_state("Drop", drop)
    m.add_state("Default_position", default_position)
    m.add_state("Finish", finish, end_state=1)
    m.set_start("Initialization")

    m.run(velma)