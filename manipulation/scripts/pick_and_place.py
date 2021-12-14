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
    p.processWorld(octomap)
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

    planAndExecute(q_map_1, p)

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
            print "The trajectory could not be completed, retrying..."
            continue
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_dest, js[1]):
        exitError(6)

def jmove(velma):

    T_B_Jar = velma.getTf("B", "frame_01")

    if T_B_Jar is None:
        exitError(1)

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
    T_B_A7d = PyKDL.Frame(PyKDL.Rotation.RPY(*rpy), PyKDL.Vector(*xyz))
    arm_q = []
    for flip_shoulder, flip_elbow, flip_ee in flips:
        for elbow_circle_angle in np.linspace(-math.pi, math.pi, 20):
            q = solv.calculateIkArm(arm_name, T_B_A7d, torso_angle, elbow_circle_angle, flip_shoulder, flip_elbow, flip_ee)

            if not q[0] is None:
                q_dict = js.copy()
                for i in range(len(q)):
                    if i > 0:
                        q_dict['right_arm_{}_joint'.format(i)] = q[i]
                    else:
                        q_dict['torso_0_joint'] = q[i]
                arm_q.append(q_dict)
    
    print(q_dict)

def grab_jar(velma):
    dest_q = [math.radians(180), math.radians(180), math.radians(180), math.radians(0)]
    velma.moveHandLeft(dest_q, [1, 1, 1, 1], [8000, 8000, 8000, 8000], 100000000, hold=False)
    if velma.waitForHandLeft() != 0:
        exitError(2)


    print("Jar grabbed succesfully")

    newState = "Departure_from_object"
    return (newState, velma)

def approach_to_object(velma):


    print "Moving left wrist to pose defined in world frame..."
    T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.039055, 0.016422, 0.9991, 0.00098 ), PyKDL.Vector( 0.43, 0.46, 0.97 ))
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
    if T_B_T_diff.vel.Norm() > 0.05 or T_B_T_diff.rot.Norm() > 0.06:
        exitError(10)

    print "Moving left wrist to pose defined in world frame..."
    T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.039, 0.016, 1, 0.0 ), PyKDL.Vector( 0.6, 0.48, 0.97 ))
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
    if T_B_T_diff.vel.Norm() > 0.05 or T_B_T_diff.rot.Norm() > 0.07:
        exitError(10)

    newState = "Grab_object"
    return (newState, velma)

def departure_from_object(velma):
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
    if T_B_T_diff.vel.Norm() > 0.05 or T_B_T_diff.rot.Norm() > 0.06:
        exitError(10)

    print "Odkladam na stol"
    T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.039, 0.016, 1, 0.0 ), PyKDL.Vector( 0.6, 0.48, 0.97 ))
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
    if T_B_T_diff.vel.Norm() > 0.05 or T_B_T_diff.rot.Norm() > 0.07:
        exitError(10)

    exitError(0)
    

    newState = "Departure_from_object"
    return (newState, velma)

def default_position(velma):
    pass








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
    m.set_start("Initialization")
    # m.add_state("Go_to_table2", ...)
    # m.add_state("Approach_to_table2_drop", ...)
    # m.add_state("Drop_object", ...)
    # m.add_state("Deafault_position", ..., end_state=1)
    m.add_state("Default_position", default_position, end_state=1)

    m.run(velma)