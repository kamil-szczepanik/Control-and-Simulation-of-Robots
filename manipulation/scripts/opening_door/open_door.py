#!/usr/bin/env python
import roslib
import rospy
from velma_common.velma_interface import VelmaInterface,isConfigurationClose
from rcprg_ros_utils import exitError
from velma_common import *
from rcprg_planner import *
from tf_conversions import posemath
import tf2_ros
import geometry_msgs.msg
import PyKDL
import numpy as np
from velma_kinematics.velma_ik_geom import KinematicsSolverLWR4, KinematicsSolverVelma
import math
from StateMachine import StateMachine

from jimp import Jimp
from cimp import Cimp
from grippers import Grippers


def initialization(velma):

    print ("Running python interface for Velma...")
    print ("Waiting for VelmaInterface initialization...")
    if not velma.waitForInit(timeout_s=10.0):
        print ("Could not initialize VelmaInterface\n")
        exitError(1)
    print ("Initialization ok!\n")

    print ("Enabling motors")
    if velma.enableMotors() != 0:
        exitError(1)

    print ("Sending head pan motor START_HOMING command...")
    velma.startHomingHP()
    if velma.waitForHP() != 0:
        exitError(1)
    print ("Head pan motor homing successful.")

    print ("Sending head tilt motor START_HOMING command...")
    velma.startHomingHT()
    if velma.waitForHT() != 0:
        exitError(1)
    print ("Head tilt motor homing successful.")

    newState = "Approach_to_handle"
    # newState = "Departure"

    return (newState, velma)

def approach_to_handle(velma):


    print("Closing grippers")
    grippers.close_grippers("right")
    grippers.close_grippers("left")

    if jimp.move_to_handle(planner, solver, hand):
        newState = "Open_door"
        return (newState, velma)
    else:
        print("Failed, trying again")
        newState = "Approach_to_handle"
        return (newState, velma)

    

def open_door(velma):


    print("Opening door")
    rospy.sleep(5)
    grippers.grippers_release_handle(hand)
    cimp.set_impedance(30, 30, 200, 100, 100, 100)
    print("Move: 1")
    cimp.move_tool(hand, solver, x=-0.1, y=0.07)
    rospy.sleep(1)
    # print("Move: 2")
    # grippers.grippers_release_handle(hand)
    # cimp.move_tool(hand, solver, x=-0.1, y=0.07)
    # rospy.sleep(1)

    # cimp.set_impedance(75, 20, 200, 100, 100, 100)
    # print("Move: 3")
    # cimp.move_tool(hand, solver, x=-0.2, y=0.07)
    # rospy.sleep(2)
    grippers.grippers_push(hand)


    newState = "Departure"
    return (newState, velma)

def departure(velma):

    cimp.set_impedance(75, 20, 200, 100, 100, 100)
    print("Move: 4")
    # cimp.move_tool(hand, solver, x=0, y=0.1)
    cimp.move_and_rotate_tool(hand, solver,x=0, y=0.1, yaw=3.14/8)
    rospy.sleep(2)
    # cimp.set_impedance(75, 20, 200, 100, 100, 100)
    print("Move: 5")
    # cimp.move_tool(hand, solver, x=0.1, y=0.15)
    cimp.move_and_rotate_tool(hand, solver,x=0.1, y=0.15, yaw=3.14/8)
    rospy.sleep(2)
    print("Move: 6")
    # cimp.move_tool(hand, solver, x=0.15, y=0.2)
    cimp.move_and_rotate_tool(hand, solver,x=0.2, y=0.3, yaw=3.14/8)
    rospy.sleep(2)
    # cimp.set_impedance(75, 20, 200, 100, 100, 100)
    print("Move: 7")
    # cimp.move_tool(hand, solver, x=0.2, y=0.25)
    # rospy.sleep(2)
    # cimp.set_impedance(75, 20, 200, 100, 100, 100)
    # print("Move: 8")
    # cimp.move_tool(hand, solver, x=0.25, y=0.3)
    # rospy.sleep(2)

    cimp.move_and_rotate_tool(hand, solver,x=0.35, y=0.45, yaw=3.14/8)


    print("Move: 8")
    cimp.move_and_rotate_tool(hand, solver,x=0.4, y=0.5)
    # cimp.move_tool(hand, solver, x=0.3, y=0.35)
    rospy.sleep(2)
    grippers.grippers_release_handle(hand)
    rospy.sleep(2)
    grippers.open_grippers(hand)

    cimp.move_and_rotate_tool(hand, solver,x=0.3, y=0.3)

    
    




    newState = "Default_position"
    return (newState, velma)

def default_position(velma):

    # TODO

    newState = "Finish"
    return (newState, velma)

def finish(velma):
    pass

    
def rotate_x_y(x, y, theta):
    xx = x*math.cos(theta) - y*math.sin(theta)
    yy = x*math.sin(theta) + y*math.cos(theta)
    return xx, yy


if __name__ == "__main__":

    rospy.init_node('open_door')
    rospy.sleep(0.5)
    print("Running python interface for Velma...")
    # conf = Conf()
    velma = VelmaInterface()
    rospy.sleep(1)
    planner = Planner(velma.maxJointTrajLen())
    if not planner.waitForInit():
        print("Could not initialize planner")
        exitError(2)
    print "Planner init ok"
    solver = KinematicsSolverVelma()
    oml = OctomapListener("/octomap_binary")
    rospy.sleep(1.0)
    octomap = oml.getOctomap(timeout_s=5.0)
    if not planner.processWorld(octomap):
        print("Processing world failed")
        exitError(1)
    jimp = Jimp(velma)
    cimp = Cimp(velma)
    grippers = Grippers(velma)
    hand = 'left'
    

    m = StateMachine()
    m.add_state("Initialization", initialization)
    m.add_state("Approach_to_handle", approach_to_handle)
    m.add_state("Open_door", open_door)
    m.add_state("Departure", departure)
    m.add_state("Default_position", default_position)
    m.add_state("Finish", finish, end_state=1)
    m.set_start("Initialization")

    m.run(velma)






