#!/usr/bin/env python

import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
import rospy
import math
from velma_common import *
from rcprg_ros_utils import exitError

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

    def run(self):
        try:
            handler = self.handlers[self.startState]
        except:
            raise InitializationError("must call .set_start() before .run()")
        if not self.endStates:
            raise  InitializationError("at least one state must be an end_state")
    
        while True:
            newState = handler()
            if newState.upper() in self.endStates:
                print("reached ", newState)
                break 
            else:
                handler = self.handlers[newState.upper()]  

def initialization():
    rospy.init_node('pick_and_place')

    rospy.sleep(0.5)

    print("This test/tutorial executes simple motions"\
        " in joint impedance mode. Planning is not used"\
        " in this example.")

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

    newState = "Approach_to_object"
    return newState

def grab_jar(velma):
    dest_q = [math.radians(180), math.radians(180), math.radians(180), math.radians(0)]
    velma.moveHandLeft(dest_q, [1, 1, 1, 1], [8000, 8000, 8000, 8000], 100000000, hold=False)
    if velma.waitForHandLeft() != 0:
        exitError(2)
    if velma.waitForHandRight() != 0:
        exitError(4)

    exitError(0)

    newState = "Go_to_table2"
    return newState

m = StateMachine()
m.add_state("Initialization", initialization)
m.add_state("Approach_to_object", ...)
m.add_state("Grab_object", ...)
m.add_state("Go_to_table2", ...)
m.add_state("Approach_to_table2_drop", ...)
m.add_state("Drop_object", ...)
m.add_state("Deafault_position", ..., end_state=1)

m.run()

