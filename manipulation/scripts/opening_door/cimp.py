#!/usr/bin/env python
from velma_common import *
from rcprg_ros_utils import exitError
import math
import PyKDL
import rospy


class Cimp:

    def __init__(self, velma):
        self.velma = velma

    def switch_to_cimp(self, hand='left'):
        print ("Switch to cart_imp mode (no trajectory)...")
        if hand == 'left':
            if not self.velma.moveCartImpLeftCurrentPos(start_time=0.2):
                exitError(3)
            if self.velma.waitForEffectorLeft() != 0:
                exitError(3)
        else:
            if not self.velma.moveCartImpRightCurrentPos(start_time=0.2):
                exitError(3)
            if self.velma.waitForEffectorRight() != 0:
                exitError(3)
    
        rospy.sleep(0.5)
    
        diag = self.velma.getCoreCsDiag()
        if not diag.inStateCartImp():
            print ("The core_cs should be in cart_imp state, but it is not")
            exitError(3)

    def move_tool(self, hand, solver, x=0.0, y= 0.0, z=0.0):
        time, velma_pos = self.velma.getLastJointState()
        configuration = (   velma_pos[hand + '_arm_0_joint'],
                        velma_pos[hand + '_arm_1_joint'],
                        velma_pos[hand + '_arm_2_joint'],
                        velma_pos[hand + '_arm_3_joint'],
                        velma_pos[hand + '_arm_4_joint'],
                        velma_pos[hand + '_arm_5_joint'],
                        velma_pos[hand + '_arm_6_joint']    )

        if hand == 'right':
            frame = solver.getRightArmFk(velma_pos['torso_0_joint'], configuration)
        else:
            frame = solver.getLeftArmFk(velma_pos['torso_0_joint'], configuration)
            
        p = PyKDL.Vector(x, y, z)
        p = frame * p
        frame.p = p

        if hand == 'right':
            if not self.velma.moveCartImpRight([frame], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(1,1,1), PyKDL.Vector(1,1,1))):
                exitError(1)
            if self.velma.waitForEffectorRight() != 0:
                exitError(1)
        else:
            if not self.velma.moveCartImpLeft([frame], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(1,1,1), PyKDL.Vector(1,1,1))):
                exitError(1)
            if self.velma.waitForEffectorLeft() != 0:
                exitError(1)

    def move_and_rotate_tool(self, hand, solver, x=0.0, y= 0.0, z=0.0, roll=0.0, pitch= 0.0, yaw=0.0):
        time, velma_pos = self.velma.getLastJointState()
        configuration = (   velma_pos[hand + '_arm_0_joint'],
                        velma_pos[hand + '_arm_1_joint'],
                        velma_pos[hand + '_arm_2_joint'],
                        velma_pos[hand + '_arm_3_joint'],
                        velma_pos[hand + '_arm_4_joint'],
                        velma_pos[hand + '_arm_5_joint'],
                        velma_pos[hand + '_arm_6_joint']    )

        if hand == 'right':
            frame = solver.getRightArmFk(velma_pos['torso_0_joint'], configuration)
        else:
            frame = solver.getLeftArmFk(velma_pos['torso_0_joint'], configuration)
            
        p = PyKDL.Vector(x, y, z)
        p = frame * p
        frame.p = p
        frame.M.DoRotX(roll)
        frame.M.DoRotY(pitch)
        frame.M.DoRotZ(yaw)

        if hand == 'right':
            if not self.velma.moveCartImpRight([frame], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(1,1,1), PyKDL.Vector(1,1,1))):
                exitError(1)
            if self.velma.waitForEffectorRight() != 0:
                exitError(1)
        else:
            if not self.velma.moveCartImpLeft([frame], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(1,1,1), PyKDL.Vector(1,1,1))):
                exitError(1)
            if self.velma.waitForEffectorLeft() != 0:
                exitError(1)

    def set_impedance(self, x, y, z, roll, pitch, yaw, hand='left'):
        if hand == 'left':
            print "Switch to cart_imp mode (no trajectory)..."
            if not self.velma.moveCartImpLeftCurrentPos(start_time=0.2):
                exitError(10)
            if self.velma.waitForEffectorLeft() != 0:
                exitError(11)
        
            rospy.sleep(0.5)
        
            diag = self.velma.getCoreCsDiag()
            if not diag.inStateCartImp():
                print "The core_cs should be in cart_imp state, but it is not"
                exitError(12)
        
            print "To see the tool frame add 'tf' in rviz and enable 'right_arm_tool' frame."
            print "At every state switch to cart_imp, the tool frames are reset."
            print "Also, the tool impedance parameters are reset to 1500N/m in every"\
                " direction for linear stiffness and to 150Nm/rad in every direction for angular"\
                " stiffness, i.e. (1500,1500,1500,150,150,150)."
            if not self.velma.moveCartImpLeft(None, None, None, None, [PyKDL.Wrench(PyKDL.Vector(x, y, z), PyKDL.Vector(roll, pitch, yaw))], [2], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
                exitError(2)
            if self.velma.waitForEffectorLeft() != 0:
                exitError(2)
        else:
            print "Switch to cart_imp mode (no trajectory)..."
            if not self.velma.moveCartImpLeftCurrentPos(start_time=0.2):
                exitError(10)
            if self.velma.waitForEffectorLeft() != 0:
                exitError(11)
        
            rospy.sleep(0.5)
        
            diag = self.velma.getCoreCsDiag()
            if not diag.inStateCartImp():
                print "The core_cs should be in cart_imp state, but it is not"
                exitError(12)
        
            print "To see the tool frame add 'tf' in rviz and enable 'right_arm_tool' frame."
            print "At every state switch to cart_imp, the tool frames are reset."
            print "Also, the tool impedance parameters are reset to 1500N/m in every"\
                " direction for linear stiffness and to 150Nm/rad in every direction for angular"\
                " stiffness, i.e. (1500,1500,1500,150,150,150)."
            if not self.velma.moveCartImpRight(None, None, None, None, [PyKDL.Wrench(PyKDL.Vector(x, y, z), PyKDL.Vector(roll, pitch, yaw))], [2], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
                exitError(2)
            if self.velma.waitForEffectorRight() != 0:
                exitError(2)