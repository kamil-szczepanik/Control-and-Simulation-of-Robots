#!/usr/bin/env python
import math
from rcprg_ros_utils import exitError
import PyKDL


class Grippers:

    def __init__(self, velma):
        self.velma = velma

    def deg2rad(self, deg):
        return float(deg)/180.0*math.pi

    def moveGripper(self, q, selected):
        if selected == 'right':
            self.velma.moveHandRight(q, [1, 1, 1, 1], [4000,4000,4000,4000], 1000, hold=False)
            if self.velma.waitForHandRight() != 0:
                exitError(1)
        if selected == 'left':
            self.velma.moveHandLeft(q, [1, 1, 1, 1], [4000,4000,4000,4000], 1000, hold=False)
            if self.velma.waitForHandLeft() != 0:
                exitError(1)

    def closeGrippers(self, selected):
        q = [self.deg2rad(180), self.deg2rad(180), self.deg2rad(180), 0]
        self.moveGripper(q, selected)

    def openGrippers(self, selected):
        q = [0, 0, 0, 0]
        self.moveGripper(q, selected)

    def grippersReleaseHandle(self, selected):
        q = [self.deg2rad(75), self.deg2rad(75), self.deg2rad(60), 0]
        self.moveGripper(q, selected)

    def grippersGrabHandle(self, selected):
        q = [self.deg2rad(90), self.deg2rad(90), self.deg2rad(90), 0]
        self.moveGripper(q, selected)
