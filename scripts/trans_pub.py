#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import PyKDL
from tf_conversions import posemath
import csv


class WriterCsv():
    def __init__(self, file_name):
        self.__file = open(file_name, "w")
        self.writer = csv.writer(self.__file)
        self.writer.writerow(["x", "y"])

    def __del__(self):
        self.__file.close()


def zad2():
    global odom
    global p


    if were_both_callbacks==2:
        tra = [-0.496, 0.046, 0.000]
        q = [0.000, 0.000, 0.981, 0.195]

        f1 = posemath.fromMsg(odom.pose.pose)
        f2 = PyKDL.Frame(PyKDL.Rotation.Quaternion(
            *q), PyKDL.Vector(*tra))
        f = f2*f1
        pos = posemath.toMsg(f)

        odomToMapWriter.writer.writerow(
            [pos.position.x, pos.position.y])

        posewriter.writer.writerow(
            [p.pose.pose.position.x, p.pose.pose.position.y])

    # mobile_base = WriterCsv("mobile_base.csv")
        # mobile_base.writer.writerow(
        #     [odom.pose.pose.position.x, odom.pose.pose.position.y])

def callback(data):
    global were_both_callbacks
    if were_both_callbacks == 0:
        were_both_callbacks = 1

    global odom
    odom = Odometry()
    odom = data




def callback_pose(data):
    global were_both_callbacks
    if were_both_callbacks == 1:
        were_both_callbacks = 2

    global p
    p = PoseWithCovarianceStamped()
    p = data


if __name__ == '__main__':
    try:

        subpose = rospy.Subscriber(
            "/robot_pose", PoseWithCovarianceStamped, callback_pose)
        sub = rospy.Subscriber("/mobile_base_controller/odom", Odometry, callback)
        rospy.init_node('talk', anonymous=True)

        global were_both_callbacks
        were_both_callbacks = 0
        global odomToMapWriter
        global posewriter
        odomToMapWriter = WriterCsv("odom_mala_predkosc.csv")
        posewriter = WriterCsv("mappose_mala_predkosc.csv")
        while not rospy.is_shutdown():
            zad2()
            rate = rospy.Rate(10)  # 10hz
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
