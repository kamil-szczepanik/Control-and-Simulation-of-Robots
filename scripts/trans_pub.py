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

    tra = [-0.396, -0.110, 0.000]
    q = [0.000, 0.000, 0.990, 0.144]

    subpose = rospy.Subscriber(
        "/robot_pose", PoseWithCovarianceStamped, callback_pose)
    sub = rospy.Subscriber("/mobile_base_controller/odom", Odometry, callback)
    rospy.init_node('talk', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    od = Odometry()
    mobile_base = WriterCsv("mobile_base.csv")
    odomToMapWriter = WriterCsv("odom.csv")
    posewriter = WriterCsv("mappose.csv")

    while not rospy.is_shutdown():
        f1 = posemath.fromMsg(odom.pose.pose)
        f2 = PyKDL.Frame(PyKDL.Rotation.Quaternion(
            *q), PyKDL.Vector(*tra))
        f = f2*f1
        pos = posemath.toMsg(f)

        odomToMapWriter.writer.writerow(
            [pos.position.x, pos.position.y])
        mobile_base.writer.writerow(
            [odom.pose.pose.position.x, odom.pose.pose.position.y])
        posewriter.writer.writerow(
            [p.pose.pose.position.x, p.pose.pose.position.y])
        rate.sleep()


def callback(data):
    global odom
    odom = Odometry()
    odom = data


def callback_pose(data):
    global p
    p = PoseWithCovarianceStamped()
    p = data


if __name__ == '__main__':
    try:
        zad2()
    except rospy.ROSInterruptException:
        pass
