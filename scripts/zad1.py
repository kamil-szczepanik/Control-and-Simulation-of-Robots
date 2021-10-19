#!/usr/bin/env python

import rospy
from rospy import Time, Duration
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
import math
from datetime import datetime
import time


def zad1():
    VEL_LIN = 0.08
    VEL_ANG = 0.09

    TURN_RIGHT = 0
    SIDE_LENGHT = 1.0

    pub = rospy.Publisher('key_vel', Twist, queue_size=10)
    sub = rospy.Subscriber("/mobile_base_controller/odom", Odometry, callback)
    rospy.init_node('zad1', anonymous=True)
    rate = rospy.Rate(50)  # 10hz

    vel = Twist()
    START_TIME = rospy.Time.now()
    global start_time
    start_time = time.time()

    while not rospy.is_shutdown():

        lin, ang = go_square(SIDE_LENGHT, TURN_RIGHT, VEL_LIN, VEL_ANG)
        vel.linear.x = lin
        vel.angular.z = ang
        pub.publish(vel)

        rate.sleep()


def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pose.pose.position.x)
    global pose
    pose = Pose()
    pose = data.pose.pose


def calc_time_lin(a, vel_lin):
    return a/vel_lin


def calc_time_ang(vel_ang):
    return (math.pi/2)/vel_ang


def go_square(side_lenght, turn_right, vel_lin, vel_ang):

    global start_time

    TIME_LIN = calc_time_lin(side_lenght, vel_lin)
    TIME_ANG = calc_time_ang(vel_ang)

    duration = time.time() - start_time
    print(duration)
    if duration < TIME_LIN:
        lin = vel_lin
        ang = 0
    elif duration >= TIME_LIN and duration <= TIME_LIN + TIME_ANG:
        lin = 0.0
        ang = vel_ang
    else:
        start_time = time.time()
        lin = 0
        ang = vel_ang

    return lin, ang


if __name__ == '__main__':
    try:
        zad1()
    except rospy.ROSInterruptException:
        pass
