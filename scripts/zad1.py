#!/usr/bin/env python

import rospy
from rospy import Time, Duration
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
import math
from datetime import datetime
import time


def zad1():
    VEL_LIN = 0.08*2.4*2.4
    VEL_ANG = 0.09*2.4*2.4

    TURN_RIGHT = 0
    SIDE_LENGHT = 1.0

    pub = rospy.Publisher('key_vel', Twist, queue_size=10)
    sub = rospy.Subscriber("/mobile_base_controller/odom", Odometry, callback)
    rospy.init_node('zad1', anonymous=True)
    rate = rospy.Rate(50)  # 10hz

    vel = Twist()

    i = 0
    while not rospy.is_shutdown():
        i+=1
        global time_now
        if i>2:
          
            now = rospy.Time.now()
            time_now = now.secs + float(now.nsecs)/1000000000
            lin, ang = go_square(SIDE_LENGHT, TURN_RIGHT, VEL_LIN, VEL_ANG)
            vel.linear.x = lin
            vel.angular.z = ang

        if i == 2:
            global start_time
            now = rospy.Time.now()
            start_time = now.secs + float(now.nsecs)/1000000000
        
        pub.publish(vel)
        rate.sleep()


def callback(data):
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

    duration = time_now - start_time
    print(duration)
    if duration < TIME_LIN:
        lin = vel_lin
        ang = 0
    elif duration >= TIME_LIN and duration <= TIME_LIN + TIME_ANG:
        lin = 0.0
        if turn_right:
            ang = -vel_ang
        else:
            ang = vel_ang
    else:
        start_time = time_now
        lin = 0
        ang = 0


    return lin, ang


if __name__ == '__main__':
    try:
        zad1()
    except rospy.ROSInterruptException:
        pass
