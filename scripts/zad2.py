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
    rate = rospy.Rate(50) # 10hz



    vel = Twist()

    while not rospy.is_shutdown():

        lin, ang = go_square(SIDE_LENGHT, TURN_RIGHT, VEL_LIN, VEL_ANG)
        vel.linear.x = lin
        vel.angular.z = ang
        pub.publish(vel)
        
        rate.sleep()


def callback(data):
    global pose
    pose = Pose()
    pose = data.pose.pose
    


def go_square(side_lenght, turn_right, vel_lin, vel_ang):





    return lin, ang



if __name__ == '__main__':
    try:
        zad1()
    except rospy.ROSInterruptException:
        pass

