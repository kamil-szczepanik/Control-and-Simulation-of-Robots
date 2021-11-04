#!/usr/bin/env python

import rospy
from rospy import Time, Duration
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
import math
from rospy.core import is_initialized
import tf


def zad2():

    VEL_LIN = 0.08*2.4*2.4
    VEL_ANG = 0.09*2.4*2.4

    # VEL_LIN = 0.3
    # VEL_ANG = 0.3
    TURN_RIGHT = False
    SIDE_LENGHT = 1.0
    global is_initialized
    is_initialized = False

    pub = rospy.Publisher('key_vel', Twist, queue_size=10)
    sub = rospy.Subscriber("/mobile_base_controller/odom", Odometry, callback)
    rospy.init_node('zad1', anonymous=True)
    rate = rospy.Rate(50)  # 50hz

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
    global is_initialized
    global goal_pose
    global goal_index
    poses = get_goal_poses(side_lenght, turn_right)
    if not is_initialized:
        goal_index = 0
        is_initialized = True
    goal_pose = poses[goal_index]
    print(goal_pose)
    if not is_angle_achived(goal_pose, tolerance=0.1):
        lin = 0.0
        ang = get_ang_vel(vel_ang, turn_right)
        print("angle not achived")
    elif not is_distance_achived(goal_pose, tolerance=0.05):
        lin = vel_lin
        ang = 0.0
        print("distance not achived")
    else:
        if len(poses)-1 != goal_index:
            goal_index += 1
            lin = 0.0
            ang = 0.0
        else:
            goal_index=0
            lin = 0.0
            ang = 0.0

    return lin, ang


def get_goal_poses(side_lenght, turn_right):
    if turn_right:
        return [(side_lenght, 0.0), (side_lenght, -side_lenght), (0.0, -side_lenght), (0.0,0.0)]
    else:
        return [(side_lenght, 0.0), (side_lenght, side_lenght), (0.0, side_lenght), (0.0,0.0)]

def get_distance(goal_pose):
    global pose
    pose
    return math.sqrt(pow(goal_pose[0] - pose.position.x, 2) +
                     pow(goal_pose[1] - pose.position.y, 2))


def get_angle_diff(goal_pose):
    global pose
    theta = tf.transformations.euler_from_quaternion(
        [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[2]
    goal_angle = math.atan2(
        goal_pose[1]-pose.position.y, goal_pose[0]-pose.position.x)
    return goal_angle - theta


def is_distance_achived(goal_pose, tolerance):
    print("distance_diff: ", get_distance(goal_pose))
    return get_distance(goal_pose) < tolerance


def is_angle_achived(goal_pose, tolerance):
    print("angle_diff: ", get_angle_diff(goal_pose))
    return abs(get_angle_diff(goal_pose)) < tolerance


def get_ang_vel(vel_ang, turn_right):
    if turn_right:
        return -vel_ang
    else:
        return vel_ang


if __name__ == '__main__':
    try:
        zad2()
    except rospy.ROSInterruptException:
        pass
