#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry

def talker():
    pub = rospy.Publisher('key_vel', Twist, queue_size=10)
    sub = rospy.Subscriber("/mobile_base_controller/odom", Odometry, callback)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    vel = Twist()
    while not rospy.is_shutdown():
        vel.linear.x = 0.1
        pub.publish(vel)
        rate.sleep()


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pose.pose.position.x)
    

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

