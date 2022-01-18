#!/usr/bin/env python
import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
import rospy
from velma_common.velma_interface import VelmaInterface
from rcprg_ros_utils import exitError
from tf_conversions import posemath

if __name__ == "__main__":
    rospy.init_node('cabinet_position')
    rospy.sleep(0.5)
    print("Running python interface for Velma...")
    velma = VelmaInterface()
    print("Waiting for VelmaInterface initialization...")
    if not velma.waitForInit(timeout_s=10.0):
        exitError(1, msg="Could not initialize VelmaInterface")
    print("Initialization ok!")
    T_B_cabinet = velma.getTf("B", "cabinet_handle")

    print(T_B_cabinet)


    obj_pose = posemath.toMsg(T_B_cabinet)
    print(obj_pose)

    # approach_pose = posemath.Pose()
    # approach_pose.position.x = obj_pose.position.x
    # approach_pose.position.z = obj_pose.position.z
    # approach_pose.position.y = obj_pose.position.y
    # approach_pose.orientation = obj_pose.orientation

    # print('\nObject pose:\n' , approach_pose)

