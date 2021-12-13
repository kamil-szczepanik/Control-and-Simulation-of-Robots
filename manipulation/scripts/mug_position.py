#!/usr/bin/env python
import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
import rospy
from velma_common.velma_interface import VelmaInterface
from rcprg_ros_utils import exitError

if __name__ == "__main__":
    rospy.init_node('mug_position')

    rospy.sleep(0.5)
    print("Running python interface for Velma...")
    velma = VelmaInterface()
    print("Waiting for VelmaInterface initialization...")
    if not velma.waitForInit(timeout_s=10.0):
        exitError(1, msg="Could not initialize VelmaInterface")
    print("Initialization ok!")
    T_B_Jar = velma.getTf("B", "frame_01")

    print(T_B_Jar)