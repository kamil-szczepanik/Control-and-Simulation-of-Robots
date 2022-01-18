#!/usr/bin/env python
import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
import rospy
from velma_common.velma_interface import VelmaInterface
from rcprg_ros_utils import exitError
from tf_conversions import posemath
import tf2_ros
import geometry_msgs.msg
import PyKDL


class Conf:

    def __init__(self):
        rospy.init_node('cabinet_position')
        rospy.sleep(0.5)
        self.velma = self.setup_velma()
    
    def setup_velma(self):
        print("Running python interface for Velma...")
        velma = VelmaInterface()
        print("Waiting for VelmaInterface initialization...")
        if not velma.waitForInit(timeout_s=10.0):
            exitError(1, msg="Could not initialize VelmaInterface")
        print("Initialization ok!")
        
        print("Motors must be enabled every time after the robot enters safe state.")
        print("If the motors are already enabled, enabling them has no effect.")
        print("Enabling motors...")
        if velma.enableMotors() != 0:
            exitError(2, msg="Could not enable motors")

        rospy.sleep(0.5)

        diag = velma.getCoreCsDiag()
        if not diag.motorsReady():
            exitError(1, msg="Motors must be homed and ready to use for this test.")

        return velma

    def switch_to_jnt_imp(self):
            print("Switch to jnt_imp mode (no trajectory)...")
            self.velma.moveJointImpToCurrentPos(start_time=0.5)
            error = self.velma.waitForJoint()
            if error != 0:
                exitError(3, msg="The action should have ended without error,"\
                            " but the error code is {}".format(error))

if __name__ == "__main__":
    conf = Conf()

    T_Wo_cabinet = conf.velma.getTf("Wo", "cabinet_handle")
    print(T_Wo_cabinet)
    T_Wo_cabinet.M.DoRotZ(3.14)


    obj_pose = posemath.toMsg(T_Wo_cabinet)
    print(obj_pose)

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = "abc"
    t.transform.translation.x = obj_pose.position.x - 0.2
    t.transform.translation.y = obj_pose.position.y
    t.transform.translation.z = obj_pose.position.z

    t.transform.rotation = obj_pose.orientation
    br.sendTransform(t)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        t.header.stamp = rospy.Time.now()
        print("sending transormation")
        br.sendTransform(t)
        rate.sleep()

