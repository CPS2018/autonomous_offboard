#!/usr/bin/env python

import roslib
#roslib.load_manifest('autonomous_offboard')
import rospy
import time
import gripper_control

from std_msgs.msg import UInt16

if __name__ == '__main__':
    rospy.init_node('action_controller_gripper')
    gc = gripper_control.gripper_control()
    print("Started")
    msg = UInt16()
    print("First")
    msg.data = 180
    gc.grippers_short(msg)
    gc.grippers_long(msg)
    print("Second")
    time.sleep(10)
    msg.data = 0
    gc.grippers_short(msg)
    gc.grippers_long(msg)


    

