#!/usr/bin/env python

import roslib
import rospy
import time
from std_msgs.msg import UInt16
class gripper_control():
    def __init__(self):

        #publishers
        self.open_shortG = rospy.Publisher('/open_shortG', UInt16, queue_size=1)
        self.close_shortG = rospy.Publisher('/close_shortG', UInt16, queue_size=1)
        self.Goff = rospy.Publisher('/Goff', UInt16, queue_size=1)

    def open_short_gripper(self, cmd):
        print("Opening short grippers")
        time.sleep(3)
        self.open_shortG.publish(cmd)

    def close_short_gripper(self, cmd):
        print("Closing short grippers")
        time.sleep(3)
        self.close_shortG.publish(cmd)

    def Goff_gripper(self, cmd):
        print("Turning servo motors off")
        self.Goff.publish(cmd)
