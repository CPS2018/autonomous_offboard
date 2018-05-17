#!/usr/bin/env python
import roslib
#roslib.load_manifest('autonomous_offboard')
import rospy
import actionlib
import mavros_state
import time

import gripper_control

from autonomous_offboard.msg import center_on_objectAction,  center_on_objectGoal, descend_on_objectAction, descend_on_objectGoal, detect_objectAction, detect_objectGoal,goto_position_velAction, goto_position_velGoal, goto_positionAction, goto_positionGoal, short_grippersAction, short_grippersGoal, long_grippersAction, long_grippersGoal, coverageAction, coverageGoal, gotoposition_sensorAction , gotoposition_sensorGoal

from std_msgs.msg import Float32, UInt16
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    rospy.init_node('action_controller_claws')
    gc = gripper_control.gripper_control()
    msg = UInt16()

    rospy.loginfo("closing grippers")
    msg.data = 180
    gc.close_short_gripper(msg)
