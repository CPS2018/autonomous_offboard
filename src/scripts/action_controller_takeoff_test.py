#!/usr/bin/env python

import roslib
#roslib.load_manifest('autonomous_offboard')
import rospy
import actionlib
import mavros_state
import time

from autonomous_offboard.msg import center_on_objectAction,  center_on_objectGoal, descend_on_objectAction, descend_on_objectGoal, detect_objectAction, detect_objectGoal, goto_positionAction, goto_positionGoal, short_grippersAction, short_grippersGoal,takeoff_currAction, takeoff_currGoal, long_grippersAction, long_grippersGoal, coverageAction, coverageGoal

from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    rospy.init_node('action_controller_takeoff_test')
    mv_state = mavros_state.mavros_state()
    print 'Setting offboard'
    mv_state.set_mode('OFFBOARD')
    print 'Arming vehicle'
    print 'Takeoff'
    mv_state.arm(True)
    print 'Sleeping 20 sec'
    time.sleep(20)
    print 'Landing'
    mv_state.land(0.0)
