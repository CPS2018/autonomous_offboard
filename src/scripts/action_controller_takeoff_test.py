#!/usr/bin/env python

import roslib
#roslib.load_manifest('autonomous_offboard')
import rospy
import actionlib
import mavros_state
import time

from autonomous_offboard.msg import center_on_objectAction,  center_on_objectGoal, descend_on_objectAction, descend_on_objectGoal, detect_objectAction, detect_objectGoal,goto_position_velAction, goto_position_velGoal, goto_positionAction, goto_positionGoal, short_grippersAction, short_grippersGoal, long_grippersAction, long_grippersGoal, coverageAction, coverageGoal

from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    rospy.init_node('action_controller_takeoff_test')
    goto_position_vel_client = actionlib.SimpleActionClient('goto_position_vel', goto_position_velAction)
    goto_position_vel_client.wait_for_server()
    goto_position_vel_goal = goto_position_velGoal()
    mv_state = mavros_state.mavros_state()
    print 'Setting offboard'
    mv_state.set_mode('OFFBOARD')
    print 'Arming vehicle'
    print 'Takeoff'
    mv_state.arm(True)
    print 'Sleeping 20 sec'
    time.sleep(20)

    rospy.loginfo("change true height position")
    goto_position_vel_goal.destination.pose.position.x = 0
    goto_position_vel_goal.destination.pose.position.y = 5
    goto_position_vel_goal.destination.pose.position.z = 5
    goto_position_vel_client.send_goal(goto_position_vel_goal)
    goto_position_vel_client.wait_for_result()
    rospy.loginfo("10 second sleep before landing")
    time.sleep(10)
    #rospy.loginfo("Landing")
    #goto_position_vel_goal.destination.pose.position.x = 0
    #goto_position_vel_goal.destination.pose.position.y = 0
    #goto_position_vel_goal.destination.pose.position.z = -0.1
    #goto_position_vel_client.send_goal(goto_position_vel_goal)
    #goto_position_vel_client.wait_for_result()
    #rospy.loginfo("Trying to land, 2 second sleep")
    #time.sleep(2)

    #mv_state.arm(False)

