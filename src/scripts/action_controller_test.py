#!/usr/bin/env python

import roslib
#roslib.load_manifest('autonomous_offboard')
import rospy
import actionlib
import mavros_state
import time

from autonomous_offboard.msg import center_on_objectAction,  center_on_objectGoal, descend_on_objectAction, descend_on_objectGoal, detect_objectAction, detect_objectGoal, goto_positionAction, goto_positionGoal, short_grippersAction, short_grippersGoal, long_grippersAction, long_grippersGoal, coverageAction, coverageGoal

from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    rospy.init_node('action_controller_test')
    goto_position_client = actionlib.SimpleActionClient('goto_position', goto_positionAction)
    goto_position_client.wait_for_server()
    goto_position_goal = goto_positionGoal()
    mv_state = mavros_state.mavros_state()
    print 'Setting offboard'
    mv_state.set_mode('OFFBOARD')
    print 'Arming vehicle'
    mv_state.arm(True)
    rospy.loginfo("Taking off")
    time.sleep(10)

    rospy.loginfo("First position")
    goto_position_goal.destination.pose.position.x = 20
    goto_position_goal.destination.pose.position.y = 0
    goto_position_goal.destination.pose.position.z = 3
    goto_position_client.send_goal(goto_position_goal)
    goto_position_client.wait_for_result()

    time.sleep(10)

    rospy.loginfo("Second position")
    goto_position_goal.destination.pose.position.x = 0
    goto_position_goal.destination.pose.position.y = -20
    goto_position_goal.destination.pose.position.z = 3
    goto_position_client.send_goal(goto_position_goal)
    goto_position_client.wait_for_result()

    time.sleep(10)

    rospy.loginfo("Third position")
    goto_position_goal.destination.pose.position.x = -20
    goto_position_goal.destination.pose.position.y = 0
    goto_position_goal.destination.pose.position.z = 3
    goto_position_client.send_goal(goto_position_goal)
    goto_position_client.wait_for_result()

    time.sleep(10)

    rospy.loginfo("Fourth position")
    goto_position_goal.destination.pose.position.x = 0
    goto_position_goal.destination.pose.position.y = 20
    goto_position_goal.destination.pose.position.z = 3
    goto_position_client.send_goal(goto_position_goal)
    goto_position_client.wait_for_result()

    time.sleep(10)

    mv_state.land(0.0)
