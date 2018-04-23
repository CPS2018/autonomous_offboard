#!/usr/bin/env python

import roslib
roslib.load_manifest('simulation_control')
import rospy
import actionlib
import mavros_state
import time

from autonomous_offboard.msg import center_on_objectAction,  center_on_objectGoal, descend_on_objectAction, descend_on_objectGoal, detect_objectAction, detect_objectGoal, goto_positionAction, goto_positionGoal, short_grippersAction, short_grippersGoal, long_grippersAction, long_grippersGoal, coverageAction, coverageGoal

from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    rospy.init_node('action_controller_descend')
    mv_state = mavros_state.mavros_state()
    print 'Setting offboard'
    mv_state.set_mode('OFFBOARD')
    print 'Arming vehicle'
    mv_state.arm(True)
    rospy.loginfo("Taking off")
    goto_position_client = actionlib.SimpleActionClient('goto_position', goto_positionAction)
    goto_position_client.wait_for_server()
    goto_position_goal = goto_positionGoal()
    goto_position_goal.destination.pose.position.z = 2
    goto_position_client.send_goal(goto_position_goal)
    goto_position_client.wait_for_result()
    rospy.loginfo("Takeoff succeded")

    time.sleep(5)

    rospy.loginfo("First position")
    goto_position_goal.destination.pose.position.x = -2
    goto_position_goal.destination.pose.position.y = 0
    goto_position_goal.destination.pose.position.z = 2
    goto_position_client.send_goal(goto_position_goal)
    goto_position_client.wait_for_result()

    time.sleep(2)
    
    rospy.loginfo("Descending on object")
    descend_on_object_client = actionlib.SimpleActionClient('descend_on_object', descend_on_objectAction)
    descend_on_object_client.wait_for_server()
    rospy.loginfo("Descending server started")
    descend_on_object_goal = descend_on_objectGoal()
    descend_on_objectGoal = 2.0
    descend_on_object_client.send_goal(descend_on_object_goal)
    descend_on_object_client.wait_for_result()
    if descend_on_object_client.get_result().position_reached.data:
        print("landing")
        mv_state.arm(False)
    else:
        rospy.loginfo("Couldnt land exiting")

    time.sleep(3)

    print 'Setting offboard'
    mv_state.set_mode('OFFBOARD')
    print 'Arming vehicle'
    mv_state.arm(True)

    time.sleep(1)

    rospy.loginfo("Going Home")
    goto_position_goal.destination.pose.position.x = 0
    goto_position_goal.destination.pose.position.y = 0
    goto_position_goal.destination.pose.position.z = 5
    goto_position_client.send_goal(goto_position_goal)
    goto_position_client.wait_for_result()
    mv_state.land(0.0)
