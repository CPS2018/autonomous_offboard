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
    rospy.init_node('action_controller_coverage_descend')
    goto_position_vel_client = actionlib.SimpleActionClient('goto_position_vel', goto_position_velAction)
    goto_position_vel_client.wait_for_server()
    goto_position_vel_goal = goto_position_velGoal()
    mv_state = mavros_state.mavros_state()
    print 'Setting offboard'
    mv_state.set_mode('OFFBOARD')
    print 'Arming vehicle'
    mv_state.arm(True)
    rospy.loginfo("Taking off")
    time.sleep(10)

    coverage_client = actionlib.SimpleActionClient('coverage', coverageAction)
    coverage_client.wait_for_server()
    coverage_goal = coverageGoal()
    coverage_goal.destination.pose.position.x = 5
    coverage_goal.destination.pose.position.y = 0
    coverage_goal.destination.pose.position.z = 5
    coverage_client.send_goal(coverage_goal)
    coverage_client.wait_for_result()
    res = PoseStamped()
    res = coverage_client.get_result().detected_object_pos
    rospy.loginfo("Detected object is at position (x,y,z)=({}, {}, {})".format(
        res.pose.position.x,
        res.pose.position.y,
        res.pose.position.z))
    if res.pose.position.x == float("inf"):
        rospy.loginfo("ABORT no object detected. going home")
        mv_state.land(0.0)
        mv_state.arm(False)
    else:
        rospy.loginfo("Going to detected position")
        goto_position_vel_goal.destination.pose.position.x = coverage_client.get_result().detected_object_pos.pose.position.x
        goto_position_vel_goal.destination.pose.position.y = coverage_client.get_result().detected_object_pos.pose.position.y
        goto_position_vel_goal.destination.pose.position.z = coverage_client.get_result().detected_object_pos.pose.position.z
        goto_position_vel_client.send_goal(goto_position_goal)
        goto_position_vel_client.wait_for_result()
        rospy.loginfo("Is at position (x,y,z)=({}, {}, {})".format(coverage_client.get_result().detected_object_pos.pose.position.x,
                                                                   coverage_client.get_result().detected_object_pos.pose.position.y,
                                                                   coverage_client.get_result().detected_object_pos.pose.position.z))

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

        time.sleep(5)

        rospy.loginfo("Takeoff")
        goto_position_goal.destination.pose.position.x = 0
        goto_position_goal.destination.pose.position.y = 0
        goto_position_goal.destination.pose.position.z = 3
        goto_position_client.send_goal(goto_position_goal)
        goto_position_client.wait_for_result()

        time.sleep(5)

        rospy.loginfo("Going Home")
        goto_position_goal.destination.pose.position.x = -5
        goto_position_goal.destination.pose.position.y = 0
        goto_position_goal.destination.pose.position.z = 3
        goto_position_client.send_goal(goto_position_goal)
        goto_position_client.wait_for_result()

        time.sleep(5)

        rospy.loginfo("Landing")
        goto_position_vel_goal.destination.pose.position.x = 0
        goto_position_vel_goal.destination.pose.position.y = 0
        goto_position_vel_goal.destination.pose.position.z = -0.1
        goto_position_vel_client.send_goal(goto_position_vel_goal)
        goto_position_vel_client.wait_for_result()
        rospy.loginfo("Trying to land, 10 second sleep")
        time.sleep(10)

        mv_state.arm(False)
