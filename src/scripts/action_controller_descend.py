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
    rospy.init_node('action_coverage_descend')
    goto_position_vel_client = actionlib.SimpleActionClient('goto_position_vel', goto_position_velAction)
    goto_position_vel_client.wait_for_server()
    goto_position_vel_goal = goto_position_velGoal()
    gotoposition_sensor_client = actionlib.SimpleActionClient('gotoposition_sensor', gotoposition_sensorAction)
    gotoposition_sensor_client.wait_for_server()
    gotoposition_sensor_goal = gotoposition_sensorGoal()
    descend_on_object_client = actionlib.SimpleActionClient('descend_on_object', descend_on_objectAction)
    descend_on_object_client.wait_for_server()
    descend_on_object_goal = descend_on_objectGoal()
    mv_state = mavros_state.mavros_state()
    print 'Setting offboard'
    mv_state.set_mode('OFFBOARD')
    print 'Arming vehicle'
    mv_state.arm(True)
    rospy.loginfo("Taking off")
    time.sleep(10)

    rospy.loginfo("Detect position")
    gotoposition_sensor_goal.destination.pose.position.x = 20.5
    gotoposition_sensor_goal.destination.pose.position.y = 45
    gotoposition_sensor_goal.destination.pose.position.z = 5
    gotoposition_sensor_client.send_goal(gotoposition_sensor_goal)
    gotoposition_sensor_client.wait_for_result()

    rospy.loginfo("Descending on object")
    rospy.loginfo("Descending server started")
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

    time.sleep(3)

    rospy.loginfo("Get some height")
    goto_position_vel_goal.destination.pose.position.x = 0
    goto_position_vel_goal.destination.pose.position.y = 0
    goto_position_vel_goal.destination.pose.position.z = 5
    goto_position_vel_client.send_goal(goto_position_vel_goal)
    goto_position_client.wait_for_result()

    time.sleep(3)
    rospy.loginfo("Going home")
    gotoposition_sensor_goal.destination.pose.position.x = 0
    gotoposition_sensor_goal.destination.pose.position.y = 0
    gotoposition_sensor_goal.destination.pose.position.z = 5
    gotoposition_sensor_client.send_goal(gotoposition_sensor_goal)
    gotoposition_sensor_client.wait_for_result()

    time.sleep(3)

    rospy.loginfo("Get some height")
    goto_position_vel_goal.destination.pose.position.x = 0
    goto_position_vel_goal.destination.pose.position.y = 0
    goto_position_vel_goal.destination.pose.position.z = -0.25
    goto_position_vel_client.send_goal(goto_position_vel_goal)
    goto_position_client.wait_for_result()


