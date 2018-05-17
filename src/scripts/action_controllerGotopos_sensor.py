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
    rospy.init_node('action_controllerGotopos_sensor')
    gc = gripper_control.gripper_control()
    gotoposition_sensor_client = actionlib.SimpleActionClient('gotoposition_sensor', gotoposition_sensorAction)
    gotoposition_sensor_client.wait_for_server()
    gotoposition_sensor_goal = gotoposition_sensorGoal()

    goto_position_vel_client = actionlib.SimpleActionClient('goto_position_vel', goto_position_velAction)
    goto_position_vel_client.wait_for_server()
    goto_position_vel_goal = goto_position_velGoal()
    mv_state = mavros_state.mavros_state()
    msg = UInt16()
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
    #DESCENDING
    rospy.loginfo("closing grippers")
    msg.data = 180
    gc.close_short_gripper(msg)


    time.sleep(3)

    rospy.loginfo("Drop off position")
    gotoposition_sensor_goal.destination.pose.position.x = 100
    gotoposition_sensor_goal.destination.pose.position.y = -27
    gotoposition_sensor_goal.destination.pose.position.z = 5
    gotoposition_sensor_client.send_goal(gotoposition_sensor_goal)
    gotoposition_sensor_client.wait_for_result()

    time.sleep(3)

    rospy.loginfo("going down to drop off position")
    gotoposition_sensor_goal.destination.pose.position.x = 100
    gotoposition_sensor_goal.destination.pose.position.y = -27
    gotoposition_sensor_goal.destination.pose.position.z = 0.25
    gotoposition_sensor_client.send_goal(gotoposition_sensor_goal)
    gotoposition_sensor_client.wait_for_result()

    rospy.loginfo("opening grippers")
    gc.open_short_gripper(msg)
    time.sleep(2)
    gc.Goff_gripper(msg)
    time.sleep(3)

    rospy.loginfo("Going to up before returning home")
    gotoposition_sensor_goal.destination.pose.position.x = 100
    gotoposition_sensor_goal.destination.pose.position.y = -27
    gotoposition_sensor_goal.destination.pose.position.z = 5
    gotoposition_sensor_client.send_goal(gotoposition_sensor_goal)
    gotoposition_sensor_client.wait_for_result()

    time.sleep(3)

    rospy.loginfo("returning home")
    gotoposition_sensor_goal.destination.pose.position.x = 0
    gotoposition_sensor_goal.destination.pose.position.y = 0
    gotoposition_sensor_goal.destination.pose.position.z = 5
    gotoposition_sensor_client.send_goal(gotoposition_sensor_goal)
    gotoposition_sensor_client.wait_for_result()

    time.sleep(3)

    rospy.loginfo("Landing")
    goto_position_vel_goal.destination.pose.position.x = 0
    goto_position_vel_goal.destination.pose.position.y = 0
    goto_position_vel_goal.destination.pose.position.z = -0.15
    goto_position_vel_client.send_goal(goto_position_vel_goal)
    goto_position_vel_client.wait_for_result()
    rospy.loginfo("Trying to land, 2 second sleep")
    time.sleep(2)

    mv_state.arm(False)



