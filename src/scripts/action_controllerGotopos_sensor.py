#!/usr/bin/env python
import roslib
#roslib.load_manifest('autonomous_offboard')
import rospy
import actionlib
import mavros_state
import time

from autonomous_offboard.msg import center_on_objectAction,  center_on_objectGoal, descend_on_objectAction, descend_on_objectGoal, detect_objectAction, detect_objectGoal,goto_position_velAction, goto_position_velGoal, goto_positionAction, goto_positionGoal, short_grippersAction, short_grippersGoal, long_grippersAction, long_grippersGoal, coverageAction, coverageGoal, gotoposition_sensorAction , gotoposition_sensorGoal

from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    rospy.init_node('action_controllerGotopos_sensor')
    gotoposition_sensor_client = actionlib.SimpleActionClient('gotoposition_sensor', gotoposition_sensorAction)
    gotoposition_sensor_client.wait_for_server()
    gotoposition_sensor_goal = gotoposition_sensorGoal()
    mv_state = mavros_state.mavros_state()
    print 'Setting offboard'
    mv_state.set_mode('OFFBOARD')
    print 'Arming vehicle'
    mv_state.arm(True)
    rospy.loginfo("Taking off")
    time.sleep(10)

    rospy.loginfo("First position")
    gotoposition_sensor_goal.destination.pose.position.x = 2
    gotoposition_sensor_goal.destination.pose.position.y = 0
    gotoposition_sensor_goal.destination.pose.position.z = 3
    gotoposition_sensor_client.send_goal(gotoposition_sensor_goal)
    gotoposition_sensor_client.wait_for_result()


    time.sleep(10)

    mv_state.land(0.0)
