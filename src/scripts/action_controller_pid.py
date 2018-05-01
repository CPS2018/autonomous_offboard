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
    rospy.init_node('action_controller_pid_test')
    mv_state = mavros_state.mavros_state()
    print 'Setting offboard'
    mv_state.set_mode('OFFBOARD')
    print 'Arming vehicle'
    mv_state.arm(True)
    rospy.loginfo("Taking off")
    time.sleep(10)

    goto_position_client = actionlib.SimpleActionClient('goto_position', goto_positionAction)
    goto_position_client.wait_for_server()
    goto_position_goal = goto_positionGoal()
    goto_position_goal.destination.pose.position.z = 2
    goto_position_client.send_goal(goto_position_goal)
    goto_position_client.wait_for_result()
    rospy.loginfo("Takeoff succeded")

    # time.sleep(5)
    # rospy.loginfo("Z PID")
    # print("sleeping 5 sec start Rosbagging. ZPID")
    # time.sleep(5)
    # ######Z PID#########
    goto_position_vel_client = actionlib.SimpleActionClient('goto_position_vel', goto_position_velAction)
    goto_position_vel_client.wait_for_server()
    goto_position_vel_goal = goto_position_velGoal()
    # goto_position_vel_goal.destination.pose.position.x = 0
    # goto_position_vel_goal.destination.pose.position.y = 0
    # goto_position_vel_goal.destination.pose.position.z = 3
    # goto_position_vel_client.send_goal(goto_position_vel_goal)
    # goto_position_vel_client.wait_for_result()
    #
    # time.sleep(10)
    # print("z pid done, stop rosbag")
    # time.sleep(5)
    # ###STABALIZE
    # rospy.loginfo("Stabalize")
    # goto_position_goal.destination.pose.position.x = 0
    # goto_position_goal.destination.pose.position.y = 0
    # goto_position_goal.destination.pose.position.z = 2
    # goto_position_client.send_goal(goto_position_goal)
    # goto_position_client.wait_for_result()

    time.sleep(5)

    rospy.loginfo("X PID")
    print("sleeping 5 sec start Rosbagging. XPID")
    time.sleep(5)
    #######X PID##############
    goto_position_vel_goal.destination.pose.position.x = 1
    goto_position_vel_goal.destination.pose.position.y = 0
    goto_position_vel_goal.destination.pose.position.z = 2
    goto_position_vel_client.send_goal(goto_position_vel_goal)
    goto_position_vel_client.wait_for_result()

    time.sleep(10)
    print("x pid done, stop rosbag")
    time.sleep(5)
    ###STABALIZE
    rospy.loginfo("Stabalize")
    goto_position_goal.destination.pose.position.x = -1
    goto_position_goal.destination.pose.position.y = 0
    goto_position_goal.destination.pose.position.z = 2
    goto_position_client.send_goal(goto_position_goal)
    goto_position_client.wait_for_result()

    time.sleep(5)
    #
    #######Y PID##############
    rospy.loginfo("Y PID")
    print("sleeping 5 sec start Rosbagging. YPID")
    time.sleep(5)
    goto_position_vel_goal.destination.pose.position.x = 0
    goto_position_vel_goal.destination.pose.position.y = 1
    goto_position_vel_goal.destination.pose.position.z = 2
    goto_position_vel_client.send_goal(goto_position_vel_goal)
    goto_position_vel_client.wait_for_result()

    time.sleep(10)
    print("y pid done, stop rosbag")
    time.sleep(5)
    ###STABALIZE
    rospy.loginfo("Stabalize")
    goto_position_goal.destination.pose.position.x = 0
    goto_position_goal.destination.pose.position.y = -1
    goto_position_goal.destination.pose.position.z = 2
    goto_position_client.send_goal(goto_position_goal)
    goto_position_client.wait_for_result()

    time.sleep(5)

    mv_state.land(0.0)

