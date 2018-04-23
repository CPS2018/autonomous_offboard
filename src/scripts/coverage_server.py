#!/usr/bin/env python
import rospy
import actionlib
import autonomous_offboard.msg
import numpy as np
import time
import math
from geometry_msgs.msg import PoseStamped
from simulation_control.msg import goto_positionAction, goto_positionGoal, detect_objectAction, detect_objectGoal


class coverage_server():

    def __init__(self):
        # variables
        self.waypoints = np.empty((0,3))
        self.destination = PoseStamped()
        self.angleOfViewWidth = 37
        self.angleOfViewHeight = 47
        self.areaWidth = 50
        self.areaHeight = 50
        self.searchHeight = 10
        self.detectedPos = PoseStamped()
        self.detectedPos.pose.position.x = float("inf")
        self.result = simulation_control.msg.coverageResult()

        self.goto_position_client = actionlib.SimpleActionClient('goto_position', goto_positionAction)
        self.detect_object_client = actionlib.SimpleActionClient('detect_object', detect_objectAction)
        self.rate = rospy.Rate(20)
        self.action_server = actionlib.SimpleActionServer('coverage',
                                                          autonomous_offboard.msg.coverageAction,
                                                          execute_cb=self.execute_cb, auto_start=False)
        self.action_server.start()

    def execute_cb(self, goal):
        goto_position_goal = goto_positionGoal()
        self.goto_position_client.wait_for_server()
        self.detect_object_client.wait_for_server()
        goto_position_goal = goto_positionGoal()
        self.destination = goal.destination
        self.calculate_waypoints()
        idx= 0
        while(self.detectedPos.pose.position.x == float("inf")):
            goto_position_goal.destination.pose.position.x = self.waypoints.item((idx, 0))
            goto_position_goal.destination.pose.position.y = self.waypoints.item((idx, 1))
            goto_position_goal.destination.pose.position.z = self.waypoints.item((idx, 2))
            self.goto_position_client.send_goal(goto_position_goal)
            self.goto_position_client.wait_for_result()
            idx = idx + 1
            if idx == self.waypoints.shape[0]:
                idx = 0

            detect_object_goal = detect_objectGoal()
            self.detect_object_client.send_goal(detect_object_goal)
            self.detect_object_client.wait_for_result()
            self.detectedPos = self.detect_object_client.get_result().detected_position
            time.sleep(0.1)
            if self.detectedPos.pose.position.x != float("inf"):

                self.result.detected_object_pos = self.detectedPos
                self.result.detected_object_pos.pose.position.z = self.searchHeight
                self.action_server.set_succeeded(self.result)



    #calculate waypoints
    def calculate_waypoints(self):
        cameraWidth = 2 * self.searchHeight * math.tan(math.radians(self.angleOfViewWidth/2))
        cameraHeight = 2 * self.searchHeight * math.tan(math.radians(self.angleOfViewHeight/2))
        lastWaypoint = PoseStamped()
        lastWaypoint.pose.position.x = self.destination.pose.position.x - self.areaWidth/2
        lastWaypoint.pose.position.y = self.destination.pose.position.y - self.areaHeight / 2
        lastWaypoint.pose.position.z = self.searchHeight
        tempArr = []
        tempArr.append([lastWaypoint.pose.position.x, lastWaypoint.pose.position.y, lastWaypoint.pose.position.z])
        while(lastWaypoint.pose.position.x + cameraWidth/2 < self.destination.pose.position.x + self.areaWidth/2):
            #check if going upwards or downwards
            if(cameraHeight > 0):
                while(lastWaypoint.pose.position.y + cameraHeight/2 < self.destination.pose.position.y + self.areaHeight/2):
                    lastWaypoint.pose.position.y = lastWaypoint.pose.position.y + cameraHeight
                    tempArr.append([lastWaypoint.pose.position.x, lastWaypoint.pose.position.y, lastWaypoint.pose.position.z])
            elif(cameraHeight < 0):
                while (lastWaypoint.pose.position.y + cameraHeight / 2 > self.destination.pose.position.y - self.areaHeight / 2):
                    lastWaypoint.pose.position.y = lastWaypoint.pose.position.y + cameraHeight
                    tempArr.append([lastWaypoint.pose.position.x, lastWaypoint.pose.position.y, lastWaypoint.pose.position.z])

            cameraHeight= cameraHeight*(-1) #make it change direction for zig-zag pattern
            lastWaypoint.pose.position.x = lastWaypoint.pose.position.x + cameraWidth
            tempArr.append([lastWaypoint.pose.position.x, lastWaypoint.pose.position.y, lastWaypoint.pose.position.z])

        self.waypoints = np.asarray(tempArr)
        rospy.loginfo(self.waypoints)





if __name__ == '__main__':
    try:

        rospy.init_node('coverage_server')
        coverage_server()
    except rospy.ROSInterruptException:
        pass