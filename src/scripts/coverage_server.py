#!/usr/bin/env python
import rospy
import actionlib
import autonomous_offboard.msg
import numpy as np
import time
import math
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Float32
from autonomous_offboard.msg import detect_objectAction, detect_objectGoal, goto_position_velAction, goto_position_velGoal


class coverage_server():

    def __init__(self):
        # variables
        self.cur_pose = PoseStamped()
        self.waypoints = np.empty((0,3))
        self.destination = PoseStamped()
        self.angleOfViewWidth = 37
        self.angleOfViewHeight = 47
        self.areaWidth = 20
        self.areaHeight = 20
        self.searchHeight = 10
        self.detectedPos = PoseStamped()
        self.detectedPos.pose.position.x = float("inf")
        self.result = autonomous_offboard.msg.coverageResult()
        rospy.Subscriber('/tensorflow_detection/cam_point', Point, self.detection_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.cur_pose_cb)

        self.set_xy_vel = rospy.Publisher('/position_control/set_xy_vel', Float32, queue_size=10)

        self.goto_position_client = actionlib.SimpleActionClient('goto_position', goto_position_velAction)
        #self.detect_object_client = actionlib.SimpleActionClient('detect_object', detect_objectAction)
        self.rate = rospy.Rate(20)
        self.action_server = actionlib.SimpleActionServer('coverage',
                                                          autonomous_offboard.msg.coverageAction,
                                                          execute_cb=self.execute_cb, auto_start=False)
        self.action_server.start()

    def execute_cb(self, goal):
        xyVel = Float32()
        xyVel.data = 1
        self.set_xy_vel.publish(xyVel)
        goto_position_goal = goto_position_velGoal()
        self.goto_position_client.wait_for_server()
        #self.detect_object_client.wait_for_server()
        self.destination = goal.destination
        self.calculate_waypoints2()
        idx= 0
        while(self.detectedPos.pose.position.x == float("inf")):
            goto_position_goal.destination.pose.position.x = self.waypoints.item((idx, 0))
            goto_position_goal.destination.pose.position.y = self.waypoints.item((idx, 1))
            goto_position_goal.destination.pose.position.z = self.waypoints.item((idx, 2))
            self.goto_position_client.send_goal(goto_position_goal)
            self.goto_position_client.wait_for_result()
            idx = idx + 1
            if idx == self.waypoints.shape[0] or self.detectedPos.pose.position.x != float("inf"):
                idx = 0
                self.result.detected_object_pos = self.detectedPos
                self.result.detected_object_pos.pose.position.z = self.searchHeight
                self.action_server.set_succeeded(self.result)

            #detect_object_goal = detect_objectGoal()
            #self.detect_object_client.send_goal(detect_object_goal)
            #self.detect_object_client.wait_for_result()
            #self.detectedPos = self.detect_object_client.get_result().detected_position
            time.sleep(0.1)





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

    def calculate_waypoints2(self):
        tempArr = []
        self.cameraWidth = 2 * self.searchHeight * math.tan(math.radians(self.angleOfViewWidth / 2))
        self.cameraHeight = 2 * self.searchHeight * math.tan(math.radians(self.angleOfViewHeight / 2))
        startPointX = self.destination.pose.position.x - self.areaWidth / 2 + self.cameraWidth / 2
        startPointY = self.destination.pose.position.y - self.areaHeight / 2 + self.cameraHeight / 2
        self.nrOfStepsHeight = int(math.floor(self.areaHeight / self.cameraHeight))
        self.nrOfStepsWidth = int(math.floor(self.areaWidth / self.cameraWidth))

        tempArr.append([startPointX, startPointY, self.searchHeight])

        for i in range(0, self.nrOfStepsWidth):
            for j in range(0, self.nrOfStepsHeight):

                if j == self.nrOfStepsHeight - 1:
                    _last_steplength = self.areaHeight - abs(self.nrOfStepsHeight * self.cameraHeight)
                    if self.cameraHeight > 0:
                        tempArr.append([0, _last_steplength, self.searchHeight])
                    else:
                        tempArr.append([0, -_last_steplength, self.searchHeight])
                else:
                    tempArr.append([0, self.cameraHeight, self.searchHeight])
            if i == self.nrOfStepsWidth-1:
                self.cameraHeight = self.cameraHeight * (-1)  # make it change direction for zig-zag pattern
                _last_steplength = self.areaHeight - abs(self.nrOfStepsWidth * self.cameraWidth)
                if self.cameraWidth > 0:
                    tempArr.append([_last_steplength, 0, self.searchHeight])
                else:
                    tempArr.append([-_last_steplength, 0, self.searchHeight])
                for j in range(0, self.nrOfStepsHeight):

                    if j == self.nrOfStepsHeight - 1:
                        _last_steplength = self.areaHeight - abs(self.nrOfStepsHeight * self.cameraHeight)
                        if self.cameraHeight > 0:
                            tempArr.append([0, _last_steplength, self.searchHeight])
                        else:
                            tempArr.append([0, -_last_steplength, self.searchHeight])
                    else:
                        tempArr.append([0, self.cameraHeight, self.searchHeight])


            elif i != self.nrOfStepsWidth:
                self.cameraHeight = self.cameraHeight * (-1)  # make it change direction for zig-zag pattern
                tempArr.append([self.cameraWidth, 0, self.searchHeight])

        self.waypoints = np.asarray(tempArr)
        rospy.loginfo(self.waypoints)

    def detection_callback(self, data):
        if (data.x != float("inf")):
            self.detectedPos = self.cur_pose
            self.detectedPos.pose.position.x = self.detectedPos.pose.position.x + data.x
            self.detectedPos.pose.position.y = self.detectedPos.pose.position.y + data.y

    def cur_pose_cb(self, data):
        self.cur_pose = data
if __name__ == '__main__':
    try:

        rospy.init_node('coverage_server')
        coverage_server()
    except rospy.ROSInterruptException:
        pass