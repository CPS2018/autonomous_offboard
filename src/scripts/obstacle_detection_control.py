#!/usr/bin/env python

import math
import rospy
import math
import numpy
import sys
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float32, Bool, String
        
class obstacle_detection_control():

    def __init__(self):

        print 'Initialising obstacle detection'
        rospy.init_node('obstacle_detection_control')
        rate = rospy.Rate(20)

        # ---------------Variables---------------#
        self.local_pose = PoseStamped()
        self.detection = LaserScan()
        self.curr_height = Float32
        self.curr_height = 0

        self.obstacle_detected = False
        self.distance_front = 5.5
        self.distance_right = 5.5
        self.distance_left = 5.5
        self.counter = 0

        # ---------------Subscribers---------------#
        rospy.Subscriber('/Laser_LidarLite', Float32, self.lidar_callback)
        ##rospy.Subscriber('/sonar_data', LaserScan,  self.sonar_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._local_pose_callback)

        # ---------------Publishers---------------#
        self.obstacle = rospy.Publisher("/obstacle_detection_control/obstacle_detected", Bool, queue_size=10)
        self.fsrp = rospy.Publisher("/obstacle_detection_control/front_reading", Float32, queue_size=10)
        self.lsrp = rospy.Publisher("/obstacle_detection_control/left_reading", Float32, queue_size=10)
        self.rsrp = rospy.Publisher("/obstacle_detection_control/right_reading", Float32, queue_size=10)
        self.hrp = rospy.Publisher("/obstacle_detection_control/height_reading", Float32, queue_size=10)

        print 'Init done'
        while not rospy.is_shutdown():
            self.obstacle.publish(self.obstacle_detected)
            self.hrp.publish(self.curr_height)
           # self.check_detect()
           # self.fsrp.publish(self.distance_front)
           # self.lsrp.publish(self.distance_left)s
           # self.rsrp.publish(self.distance_right)
            rate.sleep()

    #--------------Detection sensors--------------#
    def sonar_callback(self,data):

        self.detection=data
        self.distance_front = numpy.min(self.detection.ranges[14:17])
        self.distance_left = numpy.min(self.detection.ranges[26:29])
        self.distance_right = numpy.min(self.detection.ranges[0:3])

        #Maximum reading handler
        if self.distance_front == float("inf"):
            self.distance_front = 5.5
        if self.distance_left == float("inf"):
            self.distance_left = 5.5
        if self.distance_right == float("inf"):
            self.distance_right = 5.5

    #----------------Height sensor----------------#
    def lidar_callback(self,data):
        if self.curr_height>0:
            reading = self.exp_filter(self.curr_height,data)
            self.curr_height = reading
        else:
            self.curr_height = data

    # ---------------Detection---------------#
    def check_detect(self):
        #Distance threshold to decide if an obstacle is preasent
        distTh = 2.5

        if self.distance_right<distTh or self.distance_left<distTh or self.distance_front<distTh:
            self.obstacle_detected = True
            self.counter = 0
        else:
            # Obstacle is only considered gone if x continous reading say so
            self.counter += 1
            if self.counter >=5:
                self.obstacle_detected = False

    # -----------------Fitlering-----------------#
    def exp_filter(self,last,new):
        # Smoothing value (0-1), 0 = high smoothing, 1 = low smoothing
        s = 0.2
        # Filtering
        tempdata = s*new+(1-s)*last
        return tempdata

    def _local_pose_callback(self, data):
        self.local_pose = data

if __name__ == '__main__':
    try:

        obstacle_detection_control()

    except rospy.ROSInterruptException:
        pass

