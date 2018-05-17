#!/usr/bin/env python
import rospy
import actionlib
import math
import tf
import numpy
import time
import autonomous_offboard.msg
from std_msgs.msg import Bool, String, Float32, Float64
from geometry_msgs.msg import PoseStamped, Quaternion






class gotoposition_sensor_server():
    def __init__(self):

        #---------------Variables---------------#

        self.local_pose = PoseStamped()
        self.avoid_pose = PoseStamped()
        self.goal_pose = PoseStamped()
        self.distance = Float32()
        self.target_reached = False
        self.yaw = Float32()


        #---------------Publishers---------------#
        self.pose_control = rospy.Publisher('/position_control/set_position', PoseStamped, queue_size=10)
        self.mode_control = rospy.Publisher('/position_control/set_mode', String, queue_size=10)
        self.vel_control = rospy.Publisher('/position_control/set_velocity', PoseStamped, queue_size=10)
        self.velpose_control = rospy.Publisher('/position_control/set_velocityPose',PoseStamped,queue_size=10)


        # ---------------Subscribers---------------#

        rospy.Subscriber('/position_control/distance', Bool, self.distance_reached_cb)
        rospy.Subscriber('/position_control/Real_pose', PoseStamped, self._Real_pose_callback)
        rospy.Subscriber('/position_control/set_mode', String, self.set_mode_callback)

        # ---------------Start---------------#
        self.rate = rospy.Rate(20)
        self.action_server = actionlib.SimpleActionServer('gotoposition_sensor', autonomous_offboard.msg.gotoposition_sensorAction,
                                                          execute_cb=self.execute_cb, auto_start=False)
        self.action_server.start()

    def execute_cb(self, goal):
        #Set the orientation before start in position mode.
        self.mode_control.publish('velposctr')
        quaternion = tf.transformations.quaternion_from_euler(0, 0, math.radians(90))
        goal.destination.pose.orientation.x = quaternion[0]
        goal.destination.pose.orientation.y = quaternion[1]
        goal.destination.pose.orientation.z = quaternion[2]
        goal.destination.pose.orientation.w = quaternion[3]
        self.goal_pose = goal.destination
        self.velpose_control.publish(goal.destination)
        rospy.sleep(0.1)

        while not self.target_reached:
            self.rate.sleep()

        rospy.loginfo("Destination reached")
        self.action_server.set_succeeded()



    def distance_reached_cb(self, data):
        self.target_reached = data.data

    def _Real_pose_callback(self, data):
        self.Real_pose = data


    def set_mode_callback(self, data):
        self.current_mode = data


   




if __name__ == '__main__':
    try:

        rospy.init_node('gotoposition_sensor_server')
        gotoposition_sensor_server()
    except rospy.ROSInterruptException:
        pass
