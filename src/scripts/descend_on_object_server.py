#!/usr/bin/env python
import rospy
import actionlib
import autonomous_offboard.msg
import tf
import math
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Bool, String, Float32
import time


class descend_on_object_server():
    def __init__(self):

        # variables
        self.local_pose = PoseStamped()
        self.des_pose = PoseStamped()
        quaternion = tf.transformations.quaternion_from_euler(0, 0, math.radians(90))
        des_pose.pose.orientation.x = quaternion[0]
        des_pose.pose.orientation.y = quaternion[1]
        des_pose.pose.orientation.z = quaternion[2]
        des_pose.pose.orientation.w = quaternion[3]
        self.object_pose = Point()
        self.object_pose_center = Point()

        # publishers
        self.mode_control = rospy.Publisher('/position_control/set_mode', String, queue_size=10)
        self.vel_control = rospy.Publisher('/position_control/set_velocity', PoseStamped, queue_size=10)
        self.set_xy_vel = rospy.Publisher('/position_control/set_xy_vel', Float32, queue_size=10)
        self.set_z_vel = rospy.Publisher('/position_control/set_z_vel', Float32, queue_size=10)

        # subscribers
        rospy.Subscriber('/tensorflow_detection/cam_point', Point, self.get_cam_pos_callback)
        rospy.Subscriber('/tensorflow_detection/cam_point_center', Point, self.get_cam_pos_center_callback)
        #rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._local_pose_callback)
        rospy.Subscriber('/position_control/distance', Bool, self.distance_reached_cb)

        rospy.Subscriber('/position_control/Real_pose', PoseStamped, self._real_pose_callback)

        self.rate = rospy.Rate(20)
        self.result = autonomous_offboard.msg.descend_on_objectResult()
        self.action_server = actionlib.SimpleActionServer('descend_on_object',
                                                          autonomous_offboard.msg.descend_on_objectAction,
                                                          execute_cb=self.execute_cb,
                                                          auto_start=False)
        self.last_object_pose = Point()
        self.last_object_pose_center = Point()
        self.action_server.start()

    def execute_cb(self, goal):
        xyVel = Float32()
        xyVel.data = 0.2
        self.set_xy_vel.publish(xyVel)
        rospy.loginfo("Starting to descend")
        self.mode_control.publish('velctr')
        rospy.sleep(0.1)

        while self.local_pose.pose.position.z > 2.0:
            self.rate.sleep()
            print("x = ", self.local_pose.pose.position.x)
            print("y = ", self.local_pose.pose.position.y)
            print("z = ", self.local_pose.pose.position.z)
            rospy.sleep(0.2)

            if self.detected and (abs(self.object_pose.x) > 0.2 or abs(self.object_pose.y) > 0.2):
                self.last_object_pose = self.object_pose
                self.des_pose.pose.position.x = self.object_pose.x
                self.des_pose.pose.position.y = self.object_pose.y
                self.des_pose.pose.position.z = self.local_pose.pose.position.z
                self.vel_control.publish(self.des_pose)
                rospy.loginfo("Centering...")
                while not self.target_reached:
                    rospy.sleep(2)

            elif self.detected and abs(self.object_pose.x) < 0.2 and abs(self.object_pose.y) < 0.2:
                self.des_pose.pose.position.x = 0
                self.des_pose.pose.position.y = 0
                self.des_pose.pose.position.z = self.local_pose.pose.position.z - 0.9
                rospy.loginfo("Descending...")
                self.vel_control.publish(self.des_pose)
                while not self.target_reached:
                    rospy.sleep(2)

        while self.local_pose.pose.position.z < 2.0 and self.local_pose.pose.position.z > 0.5:
            self.rate.sleep()
            print("x = ", self.local_pose.pose.position.x)
            print("y = ", self.local_pose.pose.position.y)
            print("z = ", self.local_pose.pose.position.z)
            rospy.sleep(0.2)

            if self.detected and (abs(self.object_pose_center.x) > 0.05 or abs(self.object_pose_center.y) > 0.05):
                self.last_object_pose_center = self.object_pose_center
                self.des_pose.pose.position.x = self.object_pose_center.x
                self.des_pose.pose.position.y = self.object_pose_center.y
                self.vel_control.publish(self.des_pose)
                rospy.loginfo("Centering...")
                while not self.target_reached:
                    rospy.sleep(2)

            elif self.detected and abs(self.object_pose_center.x) < 0.05 and abs(self.object_pose_center.y) < 0.05:
                self.des_pose.pose.position.x = 0
                self.des_pose.pose.position.y = 0
                self.des_pose.pose.position.z = self.local_pose.pose.position.z - 0.1
                rospy.loginfo("Descending...")
                self.vel_control.publish(self.des_pose)
                while not self.target_reached:
                    rospy.sleep(2)

        print("Landing")
        self.des_pose.pose.position.x = 0
        self.des_pose.pose.position.y = 0
        self.des_pose.pose.position.z = -0.15
        self.vel_control.publish(self.des_pose)
        time.sleep(0.1)
        while not self.target_reached:
            time.sleep(2)
        print("x = ", self.local_pose.pose.position.x)
        print("y = ", self.local_pose.pose.position.y)
        print("z = ", self.local_pose.pose.position.z)
        self.rate.sleep()
        self.result.position_reached.data = True
        self.action_server.set_succeeded(self.result)

    def get_cam_pos_callback(self, data):
        if data.x != float("inf"):
            self.detected = True
            self.object_pose = data
        else:
            self.detected = False

    def get_cam_pos_center_callback(self, data):
        if data.x != float("inf"):
            self.detected = True
            self.object_pose_center = data
        else:
            self.detected = False

   # def _local_pose_callback(self, data):
    #    self.local_pose = data

    def distance_reached_cb(self, data):
        self.target_reached = data.data
    def _real_pose_callback(self, data):
        self.local_pose = data


if __name__ == '__main__':
    try:

        rospy.init_node('descend_on_object_server')
        descend_on_object_server()
    except rospy.ROSInterruptException:
        pass
