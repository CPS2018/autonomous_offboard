#!/usr/bin/env python
import rospy
import actionlib
import autonomous_offboard.msg

from geometry_msgs.msg import PoseStamped, Point
class detect_object_server():
    def __init__(self):

        #variables
        self.local_pose = PoseStamped()
        self.object_pose = PoseStamped()
        self.detected = False
        self.newValue = False
        #publishers

        #subscribers
        rospy.Subscriber('/tensorflow_detection/cam_point', Point, self.get_cam_pos_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._local_pose_callback)

        self.rate = rospy.Rate(20)
        self.result = simulation_control.msg.detect_objectResult()
        self.action_server = actionlib.SimpleActionServer('detect_object', autonomous_offboard.msg.detect_objectAction,
                                                          execute_cb=self.execute_cb, auto_start=False)
        self.action_server.start()


    def execute_cb(self, goal):
        self.newValue = False
        self.result.detected_position.pose.position.x = float("inf")
        while not self.newValue:
            rospy.sleep(0.1)
            rospy.loginfo(self.newValue)

        self.result.detected_position = self.object_pose
        rospy.loginfo(self.object_pose)
        self.action_server.set_succeeded(self.result)

    def get_cam_pos_callback(self, data):
        self.newValue = True
        #rospy.loginfo(self.newValue)

        if data.x != float("inf"):
            #rospy.loginfo('detected')
            #rospy.loginfo(data)
            self.detected = True
            self.object_pose.pose.position.x = data.x + self.local_pose.pose.position.x
            self.object_pose.pose.position.y = data.y + self.local_pose.pose.position.y
            self.object_pose.pose.position.z = data.z + self.local_pose.pose.position.z

        else:
            #rospy.loginfo('not detected')
            self.detected = False
            self.object_pose.pose.position = data


    def _local_pose_callback(self, data):
        self.local_pose = data

if __name__ == '__main__':
    try:

        rospy.init_node('detect_object_server')
        detect_object_server()
    except rospy.ROSInterruptException:
        pass