#!/usr/bin/env python
import rospy
import actionlib
import autonomous_offboard.msg
from std_msgs.msg import Bool, String

from geometry_msgs.msg import PoseStamped
class takeoff_curr_server():
    def __init__(self):

        #variables
        self.target_reached = False
        #publishers
        self.pose_control = rospy.Publisher('/position_control/set_position', PoseStamped, queue_size=10)
        self.mode_control = rospy.Publisher('/position_control/set_mode', String, queue_size=10)

        #subscribers
        rospy.Subscriber('/position_control/distance', Bool, self.distance_reached_cb)
        self.local_pose = PoseStamped()
        self.take_off_pose = PoseStamped()
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._local_pose_callback)

        self.rate = rospy.Rate(20)
        #self.result = simulation_control.msg.goto_positionResult()
        self.action_server = actionlib.SimpleActionServer('takeoff_curr', autonomous_offboard.msg.takeoff_currAction,
                                                          execute_cb=self.execute_cb, auto_start=False)
        self.action_server.start()


    def _local_pose_callback(self, data):
        self.local_pose = data

    def execute_cb(self, goal):
        self.take_off_pose.pose.position.x = self.local_pose.position.pose.x
        self.take_off_pose.pose.position.y = self.local_pose.position.pose.y
        self.take_off_pose.pose.position.z = goal.destination.pose.position.z
        self.mode_control.publish('posctr')
        self.pose_control.publish(self.take_off_pose)
        rospy.sleep(0.1)
        print(self.target_reached)
        while not self.target_reached:
            self.rate.sleep()

        rospy.loginfo("Destination reached")
        self.action_server.set_succeeded()


    def distance_reached_cb(self, data):
        self.target_reached = data.data

if __name__ == '__main__':
    try:

        rospy.init_node('takeoff_curr_server')
        takeoff_curr_server()
    except rospy.ROSInterruptException:
        pass
