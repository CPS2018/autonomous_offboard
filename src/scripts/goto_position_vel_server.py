#!/usr/bin/env python
import rospy
import actionlib
import autonomous_offboard.msg
from std_msgs.msg import Bool, String, Float32

from geometry_msgs.msg import PoseStamped
class goto_position_vel_server():
    def __init__(self):

        #variables
        self.target_reached = False
        #publishers
        self.pose_control = rospy.Publisher('/position_control/set_velocity', PoseStamped, queue_size=10)
        self.mode_control = rospy.Publisher('/position_control/set_mode', String, queue_size=10)
        #subscribers
        rospy.Subscriber('/position_control/distance', Bool, self.distance_reached_cb)

        self.rate = rospy.Rate(20)
        #self.result = simulation_control.msg.goto_positionResult()
        self.action_server = actionlib.SimpleActionServer('goto_position_vel', autonomous_offboard.msg.goto_position_velAction,
                                                          execute_cb=self.execute_cb, auto_start=False)
        self.action_server.start()


    def execute_cb(self, goal):
        print(goal)
        self.mode_control.publish('velctr')
        self.pose_control.publish(goal.destination)
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

        rospy.init_node('goto_position_vel_server')
        goto_position_vel_server()
    except rospy.ROSInterruptException:
        pass
