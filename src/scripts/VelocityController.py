#!/usr/bin/env python
import rospy
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3, Twist, TwistStamped
import math
import numpy
import tf
from std_msgs.msg import Header
from PID import PID


class VelocityController:
    target = PoseStamped()
    output = TwistStamped()

    def __init__(self):
        self.X = PID()
        self.Y = PID()
        self.Z = PID()
        self.yaw = PID()


        self.lastTime = rospy.get_time()
        self.target = None

    def setTarget(self, target):
        self.target = target
        self.distThreshold = 2

    def update(self, state):
        if (self.target is None):
            rospy.logwarn("Target position for velocity controller is none.")
            return None

        # simplify variables a bit
        time = state.header.stamp.to_sec()
        position = state.pose.position
        orientation = state.pose.orientation

        # create output structure
        output = TwistStamped()
        output.header = state.header
        # output velocities
        linear = Vector3()
        angular = Vector3()

        # Control in X vel
        linear.x = self.X.update(self.target.position.x, position.x, time)
        # Control in Y vel
        linear.y = self.Y.update(self.target.position.y, position.y, time)
        # Control in Z vel
        #Front sensor
        linear.z = self.Z.update(self.target.position.z, position.z, time)
        # Control yaw (no x, y angular)

        (DesRoll, DesPitch, DesYaw) = tf.transformations.euler_from_quaternion([self.target.orientation.x, self.target.orientation.y, self.target.orientation.z,self.target.orientation.w])
        (CurrRoll, CurrPitch, CurrYaw) = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z,orientation.w])

        angular.z = self.yaw.update(DesYaw,CurrYaw,time)

        # TODO
        output.twist = Twist()
        output.twist.linear = linear
        output.twist.angular = angular
        return output

    def stop(self):
        setTarget(self.current)
        update(self.current)

    def set_x_pid(self, kp, ki, kd, output):
        self.X.setKp(kp)
        self.X.setKi(ki)
        self.X.setKd(kd)
        self.X.setMaxO(output)
    def set_y_pid(self, kp, ki, kd, output):
        self.Y.setKp(kp)
        self.Y.setKi(ki)
        self.Y.setKd(kd)
        self.Y.setMaxO(output)
    def set_z_pid(self, kp, ki, kd, output):
        self.Z.setKp(kp)
        self.Z.setKi(ki)
        self.Z.setKd(kd)
        self.Z.setMaxO(output)
    def set_yaw_pid(self, kp, ki, kd, output):
        self.yaw.setKp(kp)
        self.yaw.setKi(ki)
        self.yaw.setKd(kd)
        self.yaw.setMaxO(output)




