#!/usr/bin/env python
from geometry_msgs.msg import PoseStamped, TwistStamped, Point, Quaternion
import rospy
import math
import tf
from std_msgs.msg import Float32, Bool, String
from VelocityController import VelocityController
from sensor_msgs.msg import Imu
import numpy as np
import copy


class position_control():
    def __init__(self):
        print 'Initialising position control'
        rospy.init_node('position_control', anonymous=True)
        rate = rospy.Rate(20)
        self.lidar_height = Float32
        self.lidar_height = 0.0
        self.actual_height = Float32
        self.actual_height = 0.0
        self.real_pose = PoseStamped()

        # ----------Subscribers----------#
        rospy.Subscriber('/position_control/set_mode', String, self.set_mode_callback)
        rospy.Subscriber('/position_control/set_position', PoseStamped, self.set_pose_callback)
        rospy.Subscriber('/position_control/set_velocity', PoseStamped, self.set_velocity_callback)
        rospy.Subscriber('/position_control/set_velocityPose', PoseStamped, self.set_velpose_callback)

        rospy.Subscriber('/position_control/set_x_pid', Point, self.set_x_pid)
        rospy.Subscriber('/position_control/set_y_pid', Point, self.set_y_pid)
        rospy.Subscriber('/position_control/set_z_pid', Point, self.set_z_pid)
        rospy.Subscriber('/position_control/set_yaw_pid', Point, self.set_yaw_pid)
        #Set max output velocity on PID in velocity control
        rospy.Subscriber('/position_control/set_xy_vel', Float32, self.set_xy_vel)
        rospy.Subscriber('/position_control/set_z_vel', Float32, self.set_z_vel)

        self.local_pose = PoseStamped()
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._local_pose_callback)
        self.local_velocity = TwistStamped()
        rospy.Subscriber('/mavros/local_position/velocity', TwistStamped, self._local_velocity_callback)
        self.local_imu = Imu()
        rospy.Subscriber('/mavros/imu/data', Imu, self._local_imu_callback)
        rospy.Subscriber('Laser_LidarLite', Float32, self._local_lidar_callback)

        # pos
        self._pose_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self._pose_msg = PoseStamped()
        self._vel_pose_msg = PoseStamped()

        self._pos_state = "posctr"
        # vel
        self._vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self._vel_msg = TwistStamped()
        self._vel_state = "velctr"

        self._velpose_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self._velpose_msg = PoseStamped()
        self._velpose_state = "velposctr"



        self.dist = rospy.Publisher('/position_control/distance', Bool, queue_size=10)
        self._real_pose = rospy.Publisher('/position_control/Real_pose', PoseStamped, queue_size=10)
        self.yawangle = rospy.Publisher('/position_control/Yawangle', Float32, queue_size=10)
        self.pitchangle = rospy.Publisher('/position_control/Pitchangle', Float32, queue_size=10)
        self.rollangle = rospy.Publisher('/position_control/Rollangle', Float32, queue_size=10)

        self.pid_out_pub = rospy.Publisher('/position_control/pid_out', TwistStamped, queue_size=10)

        self.current_publisher = self._pose_pub
        self.current_message = self._pose_msg
        # self._pose_msg.pose.position.z = 3
        self._pose_msg = self.local_pose
        self._pose_msg.pose.position.x = 0
        self._pose_msg.pose.position.y = 0
        self._pose_msg.pose.position.z = 3
        self.set_pose(self._pose_msg)
        self.des_vel = TwistStamped()
        self.current_mode = String()
        self.current_mode.data = 'posctr'

        self.vController = VelocityController()
        self.vController.set_x_pid(1.0, 0.0, 0.0, 1)  # 0.15  #MARCUS: 2.8, 0.913921, 0.0, 1
        self.vController.set_y_pid(1.0, 0.0, 0.0, 1)  # 2.1, 0.713921, 0.350178 #MARCUS: 2.8, 0.913921, 0.0, 1
        self.vController.set_z_pid(1.0, 0.0, 0.0, 0.3)  # 0.15 #MARCUS: 1.3, 2.4893, 0.102084, 1
        # self.vController.set_yaw_pid(3.6,1.33333,0.1875,1)
        self.vController.set_yaw_pid(1, 0, 0, 1)#1, 1.33333, 0.1875, 1

        print 'Init done'
        while not rospy.is_shutdown():
            if self.current_mode.data == 'posctr':
                self._pose_pub.publish(self._pose_msg)
            elif self.current_mode.data == 'velctr':
                self.vController.setTarget(self._vel_pose_msg.pose)
                self.des_vel = self.vController.update(self.real_pose)
                self._vel_pub.publish(self.des_vel)
                self.pid_out_pub.publish(self.des_vel)

            elif self.current_mode.data == 'velposctr':
                self.vController.setTarget(self._velpose_msg.pose)
                self.des_velpos = self.vController.update(self.real_pose)
                self._velpose_pub.publish(self.des_velpos)
                self.pid_out_pub.publish(self.des_velpos)

            else:
                print "No such position mode"
            self._real_pose.publish(self.real_pose)
            self.check_distance()
            self.get_angles()
            rate.sleep()

    def set_mode_callback(self, data):
        self.current_mode = data

    def set_vel(self, vel):
        self._vel_msg.twist.linear.x = vel.twist.linear.x
        self._vel_msg.twist.linear.y = vel.twist.linear.y
        self._vel_msg.twist.linear.z = vel.twist.linear.z

    def set_pose(self, pose):
        self._pose_msg.pose.position.x = self.local_pose.pose.position.x + pose.pose.position.x
        self._pose_msg.pose.position.y = self.local_pose.pose.position.y + pose.pose.position.y
        self._pose_msg.pose.position.z = pose.pose.position.z - (self.actual_height - self.local_pose.pose.position.z)

        #self._pose_msg.pose.orientation.x = self._orient_msg.pose.orientation.x
        #self._pose_msg.pose.orientation.y = self._orient_msg.pose.orientation.y
        #self._pose_msg.pose.orientation.z = self._orient_msg.pose.orientation.z
        #self._pose_msg.pose.orientation.w = self._orient_msg.pose.orientation.w

    def set_vel_pose(self, vel_pose):
        # print(vel_pose.pose)
        self._vel_pose_msg.pose.position.x = self.local_pose.pose.position.x + vel_pose.pose.position.x
        self._vel_pose_msg.pose.position.y = self.local_pose.pose.position.y + vel_pose.pose.position.y

        self._vel_pose_msg.pose.position.z = vel_pose.pose.position.z
        print(self._vel_pose_msg)
        # print(self._vel_pose_msg.pose)
        self._vel_pose_msg.pose.orientation.x = vel_pose.pose.orientation.x
        self._vel_pose_msg.pose.orientation.y = vel_pose.pose.orientation.y
        self._vel_pose_msg.pose.orientation.z = vel_pose.pose.orientation.z
        self._vel_pose_msg.pose.orientation.w = vel_pose.pose.orientation.w

    def set_velpose_pose(self, vel_pose):
        self._velpose_msg.pose.position.x = vel_pose.pose.position.x
        self._velpose_msg.pose.position.y = vel_pose.pose.position.y
        self._velpose_msg.pose.position.z = vel_pose.pose.position.z
        # print(self._vel_pose_msg.pose)
        self._velpose_msg.pose.orientation.x = vel_pose.pose.orientation.x
        self._velpose_msg.pose.orientation.y = vel_pose.pose.orientation.y
        self._velpose_msg.pose.orientation.z = vel_pose.pose.orientation.z
        self._velpose_msg.pose.orientation.w = vel_pose.pose.orientation.w

    def _local_pose_callback(self, data):
        self.local_pose = data
        self.real_pose = copy.deepcopy(self.local_pose)
        self.real_pose.pose.position.z = self.actual_height
       

    def _local_velocity_callback(self, data):
        self.local_velocity = data

    def _local_imu_callback(self, data):
        self.local_imu = data

    def _local_lidar_callback(self, data):
        self.lidar_height = (data.data / 100)

        X = self.local_imu.orientation.x
        Y = self.local_imu.orientation.y
        Z = self.local_imu.orientation.z
        W = self.local_imu.orientation.w

        (roll, pitch, Yaw) = tf.transformations.euler_from_quaternion([X, Y, Z, W])
        Comp = -math.sin(pitch) * 0.22  # 0.22 = 22cm from rotation centrum

      

        self.actual_height = ((self.lidar_height * (math.cos(pitch) * math.cos(roll))) - Comp)-0.3

    def set_pose_callback(self, data):
        self.set_pose(data)

    def set_velocity_callback(self, data):
        self.set_vel_pose(data)

    def set_velpose_callback(self, data):
        self.set_velpose_pose(data)

    def set_x_pid(self, data):
        self.vController.set_x_pid(data.x, data.y, data.z)

    def set_y_pid(self, data):
        self.vController.set_y_pid(data.x, data.y, data.z)

    def set_z_pid(self, data):
        self.vController.set_z_pid(data.x, data.y, data.z)

    def set_yaw_pid(self, data):
        self.vController.set_yaw_pid(data.x, data.y, data.z)

    def set_xy_vel(self, data):
        self.vController.set_xy_vel(data)
    def set_z_vel(self, data):
        self.vController.set_z_vel(data)


    def get_angles(self):

        X = self.local_imu.orientation.x
        Y = self.local_imu.orientation.y
        Z = self.local_imu.orientation.z
        W = self.local_imu.orientation.w
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([X, Y, Z, W])


        self.yawangle.publish(math.degrees(yaw))  # Yaw in degrees
        self.pitchangle.publish(math.degrees(pitch))  # Pitch in degrees
        self.rollangle.publish(math.degrees(roll))  # Roll in degrees

    def check_distance(self):
        if self.current_mode.data == 'posctr':
            booldist = self.is_at_position(self.local_pose.pose.position, self._pose_msg.pose.position, 0.5)
            boolvel = self.hover_velocity()
            self.dist.publish(booldist and boolvel)
        elif self.current_mode.data == 'velctr':
            # print("target vel_pos: {}".format(vel_pose_tot))
            booldist = self.is_at_position(self.real_pose.pose.position, self._vel_pose_msg.pose.position, 0.2)
            boolvel = self.hover_velocity()
            self.dist.publish(booldist and boolvel)
        elif self.current_mode.data == 'velposctr':
            # print("target vel_pos: {}".format(vel_pose_tot))
            booldist = self.is_at_position(self.real_pose.pose.position, self._velpose_msg.pose.position, 0.3)
            boolvel = self.hover_velocity()
            self.dist.publish(booldist and boolvel)

    def is_at_position(self, p_current, p_desired, offset):
        des_pos = np.array((p_desired.x,
                            p_desired.y,
                            p_desired.z))
        cur_pos = np.array((p_current.x,
                            p_current.y,
                            p_current.z))
        distance = np.linalg.norm(des_pos - cur_pos)
        return distance < offset

    def hover_velocity(self):
        return self.local_velocity.twist.linear.x < 0.2 and self.local_velocity.twist.linear.y < 0.2 and self.local_velocity.twist.linear.z < 0.2


if __name__ == '__main__':
    try:

        position_control()

    except rospy.ROSInterruptException:
        pass

