#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from std_msgs.msg import Float64, Bool, String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from norlab_icp_mapper_ros.srv import SaveMap
import message_filters

import numpy as np
import pandas as pd

from multiprocessing import Lock

class LoggerNode(Node):

    def __init__(self):
        super().__init__('logger_node')

        self.calib_sub = self.create_subscription(
            Odometry,
            'calib_switch',
            self.switch_callback,
            10)
        self.joy_sub = self.create_subscription(
            Odometry,
            'joy_switch',
            self.joy_callback,
            10)
        # self.estop_sub = self.create_subscription(
        #     Odometry,
        #     'mcu/status',
        #     self.estop_callback,
        #     10)
        self.calib_state_sub = self.create_subscription(
            String,
            'calib_state',
            self.calib_state_callback,
            10)
        self.icp_sub = self.create_subscription(
            Odometry,
            'icp_odom',
            self.pose_callback,
            10)
        self.encoder_left_sub = self.create_subscription(
            Float64,
            'left_encoder_angular_velocity',
            self.velocity_left_meas_callback,
            10)
        self.encoder_right_sub = self.create_subscription(
            Float64,
            'right_encoder_angular_velocity',
            self.velocity_right_meas_callback,
            10)
        self.imu_sub = self.create_subscription(
            Imu,
            'MTI_imu/data_raw',
            self.imu_callback,
            10)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'doughnut_cmd_vel',
            self.cmd_vel_callback,
            10)

        self.calib_switch = Bool()
        self.joy_switch = Bool()
        self.pose = Odometry()
        self.velocity_left_cmd = Float64()
        self.velocity_left_meas = Float64()
        self.volt_left = Float64()
        self.current_left = Float64()
        self.velocity_right_cmd = Float64()
        self.velocity_right_meas = Float64()
        self.volt_right = Float64()
        self.current_right = Float64()
        self.imu_vel = Imu()
        self.cmd_vel = Twist()
        self.calib_state = String()

        self.rate = self.create_rate(20)

        self.ave_service = self.create_service(SaveMap, 'save_data', self.save_data_callback)

        self.array = np.zeros((1, 18))
        self.odom_index = 0
        self.prev_icp_x = 0
        self.prev_icp_y = 0
        self.icp_index = 0

    def switch_callback(self, msg):
        self.calib_switch = msg

    def joy_callback(self, msg):
        self.joy_switch = msg
    def calib_state_callback(self, msg):
        self.calib_state = msg

    def pose_callback(self, msg):
        self.pose = msg

    def velocity_left_meas_callback(self, msg):
        self.velocity_left_meas = msg

    def velocity_right_meas_callback(self, msg):
        self.velocity_right_cmd = msg

    def imu_callback(self, msg):
        self.imu_vel = msg

    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg
        ## TODO: Find a better way to run the self.log_msgs() function when spinning
        self.log_msgs()

    def log_msgs(self):
        # Create numpy array with adequate poses
        if (self.pose.pose.pose.position.x != self.prev_icp_x
                and self.pose.pose.pose.position.y != self.prev_icp_y):
            self.prev_icp_x = self.pose.pose.pose.position.x
            self.prev_icp_y = self.pose.pose.pose.position.y
            self.icp_index += 1

        ## TODO: Fix clock call
        new_row = np.array(([self.get_clock().now(), self.joy_switch.data, self.icp_index, self.calib_state.data,
                             self.velocity_left_meas.data, self.velocity_right_meas.data,
                             self.cmd_vel.linear.x, self.cmd_vel.angular.z,
                             self.pose.pose.pose.position.x, self.pose.pose.pose.position.y, self.pose.pose.pose.position.z,
                             self.pose.pose.pose.orientation.x, self.pose.pose.pose.orientation.y,
                             self.pose.pose.pose.orientation.z, self.pose.pose.pose.orientation.w,
                             self.imu_vel.angular_velocity.x, self.imu_vel.angular_velocity.y,
                             self.imu_vel.angular_velocity.z]))

        self.array = np.vstack((self.array, new_row))
        self.get_logger().info('test')

# TODO: Add /mcu/status/stop_engaged listener

    def save_data_callback(self, req, res):
        self.get_logger().info('Converting Array to DataFrame')
        df = pd.DataFrame(data=self.array, columns=['ros_time', 'joy_switch', 'icp_index', 'calib_state',
                                                   'meas_left_vel', 'meas_right_vel',
                                                   'cmd_vel_x', 'cmd_vel_omega',
                                                   'icp_pos_x', 'icp_pos_y', 'icp_pos_z',
                                                   'icp_quat_x', 'icp_quat_y',
                                                   'icp_quat_z', 'icp_quat_w',
                                                   'imu_x', 'imu_y', 'imu_z'])
        self.get_logger().info('Exporting DataFrame as .pkl')
        df.to_pickle(req.map_file_name.data)
        self.get_logger().info('Data export done!')
        return None

def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)

    try:
        # declare the node constructor
        logger_node = LoggerNode()
        executor = MultiThreadedExecutor()
        executor.add_node(logger_node)

        try:
            # pause the program execution, waits for a request to kill the node (ctrl+c)
            executor.spin()
        finally:
            executor.shutdown()
            controller_node.destroy_node()
    finally:
        # shutdown the ROS communication
        rclpy.shutdown()

if __name__ == '__main__':
    main()