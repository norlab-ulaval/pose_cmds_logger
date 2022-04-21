#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import numpy as np
import pandas as pd

class Server:
    def __init__(self):
        self.calib_switch = Bool()
        self.joy_switch = Bool()
        self.pose = Odometry()
        self.imu = Imu()
        # self.velocity_left_cmd = Float64()
        # self.velocity_left_meas = Float64()
        # self.velocity_right_cmd = Float64()
        # self.velocity_right_meas = Float64()

    def switch_callback(self, msg):
        # "Store" message received.
        self.calib_switch = msg

    def joy_callback(self, msg):
        # "Store" message received.
        self.joy_switch = msg

    def pose_callback(self, msg):
        # "Store" message received.
        self.pose = msg

    def imu_callback(self, msg):
        # "Store" message received.
        self.imu = msg

    def twist_callback(self, msg):
        # "Store" message received.
        self.twist = msg

    def log_msgs(self, array):
        # Create numpy array with adequate poses
        global prev_icp_x
        global prev_icp_y
        global icp_index
        new_row = np.zeros((1, 16))

        if (self.pose.pose.pose.position.x != prev_icp_x
                and self.pose.pose.pose.position.y != prev_icp_y):
            prev_icp_x = self.pose.pose.pose.position.x
            prev_icp_y = self.pose.pose.pose.position.y
            icp_index += 1

        new_row = np.array(([rospy.get_rostime(), self.joy_switch.data, icp_index, # util data
                             self.twist.linear.x, self.twist.angular.z, # cmd data
                             self.imu.orientation.x, self.imu.orientation.y,
                             self.imu.orientation.z, self.imu.orientation.z,
                             self.imu.angular_velocity.z, # IMU data
                             self.pose.pose.pose.position.x, self.pose.pose.pose.position.y,
                             self.pose.pose.pose.orientation.x, self.pose.pose.pose.orientation.y,
                             self.pose.pose.pose.orientation.z, self.pose.pose.pose.orientation.w])) # icp pose data

        return np.vstack((array, new_row))

    def export_array(self, array):
        rospy.loginfo('Converting Array to DataFrame')
        df = pd.DataFrame(data=array, columns=['ros_time', 'joy_switch', 'icp_index',
                                               'cmd_lin', 'cmd_ang',
                                               'imu_quat_x', 'imu_quat_y',
                                               'imu_quat_z', 'imu_quat_w',
                                               'imu_yaw_rate',
                                               'icp_pos_x', 'icp_pos_y',
                                               'icp_quat_x', 'icp_quat_y',
                                               'icp_quat_z', 'icp_quat_w'])
        rospy.loginfo('Exporting DataFrame as .csv')
        df.to_csv('/home/dominic/Desktop/data.csv')
        rospy.loginfo('Data export done!')

if __name__ == '__main__':
    rospy.init_node('pose_cmds_logger')
    rate = rospy.Rate(20)  # 20hz

    server = Server()

    rospy.Subscriber('calib_switch', Bool, server.switch_callback)
    rospy.Subscriber('joy_switch', Bool, server.joy_callback)
    rospy.Subscriber('/icp_odom', Odometry , server.pose_callback)
    rospy.Subscriber('/imu/data', Imu , server.imu_callback)
    rospy.Subscriber('cmd_vel', Twist, server.twist_callback)

    array = np.zeros((1, 16))
    odom_index = 0
    global prev_icp_x
    prev_icp_x = 0
    global prev_icp_y
    prev_icp_y = 0
    global icp_index
    icp_index = 0
    while not rospy.is_shutdown():
        if server.calib_switch.data:
            #rospy.loginfo('on')
            array = server.log_msgs(array)
        elif not server.calib_switch.data:
            #rospy.loginfo('false')
            server.export_array(array)
            rospy.sleep(5)
        rate.sleep()