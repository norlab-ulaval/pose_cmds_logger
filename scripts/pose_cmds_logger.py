#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import numpy as np
import pandas as pd

class Server:
    def __init__(self):
        self.calib_switch = Bool()
        self.joy_switch = Bool()
        self.pose = Odometry()
        self.velocity_left_cmd = Float64()
        self.velocity_left_meas = Float64()
        self.velocity_right_cmd = Float64()
        self.velocity_right_meas = Float64()

    def switch_callback(self, msg):
        # "Store" message received.
        self.calib_switch = msg

    def joy_callback(self, msg):
        # "Store" message received.
        self.joy_switch = msg

    def pose_callback(self, msg):
        # "Store" message received.
        self.pose = msg

    def velocity_left_cmd_callback(self, msg):
        # "Store" the message received.
        self.velocity_left_cmd = msg

    def velocity_left_meas_callback(self, msg):
        # "Store" the message received.
        self.velocity_left_meas = msg

    def velocity_right_cmd_callback(self, msg):
        # "Store" the message received.
        self.velocity_right_cmd = msg

    def velocity_right_meas_callback(self, msg):
        # "Store" the message received.
        self.velocity_right_meas = msg

    def log_msgs(self, array):
        # Create numpy array with adequate poses
        global prev_icp_x
        global prev_icp_y
        global icp_index
        new_row = np.zeros((1, 13))

        if (self.pose.pose.pose.position.x != prev_icp_x
                and self.pose.pose.pose.position.y != prev_icp_y):
            prev_icp_x = self.pose.pose.pose.position.x
            prev_icp_y = self.pose.pose.pose.position.y
            icp_index += 1

        new_row = np.array(([rospy.get_rostime(), self.joy_switch.data, icp_index,
                             self.velocity_left_cmd.data, self.velocity_left_meas.data,
                             self.velocity_right_cmd.data, self.velocity_right_meas.data,
                             self.pose.pose.pose.position.x, self.pose.pose.pose.position.y,
                             self.pose.pose.pose.orientation.x, self.pose.pose.pose.orientation.y,
                             self.pose.pose.pose.orientation.z, self.pose.pose.pose.orientation.w]))

        return np.vstack((array, new_row))

    def export_array(self, array):
        rospy.loginfo('Converting Array to DataFrame')
        df = pd.DataFrame(data=array, columns=['ros_time', 'joy_switch', 'icp_index',
                                               'cmd_left_vel', 'meas_left_vel',
                                               'cmd_right_vel', 'meas_right_vel',
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
    rospy.Subscriber('/left_drive/velocity', Float64, server.velocity_left_cmd_callback)
    rospy.Subscriber('/left_drive/status/speed', Float64, server.velocity_left_meas_callback)
    rospy.Subscriber('/right_drive/velocity', Float64, server.velocity_right_cmd_callback)
    rospy.Subscriber('/right_drive/status/speed', Float64, server.velocity_right_meas_callback)

    array = np.zeros((1, 13))
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