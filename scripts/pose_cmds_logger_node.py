#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64, Bool, String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from pose_cmds_logger.srv import *
import message_filters
from warthog_msgs.msg import Status

import numpy as np
import pandas as pd


calib_switch = Bool()
joy_switch = Bool()
pose = Odometry()
velocity_left_cmd = Float64()
velocity_left_meas = Float64()
volt_left = Float64()
current_left = Float64()
velocity_right_cmd = Float64()
velocity_right_meas = Float64()
volt_right = Float64()
current_right = Float64()
imu_vel = Imu()
cmd_vel = Twist()
calib_state = String()
estop = Status()

def switch_callback(msg):
    # "Store" message received.
    global calib_switch
    calib_switch = msg

def joy_callback(msg):
    # "Store" message received.
    global joy_switch
    joy_switch = msg

def calib_state_callback(msg):
    global calib_state
    calib_state = msg

def pose_callback(msg):
    # "Store" message received.
    global pose
    pose = msg

def velocity_left_cmd_callback(msg):
    # "Store" the message received.
    global velocity_left_cmd
    velocity_left_cmd = msg

def velocity_left_meas_callback(msg):
    # "Store" the message received.
    global velocity_left_meas
    velocity_left_meas = msg

def velocity_right_cmd_callback(msg):
    # "Store" the message received.
    global velocity_right_cmd
    velocity_right_cmd = msg

def velocity_right_meas_callback(msg):
    # "Store" the message received.
    global velocity_right_meas
    velocity_right_meas = msg

def voltage_left_callback(msg):
    # "Store" the message received.
    global volt_left
    volt_left= msg

def voltage_right_callback(msg):
    # "Store" the message received.
    global volt_right
    volt_right = msg

def current_left_callback(msg):
    # "Store" the message received.
    global current_left
    current_left= msg

def current_right_callback(msg):
    # "Store" the message received.
    global current_right
    current_right = msg

def imu_callback(msg):
    # "Store" message received.
    global imu_vel
    imu_vel = msg

def cmd_vel_callback(msg):
    global cmd_vel
    cmd_vel = msg

def estop_callback(msg):
    # "Store" message received.
    global estop
    estop = msg.stop_engaged

def log_msgs(array):
    # Create numpy array with adequate poses
    global prev_icp_x
    global prev_icp_y
    global icp_index
    new_row = np.zeros((1, 25))

    if (pose.pose.pose.position.x != prev_icp_x
            and pose.pose.pose.position.y != prev_icp_y):
        prev_icp_x = pose.pose.pose.position.x
        prev_icp_y = pose.pose.pose.position.y
        icp_index += 1

    new_row = np.array(([rospy.get_rostime(), joy_switch.data, icp_index, calib_state, estop,
                         velocity_left_cmd.data, velocity_left_meas.data,
                         velocity_right_cmd.data, velocity_right_meas.data,
                         cmd_vel.linear.x, cmd_vel.angular.z,
                         pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z,
                         pose.pose.pose.orientation.x, pose.pose.pose.orientation.y,
                         pose.pose.pose.orientation.z, pose.pose.pose.orientation.w,
                         volt_left.data, current_left.data, volt_right.data, current_right.data,
                         imu_vel.angular_velocity.x, imu_vel.angular_velocity.y,
                         imu_vel.angular_velocity.z]))

    return np.vstack((array, new_row))

# TODO: Add /mcu/status/stop_engaged listener

def save_data_handle(req, array):
        rospy.loginfo('Converting Array to DataFrame')
        df = pd.DataFrame(data=array, columns=['ros_time', 'joy_switch', 'icp_index', 'calib_state', 'estop',
                                               'cmd_left_vel', 'meas_left_vel',
                                               'cmd_right_vel', 'meas_right_vel',
                                               'cmd_vel_x', 'cmd_vel_omega',
                                               'icp_pos_x', 'icp_pos_y', 'icp_pos_z',
                                               'icp_quat_x', 'icp_quat_y',
                                               'icp_quat_z', 'icp_quat_w',
                                               'voltage_left', 'current_left',
                                               'voltage_right', 'current_right',
                                               'imu_x', 'imu_y', 'imu_z'])
        rospy.loginfo('Exporting DataFrame as .pkl')
        df.to_pickle(req.data_file_name.data)
        rospy.loginfo('Data export done!')
        return []

if __name__ == '__main__':
    rospy.init_node('pose_cmds_logger')
    rate = rospy.Rate(20)  # 20hz


    calib_sub = rospy.Subscriber('calib_switch', Bool, switch_callback)
    joy_sub = rospy.Subscriber('joy_switch', Bool, joy_callback)
    estop_sub = rospy.Subscriber('mcu/status', Status, estop_callback)
    calib_state_sub = rospy.Subscriber('calib_state', String, calib_state_callback)
    icp_sub = rospy.Subscriber('/icp_odom', Odometry , pose_callback)
    cmd_left_sub = rospy.Subscriber('/left_drive/velocity', Float64, velocity_left_cmd_callback)
    meas_left_sub = rospy.Subscriber('/left_drive/status/speed', Float64, velocity_left_meas_callback)
    cmd_right_sub = rospy.Subscriber('/right_drive/velocity', Float64, velocity_right_cmd_callback)
    meas_right_sub = rospy.Subscriber('/right_drive/status/speed', Float64, velocity_right_meas_callback)
    volt_left_sub = rospy.Subscriber('/left_drive/status/battery_voltage', Float64, voltage_left_callback)
    current_left_sub = rospy.Subscriber('/left_drive/status/battery_current', Float64, current_left_callback)
    volt_right_sub = rospy.Subscriber('/right_drive/status/battery_voltage', Float64, voltage_right_callback)
    current_right_sub = rospy.Subscriber('/right_drive/status/battery_current', Float64, current_right_callback)
    imu_sub = rospy.Subscriber('/MTI_imu/data_raw', Imu, imu_callback)
    cmd_vel_sub = rospy.Subscriber('/doughnut_cmd_vel', Twist, cmd_vel_callback)

    save_service = rospy.Service('save_data', SaveData, lambda msg: save_data_handle(msg, array))

    array = np.zeros((1, 25))
    odom_index = 0
    global prev_icp_x
    prev_icp_x = 0
    global prev_icp_y
    prev_icp_y = 0
    global icp_index
    icp_index = 0

    while not rospy.is_shutdown():
        array = log_msgs(array)
        rate.sleep()
    # try:
    #     array = log_msgs(array)
    #     print('test')
    #     rospy.spin()
    # except KeyboardInterrupt:
    #     print("Shutting down")