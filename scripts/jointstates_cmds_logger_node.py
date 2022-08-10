#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from pose_cmds_logger.srv import *
import message_filters

import numpy as np
import pandas as pd


calib_switch = Bool()
good_calib_step = Bool()
joy_switch = Bool()
pose = Odometry()
joint_state = JointState()
joint_state.position = [0, 0, 0, 0]
imu_vel = Imu()
cmd_vel = Twist()

def switch_callback(msg):
    # "Store" message received.
    global calib_switch
    calib_switch = msg

def joy_callback(msg):
    # "Store" message received.
    global joy_switch
    joy_switch = msg

def good_calib_step_callback(msg):
    # "Store" message received.
    global good_calib_step
    good_calib_step = msg

def pose_callback(msg):
    # "Store" message received.
    global pose
    pose = msg

def jointstate_callback(msg):
    global joint_state
    joint_state = msg

def imu_callback(msg):
    # "Store" message received.
    global imu_vel
    imu_vel = msg

def cmd_vel_callback(msg):
    global cmd_vel
    cmd_vel = msg

def log_msgs(array):
    # Create numpy array with adequate poses
    global prev_icp_x
    global prev_icp_y
    global icp_index
    new_row = np.zeros((1, 18))

    if (pose.pose.pose.position.x != prev_icp_x
            and pose.pose.pose.position.y != prev_icp_y):
        prev_icp_x = pose.pose.pose.position.x
        prev_icp_y = pose.pose.pose.position.y
        icp_index += 1

    new_row = np.array(([rospy.get_rostime(), joy_switch.data, icp_index, good_calib_step.data,
                         joint_state.position[0], joint_state.position[1],
                         cmd_vel.linear.x, cmd_vel.angular.z,
                         pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z,
                         pose.pose.pose.orientation.x, pose.pose.pose.orientation.y,
                         pose.pose.pose.orientation.z, pose.pose.pose.orientation.w,
                         imu_vel.angular_velocity.x, imu_vel.angular_velocity.y,
                         imu_vel.angular_velocity.z]))

    return np.vstack((array, new_row))

def save_data_handle(req, array):
        rospy.loginfo('Converting Array to DataFrame')
        df = pd.DataFrame(data=array, columns=['ros_time', 'joy_switch', 'icp_index', 'good_calib_step',
                                               'wheel_pos_left', 'wheel_pos_right',
                                               'cmd_vel_x', 'cmd_vel_y',
                                               'icp_pos_x', 'icp_pos_y', 'icp_pos_z',
                                               'icp_quat_x', 'icp_quat_y',
                                               'icp_quat_z', 'icp_quat_w',
                                               'imu_x', 'imu_y', 'imu_z'])
        rospy.loginfo('Exporting DataFrame as .csv')
        df.to_csv(req.data_file_name.data)
        rospy.loginfo('Data export done!')
        return []

if __name__ == '__main__':
    rospy.init_node('pose_cmds_logger')
    rate = rospy.Rate(20)  # 20hz


    calib_sub = rospy.Subscriber('calib_switch', Bool, switch_callback)
    joy_sub = rospy.Subscriber('joy_switch', Bool, joy_callback)
    good_calib_step_sub = rospy.Subscriber('good_calib_step', Bool, good_calib_step_callback)
    icp_sub = rospy.Subscriber('/icp_odom', Odometry , pose_callback)
    cmd_left_sub = rospy.Subscriber('/joint_states', JointState, jointstate_callback)
    imu_sub = rospy.Subscriber('/MTI_imu/data_raw', Imu, imu_callback)
    cmd_vel_sub = rospy.Subscriber('/doughnut_cmd_vel', Twist, cmd_vel_callback)

    save_service = rospy.Service('save_data', SaveData, lambda msg: save_data_handle(msg, array))

    array = np.zeros((1, 18))
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