#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist

import numpy as np
import pandas as pd

class Server:
    def __init__(self):
        self.switch = Bool()
        self.pose = Twist()
        self.velocity_left_cmd = Float64()
        self.velocity_left_meas = Float64()
        self.velocity_right_cmd = Float64()
        self.velocity_right_meas = Float64()

    def switch_callback(self, msg):
        # "Store" message received.
        self.switch = msg

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
        global prev_left_meas_velocity
        global prev_right_meas_velocity
        global velocity_meas_index
        new_row = np.zeros((1, 6))

        if (self.velocity_left_meas.data != prev_left_meas_velocity
                and self.velocity_left_cmd.data != prev_left_meas_velocity):
            prev_left_meas_velocity = self.velocity_left_meas.data
            prev_right_meas_velocity = self.velocity_right_meas.data
            velocity_meas_index += 1

        new_row = np.array(([rospy.get_rostime().secs, velocity_meas_index,
                             self.velocity_left_cmd.data, prev_left_meas_velocity,
                             self.velocity_right_cmd.data, prev_right_meas_velocity]))
        return np.vstack((array, new_row))

    def export_array(self, array):
        df = pd.DataFrame(data=array, columns=['ros_time', 'wheel_meas_index', 'cmd_left_vel', 'meas_left_vel',
                                               'cmd_right_vel', 'meas_right_vel'])
        df.to_csv('/home/dominic/Desktop/data.csv')

if __name__ == '__main__':
    rospy.init_node('pose_cmds_logger')
    rate = rospy.Rate(20)  # 20hz

    server = Server()

    rospy.Subscriber('calib_switch', Bool, server.switch_callback)
    rospy.Subscriber('odom', Twist , server.pose_callback)
    rospy.Subscriber('/left_drive/velocity', Float64, server.velocity_left_cmd_callback)
    rospy.Subscriber('/left_drive/status/speed', Float64, server.velocity_left_meas_callback)
    rospy.Subscriber('/right_drive/velocity', Float64, server.velocity_right_cmd_callback)
    rospy.Subscriber('/right_drive/status/speed', Float64, server.velocity_right_meas_callback)

    array = np.zeros((1, 6))
    odom_index = 0
    global prev_left_meas_velocity
    prev_left_meas_velocity = 0
    global prev_right_meas_velocity
    prev_right_meas_velocity = 0
    global velocity_meas_index
    velocity_meas_index = 0
    while not rospy.is_shutdown():
        if server.switch.data:
            #rospy.loginfo('on')
            array = server.log_msgs(array)
        else:
            #rospy.loginfo('false')
            server.export_array(array)
        rate.sleep()