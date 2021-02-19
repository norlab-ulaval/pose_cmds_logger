#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState

import numpy as np
import pandas as pd

class Server:
    def __init__(self):
        self.switch = Bool()
        self.imu_vel = Imu()
        self.js = JointState()
        self.velocity_right_cmd = Float64()
        self.velocity_right_meas = Float64()

    def switch_callback(self, msg):
        # "Store" message received.
        self.switch = msg

    def js_callback(self, msg):
        # "Store" message received.
        self.js = msg

    def imu_callback(self, msg):
        # "Store" message received.
        self.imu_vel = msg

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
        global right_js_vel
        new_row = np.zeros((1, 6))

        if self.velocity_right_meas.data != prev_right_meas_velocity:
            prev_right_meas_velocity = self.velocity_right_meas.data
            velocity_meas_index += 1

        if (self.js.name == ['front_left_wheel_joint', 'front_right_wheel_joint',
                             'rear_left_wheel_joint', 'rear_right_wheel_joint']):
            right_js_vel = self.js.velocity[1]

        new_row = np.array(([rospy.get_rostime(), velocity_meas_index, self.velocity_right_cmd.data,
                             prev_right_meas_velocity, right_js_vel, self.imu_vel.angular_velocity.z]))
        return np.vstack((array, new_row))

    def export_array(self, array):
        df = pd.DataFrame(data=array, columns=['ros_time', 'wheel_meas_index', 'cmd_right_vel',
                                               'meas_right_vel', 'js_right_vel', 'imu_vel'])
        df.to_csv('/home/dominic/Desktop/data_imu.csv')

if __name__ == '__main__':
    rospy.init_node('imu_cmds_logger')
    rate = rospy.Rate(100)  # 100hz

    server = Server()

    rospy.Subscriber('calib_switch', Bool, server.switch_callback)
    rospy.Subscriber('/joint_states', JointState, server.js_callback)
    rospy.Subscriber('/MTI30_imu/data_raw', Imu, server.imu_callback)
    rospy.Subscriber('/right_drive/velocity', Float64, server.velocity_right_cmd_callback)
    rospy.Subscriber('/right_drive/status/speed', Float64, server.velocity_right_meas_callback)

    array = np.zeros((1, 6))
    odom_index = 0
    global prev_right_meas_velocity
    prev_right_meas_velocity = 0
    global velocity_meas_index
    velocity_meas_index = 0
    global right_js_vel
    right_js_vel = 0
    while not rospy.is_shutdown():
        if server.switch.data:
            #rospy.loginfo('on')
            array = server.log_msgs(array)
        elif not server.switch.data:
            #rospy.loginfo('false')
            server.export_array(array)
        rate.sleep()