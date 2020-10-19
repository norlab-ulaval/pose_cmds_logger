#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

import numpy as np
import pandas as pd

class Server:
    def __init__(self):
        self.pose = Twist()
        self.velocity_left_cmd = Float64()
        self.velocity_right_cmd = Float64()

    def pose_callback(self, msg):
        # "Store" message received.
        self.pose = msg

    def velocity_left_cmd_callback(self, msg):
        # "Store" the message received.
        self.velocity_left_cmd = msg

    def velocity_right_cmd_callback(self, msg):
        # "Store" the message received.
        self.velocity_right_cmd = msg

    def log_msgs(self):
        # Create numpy array with adequate poses
        data = np.zeros((1, 4))


if __name__ == '__main__':
    rospy.init_node('pose_cmds_logger')

    server = Server()

    rospy.Subscriber('odom', Twist , server.pose_callback)
    rospy.Subscriber('/left_drive/velocity', Float64, server.velocity_right_cmd_callback)
    rospy.Subscriber('/right_drive/velocity', Float64, server.velocity_left_cmd_callback)

    while not rospy.is_shutdown():
        rospy.loginfo(server.velocity_right_cmd)

    rospy.spin()