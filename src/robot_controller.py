#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import degrees
import numpy as np

class RobotMove(object):
    def __init__(self):
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.publisher_rate = rospy.Rate(10)
        self.vel_cmd = Twist()

    def set_move_cmd(self, linear = 0.0, angular = 0.0):
        self.vel_cmd.linear.x = linear
        self.vel_cmd.angular.z = angular

    def publish(self):
        self.publisher.publish(self.vel_cmd)

    def stop(self):
        self.set_move_cmd()
        self.publish()


class RobotLaserScan(object):
    def laserscan_cb(self, scan_data):
        left_arc = scan_data.ranges[0:21]
        right_arc = scan_data.ranges[-20:]
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])

        self.min_distance = front_arc.min()
        arc_angles = np.arange(-20, 21)
        self.closest_object_position = arc_angles[np.argmin(front_arc)]

    def __init__(self):
        self.min_distance = 0.0
        self.closest_object_position = 0.0
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.laserscan_cb)
