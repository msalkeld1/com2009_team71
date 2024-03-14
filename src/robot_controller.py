#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
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
