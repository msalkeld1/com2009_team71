#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class WallFollowing():
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('wall_follower', anonymous=True)
        
        # Publisher for commanding robot movement
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        
        # Rate for the main loop
        self.rate = rospy.Rate(10)  # 10 Hz
        
        # Object for holding velocity commands
        self.vel_cmd = Twist()
        
        # Array to hold the laser scan data in the front arc
        self.front_arc = np.zeros(360)
        
        # Variables to hold current left and right distances
        self.current_left = float('inf')
        self.current_right = float('inf')
        
        # Flag to indicate shutdown
        self.ctrl_c = False
        
        # Subscriber to the LiDAR data topic
        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.callback_lidar)
        
        # Register shutdown hook
        rospy.on_shutdown(self.shutdownhook)
        
        rospy.loginfo("Wall following node is active...")

    def shutdownhook(self):
        # Stop the robot when shutting down the node
        self.vel_cmd.linear.x = 0.0
        self.vel_cmd.angular.z = 0.0
        self.pub.publish(self.vel_cmd)
        self.ctrl_c = True
    def callback_odom(self, data):
        # Extract the position and orientation from odometry data
        position = data.pose.pose.position
        self.current_position = (position.x, position.y)
        
        # Check if current position is close to any previously visited position
        if any(math.hypot(position.x - pos[0], position.y - pos[1]) < self.position_threshold for pos in self.position_history):
            self.at_previous_position = True
        else:
            self.at_previous_position = False
            self.position_history.append((position.x, position.y))
    def callback_lidar(self, lidar_data):
        # Extract relevant slices from LiDAR data arrays
        left_arc = lidar_data.ranges[0:91]
        right_arc = lidar_data.ranges[270:360]
        self.front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        
        # Update the current left and right distances
        self.current_left = lidar_data.ranges[90]
        self.current_right = lidar_data.ranges[270]

    def main_loop(self):
        while not self.ctrl_c:
            # Define sectors from the LiDAR data
            min_right = np.min(self.front_arc[120:140])
            min_front = np.min(self.front_arc[80:100])
            min_left = np.min(self.front_arc[20:40])
            max_left = np.max(self.front_arc[20:40])  # Sector for left side

            # Simple reactive controller
            if min_front < 0.5 and min_left < 0.35:
                # Too close to a front obstacle, need to turn right
                self.vel_cmd.angular.z = -2
                self.vel_cmd.linear.x = 0
                print('1')            
            elif min_front < 0.5 and max_left > 0.6:
                # Too close to the left wall, turn right slightly
                self.vel_cmd.angular.z = 2
                self.vel_cmd.linear.x = 0
                print('3')
            
            elif min_left > 0.6 and min_right > 0.6 and min_front > 0.6:
                # Both sides are clear, move forward faster
                self.vel_cmd.linear.x = 0.1
                self.vel_cmd.angular.z = 0.8
                print('5')
            
            
            else:
                # Default behavior to move forward
                print('6')
                self.vel_cmd.linear.x = 0.2
                self.vel_cmd.angular.z = 0

            # Publish the velocity command
            self.pub.publish(self.vel_cmd)
            self.rate.sleep()

if __name__ == '__main__':
    wall_follower = WallFollowing()
    try:
        wall_follower.main_loop()
    except rospy.ROSInterruptException:
        pass
