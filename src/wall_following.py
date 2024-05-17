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
        self.rate = rospy.Rate(10)  # 50 Hz
        
        # Object for holding velocity commands
        self.vel_cmd = Twist()
        
        # Array to hold the laser scan data in the front arc
        self.front_arc = np.zeros(360)
        self.right_arc = np.zeros(360)
        self.lef_arc = np.zeros(360)


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

    def pid(self, min_left, min_right):
        kp = 3
        reference_input = 0
        feedback_signal = min_left - min_right
        error = feedback_signal - reference_input 

        ang_vel = kp * error
        ang_vel = max(min(ang_vel, 1.82), -1.82)
        print(f"Error = {error:.1f} | Control Signal = {ang_vel:.2f} rad/s")
        return ang_vel

    def callback_lidar(self, lidar_data):
        # Extract relevant slices from LiDAR data arrays
        left_arc = lidar_data.ranges[0:91]
        right_arc = lidar_data.ranges[270:360]
        self.front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        
    def main_loop(self):
        while not self.ctrl_c:
            # Define sectors from the LiDAR data
            min_right = np.amin(self.front_arc[120:140])
            min_front = np.amin(self.front_arc[80:100])
            min_left = np.amin(self.front_arc[20:40])

            # Simple reactive controller
            if min_front <= 0.5 and min_left < 0.5:
                # Too close to a front obstac
                # le, need to turn right
                self.vel_cmd.angular.z = -1.8
                self.vel_cmd.linear.x = 0
            elif min_left <= 0.3:
                self.vel_cmd.angular.z = -1
                self.vel_cmd.linear.x = 0.1
            elif min_front < 0.4 and min_left > 0.5:
                # Too close to the left wall, turn right slightly
                self.vel_cmd.angular.z = 1.5
                self.vel_cmd.linear.x = 0.01
            elif min_front >= 0.4 and min_left > 0.5:
                # Too close to the left wall, turn right slightly
                self.vel_cmd.angular.z = 0.9
                self.vel_cmd.linear.x = 0.26
            elif min_left > 0.6 and min_right > 0.6 and min_front > 0.6:
                # Both sides are clear, move forward faster
                self.vel_cmd.linear.x = 0.26
                self.vel_cmd.angular.z = 0.5
            
            else:
                # Default behavior to move forward
                
                if min_left < 0.5 and min_right < 0.5:
                    self.vel_cmd.linear.x = 0.2
                    self.vel_cmd.angular.z = self.pid(min_left, min_right)
                else:
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
