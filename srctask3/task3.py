#!/usr/bin/env python3


import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class MazeNavigation():

    def __init__(self):

        #cmd_vel and twist msg initialization as publisher
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.init_node('task3', anonymous=True)
        self.rate = rospy.Rate(10) 

        self.vel_cmd = Twist()
        #initializing an array with 360 (integer 0) items for 360 degrees
        self.right_arc = np.zeros(360)
        self.front_arc = np.zeros(360)
        self.ctrl_c = False
        #scan and Laserscan msg initialization as subscriber
        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.callback_lidar)
        rospy.on_shutdown(self.shutdownhook)

        
        rospy.loginfo("the 'Maze Navigation' node is active...")

    def shutdownhook(self):
        self.vel_cmd.linear.x = 0.0 
        self.vel_cmd.angular.z = 0.0

        # publish
        self.pub.publish(self.vel_cmd)
        self.ctrl_c = True

    def callback_lidar(self, lidar_data):
        
        #left arc for 0 to 90 degrees, right arc for 270 to 0 degrees
        left_arc = lidar_data.ranges[0:91]
        right_arc = lidar_data.ranges[-90:]
        #front arc is 270 to 0 + 0 to 90, so 270 to 90 degrees
        self.front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        self.all_angles =  lidar_data.ranges

        # i = 0
        # while i < 360:
        #     print(f"  @@@ starting self.all_angles = {i} =  {self.all_angles[i]} meters")
        #     i = i + 1

        # print(f"  **** starting left_arc =  {left_arc} meters")
        # print(f"  **** starting sright_arc =  {right_arc} meters")
        # print(f"  **** starting self.front_arc =  {self.front_arc} meters")
                

    def main_loop(self):

        while not self.ctrl_c:
           
            min_left = np.amin(self.front_arc[40:60])
            min_front = np.amin(self.front_arc[80:100])

            # reverse left if needed
            if (min_front < 0.1):
                self.vel_cmd.angular.z = 1.3
                self.vel_cmd.linear.x = -0.1

                print("reversing")
            #adjust towards right in place if needed
            elif (min_front < 0.4):
                self.vel_cmd.angular.z = -1.3
                self.vel_cmd.linear.x = 0.00

                print("reorienting")

            #move forward if no obsticle 
            elif (min_left > 0.33 and min_left < 0.43 and min_front >= 0.4):
                self.vel_cmd.angular.z = 0
                self.vel_cmd.linear.x = 0.26
                print("forward")

            #move left if left wall and front wall are far away
            elif(min_left >= 0.43 and min_front >= 0.4):

                self.vel_cmd.angular.z = 0.90
                self.vel_cmd.linear.x = 0.26
                print("left")
            
            #move right if front wall is at 0.4m and maintain until it is > than 0.4 and left wall is too close
            elif(min_left <= 0.33 and min_front >= 0.4):
                
                self.vel_cmd.angular.z = -0.90
                self.vel_cmd.linear.x = 0.26
                print("right")
            
            self.pub.publish(self.vel_cmd)

            self.rate.sleep()

if __name__ == '__main__':
    vel_ctlr = MazeNavigation()
    try:
        vel_ctlr.main_loop()
    except rospy.ROSInterruptException:
        pass
 
