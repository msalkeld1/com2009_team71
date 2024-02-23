#!/usr/bin/env python3

import rospy 
from geometry_msgs.msg import Twist
import math


class Circle():

    def __init__(self):
        self.node_name = "circle_pub"
        topic_name = "cmd_vel"
        self.vel_msg = Twist()
        self.r = 0.5
        self.t = 30

        self.pub = rospy.Publisher(topic_name, Twist, queue_size=10) 
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)

        self.ctrl_c = False 
        rospy.on_shutdown(self.shutdownhook) 

        rospy.loginfo(f"The '{self.node_name}' node is active...") 

    def shutdownhook(self):
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.pub.publish(self.vel_msg)
        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
        self.ctrl_c = True

    def main(self):
        self.vel_msg.linear.x = (2* math.pi * self.r) / self.t
        self.vel_msg.angular.z = self.vel_msg.linear.x / self.r
        self.start_time = rospy.get_time()
        self.circle_counter = 0
        while not self.ctrl_c and self.circle_counter < 2:
            self.pub.publish(self.vel_msg)

            if rospy.get_time() - self.start_time > self.t:
                self.vel_msg.angular.z *= -1
                self.start_time = rospy.get_time()
                self.circle_counter += 1

            self.rate.sleep()

        self.circle_counter = 0
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.pub.publish(self.vel_msg)

if __name__ == '__main__':
    node = Circle()
    node.main()