#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry

class OdomSubscriber():

    def callback(self, topic_data: Odometry):
        pose = topic_data.pose.pose
        position = pose.position
        orientation = pose.orientation 
        pos_x = position.x
        pos_y = position.y

        if self.counter > 30:
            self.counter = 0
            print(f"x= {pos_x:.2f} [m], y = {pos_y:.2f} [m], yaw= {orientation.z:.1f} [degrees]")
        else:
            self.counter += 1

    def __init__(self):
        node_name = "odom_subscriber"
        rospy.init_node(node_name, anonymous=True)
        self.sub = rospy.Subscriber("odom", Odometry, self.callback)
        rospy.loginfo(f"The '{node_name}' node is active...")
        
        self.counter = 0 

    def main(self):
        # set the node to remain active until closed manually:
        rospy.spin()

if __name__ == '__main__':
    node = OdomSubscriber()
    node.main()