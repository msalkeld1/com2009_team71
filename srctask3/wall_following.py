#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class WallFollowing:
    frontLeft = 0
    frontRight = 0
    front = 10
    left = 0
    right = 0
    
    state = 0
    states = {
        0: 'find the wall',
        1: 'turn right',
        2: 'follow the wall'
    }

    def change_state(self, newState):
        self.state = newState

    def findTheWall(self):
        if self.front >= 0.5:
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = 0.2
            cmd_vel_msg.angular.z = 0
            self.cmd_pub.publish(cmd_vel_msg)          
            return False
        else:
            self.turn_right()
            self.change_state(2)
            return True
    def go_straight(self):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.2
        cmd_vel_msg.angular.z = 0
        self.cmd_pub.publish(cmd_vel_msg)

    def follow_wall(self):
            d = 0.5

            # Check if there's enough space on the left to move forward
            if self.frontLeft > d and self.left < d:
                self.go_straight()
                state_description = 'case 1 - moving forward'
            # Check if there's an obstacle in front and to the left, indicating a need to turn right
            elif self.front < d and self.left < d:
                self.turn_right()
                state_description = 'case 2 - turning right to follow the wall'
            # Check if there's an obstacle in front but enough space on the right to move forward
            elif self.front < d and self.frontRight > d:
                self.go_straight()
                state_description = 'case 3 - moving forward'
            # Check if there's an obstacle on the right and in front, indicating the need to turn left
            elif self.frontRight < d and self.front < d:
                self.turn_left()
                state_description = 'case 4 - turning left to follow the wall'
            # If none of the above conditions are met, keep moving forward
            else:
                self.go_straight()
                state_description = 'case 5 - moving forward'

            rospy.loginfo(state_description)


            rospy.loginfo(state_description)





    def stop(self):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0
        cmd_vel_msg.angular.z = 0
        self.cmd_pub.publish(cmd_vel_msg)

    def main(self):
        rospy.init_node('wall_following')
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)        
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.state == 0:
                print(self.state)
                self.findTheWall()          
            elif self.state == 2:
                print(self.state)
                self.follow_wall()
            else:
                rospy.logerr('Unknown state!')
            rate.sleep()
    
    def turn_right(self):
         cmd_vel_msg = Twist()
         cmd_vel_msg.linear.x = 0
         cmd_vel_msg.angular.z = -1.0
         self.cmd_pub.publish(cmd_vel_msg)
         
    def turn_left(self):
         cmd_vel_msg = Twist()
         cmd_vel_msg.linear.x = 0
         cmd_vel_msg.angular.z = 1.0
         self.cmd_pub.publish(cmd_vel_msg)

    def scan_callback(self, msg):
        self.front = msg.ranges[0]
        self.right = msg.ranges[270]
        self.frontLeft = msg.ranges[40]
        self.frontRight = msg.ranges[300]
        self.left = msg.ranges[90]


if __name__ == '__main__':
    try:
        wall_following = WallFollowing()
        wall_following.main()
    except rospy.ROSInterruptException:
        pass



