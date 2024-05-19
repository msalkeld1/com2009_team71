#!/usr/bin/env python3


import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from math import degrees, pi
from tf.transformations import euler_from_quaternion
import numpy as np

class MazeNavigation():

    def __init__(self):

        #cmd_vel and twist msg initialization as publisher
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.init_node('task3', anonymous=True)
        self.rate = rospy.Rate(10) # hz

        self.vel_cmd = Twist()
        #initializing an array with 360 items for 360 degrees
        self.front_arc = np.zeros(360)
        self.back_front_arc = np.zeros(360)

        self.ctrl_c = False
        #scan and Laserscan msg initialization as subscriber
        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.callback_lidar)
        rospy.on_shutdown(self.shutdownhook)

        self.subscriber = rospy.Subscriber('/odom', Odometry, self.odom_cb)

        self.all_angles = np.zeros(360)
        self.walls_blocked = False
        self.min_front = 0.0
        self.min_left = 0.0
        self.start_time = 0.0

        self.firstturn = False

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

        #back left arc 
        back_left_arc = lidar_data.ranges[90:181]
        back_right_arc = lidar_data.ranges[180:271]

        #front arc is 270 to 0 + 0 to 90, so 270 to 90 degrees
        self.back_front_arc = np.array(back_left_arc[::-1] + back_right_arc[::-1])

        self.all_angles =  lidar_data.ranges

        # i = 0
        # while i < 360:
        #     print(f"  @@@ starting self.all_angles = {i} =  {self.all_angles[i]} meters")
        #     i = i + 1

        # print(f"  **** starting left_arc =  {left_arc} meters")
        # print(f"  **** starting sright_arc =  {right_arc} meters")
        # print(f"  **** starting self.front_arc =  {self.front_arc} meters")

    def odom_cb(self, odom_data):
        orientation = odom_data.pose.pose.orientation
        position = odom_data.pose.pose.position
        (_, _, yaw) = euler_from_quaternion([orientation.x,
            orientation.y, orientation.z, orientation.w],'sxyz')
        
        self.theta_zd = self.round(degrees(yaw), 4)
                
    def round(self, value, precision):
        value = int(value * (10**precision))
        return float(value) / (10**precision)        

    def exit_block(self):

        print("turn early if going into close space")

        # print(f" before self.walls_blocked == True min_left = {self.min_left}  min_front = {self.min_front}  meters")        

        turn_right1_complete = False
        turn_right2_complete = False
        target_for_right1 = 0.40
        target_for_right2 = 0.60
        u_turn_complete = False

        self.start_time = rospy.get_time()

        while u_turn_complete == False:

            self.min_left = np.amin(self.front_arc[40:60])
            self.min_front = np.amin(self.front_arc[80:100])      

            self.back_min_left = np.amin(self.back_front_arc[40:60])                          
            self.back_min_right = np.amin(self.back_front_arc[120:140])        

            # print(f" inside min_front = {self.min_front}  target_for_right1 = {target_for_right1}  target_for_right2 = {target_for_right2} meters")     

            # reverse left if needed
            if (self.min_front < 0.1):
                self.vel_cmd.angular.z = 1.3
                self.vel_cmd.linear.x = -0.1               
   
            # new code for back touching
            elif (self.back_min_left < 0.1) :
                self.vel_cmd.angular.z = -1.3
                self.vel_cmd.angular.z = 0.0
                self.vel_cmd.linear.x = 0.1 

            elif self.min_front > target_for_right1 and turn_right1_complete == False:
                self.vel_cmd.angular.z = -1.3
                self.vel_cmd.linear.x = 0.0         
            elif self.min_front < target_for_right1 and turn_right1_complete == False:
                turn_right1_complete = True
                # print(f" after turn_right1_complete--- target_for_right1 = {target_for_right1} ")    
                self.vel_cmd.angular.z = 0.0
                self.vel_cmd.linear.x = 0.0   
            elif self.min_front < target_for_right2 and  turn_right1_complete == True and turn_right2_complete == False:
                self.vel_cmd.angular.z = -1.3
                self.vel_cmd.linear.x = 0.0  
            elif self.min_front > target_for_right2 and  turn_right1_complete == True and turn_right2_complete == False: 
                turn_right2_complete = True
                self.vel_cmd.angular.z = 0.0
                self.vel_cmd.linear.x = 0.0        
                # print(f" after turn_right2_complete--- target_for_right2 = {target_for_right2} ")    

            if turn_right1_complete == True and turn_right2_complete == True :
                u_turn_complete = True

            if (rospy.get_time() - self.start_time) >= 10.0:
                print(f" STUCK breaking from loop ")    
                break

            self.pub.publish(self.vel_cmd)
            self.rate.sleep()                    
        
        # print(f" after self.walls_blocked == True min_left = {self.min_left}  min_front = {self.min_front}  meters")        
        self.walls_blocked == False   


    def main(self):

  
        while not self.ctrl_c:

            no_of_block = 0

            self.min_left = np.amin(self.front_arc[40:60])
            self.min_front = np.amin(self.front_arc[80:100])

            self.back_min_left = np.amin(self.back_front_arc[40:60])                          
            self.back_min_right = np.amin(self.back_front_arc[120:140])        
            

            # print(f" @@@ min_left = { self.min_left }  min_front = { self.min_front }  meters")     
            # print(f" *** [0] = {self.front_arc[0]} ** [5] = {self.front_arc[5]}  [25] = {self.front_arc[25]}  [35]= {self.front_arc[35]} [45]= {self.front_arc[45]}  [55] {self.front_arc[55]} [65] {self.front_arc[65]} ")        
            # print(f" *** [115] = {self.front_arc[115]}  [125] = {self.front_arc[125]}  [135]= {self.front_arc[135]} [145]= {self.front_arc[145]}  [155] {self.front_arc[155]} ")        

            if ( self.min_front  < 0.8 and 
                self.front_arc[5] < 0.5 and self.front_arc[25] < 0.5 and self.front_arc[35] < 0.5 and self.front_arc[45] < 0.5 and self.front_arc[55] < 0.7 and
                self.front_arc[65] < 0.8 and
                self.front_arc[115] <= 0.8 and self.front_arc[125] <= 0.6 and self.front_arc[135] <= 0.6 and self.front_arc[145] <= 0.6 and self.front_arc[155] <= 0.6 and
                self.front_arc[175] <= 0.6 ):   
               

                self.walls_blocked = True
                # print(" @@@@ reached EXIT block ")        
            else:
                self.walls_blocked = False

            # reverse left if needed
            if (self.min_front < 0.1):
                self.vel_cmd.angular.z = 1.3
                self.vel_cmd.linear.x = -0.1

                # print("reversing")

            # aviod back obsticle
            elif (self.back_min_left < 0.1) :
                self.vel_cmd.angular.z = 0.0
                self.vel_cmd.linear.x = 0.1    
                                   

            #adjust towards right in place if needed
            elif (self.min_front < 0.4):
                self.vel_cmd.angular.z = -1.3
                self.vel_cmd.linear.x = 0.00
                # print("reorienting")
                                    
            elif self.walls_blocked == True :       
                no_of_block = no_of_block + 1
                print(f"exit_block = {no_of_block} code starts")   

                self.exit_block()
            #move forward if no obsticle 
            elif (self.min_left > 0.33 and self.min_left < 0.43 and self.min_front >= 0.4 and self.walls_blocked == False) :

                self.vel_cmd.angular.z = 0
                self.vel_cmd.linear.x = 0.26
                # print("forward")

            #move left if left wall and front wall are far away
            elif(self.min_left >= 0.43 and self.min_front >= 0.4 and self.walls_blocked == False):
                self.vel_cmd.angular.z = 0.90
                self.vel_cmd.linear.x = 0.26
                # print("left")

            #move right if front wall is at 0.4m and maintain until it is > than 0.4 and left wall is too close
            elif(self.min_left <= 0.33 and self.min_front >= 0.4):
                self.vel_cmd.angular.z = -0.90
                self.vel_cmd.linear.x = 0.26
                # print("right")

            
            self.pub.publish(self.vel_cmd)
            self.rate.sleep()

if __name__ == '__main__':
    vel_ctlr = MazeNavigation()
    try:
        vel_ctlr.main()
    except rospy.ROSInterruptException:
        pass
 