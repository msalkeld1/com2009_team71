#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import roslaunch
import math

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from pathlib import Path

class MazeNavigation():
    def __init__(self):
        #cmd_vel and twist msg initialization as publisher
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.init_node('task3', anonymous=True)
        self.rate = rospy.Rate(10) 

        self.vel_cmd = Twist()
        #initializing an array with 360 items for 360 degrees
        self.right_arc = np.zeros(360)
        self.front_arc = np.zeros(360)
        self.ctrl_c = False
        #scan and Laserscan msg initialization as subscriber
        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.callback_lidar)
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo("the 'Maze Navigation' node is active...")
        # photoing part
        # Subscriber to the camera data topic
        self.camera_subscriber = rospy.Subscriber("/camera/color/image_raw",
            Image, self.camera_callback)
        
        # Liking to the CvBridege
        self.cvbridge_interface = CvBridge()

        # Defined the target beacon colour
        #self.target = "yellow" # choose a target colour
        self.target = rospy.get_param('~target_colour')

        print(f"YOUR TARGET BEACON COLOUR IS '{self.target}'!\n"
              f"WILL CAPTURE THE IMAGE OF IT IF SHOWS!")
        
        # initializa the data
        self.m00 = 0
        self.m00_min = 10000

        #defined the target saving path 
        self.base_image_path = Path.home().joinpath("catkin_ws/src/com2009_team71/snaps/")
        self.base_image_path.mkdir(parents=True, exist_ok=True)
        self.full_image_path = self.base_image_path.joinpath("task4_beacon.jpg")

    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        # crop the original image
        height, width, _ = cv_img.shape
        crop_width = width - 800
        crop_height = 400
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))
        self.crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(self.crop_img, cv2.COLOR_BGR2HSV)

       #recognize the beacon in the robot view
        if self.target == "yellow":  
            mask = cv2.inRange(hsv_img, ( 20, 100, 100), (30, 255, 255))
        elif self.target == "red":
            mask = cv2.inRange(hsv_img, ( 0, 185, 100), (10, 255, 255))
        elif self.target == "green":
            mask = cv2.inRange(hsv_img, (50, 150, 100), (65, 255, 255))
        elif self.target == "blue":
            mask = cv2.inRange(hsv_img, (115, 224, 100), (130, 255, 255)) 
        else:
            print("WRONG TARGET COLOUR!!!")                

        # image data processing
        res = cv2.bitwise_and(self.crop_img, self.crop_img, mask = mask)
        m = cv2.moments(mask)
        self.m00 = m['m00']
        self.cy = m['m10'] / (m['m00'] + 1e-5)

        if self.m00 > self.m00_min:
            cv2.circle(self.crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
        
        # show the robot view
        cv2.imshow(f"TARGET BEACON ----> '{self.target}'", self.crop_img)

        if self.m00 > self.m00_min:  #if target beacon show in the view
            cv2.imwrite(str(self.full_image_path), self.crop_img)
            print(f"Captured the target coulour(->'{self.target}'<-) beacon!!! \n"
                  f"Saving the the target location '{self.full_image_path}'")
            self.save_map()
            rospy.sleep()
        else:
            print(f"Target Beacon -> '{self.target}' <- No Here!!!")        
        cv2.waitKey(1)

    def shutdownhook(self):
        self.vel_cmd.linear.x = 0.0 
        self.vel_cmd.angular.z = 0.0
        # publish
        self.pub.publish(self.vel_cmd)
        self.ctrl_c = True

    def save_map(self):
        map_name = "task4_map"
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        print(f"Saving map at time: {rospy.get_time()}")
        node = roslaunch.core.Node(
            package="map_server",
            node_type="map_saver",
            args=f"-f {map_name}",
            output="screen"
        )
        process = launch.launch(node)

    def callback_lidar(self, lidar_data):        
        #left arc for 0 to 90 degrees, right arc for 270 to 0 degrees
        left_arc = lidar_data.ranges[0:91]
        right_arc = lidar_data.ranges[-90:]
        #front arc is 270 to 0 + 0 to 90, so 270 to 90 degrees
        self.front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        self.all_angles =  lidar_data.ranges              

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
