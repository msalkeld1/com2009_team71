#!/usr/bin/env python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import actionlib
import argparse
import roslaunch
# Import all the necessary ROS message types:
from tuos_msgs.msg import SearchFeedback, SearchResult, SearchAction, SearchGoal
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import String
# Import the tb3 modules from tb3.py
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan
from pathlib import Path

# Import some other useful Python Modules
from math import sqrt, pow
import numpy as np

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge, CvBridgeError

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from math import sqrt, pow, pi, degrees , atan2, sin, cos, pi


class SearchActionServer(object):
    feedback = SearchFeedback() 
    result = SearchResult()

    def __init__(self):
        self.actionserver = actionlib.SimpleActionServer("/search_action_server", 
            SearchAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()
        self.pic_path = Path.home().joinpath("catkin_ws/src/com2009_team71/snaps/the_beacon.jpg")
        self.ctrl_c = False
        self.take_dummy_photo = True
        rospy.on_shutdown(self.shutdown_ops)
        self.rate = rospy.Rate(10)

        self.vel_controller = Tb3Move()
        self.tb3_odom = Tb3Odometry()
        self.tb3_lidar = Tb3LaserScan()
        self.lidar_subscriber = self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)


        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        #zeroing robot pose
        self.x0 = 0.0
        self.y0 = 0.0
        self.z0 = 0.0

        self.posx0 = 0.0
        self.posy0 = 0.0
        self.posz0 = 0.0

        self.startposx0 = 0.0
        self.startposy0 = 0.0

        self.zone1 = False
        self.zone2 = False
        self.zone3 = False
        self.zone4 = False

        self.stop_counter = 0
        self.move_rate = "" # fast, slow or stop

        self.distance = 0
        self.firstleft = False
        self.zonesloc = np.zeros(8)

        #zeroing lidar
        self.front_min = 0
        self.right_min = 0
        self.left_min = 0

        # Thresholds for ["Blue", "Red", "Green", "Turquoise"]
        self.search = False
        self.detected = False
        self.lower = []
        self.upper = []
        self.mask = np.zeros((1080,1920,1), np.uint8)
        self.hsv_img = np.zeros((1080,1920,3), np.uint8)
        self.m00 = 0
        self.m00_min = 100000
        self.turn_vel_fast = -0.5      
        self.turn_vel_slow = -0.1

        self.m00 = 0
        self.cy = 0
        self.cz = 0        


        # Command-Line Interface:
        cli = argparse.ArgumentParser(description=f"Command-line interface for the 'argparse' node.")
       
        cli.add_argument("-target_colour", metavar="COL", type=String,
            default="red", 
            help="The name of a colour (for example)")
               
        # obtain the arguments passed to this node from the command-line:
        self.args = cli.parse_args(rospy.myargv()[1:])
        
        self.target_colour = self.args.target_colour.data
        # self.target_colour = self.args.colour.data

        # self.target_colour = "red"

        self.gotoblock = 1

        print (f" **** COLOUR targetted **** == {self.target_colour}")
        
    
    def scan_callback(self, scan_data):
        f_left_arc = scan_data.ranges[0:25]
        f_right_arc = scan_data.ranges[-25:]

        right_arc = np.array(scan_data.ranges[320:340])
        left_arc = np.array(scan_data.ranges[26:47])

        front_arc = np.array(f_left_arc[::-1] + f_right_arc[::-1])
        self.front_min = np.amin(front_arc)

        self.right_min = np.amin(right_arc)

        self.left_min = np.amin(left_arc)

        self.rightwallat270 = scan_data.ranges[270]
        self.rightwallat280 = scan_data.ranges[280]        


    
    def set_the_color(self):
        # Thresholds for ["Blue", "Red", "Green", "Turquoise"]
        color_threshold = {
            "blue": [(97, 62, 100), (103, 256, 255)],
            "red": [(145, 80, 100), (180, 247, 255)],
            "green": [(81, 60, 100), (101, 256, 255)],
            "turquoise": [(75, 150, 100), (100, 255, 255)],
            "purple": ([145, 190, 100], [155, 255, 255]),
            "yellow": ([23, 102, 100], [28, 176, 255])
        }

        for color, (lower, upper) in color_threshold.items():
            lower = np.array(lower)
            upper = np.array(upper)
            if self.target_colour == color:
                self.color = color
                self.lower = lower
                self.upper = upper
                self.search = True

    def show_and_save_image(self, img):
        full_image_path = self.pic_path
        cv2.imwrite(str(full_image_path), img)

    def camera_callback(self, img_data):
        try:
            self.cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        if (self.take_dummy_photo == True):
            self.show_and_save_image(self.cv_img)
            self.take_dummy_photo = False
        height, width, _ = self.cv_img.shape
        crop_width = width - 800
        crop_height = 400
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2)) + 200

        crop_img = self.cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        self.hsv_img = hsv_img


        # for i in range(4):
        #     if i == 0:
        #         mask = cv2.inRange(hsv_img, self.lower[i], self.upper[i])
        #         # print("mask123")
        #     else:
        #         mask = mask + cv2.inRange(hsv_img, self.lower[i], self.upper[i])
        #         # print("maskmasktest")


        #         BLUE             GREEN           RED             TURQUOISE
        # lowers = [(115, 224, 100), (50, 150, 100), ( 0, 185, 100), ( 75, 150, 100)]
        # uppers = [(130, 255, 255), (65, 255, 255), (10, 255, 255), (100, 255, 255)]
        
        # self.lower = (115, 224, 100)
        # self.upper = (130, 255, 255)
        mask = cv2.inRange(hsv_img, self.lower, self.upper)
        res = cv2.bitwise_and(crop_img, crop_img, mask = mask)

        m = cv2.moments(self.mask)
            
        self.m00 = m["m00"]
        self.cy = m["m10"] / (m["m00"] + 1e-5)
        self.cz = m['m01'] / (m['m00'] + 1e-5)

        # print(f" @ continue_search {self.m00} = {self.m00_min} = self.cy = {self.cy} self.c = {self.cz}")


        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), int(self.cz)), 10, (0, 0, 255), 2)
        
        # cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)

    def shutdown_ops(self):
        self.vel_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True

    def setzone_location(self):
        xpositive = False
        ypositive = False  
        if self.tb3_odom.posx > 0 : 
            xpositive = True    
        else:
            xpositive = False    

        if self.tb3_odom.posy > 0 : 
            ypositive = True    
        else:
            ypositive = False    

        if  xpositive == True and ypositive == True and self.zone1 == False: 
            self.zone1 = True
            self.zonesloc[0] = self.tb3_odom.posx
            self.zonesloc[1] = self.tb3_odom.posy
            print(f" @@@@@@   ZONE zone1 = {self.zone1} x == {self.zonesloc[0]}, y == {self.zonesloc[1]}")

        elif  xpositive == True and ypositive == False and  self.zone2 == False: 
            self.zone2 = True  
            self.zonesloc[2] = self.tb3_odom.posx
            self.zonesloc[3] = self.tb3_odom.posy           
            print(f" @@@@@@   ZONE zone2 = {self.zone2} x == {self.zonesloc[2]}, y == {self.zonesloc[3]}")

        elif  xpositive == False and ypositive == False and  self.zone3 == False: 
            self.zone3 = True
            self.zonesloc[4] = self.tb3_odom.posx
            self.zonesloc[5] = self.tb3_odom.posy    
            print(f" @@@@@@   ZONE zone3 = {self.zone3} x == {self.zonesloc[4]}, y == {self.zonesloc[5]}")

        elif  xpositive == False and ypositive == True and  self.zone4 == False: 
            self.zone4 = True      
            self.zonesloc[6] = self.tb3_odom.posx
            self.zonesloc[7] = self.tb3_odom.posy   
            print(f" @@@@@@   ZONE zone4= {self.zone4} x == {self.zonesloc[6]}, y == {self.zonesloc[7]}")        


    def colour_search(self):

        while not self.ctrl_c:
            if self.stop_counter > 0:
                self.stop_counter -= 1

            if self.m00 > self.m00_min:
                # blob detected
                if self.cy >= 560-100 and self.cy <= 560+100:
                    if self.move_rate == 'slow':
                        self.move_rate = 'stop'
                        self.stop_counter = 30
                else:
                    self.move_rate = 'slow'
            else:
                self.move_rate = 'fast'
                
            if self.move_rate == 'fast':
                print("MOVING FAST: I can't see anything at the moment, scanning the area...")
                self.vel_controller.set_move_cmd(0.0, self.turn_vel_fast)
            elif self.move_rate == 'slow':
                print(f"MOVING SLOW: A blob of colour of size {self.m00:.0f} pixels is in view at y-position: {self.cy:.0f} pixels.")
                self.vel_controller.set_move_cmd(0.0, self.turn_vel_slow)
            elif self.move_rate == 'stop' and self.stop_counter > 0:
                print(f"STOPPED: The blob of colour is now dead-ahead at y-position {self.cy:.0f} pixels... Counting down: {self.stop_counter}")
                self.vel_controller.set_move_cmd(0.0, 0.0)
            else:
                print(f"MOVING SLOW: A blob of colour of size {self.m00:.0f} pixels is in view at y-position: {self.cy:.0f} pixels.")
                self.vel_controller.set_move_cmd(0.0, self.turn_vel_slow)
            
            self.vel_controller.publish()
            self.rate.sleep()        
        

    def continue_search(self):
           
        # print(f" @ continue_search {self.m00} = {self.m00_min} = self.cy = {self.cy}")

        if self.m00 > self.m00_min and self.detected == False:
            # print(f" @@@@@@@@@ blob detected . self.cy = {self.cy}")

            self.colour_search()

        elif self.front_min > self.distance:
            # #both sides are clear     

            if self.right_min > self.distance and self.left_min >self.distance:
                self.vel_controller.set_move_cmd(0.22, 0.3)
                self.vel_controller.publish()
                self.vel_controller.set_move_cmd(0.22, -0.30)
                self.vel_controller.publish()
 
            # Right is not clear
            elif self.right_min < self.distance and self.left_min > self.distance:
                self.vel_controller.set_move_cmd(0.15, 1)
                self.vel_controller.publish()
            # left is not clear
            elif self.right_min > self.distance and self.left_min < self.distance:
                self.vel_controller.set_move_cmd(0.15, -1)
                self.vel_controller.publish()

            #  right and left are not clear
            else:
                self.vel_controller.set_move_cmd(0.26, 0)
                self.vel_controller.publish()

        else:
            # right and left is clear
          
            if self.right_min > self.distance and self.left_min > self.distance:
                if self.right_min > self.left_min:
                    self.vel_controller.set_move_cmd(0, -1.82)
                    self.vel_controller.publish()
                else:
                    self.vel_controller.set_move_cmd(0, 1.82)
                    self.vel_controller.publish()
            #  both are not clear backwards
            elif self.right_min < self.distance and self.left_min < self.distance:
                self.vel_controller.set_move_cmd(-0.05, -1)
                self.vel_controller.publish()
            #  left is not clear
            elif self.right_min > self.distance and self.left_min < self.distance:
                self.vel_controller.set_move_cmd(-0.05, -1)
                self.vel_controller.publish()
            # right is not clear 
            elif self.right_min < self.distance and self.left_min > self.distance:
                self.vel_controller.set_move_cmd(-0.05, 1)
                self.vel_controller.publish()

    def turn_right(self):
        self.vel_controller.set_move_cmd(0.0, -0.3)
        start_time = rospy.get_time()
        # CHECK the time
        while rospy.get_time() - start_time < 5.0:
            self.vel_controller.publish() 

        self.vel_controller.set_move_cmd(linear=0.0, angular=0.0)
        self.vel_controller.publish()    
        


    def turn_left(self):

        self.vel_controller.set_move_cmd(0.0, 0.3)
        start_time = rospy.get_time()
        # CHECK the time
        while rospy.get_time() - start_time < 5.0:
            self.vel_controller.publish() 

        self.vel_controller.set_move_cmd(linear=0.0, angular=0.0)
        self.vel_controller.publish()    


    def turn_right_wall(self):

        # #move 0.2m
        # print("before move 0.2m")
        self.move_forward(linear=0.2, time=0, meters=0.2)
        # print("after move 0.2m")

        self.vel_controller.set_move_cmd(linear=0.0, angular=0.0)
        self.vel_controller.publish()  
                
        self.vel_controller.set_move_cmd(0.0, -0.3)
        start_time = rospy.get_time()
        # CHECK the time
        while rospy.get_time() - start_time < 5.0:
            self.vel_controller.publish() 
            continue    
        # #move 0.6m
        # print("before move 0.6m")
        self.move_forward(linear=0.2, time=0, meters=0.2 )
        # print("after move 0.6m")

        self.vel_controller.set_move_cmd(0.0, 0.0)



    def move_forward(self, linear=0.0, time = 0, meters=0.0):

        # print(f"  ** self.tb3_lidarLeft.min_distance = {self.tb3_lidarLeft.min_distance} meters")
        # print(f"  ** self.posx = {self.tb3_odom.posx} ")
        # print(f"  ** self.posy = {self.tb3_odom.posy} ")

        self.vel_controller.set_move_cmd(linear=linear, angular=0.0)
        self.vel_controller.publish()

        if self.front_min > (self.distance + 0.6):
            if time != 0 and meters==0.0:
                self.start_time = rospy.get_time()
                while rospy.get_time() - self.start_time < time:
                    continue

            if time == 0 and meters!=0.0:
                x0 = self.tb3_odom.posx
                y0 = self.tb3_odom.posy      
                displacement = 0.0
                # print(f"  ** displacement = {displacement} ")

                while displacement <=meters:
                    displacement = sqrt(pow(self.tb3_odom.posx-x0, 2) + pow(self.tb3_odom.posy-y0, 2))
        self.vel_controller.set_move_cmd(linear=0.0, angular=0.0)
        # self.vel_controller.publish()


    def action_server_launcher(self, goal: SearchGoal):

        self.startposx0 = self.tb3_odom.posx
        self.startposy0 = self.tb3_odom.posy

        self.distance = goal.approach_distance
        # Get the current robot odometry:
        self.posx0 = self.tb3_odom.posx
        self.posy0 = self.tb3_odom.posy
        # self.posz0 = self.tb3_odom.posz        

        # self.search_start_time = rospy.get_time()

        while not self.ctrl_c:
            
            self.set_the_color()

            success = True
            if goal.fwd_velocity <= 0 or goal.fwd_velocity > 10:
                print("Invalid velocity. ")
                success = False
            if goal.approach_distance <= 0:
                print("Invalid approach distance")
                success = False
            elif goal.approach_distance > 3.5:
                print("Invalid approach distance")
                success = False

            if not success:
                self.actionserver.set_aborted()
                return


            self.setzone_location()            

            # print(f" @@@@@@@@@ STARTING self.posx0 =  {self.posx0} self.posy0 {self.posy0} self.tb3_odom.posz = {self.tb3_odom.posz}")

            if self.firstleft == False:
                print(f" ### FIRST TAKE LEFT")   
                # print(f" ### before self.front_min = {self.front_min} ")                 

                self.turn_left()          
                # print(f" ### after self.front_min = {self.front_min} ")                 

                self.firstleft = True  

                self.vel_controller.set_move_cmd(0, 0)
                self.vel_controller.publish()    

            self.continue_search()
 
            # if self.zone1 == True and self.zone2 == True and self.zone3 == True and self.zone4 == True:
                # print(f" ### ALL ZONES DONE ")                 

            self.feedback.current_distance_travelled = sqrt(pow(self.posx0 - self.tb3_odom.posx, 2) + pow(self.posy0 - self.tb3_odom.posy, 2))
            self.actionserver.publish_feedback(self.feedback)

            
if __name__ == '__main__':
    rospy.init_node("search_action_server")
    SearchActionServer()
    rospy.spin()