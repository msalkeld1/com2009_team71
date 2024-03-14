#!/usr/bin/env python3

import rospy
import actionlib

from tuos_msgs.msg import SearchAction, SearchFeedback, SearchResult, SearchGoal

from math import sqrt, pow

class SearchActionServer():
    feedback = SearchFeedback()
    result = SearchResult()

    def __init__(self):
        rospy.init_node("search_action_server")

        self.actionserver = actionlib.SimpleActionServer("/bot_search",SearchAction, self.action_server_launcher, auto_start=False )
        self.actionserver.start()

        rospy.loginfo("The 'Search Action Server' is active...")

    def action_server_launcher(self, goal: SearchGoal):
        rate = rospy.Rate(10)

        success = True
        vel = goal.fwd_velocity
        dist = goal.approach_distance
        if vel > 0.26 or vel < 0:
            print("Invalid velocity!")
            success = False

        if dist < 0.2:
            success = False
            print("Invalid distance!")

        if not success:
            self.result.total_distance_travelled = -1.0
            self.result.closest_object_angle = -1.0
            self.result.closest_object_distance = -1.0
            self.actionserver.set_aborted(self.result)
            return
        
        print(f"Search goal received: fwd_vel = {vel:.2f} m/s, approach_distance = {dist:.2f} m.")

        #TODO: implement moving and obstacle avoidance


        if success:
            rospy.loginfo("approach completed successfully.")
            #TODO: stop robot
            self.actionserver.set_succeeded(self.result)

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    node = SearchActionServer()
    node.main()