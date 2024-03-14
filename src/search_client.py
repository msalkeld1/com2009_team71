#! /usr/bin/env python3

import rospy
import actionlib

from tuos_msgs.msg import SearchAction, SearchGoal, SearchFeedback, SearchResult

class SearchActionClient():
    goal = SearchGoal()
    msg_counter = 0

    def feedback_callback(self, feedback_data: SearchFeedback):
        self.distance = feedback_data.current_distance_travelled
        if self.msg_counter > 5:
            print(f"FEEDBACK: distance travelled = {self.distance:.3f} meters.")
            self.msg_counter = 0
        else:
            self.msg_counter += 1

    def __init__(self):
        self.distance = 0.0
        self.action_complete = False
        rospy.init_node("search_action_client")
        self.rate = rospy.Rate(1)

        self.client = actionlib.SimpleActionClient("/bot_search", SearchAction)
        self.client.wait_for_server()
        rospy.on_shutdown(self.shutdownhook)

    def shutdownhook(self):
        if not self.action_complete:
            rospy.logwarn("Received a shutdown request. Cancelling Goal...")
            self.client.cancel_goal()
            rospy.logwarn("Goal cancelled...")
        
        rospy.sleep(1)
        result: SearchResult = self.client.get_result()
        print("RESULT:")
        print(f"  * Action State = {self.client.get_state()}")
        print(f"  * total_distance_travelled = {result.total_distance_travelled:.3f} meters")
        print(f"  * closest_object_distance = {result.closest_object_distance:.3f} meters")
        print(f"  * closest_object_angle = {result.closest_object_angle:.1f} degrees.")

    def main(self):
        #TODO: implement goals 
        self.goal.approach_distance = 0.0
        self.goal.fwd_velocity = 0.0
        self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)

        while self.client.get_state() < 2:
            if self.distance > 2:
                print("STOP: distance exceeded 2 meters!!")
                break

            self.rate.sleep()

        self.action_complete = True if self.client.get_state() == 3 else False

if __name__  == '__main__':
    node = SearchActionClient()
    try:
        node.main()
    except rospy.ROSInterruptException:
        pass