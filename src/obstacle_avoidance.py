import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidanceNode:
    def __init__(self):
        rospy.init_node('obstacle_avoidance_node', anonymous=True)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/lidar_topic', LaserScan, self.lidar_callback)
        self.obstacle_threshold = 0.5  # Threshold distance to consider an obstacle
        self.max_linear_speed = 0.2    # Maximum linear speed of the robot
        self.min_linear_speed = 0.05   # Minimum linear speed to avoid stopping
        self.angular_scale = 0.5       # Scaling factor for angular velocity

    def lidar_callback(self, msg):
        # Process LIDAR data
        obstacle_detected = False
        min_distance = min(msg.ranges)

        if min_distance < self.obstacle_threshold:
            obstacle_detected = True

        # Generate control commands
        twist_msg = Twist()
        if obstacle_detected:
            twist_msg.linear.x = max(min_distance * self.angular_scale, self.min_linear_speed)
            twist_msg.angular.z = 0.5  # Turn away from the obstacle
        else:
            twist_msg.linear.x = self.max_linear_speed
            twist_msg.angular.z = 0.0  # Continue straight

        # Publish control commands
        self.publisher.publish(twist_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = ObstacleAvoidanceNode()
    node.run()
