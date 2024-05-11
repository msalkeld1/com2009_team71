import roslaunch
import rospy

map_name = "task4_map"

rospy.init_node("map_saving_script", anonymous=True)
rate = rospy.Rate(0.5)

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
rate.sleep()