#!/usr/bin/env python3

import rospy
import roslaunch
import rospkg
import rosparam
from heuristic_planners.srv import *
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

rospy.init_node('cost_aware_astar_test_node', anonymous=True)

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
rospack = rospkg.RosPack()
path = rospack.get_path('heuristic_planners')
launch = roslaunch.parent.ROSLaunchParent(uuid, [ str(path) +"/launch/planner2d_example.launch"])
launch.start()

set_algorithm_request = SetAlgorithmRequest()
path_request = GetPathRequest()

# Test Options: Goal, Start, Algorithm type and range of costs
costs = [ x*0.1 for x in range(60,81,1)]
path_request.start = Point(1.5, 0.5 , 0.0)
path_request.goal = Point(1.0, 3.0 , 0.0)
set_algorithm_request.algorithm.data = str("costastar")
## End of options

markerPub = rospy.Publisher('test_text_marker', Marker, queue_size=1)
text_marker = Marker()
text_marker.header.frame_id = "map"
text_marker.header.stamp    = rospy.get_rostime()
text_marker.ns = "planners"
text_marker.id = 0
text_marker.type = 9 # text
text_marker.action = 0
text_marker.pose.orientation.x = 0
text_marker.pose.orientation.y = 0
text_marker.pose.orientation.z = 0
text_marker.pose.orientation.w = 1.0
text_marker.color.a = 1.0
text_marker.scale.z = 0.5 # text size
text_marker.text = ""

rospy.sleep(10)

for cost in costs:
    print("Testing cost "+ str(cost))
    try:
        rosparam.set_param_raw("/planner_ros_node/cost_weight", cost, True)
        text_marker.text = "Cost " + str(round(cost,3))
        markerPub.publish(text_marker)
        rospy.sleep(1)
        
        rospy.wait_for_service('/planner_ros_node/set_algorithm')
        set_algorithm = rospy.ServiceProxy('/planner_ros_node/set_algorithm', SetAlgorithm)
        set_algorithm.call(set_algorithm_request)

        rospy.sleep(2)
        rospy.wait_for_service('/planner_ros_node/request_path')
        get_path = rospy.ServiceProxy('/planner_ros_node/request_path', GetPath)
        get_path.call(path_request)
        
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

launch.shutdown()