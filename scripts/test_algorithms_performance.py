#!/usr/bin/env python3

from os import listdir
from os.path import isfile, join
import argparse
import numpy as np
import matplotlib.pyplot as plt

import rospy
import roslaunch
import rospkg
import rosparam

from heuristic_planners.srv import *
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

rospack = rospkg.RosPack()
launch_path = str(rospack.get_path('heuristic_planners')) + str("/launch/")
parser = argparse.ArgumentParser()

launch_list = [f for f in listdir(launch_path) if isfile(join(launch_path, f))]

parser.add_argument("--launch",     help="name of the launch file",
                    nargs='+', type=str, default=["planner.launch"],
                    choices=launch_list)
parser.add_argument("--algorithm",  help="name of the algorithm",
                    nargs='+', type=str, required=True,
                    choices=["astar", "costastar", "astarsafetycost", "thetastar", "costhetastar", "thetastarsafetycost", "lazythetastar", "costlazythetastar", "costlazythetastarmodified", "lazythetastarsafetycost"])
parser.add_argument("--start-coords", help="start coordinates (x,y,z). Set z to 0 when testing with 2D",
                    nargs=3,   required=True)
parser.add_argument("--goal-coords", help="goal coordinates (x,y,z). Set z to 0 when testing with 2D",
                    nargs=3,   required=True)
parser.add_argument("--iterations", help="Number of iterations to evaluate",
                    nargs=1,   default=[100])
parser.add_argument("--cost-weight", help="cost",  
                    nargs=1,   default=[1])

parser.add_argument("--tries", help="Number of tries in each call",  
                    nargs=1,   default=[1])
parser.add_argument("--heuristic", help="Heuristic to use",  
                    nargs=1,   default=[""], type=str,
                    choices=["euclidean", "euclidean_optimized", "manhattan", "octogonal", "dijkstra"])

args = parser.parse_args()


cli_args = [launch_path + args.launch[0],'cost_weight:='+args.cost_weight[0].split('.')[0],'output:=log']
roslaunch_args = cli_args[1:]
roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file,  is_core=True)

launch.start()

rospy.init_node('planners_speed_test_node', anonymous=True)

set_algorithm_request = SetAlgorithmRequest()
path_request = GetPathRequest()


path_request.start = Point(float(args.start_coords[0]), float(
    args.start_coords[1]), float(args.start_coords[2]))
path_request.goal = Point(float(args.goal_coords[0]),  float(
    args.goal_coords[1]),  float(args.goal_coords[2]))
set_algorithm_request.algorithm.data = str(args.algorithm[0])
path_request.tries.data = int(args.tries[0])
path_request.heuristic.data = args.heuristic[0]
# End of options

markerPub = rospy.Publisher('test_text_marker', Marker, queue_size=1)
text_marker = Marker()
text_marker.header.frame_id = "map"
text_marker.header.stamp = rospy.get_rostime()
text_marker.ns = "planners"
text_marker.id = 0
text_marker.type = 9  # text
text_marker.action = 0
text_marker.pose.orientation.x = 0
text_marker.pose.orientation.y = 0
text_marker.pose.orientation.z = 0
text_marker.pose.orientation.w = 1.0
text_marker.color.a = 1.0
text_marker.scale.z = 0.5  # text size
text_marker.text = ""

it, durations = [], []

fig, ax1 = plt.subplots(1, sharex=True)

ax1.set_ylabel("Time spent")
fig.suptitle('Resulting data')
ax1.set_xlim(0, int(args.iterations[0]))

rospy.sleep(2)

rospy.wait_for_service('/planner_ros_node/set_algorithm')
set_algorithm = rospy.ServiceProxy('/planner_ros_node/set_algorithm', SetAlgorithm)
set_algorithm.call(set_algorithm_request)

rospy.sleep(20)
total_time = 0
for iter in range(0,int(args.iterations[0])):
    try:

        rospy.wait_for_service('/planner_ros_node/request_path')
        get_path = rospy.ServiceProxy(
            '/planner_ros_node/request_path', GetPath)

        resp = get_path.call(path_request)
        text_marker.text = "\nTime spent: " + str(resp.time_spent.data) + " microsecs"
        total_time += resp.time_spent.data
        markerPub.publish(text_marker)
        
        it.append(iter)
        durations.append(resp.time_spent.data)
        plt.scatter(it, durations)
        # ax1.plot(it, durations,      color='r')
        fig.canvas.draw()
        fig.show()
        plt.pause(0.05)

    except rospy.ServiceException as e:
        print("Service call failed: %s" %e)

average=total_time/int(args.iterations[0])
print("Average time: " + str(average))
launch.shutdown()
