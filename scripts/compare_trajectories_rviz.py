#!/usr/bin/env python3

## Import libraries
from os import listdir
from os.path import isfile, join
import os
import argparse
import numpy as np
import rospy
import roslaunch
import rospkg
import rosparam

## Import ROS Messages 
from heuristic_planners.srv import *
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class colors: # You may need to change color settings
    RED = '\033[31m'
    ENDC = '\033[m'
    GREEN = '\033[32m'
    YELLOW = '\033[33m'
    BLUE = '\033[34m'

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

parser = argparse.ArgumentParser()

## Get availables launch and map lists
rospack = rospkg.RosPack()
launch_path = str(rospack.get_path('heuristic_planners') )+ str("/launch/")
map2dpath = str(rospack.get_path('heuristic_planners') )+ str("/resources/2dmaps")
map3dpath = str(rospack.get_path('heuristic_planners') )+ str("/resources/3dmaps")

launch_list=[]
maps_list=[]

## Fill launch list
for file in listdir(launch_path):
    file_path = join(launch_path, file)
    if isfile(file_path):
        file_extension  = os.path.splitext(file_path)[1]
        if file_extension.lower() == ".launch":
            launch_list.append(file)

## Fill 2D Map list
for file in listdir(map2dpath):
    file_path = join(map2dpath, file)
    if isfile(file_path):
        file_extension  = os.path.splitext(file_path)[1]
        if file_extension.lower() == ".pgm":
            maps_list.append(file)

## Fill 3D Map list
for file in listdir(map3dpath):
    file_path = join(map3dpath, file)
    if isfile(file_path):
        file_extension  = os.path.splitext(file_path)[1]
        if file_extension.lower() == ".bt":
            maps_list.append(file)


print( colors.YELLOW + "Available Launch list: " + str(launch_list) + colors.ENDC)
print( colors.YELLOW + "Available 2D Map list: " + str(maps_list)   + colors.ENDC)

## ARGUMENT PARSING

parser.add_argument("--launch",     help="name of the launch file",     
                    nargs='+', type=str, required=True,
                    choices=launch_list)
parser.add_argument("--algorithm",  help="name of the algorithm",  
                    nargs='*', type=str, required=True, default=["astar", "costastar", "astarsafetycost"], 
                    choices=["astar", "costastar", "astarsafetycost", "thetastar", "costhetastar", "thetastarsafetycost", "lazythetastar", "costlazythetastar", "lazythetastarsafetycost"])

parser.add_argument("--map-name",     help="name of the map to use. This map should be under the 3d/2d maps folder",     
                    nargs='+', type=str, required=True,
                    choices=maps_list)

parser.add_argument("--start-coords", help="start coordinates (x,y,z). Set z to 0 when testing with 2D",         
                    nargs=3,   required=True)
parser.add_argument("--goal-coords", help="goal coordinates (x,y,z). Set z to 0 when testing with 2D",           
                    nargs=3,   required=True)

parser.add_argument("--cost-value", help="cost to evaluate",  
                    nargs=1,   default=[1])
parser.add_argument("--lof-value", help="Line of sight to evaluate.",  
                    nargs=1,   default=[1])

args = parser.parse_args()

print(colors.GREEN + "\nUsing the following algorithms: " +        colors.RED + str(args.algorithm) + colors.ENDC)
print(colors.GREEN + "Using the following map: " +                 colors.RED + str(args.map_name)           + colors.ENDC)
print(colors.GREEN + "Using the following cost value: " +          colors.RED + str(args.cost_value)           + colors.ENDC)
print(colors.GREEN + "Using the following line of sight value: " + colors.RED + str(args.lof_value)           + colors.ENDC)

### END OF ARGUMENT PARSING

# LAUNCH SELECTED launch file

cli_args = [launch_path + args.launch[0],'map_name:='+args.map_name[0].split('.')[0], 'overlay_markers:=true']
roslaunch_args = cli_args[1:]
roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file,  is_core=True)

launch.start()

rospy.init_node('planners_test_node', anonymous=True)

set_algorithm_request = SetAlgorithmRequest()
path_request = GetPathRequest()


path_request.start = Point(float(args.start_coords[0]), float(args.start_coords[1]), float(args.start_coords[2]))
path_request.goal  = Point(float(args.goal_coords[0]),  float(args.goal_coords[1]),  float(args.goal_coords[2]))

set_algorithm_request.algorithm.data = str(args.algorithm[0])

## End of options

markerPub = rospy.Publisher('test_text_marker', Marker, queue_size=1)
text_marker = Marker()
text_marker.header.frame_id = "map"
text_marker.header.stamp    = rospy.get_rostime()
text_marker.ns = "planners"
text_marker.id = 0
text_marker.type = 9 # text
text_marker.action = 0
text_marker.pose.position.x = float(args.start_coords[0])
text_marker.pose.position.y = float(args.start_coords[1])
text_marker.pose.position.z = float(args.start_coords[2])
text_marker.pose.orientation.x = 0
text_marker.pose.orientation.y = 0
text_marker.pose.orientation.z = 0
text_marker.pose.orientation.w = 1.0
text_marker.color.a = 1.0
text_marker.scale.z = 1 # text size
text_marker.text = ""

rospy.sleep(10)
## FOR EACH ALGORITHM
for algorithm in args.algorithm:

    set_algorithm_request.algorithm.data = str(algorithm)

    print(colors.GREEN + "Testing algorithm: " + str(algorithm) + " with line of sight " + str(args.lof_value[0]) + " with cost " + str(args.cost_value[0]) + colors.ENDC)
    
    try:
        rosparam.set_param_raw("/planner_ros_node/cost_weight", float(args.cost_value[0]), False)
        rosparam.set_param_raw("/planner_ros_node/max_line_of_sight_distance", float(args.lof_value[0]), False)
        rospy.sleep(1)
        rospy.wait_for_service('/planner_ros_node/set_algorithm')
        set_algorithm = rospy.ServiceProxy('/planner_ros_node/set_algorithm', SetAlgorithm)
        set_algorithm.call(set_algorithm_request)
        rospy.sleep(2)
        rospy.wait_for_service('/planner_ros_node/request_path')
        get_path = rospy.ServiceProxy('/planner_ros_node/request_path', GetPath)
        resp = get_path.call(path_request)
        text_marker.text = "Algorithm: "            + str(algorithm) + \
                         "\nCost "                  + str(args.cost_value[0]) + \
                         "\nLoS "                   + str(args.lof_value[0]) + \
                         "\nTime spent: "           + str(resp.time_spent.data) + " ms" + \
                         "\nExplored nodes: "       + str(resp.explored_nodes.data) + \
                         "\nLine of sight checks: " + str(resp.line_of_sight_checks.data) + \
                         "\nPath Length: "          + str(round(resp.path_length.data,3)) + " m"
        markerPub.publish(text_marker)

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

input("Press Enter to close the script")
launch.shutdown()