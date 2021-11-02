#!/usr/bin/env python3

from os import listdir, getpid, system
from os.path import isfile, join
import argparse
import numpy as np
import matplotlib.pyplot as plt
import random

import rospy
import roslaunch
import rospkg
import rosparam

from heuristic_planners.srv import *
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

def generate_centered_rnd_coords(center, margins):
    coords = [float(0)] * 3
    
    coords[0] = float( float(center[0]) - float(margins[0]) + random.randint(0, (int(margins[0]) + int(margins[1])) * 100 ) / 100)
    coords[1] = float( float(center[1]) - float(margins[2]) + random.randint(0, (int(margins[2]) + int(margins[3])) * 100 ) / 100)
    coords[2] = float( float(center[2]) - float(margins[4]) + random.randint(0, (int(margins[4]) + int(margins[5])) * 100 ) / 100)

    return coords

pid = getpid()
system("sudo renice -20 -p " + str(pid))

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

rospack = rospkg.RosPack()
launch_path = str(rospack.get_path('heuristic_planners')) + str("/launch/")
parser = argparse.ArgumentParser()

launch_list = [f for f in listdir(launch_path) if isfile(join(launch_path, f))]

parser.add_argument("--launch",     help="name of the launch file",
                    nargs='+', type=str, required=True,
                    choices=launch_list)
parser.add_argument("--algorithm",  help="name of the algorithm",
                    nargs='+', type=str, required=True,
                    choices=["astar", "costastar", "thetastar", "lazythetastar", "costlazythetastar"])
parser.add_argument("--start-coords", help="start coordinates (x,y,z). Set z to 0 when testing with 2D",
                    nargs=3,   required=True)
parser.add_argument("--goal-coords", help="goal coordinates (x,y,z). Set z to 0 when testing with 2D",
                    nargs=3,   required=True)
parser.add_argument("--random-margins", help="(min x, max x, min y, max y, min z, max x) to generate start and goals coordinates",
                    nargs=6, required=True)
parser.add_argument("--iterations", help="Number of iterations to evaluate",
                    nargs=1,   default=[20])
args = parser.parse_args()


cli_args = [launch_path + args.launch[0],'output:=screen']
roslaunch_args = cli_args[1:]
roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file,  is_core=True)

launch.start()

rospy.init_node('planners_speed_test_node', anonymous=True)

set_algorithm_request = SetAlgorithmRequest()
set_algorithm_request.algorithm.data = str(args.algorithm[0])

path_request = GetPathRequest()

path_request.start = Point(float(args.start_coords[0]), 
                           float(args.start_coords[1]), 
                           float(args.start_coords[2]))

path_request.goal = Point(float(args.goal_coords[0]),  
                          float(args.goal_coords[1]),  
                          float(args.goal_coords[2]))
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

rospy.sleep(2)

rospy.wait_for_service('/planner_ros_node/set_algorithm')
set_algorithm = rospy.ServiceProxy('/planner_ros_node/set_algorithm', SetAlgorithm)
set_algorithm.call(set_algorithm_request)

rospy.sleep(20)
success_iterations = 0
while success_iterations < int(args.iterations[0]):
    try:
        rand_start = generate_centered_rnd_coords(args.start_coords, args.random_margins)
        rand_goal  = generate_centered_rnd_coords(args.goal_coords, args.random_margins)
        path_request.start = Point(float(rand_start[0]), 
                                   float(rand_start[1]), 
                                   float(rand_start[2]))

        path_request.goal = Point(float(rand_goal[0]),  
                                  float(rand_goal[1]),  
                                  float(rand_goal[2]))

        print("Running path from " + str(rand_start) + " to " + str(rand_goal))          
        # End of options
        rospy.wait_for_service('/planner_ros_node/request_path')
        get_path = rospy.ServiceProxy(
            '/planner_ros_node/request_path', GetPath)

        resp = get_path.call(path_request)
        text_marker.text = "\nTime spent: " + str(resp.time_spent.data) + " ms"
        markerPub.publish(text_marker)
        rospy.sleep(2)
        
    except rospy.ServiceException as e:
        print("Service call failed: %s" %e)

launch.shutdown()
