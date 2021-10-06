#!/usr/bin/env python3

## Import libraries
from os import listdir
from os.path import isfile, join
import os
import argparse
import numpy as np
import matplotlib.pyplot as plt
import rospy
import roslaunch
import rospkg
import rosparam

## Import ROS Messages 
from heuristic_planners.srv import *
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


def get_file_list(path : str, extension : str, result_list : list):
    ## Fill launch list
    for file in listdir(path):
        file_path = join(path, file)
        if isfile(file_path):
            file_extension  = os.path.splitext(file_path)[1]
            if file_extension.lower() == extension:
                result_list.append(file)

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

get_file_list(launch_path, ".launch", launch_list)
get_file_list(map2dpath, ".pgm", maps_list)
get_file_list(map3dpath, ".bt", maps_list)

print( colors.YELLOW + "Available Launch list: " + str(launch_list) + colors.ENDC)
print( colors.YELLOW + "Available Maps list: " + str(maps_list)   + colors.ENDC)

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

parser.add_argument("--cost-range", help="cost range to evaluate. (min, max, step). To test only one value set COST COST+1 1",  
                    nargs=3,   default=[1, 8, 1])
parser.add_argument("--lof-value", help="Line of sight range to evaluate. (min, max, step). To test only one value set LOF_VALUE LOF_VALUE+1 1",  
                    nargs=3,   default=[1, 3, 1])

args = parser.parse_args()

print(colors.GREEN + "\nUsing the following algorithms: " +        colors.RED + str(args.algorithm) + colors.ENDC)
print(colors.GREEN + "Using the following map: " +                 colors.RED + str(args.map_name)           + colors.ENDC)
print(colors.GREEN + "Using the following cost range: " +          colors.RED + str(args.cost_range)           + colors.ENDC)
print(colors.GREEN + "Using the following line of sight range: " + colors.RED + str(args.lof_value)           + colors.ENDC)

### END OF ARGUMENT PARSING

# LAUNCH SELECTED launch file

cli_args = [launch_path + args.launch[0],'map_name:='+args.map_name[0].split('.')[0]]
roslaunch_args = cli_args[1:]
roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file,  is_core=True)

launch.start()

rospy.init_node('planners_test_node', anonymous=True)

path_request = GetPathRequest()

# Test Options: Goal, Start, Algorithm type and range of costs
costs          = np.arange(float(args.cost_range[0]), float(args.cost_range[1]), float(args.cost_range[2]))
line_of_sights = np.arange(float(args.lof_value[0]), float(args.lof_value[1]), float(args.lof_value[2]))

path_request.start = Point(float(args.start_coords[0]), float(args.start_coords[1]), float(args.start_coords[2]))
path_request.goal  = Point(float(args.goal_coords[0]),  float(args.goal_coords[1]),  float(args.goal_coords[2]))


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

    path_request.algorithm.data = str(algorithm)

    ## FOR EACH LINE OF SIGHT
    for lof in line_of_sights:
        ## FOR EACH COST
        ev_cost, explored_nodes, durations, path_lens, sight_checks = [], [], [], [], []
        fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, sharex=True)
        ax1.set_ylabel("Explored nodes")
        ax2.set_ylabel("Time spent")
        ax3.set_ylabel("Path Lenght")
        ax4.set_ylabel("Sight Checks")
        ax1.set_xlim(float(args.cost_range[0]), float(float(args.cost_range[1]) - float(args.cost_range[2])))
        ax2.set_xlim(float(args.cost_range[0]), float(float(args.cost_range[1]) - float(args.cost_range[2])))
        ax3.set_xlim(float(args.cost_range[0]), float(float(args.cost_range[1]) - float(args.cost_range[2])))
        ax4.set_xlim(float(args.cost_range[0]), float(float(args.cost_range[1]) - float(args.cost_range[2])))
        fig.suptitle('Resulting data for ' + str(algorithm) + ' with line of sight ' + str(lof))
        for cost in costs:

            print(colors.GREEN + "Testing algorithm: " + str(algorithm) + " with line of sight " + str(lof) + " with cost " + str(cost) + colors.ENDC)
            try:
                rosparam.set_param_raw("/planner_ros_node/cost_weight", float(cost), False)
                rosparam.set_param_raw("/planner_ros_node/max_line_of_sight_distance", float(lof), False)
                rospy.sleep(1)

                rospy.wait_for_service('/planner_ros_node/request_path')
                get_path = rospy.ServiceProxy('/planner_ros_node/request_path', GetPath)
                resp = get_path.call(path_request)

                text_marker.text = "Algorithm: "            + str(algorithm) +\
                                 "\nCost "                  + str(round(cost,3)) +\
                                 "\nLoS "                   + str(round(lof,3)) +\
                                 "\nTime spent: "           + str(resp.time_spent.data) + " ms" +\
                                 "\nExplored nodes: "       + str(resp.explored_nodes.data) +\
                                 "\nLine of sight checks: " + str(resp.line_of_sight_checks.data) +\
                                 "\nPath Length: "          + str(round(resp.path_length.data,3)) + " m"
                markerPub.publish(text_marker)

                ev_cost.append(cost)
                explored_nodes.append(resp.explored_nodes.data)
                durations.append(resp.time_spent.data)
                path_lens.append(resp.path_length.data)
                sight_checks.append(resp.line_of_sight_checks.data)

                ax1.plot(ev_cost, explored_nodes, color='b')
                ax2.plot(ev_cost, durations,      color='r')
                ax3.plot(ev_cost, path_lens,      color='g')
                ax4.plot(ev_cost, sight_checks,   color='b')

                fig.canvas.draw()
                plt.pause(0.05)

            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

        fig.show()
        fig_name = str(algorithm)+'_lof_'+str(round(float(lof)))+'_cost_range_' + str(round(float(costs[0]),3)) + '_' + str(round(float(costs[-1]),3))    
        plt.savefig(fig_name+".png")
        plt.close()
        # plt.cla()
        # plt.clf()

launch.shutdown()