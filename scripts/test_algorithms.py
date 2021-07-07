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
launch_path = str(rospack.get_path('heuristic_planners') )+ str("/launch/")
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
parser.add_argument("--cost-range", help="cost range to evaluate. (min, max, step). To test only one value set COST COST+1 1",  
                    nargs=3,   default=[100, 150, 1])
parser.add_argument("--lof-value", help="Line of sight range to evaluate. (min, max, step). To test only one value set LOF_VALUE LOF_VALUE+1 1",  
                    nargs=3,   default=[1, 2, 1])

args = parser.parse_args()

launch = roslaunch.parent.ROSLaunchParent(uuid, [ launch_path + args.launch[0] ], is_core=True)
launch.start()

rospy.init_node('planners_test_node', anonymous=True)

set_algorithm_request = SetAlgorithmRequest()
path_request = GetPathRequest()

# Test Options: Goal, Start, Algorithm type and range of costs
costs          = np.arange(float(args.cost_range[0]), float(args.cost_range[1]), float(args.cost_range[2]))
line_of_sights = np.arange(float(args.lof_value[0]), float(args.lof_value[1]), float(args.lof_value[2]))

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
text_marker.pose.orientation.x = 0
text_marker.pose.orientation.y = 0
text_marker.pose.orientation.z = 0
text_marker.pose.orientation.w = 1.0
text_marker.color.a = 1.0
text_marker.scale.z = 0.5 # text size
text_marker.text = ""

ev_cost, explored_nodes, durations, path_lens, sight_checks = [], [], [], [], []


rospy.sleep(10)

for lof in line_of_sights:
    
    fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, sharex=True)
    ax1.set_ylabel("Explored nodes")
    ax2.set_ylabel("Time spent")
    ax3.set_ylabel("Path Lenght")
    ax4.set_ylabel("Sight Checks")
    ax1.set_xlim(int(args.cost_range[0]), int(args.cost_range[1]))
    ax2.set_xlim(int(args.cost_range[0]), int(args.cost_range[1]))
    ax3.set_xlim(int(args.cost_range[0]), int(args.cost_range[1]))
    ax4.set_xlim(int(args.cost_range[0]), int(args.cost_range[1]))
    fig.suptitle('Resulting data')
    
    for cost in costs:
        print("Testing cost "+ str(cost))
        print("Testing Line of sight:  "+ str(lof))
        try:
            rosparam.set_param_raw("/planner_ros_node/cost_weight", float(cost), True)
            rosparam.set_param_raw("/planner_ros_node/max_line_of_sight_distance", float(lof), True)

            text_marker.text = "Cost " + str(round(float(cost),3)) + "\nLine of sight: " + str(round(float(lof),3))
            markerPub.publish(text_marker)
            rospy.sleep(1)

            rospy.wait_for_service('/planner_ros_node/set_algorithm')
            set_algorithm = rospy.ServiceProxy('/planner_ros_node/set_algorithm', SetAlgorithm)
            set_algorithm.call(set_algorithm_request)

            rospy.sleep(2)
            rospy.wait_for_service('/planner_ros_node/request_path')
            get_path = rospy.ServiceProxy('/planner_ros_node/request_path', GetPath)
            resp = get_path.call(path_request)

            text_marker.text = "Cost " + str(round(cost,3)) + "\nTime spent: " + str(resp.time_spent.data) + " ms" + "\nExplored nodes: " + str(resp.explored_nodes.data)
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
            fig.show()
            plt.pause(0.05)

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            
    fig_name = str(args.algorithm[0])+'_lof_'+str(round(float(lof)))+'_cost_range_' + str(round(float(costs[0]),3)) + '_' + str(round(float(costs[-1]),3))    
    plt.savefig(fig_name+".png")
    plt.cla()
    plt.clf()

launch.shutdown()