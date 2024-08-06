#!/usr/bin/env python3

import argparse
import rospy
from heuristic_planners.srv import *
from geometry_msgs.msg import Point

parser = argparse.ArgumentParser()

parser.add_argument("--start", help="start coordinates (x,y,z). Set z to 0 when testing with 2D",
                    nargs=3,   required=True)
parser.add_argument("--goal", help="goal coordinates (x,y,z). Set z to 0 when testing with 2D",
                    nargs=3,   required=True)

parser.add_argument("--tries", help="Number of tries in each call",  
                    nargs=1,   default=[1])
parser.add_argument("--heuristic", help="Heuristic to use",  
                    choices=["euclidean", "euclidean_optimized", "manhattan", "octogonal", "dijkstra"], default='')

# args = parser.parse_args()
args, unknown = parser.parse_known_args()

rospy.init_node('planner_service_caller', anonymous=True)

path_request = GetPathRequest()

path_request.start = Point(float(args.start[0]), 
                           float(args.start[1]), 
                           float(args.start[2]))

path_request.goal = Point(float(args.goal[0]),  
                          float(args.goal[1]),  
                          float(args.goal[2]))

path_request.tries.data = int(args.tries[0])
path_request.heuristic.data = str(args.heuristic)
# End of options

rospy.sleep(5)

rospy.wait_for_service('/planner_ros_node/request_path')
get_path = rospy.ServiceProxy(
            '/planner_ros_node/request_path', GetPath)

resp = get_path.call(path_request)
