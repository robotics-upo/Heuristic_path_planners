#!/usr/bin/env python3

import numpy as np
import argparse
import random
import roslaunch
import rospkg

import time

parser = argparse.ArgumentParser()

parser.add_argument("--name",     help="Name of the file",     
                    nargs='+', type=str, required=True)

parser.add_argument("--size", help="X Y Z size (integer) in pixels of the resulting image",         
                    nargs=3,required=True)

parser.add_argument("--obstacles", help="Number of obstacles",         
                    nargs=1,required=True)

parser.add_argument("--resolution", help="Number of obstacles",         
                    nargs=1,required=True)

args = parser.parse_args()
filename = str(args.name[0])
## Create white image of w x h size in pixels
w = float(args.size[0])
h = float(args.size[1])
d = float(args.size[2])

if w < 0 or h < 0 or d < 0:
    print("Error size should be > 0")
    exit

total_obstacles = int(args.obstacles[0])
resolution = float(args.resolution[0])
if total_obstacles <= 0:
    print("Error obstacle number should be > 0")
    exit

text_file = open( filename + ".pcd", "w")
text_file.write("# .PCD v.7 - Point Cloud Data file format\n")
text_file.write("VERSION .7\n")
text_file.write("FIELDS x y z\n") 
text_file.write("SIZE 4 4 4\n")
text_file.write("TYPE F F F\n")
text_file.write("COUNT 1 1 1\n")
text_file.write("WIDTH %s\n" % str(total_obstacles))
text_file.write("HEIGHT 1\n")
text_file.write("VIEWPOINT 0 0 0 1 0 0 0\n")
text_file.write("POINTS %s\n" % str(total_obstacles))
text_file.write("DATA ascii\n")

# Obstacles
valid_obstacles = 0
## We will also skip obstacles in the upper left border and the down right border to be always sure that start/goal points will be free
safe_area_size = 5 # In resolution units

while valid_obstacles < total_obstacles:
    # coords = [random.uniform(0, w), random.uniform(0, h), random.uniform(0, d)]
    coords = [random.randint(0, (w/resolution)-1 ), random.randint(0, (h/resolution)-1), random.randint(0, (d/resolution)-1 )]

    if coords[0] < safe_area_size and coords[1] < safe_area_size and coords[2] < safe_area_size:
        continue

    if coords[0] > (w/resolution-safe_area_size) and coords[1] > (h/resolution-safe_area_size) and coords[2] > ( d/resolution - safe_area_size):
        continue

    data_str = str(str( round(coords[0]*resolution, 3) ) + ", " + str( round(coords[1]*resolution, 3) ) + ", " + str( round(coords[2]*resolution, 3) ))
    
    text_file.write("%s\n" % data_str)
    valid_obstacles += 1


text_file.close()  

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
rospack = rospkg.RosPack()
launch_path = str(rospack.get_path('heuristic_planners') )+ str("/launch/utils/create_octomap.launch")

cli_args = [launch_path ,'pcd_file:='+ filename, 'output_name:='+filename]
roslaunch_args = cli_args[1:]
roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file,  is_core=True)

launch.start()

time.sleep(10)