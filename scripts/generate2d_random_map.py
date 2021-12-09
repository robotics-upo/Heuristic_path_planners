#!/usr/bin/env python3

import numpy as np
import argparse
import random
from PIL import Image

parser = argparse.ArgumentParser()

parser.add_argument("--name",     help="Name of the file",     
                    nargs='+', type=str, required=True)

parser.add_argument("--size", help="X Y size (integer) in pixels of the resulting image",         
                    nargs=2,required=True)

parser.add_argument("--obstacles", help="Obstacle density (float). 1 represent full obstacle map (black image)",         
                    nargs=1,required=True)


args = parser.parse_args()
filename = str(args.name[0])
## Create white image of w x h size in pixels
w = int(args.size[0])
h = int(args.size[1])
if w < 0 or h < 0:
    print("Error size should be > 0")
    exit


obstacles_fraction = float(args.obstacles[0])
if obstacles_fraction <= 0 or obstacles_fraction >= 1:
    print("Error obstacle rate should be in the range (0,1)")
    exit

total_pixels = w*h
total_obstacles = round(total_pixels*obstacles_fraction)

# White Image
data = np.zeros((h,w,3), dtype=np.uint8)
data[0:w, 0:h] = [255, 255, 255] 

# Obstacles
valid_obstacles = 0
## We will also skip obstacles in the upper left border and the down right border to be always sure that start/goal points will be free
safe_area_size = 10 # 10x10 pixels area

while valid_obstacles < total_obstacles:
    coords = [random.randint(0, w-1), random.randint(0, h-1)]
    if coords[0] < safe_area_size and coords[1] < safe_area_size:
        continue
    if coords[0] > (w-safe_area_size) and coords[1] > (h-safe_area_size):
        continue

    value = data[coords[0], coords[1]]

    while (value[0] == 0 and value[1] == 0 and value[2] == 0):
        coords = [random.randint(0, w-1), random.randint(0, h-1)]
        if coords[0] < safe_area_size and coords[1] < safe_area_size:
            continue
        if coords[0] > (w-safe_area_size) and coords[1] > (h-safe_area_size):
            continue

        value = data[coords[0], coords[1]]

    valid_obstacles += 1
    data[coords[0], coords[1]] = [0, 0, 0] 
        
img = Image.fromarray(data, 'RGB')
### Comparing algorithms in RViz
print("Entropy of resulting map: " + str(img.entropy()))
img.save(filename + '.pgm')