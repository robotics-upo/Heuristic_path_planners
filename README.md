# 3D Heuristic Path Planners

This repo contains a series of Path Planning heuristic algorithms such as A*, Theta* and Lazy Theta*. The purpose of this repository is to provide som classes with a pure C++ implementation with the minimum dependencies, but at the same time, to provide ROS Integration to easily run these algorithms inside a ROS Network through some ROS Nodes.

## Dependencies

To compile everything you need a ROS Noetic installation in Ubuntu 20.04. If you want to use only the algorithm libraries you need a compiler with C++17 support (Gcc 7 or newer). 

## Running it

To easily run the algorithms, some launch files are provided for ROS Nodes. 

### A*

```bash
roslaunch heuristic_planners astar.launch
```

It will open an RViz window an load the default map *mbzirc_challenge3* in the folder ```resources/maps```. First time you open a map the Grid3 class will compute the gridmap so you will need to wait a little bit before starting.

To request path you need to call the service ```/astar_ros_node/request_path``` filling the start and goal coordinates in meters(m). 


## TODOs

- [ ] Allow using ```std::experimental``` for ```std::any``` and other C++17 functions to allow compiling it with GCC 5/6. 
- [ ] Refine save data integration
- [ ] Add Theta* class
- [ ] Add Lazy Theta* class
- [ ] Add bash/python scripts to generate data from a set of parameters
- [ ] Clean grid3d.hpp class