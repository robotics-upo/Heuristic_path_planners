# 3D Heuristic Path Planners

This repo contains a series of Path Planning heuristic algorithms such as A*, Theta* and Lazy Theta*. The purpose of this repository is to provide som classes with a pure C++ implementation with the minimum dependencies, but at the same time, to provide ROS Integration to easily run these algorithms inside a ROS Network through some ROS Nodes.

## Dependencies

To compile everything you need a ROS Noetic installation in Ubuntu 20.04. If you want to use only the algorithm libraries you need a compiler with C++17 support (Gcc 7 or newer). 

The user can also choose to compile the class ```AStarGenerator``` and related ones with ROS debug features such as publication of the inflated map with markers and publication of the explored nodes with markers for RViz. To enable it uncommented the line

```
add_definitions(-DROS)
```

in the CMakeLists.txt. To enable explored nodes visualization marker publication uncomment the following line:

```
add_definitions(-DPUB_EXPLORED_NODES)
```


## Running it

To easily run the algorithms, some launch files are provided for ROS Nodes. 

### A*

```bash
roslaunch heuristic_planners astar.launch
```

#### Parameters

You can pass some args to the launch file such as:

- *map*: The .bt file path, by default the mbzirc challenge3
- *world_size_x*: world x size in meters 
- *world_size_y*: world y size in meters 
- *world_size_z*: world z size in meters
- *resolution*: Resolution of the map in meters 
- *inflate_map*: Boolean to inflate or not the map 
- *inflation_size*: The inflation size (It's performed as an isotropic inflation in x,y,z). By default is the same as the resolution because it's recommended as minimum inflation to fill empty blocks between cells and avoid bad paths crosing walls etc.
- *save_data*: Boolean to enable data saving
- *data_file_path*: Path of the data file, by default $HOME/planning.txt


It will open an RViz window an load the default map *mbzirc_challenge3* in the folder ```resources/maps```. First time you open a map the Grid3 class will compute the gridmap so you will need to wait a little bit before starting.

To request path you need to call the service ```/astar_ros_node/request_path``` filling the start and goal coordinates in meters(m). 


## TODOs

- [ ] Allow using ```std::experimental``` for ```std::any``` and other C++17 functions to allow compiling it with GCC 5/6. 
- [ ] Refine save data integration
- [ ] Add Theta* class
- [ ] Add Lazy Theta* class
- [ ] Add bash/python scripts to generate data from a set of parameters
- [ ] Clean grid3d.hpp class