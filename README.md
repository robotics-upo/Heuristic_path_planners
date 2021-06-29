[![Build for Ubuntu 18.04 and ROS Melodic](https://github.com/RafaelRey/3D_heuristic_path_planners/actions/workflows/build_melodic.yml/badge.svg)](https://github.com/RafaelRey/3D_heuristic_path_planners/actions/workflows/build_melodic.yml)

[![Build for Ubuntu 20.04 and ROS noetic](https://github.com/RafaelRey/3D_heuristic_path_planners/actions/workflows/build_noetic.yml/badge.svg)](https://github.com/RafaelRey/3D_heuristic_path_planners/actions/workflows/build_noetic.yml)

# 3D Heuristic Path Planners

This repo contains a series of Path Planning heuristic algorithms such as A*, Theta* and Lazy Theta*. The purpose of this repository is to provide som classes with a pure C++ implementation with the minimum dependencies, but at the same time, to provide ROS Integration to easily run these algorithms inside a ROS Network through some ROS Nodes.

## Code Documentation

By default the CMake BUILD_DOC option is enabled so the doxygen documentation will be generated every time you run catkin_make. You can also read it [Online](https://codedocs.xyz/robotics-upo/3D_heuristic_path_planners)
## Dependencies

If you want to easily run it go to the [Releases](https://github.com/RafaelRey/3D_heuristic_path_planners/releases) and download and install the desired debian package with:

```bash
sudo apt install ./ros-$ROS_DISTRO-heuristic_planners....deb
```

To compile everything you need a ROS Melodic/Noetic installation in Ubuntu 18.04/20.04. If you want to use only the algorithm libraries you need a compiler with C++17 support (Gcc 7 or newer). 

The user can also choose to compile the class ```PathGenerator``` and related ones with ROS debug features such as publication of the inflated map with markers and publication of the explored nodes with markers for RViz. To enable it uncommented the line

```
add_definitions(-DROS)
```

in the CMakeLists.txt. To enable explored nodes visualization marker publication uncomment the following line:

```
add_definitions(-DPUB_EXPLORED_NODES)
```
But for "production" uses and real tests you should uncomment these definitions, specially the publishing explores nodes one because it increases a lot the computation time because of the marker messages sending. Debians files on the Release tab are compiled only with the ROS definition, not with the PUB_EXPLORED_NODES one.

To easily install the dependencies, download the source code onto your catkin workspace source folder, after that go to the catkin workspace root folder and run:

```bash
rosdep update && rosdep install --from-paths src/ -y -r
```

Make sure you have sourced your ```devel/setup.bash``` in case you compile it or ```/opt/ros/$ROS_DISTRO/setup.bash``` in case you installed it.

## Running the demo ROS Node 

The algorithms can be used for 2D and 3D. 2D is a special case of the 3D world size configuration. The world size Z must be equal to the resolution of the map, and the start/goals coordinates should have a 0 z component. The main internal difference for the algorithms is the number of directions that can be used to explore the space, that is 8 in 2D and 26 in 3D.

Also, the header ROSInterfaces.hpp include two interfaces to load PointClouds and OccupancyGrid as maps, for the 3D and 2D cases.

To easily run the algorithms, some launch files are provided for ROS Nodes. Also 2D maps and 3D maps are provided 

### 3D 

```bash
roslaunch heuristic_planners planner.launch algorithm_name:=<algorithm_name>
```

### Theta*

```bash
roslaunch heuristic_planners planner2d_example.launch algorithm_name:=<algorithm_name>
```

#### Parameters

You can pass some args to the launchs files such as:

- *map*: The .bt file path, by default the mbzirc challenge3
- *algorithm_name*: Can be "astar", "thetastar" or "lazythetastar" for the moment. At startup the node will echo the selected one to make you sure you are using the one you want. 
- *world_size_x*: world x size in meters 
- *world_size_y*: world y size in meters 
- *world_size_z*: world z size in meters
- *resolution*: Resolution of the map in meters 
- *inflate_map*: Boolean to inflate or not the map 
- *inflation_size*: The inflation size (It's performed as an isotropic inflation in x,y,z). By default is the same as the resolution because it's recommended as minimum inflation to fill empty blocks between cells and avoid bad paths crosing walls etc.
- *save_data*: Boolean to enable data saving
- *data_file_path*: Path of the data file, by default $HOME/planning.txt. It saves algorithm name, start and goal coordinates, explored nodes, time etc.


In the 3D case, it will open an RViz window an load the default map *mbzirc_challenge3* in the folder ```resources/3dmaps```. First time you open a map the Grid3 class will compute the gridmap so you will need to wait a little bit before starting.

To request path you need to call the service ```/planner_ros_node/request_path``` filling the start and goal coordinates in meters(m). 


## TODOs

- [ ] Allow using ```std::experimental``` for ```std::any``` and other C++17 functions to allow compiling it with GCC 5/6. 
- [x] Refine save data integration: Using std variant that reduces duplicated code.
- [x] Add Theta* class
- [x] Add Lazy Theta* class
- [ ] Add bash/python scripts to generate data from a set of parameters
- [ ] Clean grid3d.hpp class