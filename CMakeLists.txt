cmake_minimum_required(VERSION 2.8.3)
project(heuristic_planners)

add_compile_options(-std=c++17 -Wall -Wextra -pedantic)
add_definitions(-DROS)
# add_definitions(-DPUB_EXPLORED_NODES)
option(BUILD_DOC "Build documentation" ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  message_generation
  roscpp
  pcl_conversions
  pcl_ros
  octomap_ros
  costmap_2d
)

# check if Doxygen is installed
find_package(Doxygen)
if (DOXYGEN_FOUND)
    # set input and output files
    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    # request to configure the file
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
    message("Doxygen build started")

    # note the option ALL which allows to build the docs together with the application
    add_custom_target( doc_doxygen ALL
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM )
else (DOXYGEN_FOUND)
  message("Doxygen need to be installed to generate the doxygen documentation")
endif (DOXYGEN_FOUND)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  CellCostValue.srv
  GetPath.srv
  SetAlgorithm.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs std_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES PathGenerator AStarGenerator ThetaStarGenerator LazyThetaStarGenerator
 CATKIN_DEPENDS std_msgs visualization_msgs geometry_msgs nav_msgs roscpp message_runtime costmap_2d
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

list(APPEND ${PROJECT_NAME}_UTILS_SOURCES src/utils/geometry_utils.cpp
                                          src/utils/heuristic.cpp
                                          src/utils/LineOfSight.cpp
                                          src/utils/utils.cpp       
                                          src/utils/ros/ROSInterfaces.cpp
                                          )
add_library(PathGenerator          src/Planners/PathGenerator.cpp 
                                   ${${PROJECT_NAME}_UTILS_SOURCES})

add_library(AStarGenerator         src/Planners/AStarGenerator.cpp 
                                   src/Planners/PathGenerator.cpp
                                   ${${PROJECT_NAME}_UTILS_SOURCES})
                                   
add_library(ThetaStarGenerator     src/Planners/ThetaStarGenerator.cpp 
                                   src/Planners/AStarGenerator.cpp 
                                   src/Planners/PathGenerator.cpp
                                   ${${PROJECT_NAME}_UTILS_SOURCES})

add_library(LazyThetaStarGenerator src/Planners/LazyThetaStarGenerator.cpp 
                                   src/Planners/ThetaStarGenerator.cpp 
                                   src/Planners/AStarGenerator.cpp 
                                   src/Planners/PathGenerator.cpp
                                   ${${PROJECT_NAME}_UTILS_SOURCES})

list(APPEND ${PROJECT_NAME}_LIBRARIES PathGenerator AStarGenerator ThetaStarGenerator LazyThetaStarGenerator)
target_link_libraries(${${PROJECT_NAME}_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies( ${${PROJECT_NAME}_LIBRARIES} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(planner_ros_node src/ROS/planner_ros_node.cpp )
add_dependencies(planner_ros_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_LIBRARIES})
target_link_libraries(planner_ros_node ${catkin_LIBRARIES} ${${PROJECT_NAME}_LIBRARIES})

list(APPEND ${PROJECT_NAME}_TARGETS ${${PROJECT_NAME}_LIBRARIES} planner_ros_node)
#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${${PROJECT_NAME}_TARGETS}
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

## Mark other files for installation (e.g. launch and bag files, etc.)

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY rviz/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz)
install(DIRECTORY resources/ 
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resources  
        PATTERN ".gridm" EXCLUDE)

