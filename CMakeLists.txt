cmake_minimum_required(VERSION 3.10.2)
project(heuristic_planners)

add_compile_options(-std=gnu++17 -Wall )

option(BUILD_DOC            "Build documentation   " ON)
option(BUILD_ROS_SUPPORT    "Build with ROS Support" ON)
option(OPTIMIZE_FLAG        "Enable Compiler Optimization for Runtime Performance" ON)
option(BUILD_DEBUG          "Build debug features  " OFF)
option(BUILD_COMPUTE_STATS  "Build Algorithms with statistics" ON)
option(BUILD_VOROCPP        "Build voro++ features  " OFF)

if(OPTIMIZE_FLAG)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
else()
  add_compile_options(-fsanitize=address,undefined -static-libasan -g3)
endif()

if(BUILD_COMPUTE_STATS)
  add_compile_definitions(COMPUTE_STATISTICS)
endif()

if(BUILD_DOC)
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
endif()
unset(BUILD_DOC CACHE)

find_package(Boost)

if(BUILD_ROS_SUPPORT)
  add_compile_definitions(ROS)
  ## Find catkin macros and libraries
  find_package(catkin REQUIRED COMPONENTS
    std_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    message_generation
    rospy
    roscpp
    pcl_conversions
    pcl_ros
    octomap_ros
    costmap_2d
    sensor_msgs
  )
  #Eigen is used to calculate metrics parameters
  find_package (Eigen3 REQUIRED NO_MODULE)
  find_package(OpenSSL REQUIRED)
  ####### TORCH DIR SHOULD BE SET TO LIBTORCH FOLDER ########
  set(Torch_DIR "/home/ros/libtorch/share/cmake/Torch")
  find_package(Torch REQUIRED)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
  if(BUILD_DEBUG)
    add_definitions(-DPUB_EXPLORED_NODES)
  endif()
  unset(BUILD_DEBUG CACHE)

################################################
## Declare ROS messages, services and actions ##
################################################
catkin_python_setup()

## Add message files
add_message_files(
  FILES
  Vec3i.msg
  CoordinateList.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetPath.srv
  SetAlgorithm.srv
  ShareWeights.srv
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
 LIBRARIES AlgorithmBase AStar AStarM1 AStarM2 AStarSIREN ThetaStar ThetaStarM1 ThetaStarM2 ThetaStarSIREN LazyThetaStar LazyThetaStarM1 LazyThetaStarM1Mod LazyThetaStarM2 LazyThetaStarSIREN
 # 
 CATKIN_DEPENDS std_msgs visualization_msgs geometry_msgs nav_msgs rospy roscpp message_runtime costmap_2d sensor_msgs pcl_ros pcl_conversions
#  DEPENDS system_lib
)
endif()
###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${TORCH_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/voro++-0.4.6
)
if( BUILD_VOROCPP )
  list(APPEND VORO_SOURCES  ${CMAKE_CURRENT_SOURCE_DIR}/include/voro++-0.4.6/src/voro++.cc
                            ${CMAKE_CURRENT_SOURCE_DIR}/include/voro++-0.4.6/src/cell.cc
                            ${CMAKE_CURRENT_SOURCE_DIR}/include/voro++-0.4.6/src/c_loops.cc
                            ${CMAKE_CURRENT_SOURCE_DIR}/include/voro++-0.4.6/src/common.cc
                            ${CMAKE_CURRENT_SOURCE_DIR}/include/voro++-0.4.6/src/container_prd.cc
                            ${CMAKE_CURRENT_SOURCE_DIR}/include/voro++-0.4.6/src/container.cc
                            ${CMAKE_CURRENT_SOURCE_DIR}/include/voro++-0.4.6/src/pre_container.cc
                            ${CMAKE_CURRENT_SOURCE_DIR}/include/voro++-0.4.6/src/unitcell.cc
                            ${CMAKE_CURRENT_SOURCE_DIR}/include/voro++-0.4.6/src/v_base.cc
                            ${CMAKE_CURRENT_SOURCE_DIR}/include/voro++-0.4.6/src/v_compute.cc
                            ${CMAKE_CURRENT_SOURCE_DIR}/include/voro++-0.4.6/src/wall.cc
  )
  add_library(Voro++ STATIC ${VORO_SOURCES})
  list(APPEND ${PROJECT_NAME}_LIBRARIES Voro++)
  list(APPEND ${PROJECT_NAME}_TARGETS voro++)
  add_executable(voro++ include/voro++-0.4.6/src/cmd_line.cc)
  target_link_libraries(voro++ Voro++)
endif()
##This is True by default but it's good to have it here as a reminder
set(BUILD_SHARED_LIBS TRUE)
list(APPEND ${PROJECT_NAME}_UTILS_SOURCES src/utils/geometry_utils.cpp
                                          src/utils/heuristic.cpp
                                          src/utils/LineOfSight.cpp
                                          src/utils/utils.cpp       
                                          src/utils/metrics.cpp
                                          src/utils/FCNet.cpp       
                                          )
if(BUILD_ROS_SUPPORT)
  list(APPEND ${PROJECT_NAME}_UTILS_SOURCES src/utils/ros/ROSInterfaces.cpp)
endif()

add_library(AlgorithmBase                   src/Planners/AlgorithmBase.cpp 
                                            ${${PROJECT_NAME}_UTILS_SOURCES})

add_library(AStar                  src/Planners/AStar.cpp
                                            src/Planners/AlgorithmBase.cpp 
                                            ${${PROJECT_NAME}_UTILS_SOURCES}
                                            )

add_library(AStarM1            src/Planners/AStarM1.cpp ) 
add_library(AStarM2            src/Planners/AStarM2.cpp )
add_library(AStarSIREN         src/Planners/AStarSIREN.cpp ) 
add_library(ThetaStar          src/Planners/ThetaStar.cpp )
add_library(ThetaStarM1        src/Planners/ThetaStarM1.cpp )
add_library(ThetaStarM2        src/Planners/ThetaStarM2.cpp )
add_library(ThetaStarSIREN     src/Planners/ThetaStarSIREN.cpp )
add_library(LazyThetaStar      src/Planners/LazyThetaStar.cpp )
add_library(LazyThetaStarM1    src/Planners/LazyThetaStarM1.cpp )
add_library(LazyThetaStarM1Mod src/Planners/LazyThetaStarM1Mod.cpp )
add_library(LazyThetaStarM2    src/Planners/LazyThetaStarM2.cpp )
add_library(LazyThetaStarSIREN src/Planners/LazyThetaStarSIREN.cpp )



list(APPEND ${PROJECT_NAME}_LIBRARIES AlgorithmBase AStar AStarM1 AStarM2 AStarSIREN ThetaStar ThetaStarM1 ThetaStarM2 ThetaStarSIREN LazyThetaStar LazyThetaStarM1 LazyThetaStarM1Mod LazyThetaStarM2 LazyThetaStarSIREN)  
target_link_libraries(${${PROJECT_NAME}_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TORCH_LIBRARIES})
add_dependencies( ${${PROJECT_NAME}_LIBRARIES} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
list(APPEND ${PROJECT_NAME}_TARGETS ${${PROJECT_NAME}_LIBRARIES})

if(BUILD_ROS_SUPPORT)
  add_executable(planner_ros_node src/ROS/planner_ros_node.cpp )
  ##set_property(TARGET planner_ros_node PROPERTY CXX_STANDARD 17)
  add_dependencies(planner_ros_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_LIBRARIES})
  target_link_libraries(planner_ros_node ${catkin_LIBRARIES} ${TORCH_LIBRARIES} ${${PROJECT_NAME}_LIBRARIES} -lstdc++fs Eigen3::Eigen -lcrypto -lssl)
  list(APPEND ${PROJECT_NAME}_TARGETS planner_ros_node)
  add_executable(local_planner_ros_node src/ROS/local_planner_ros_node.cpp )
  add_dependencies(local_planner_ros_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_LIBRARIES})
  target_link_libraries(local_planner_ros_node ${catkin_LIBRARIES} ${${PROJECT_NAME}_LIBRARIES} -lstdc++fs Eigen3::Eigen -lcrypto -lssl)
  list(APPEND ${PROJECT_NAME}_TARGETS local_planner_ros_node)
#############
  ## Install ##
#############


## Mark executables and/or libraries for installation
  install(TARGETS ${${PROJECT_NAME}_TARGETS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  
  catkin_install_python(PROGRAMS  scripts/test_algorithms.py
                                  scripts/test_algorithms_compare.py
                                  scripts/test_algorithms_performance.py
                                  scripts/test_algorithms_pseudo_random_paths.py
                                  scripts/compare_trajectories_rviz.py
                                  scripts/plan_caller.py
                                  scripts/generate2d_random_map.py
                                  setup.py
                                  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

## Mark cpp header files for installation
  install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  )

## Mark other files for installation (e.g. launch and bag files, etc.)

  install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
  install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
  install(DIRECTORY rviz/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz)
  install(DIRECTORY resources/ 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resources  
  PATTERN ".gridm" EXCLUDE)

else()
  
  install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include
    FILES_MATCHING PATTERN "*.hpp"
  )
  
  install(TARGETS ${${PROJECT_NAME}_TARGETS}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  )

endif()
unset(BUILD_ROS_SUPPORT CACHE)
