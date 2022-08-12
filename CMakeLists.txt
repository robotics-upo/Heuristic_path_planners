cmake_minimum_required(VERSION 3.10.2)
project(heuristic_planners)

add_compile_options(-std=c++17 -Wall -Werror -Wextra -pedantic )

option(BUILD_DOC            "Build documentation   " OFF)
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

add_compile_definitions(ROS)
  # find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  ament_cmake_ros
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  builtin_interfaces
  pcl_conversions
  pcl_ros
  octomap_ros
  nav2_costmap_2d
)


foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

  #Eigen is used to calculate metrics parameters
  find_package (Eigen3 REQUIRED NO_MODULE)
  find_package(OpenSSL REQUIRED)
  if(BUILD_DEBUG)
    add_definitions(-DPUB_EXPLORED_NODES)
  endif()
  unset(BUILD_DEBUG CACHE)

################################################
## Declare ROS messages, services and actions ##
################################################
# catkin_python_setup()

## Generate services in the 'srv' folder
find_package(rosidl_default_generators REQUIRED)

set(SRVS_FILES
  "srv/GetPath.srv"
  "srv/SetAlgorithm.srv"
)

## Generate added messages and services with any dependencies listed here

rosidl_generate_interfaces(${PROJECT_NAME}
  ${SRVS_FILES} 
  DEPENDENCIES geometry_msgs std_msgs
)

# list(APPEND PROJECT_DEPENDENCIES ${PROJECT_NAME}_msgs)

##########
## Build ##
###########

include_directories(
  include
  include/Planners
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
                                          )
  list(APPEND ${PROJECT_NAME}_UTILS_SOURCES src/utils/ros/ROSInterfaces.cpp)

add_library(AlgorithmBase                   src/Planners/AlgorithmBase.cpp 
                                            ${${PROJECT_NAME}_UTILS_SOURCES})

add_library(AStar                  src/Planners/AStar.cpp
                                            src/Planners/AlgorithmBase.cpp 
                                            ${${PROJECT_NAME}_UTILS_SOURCES}
                                            )

add_library(AStarM1            src/Planners/AStarM1.cpp ) 
add_library(AStarM2            src/Planners/AStarM2.cpp ) 
add_library(ThetaStar          src/Planners/ThetaStar.cpp )
add_library(ThetaStarM1        src/Planners/ThetaStarM1.cpp )
add_library(ThetaStarM2        src/Planners/ThetaStarM2.cpp )
add_library(LazyThetaStar      src/Planners/LazyThetaStar.cpp )
add_library(LazyThetaStarM1    src/Planners/LazyThetaStarM1.cpp )
add_library(LazyThetaStarM1Mod src/Planners/LazyThetaStarM1Mod.cpp )
add_library(LazyThetaStarM2    src/Planners/LazyThetaStarM2.cpp )

list(APPEND ${PROJECT_NAME}_LIBRARIES AlgorithmBase AStar AStarM1 AStarM2 ThetaStar ThetaStarM1 ThetaStarM2 LazyThetaStar LazyThetaStarM1 LazyThetaStarM1Mod LazyThetaStarM2)  

foreach(LIB ${${PROJECT_NAME}_LIBRARIES})
  message("Building ${LIB}")
  target_include_directories(${LIB}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${LIB} ${PROJECT_DEPENDENCIES})
  ament_export_targets(export_${LIB} HAS_LIBRARY_TARGET)
  ament_export_dependencies(${LIB} ${PROJECT_DEPENDENCIES})
  target_link_libraries(${LIB}  ${Boost_LIBRARIES})
endforeach()

#add_executable(local_planner_node src/ROS/local_planner_node.cpp src/Planners/LocalPlanner.cpp include/Planners/LocalPlanner.hpp)
#add_executable(local_planner_node src/ROS/local_planner_node.cpp )
#ament_target_dependencies(local_planner_node ${PROJECT_DEPENDENCIES})

add_executable(planner_ros_node src/ROS/planner_ros_node.cpp )
ament_target_dependencies(planner_ros_node ${PROJECT_DEPENDENCIES})
# rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
# target_link_libraries(planner_ros_node "${cpp_typesupport_target}")
rosidl_target_interfaces(planner_ros_node
      ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(planner_ros_node ${${PROJECT_NAME}_LIBRARIES} -lstdc++fs Eigen3::Eigen -lcrypto -lssl)

#rosidl_target_interfaces(local_planner_node
#      ${PROJECT_NAME} "rosidl_typesupport_cpp")
#target_link_libraries(local_planner_node ${${PROJECT_NAME}_LIBRARIES} -lstdc++fs Eigen3::Eigen -lcrypto -lssl)

install(TARGETS
  planner_ros_node
  #local_planner_node
  DESTINATION lib/${PROJECT_NAME})

#############
  ## Install ##
#############

ament_export_dependencies(${PROJECT_DEPENDENCIES})

install(
  DIRECTORY include/
  DESTINATION include
)

foreach(LIB ${${PROJECT_NAME}_LIBRARIES})
  install(
    TARGETS ${LIB}
    EXPORT export_${LIB}
    ARCHIVE DESTINATION libs/${LIB}
    LIBRARY DESTINATION libs/${LIB}
    RUNTIME DESTINATION bin
  )
endforeach()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  resources 
  DESTINATION share/${PROJECT_NAME})

ament_package()

