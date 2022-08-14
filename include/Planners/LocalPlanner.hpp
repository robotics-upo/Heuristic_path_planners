#ifndef LOCALPLANNER_H_
#define LOCALPLANNER_H_

// #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <cstdlib>
#include <string>
#include <math.h>
#include <ctime>
#include <sys/timeb.h>
#include <fstream>

#include "tf2/LinearMath/Transform.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_msgs/msg/tf_message.hpp"

// #include <tf2_ros/transform_listener.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include "tf2_msgs/msg/tf_message.hpp"
// #include "tf2/LinearMath/Transform.h"
// #include "tf2/exceptions.h"
// #include "tf2_ros/buffer.h"
// #include <tf2_ros/message_filter.h>

#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

// JAC: Add the algorithms needed
#include "Planners/AStar.hpp"
#include "Planners/AStarM2.hpp"
#include "Planners/AStarM1.hpp"
#include "Planners/ThetaStar.hpp"
#include "Planners/ThetaStarM1.hpp"
#include "Planners/ThetaStarM2.hpp"
#include "Planners/LazyThetaStar.hpp"
#include "Planners/LazyThetaStarM1.hpp"
#include "Planners/LazyThetaStarM1Mod.hpp"
#include "Planners/LazyThetaStarM2.hpp"
#include "utils/ros/ROSInterfaces.hpp"
#include "utils/SaveDataVariantToFile.hpp"
#include "utils/misc.hpp"
#include "utils/geometry_utils.hpp"
#include "utils/metrics.hpp"

#include "Grid3D/grid3d.hpp"
#include <pcl/conversions.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/logging.hpp>
#include <visualization_msgs/msg/detail/marker__struct.hpp>
#include <visualization_msgs/msg/marker.hpp>

// #include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <heuristic_planners/srv/set_algorithm.hpp>
#include <heuristic_planners/srv/get_path.hpp>

// ROS1
// #include <visualization_msgs/Marker.h>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>


// #include <dynamic_reconfigure/server.h>
// #include <theta_star_2d/LocalPlannerConfig.h>
// #include <nix_common/CheckObstacles.h>
// #include <actionlib/server/simple_action_server.h>
// #include <actionlib/client/simple_action_client.h>

// #include <upo_actions/ExecutePathAction.h>
// #include <upo_actions/NavigateAction.h>
// #include <upo_actions/Navigate3DAction.h>

// #include <std_srvs/Empty.h>
#include <std_srvs/srv/empty.hpp>


// #include <std_msgs/Float32.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/u_int8.hpp>

// #include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
// #include <octomap_msgs/Octomap.h>
#include "octomap_msgs/msg/octomap.hpp"
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
// #include <pcl_ros/point_cloud.hpp>
// #include <pcl_ros/transforms.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/conversions.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

namespace PathPlanners
{
class LocalPlanner : public rclcpp::Node {

public:
    LocalPlanner();

    void plan();

    // JAC: Removed when topics and/or services are confirmed about the reception of the global goal.
    // void executePathGoalServerCB();
    // void executePathPreemptCB();

private:
    void resetFlags();
    void clearMarkers();
    //These functions gets parameters from param server at startup if they exists, if not it passes default values

    void configTopics();
    //Config thetastar class parameters
    void configServices();
    // JAC: Decide if this is considered from AS2.
    // void publishExecutePathFeedback();
    geometry_msgs::msg::TransformStamped getTfMapToRobot();
    // void publishTrajMarker3D();
    void configureAlgorithm(const std::string &algorithm_name, const std::string &_heuristic);
    bool calculateLocalGoal3D();
    void calculatePath3D();

    //Auxiliar functions
    geometry_msgs::msg::Point makePoint(const geometry_msgs::msg::Vector3 &vec);

    void pointsSub(const sensor_msgs::msg::PointCloud2::RawPtr &msg);
    void configMarkers(std::string ns, std::string frame);

    //Variables
    // ros::NodeHandlePtr nh;
    // ros::ServiceClient costmap_clean_srv;
    // rclcpp::Service<std_srvs::srv::Trigger> costmap_clean_srv;

    // ros::Subscriber local_map_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_sub_;
    // rclcpp::Subscription local_map_sub_;

    // ros::Publisher visMarkersPublisher, trajPub, costmap_inflated_pub_;
    // JAC: trajPub with the corresponding message
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr visMarkersPublisher, trajPub;
    // rclcpp::Publisher visMarkersPublisher, trajPub, costmap_inflated_pub_;

    //Flow control flags
    //To calculate planning time
    // struct timeb startT, finishT;

    //Theta star algorithm parameters
    //Geometric params
    //Algorithm params
    float cost_weight;
    float lof_distance;
    float occ_threshold; 
    
    double initialSearchAround, finalSearchAround;

    int occGoalCnt, badGoal;

    int number_of_points;
    bool debug;
    float seconds, milliseconds;

    std::string robot_base_frame, world_frame,traj_dest_frame;
    
    // JAC: Change the type of messafe
    // trajectory_msgs::msg::MultiDOFJointTrajectory globalTrajectory, localTrajectory;
    //Markers
    visualization_msgs::msg::Marker lineMarker, waypointsMarker;

    geometry_msgs::msg::Vector3 localGoal, robotPose;
    tf2_ros::Buffer *tfBuffer;

    std_msgs::msg::Bool is_running;

    //action server stufff
    //MFC: CHECK ALL THIS ACTION STUFF
    /* std::unique_ptr<ExecutePathServer> execute_path_srv_ptr;

    upo_actions::ExecutePathFeedback exec_path_fb;
    std_msgs::msg::Float32 planningRate;
    std_msgs::msg::UInt8 waypointGoingTo;
    std_msgs::msg::String planningStatus;

    upo_actions::ExecutePathResult action_result;

    rclcpp::Time start_time;
    std_msgs::msg::Float32 d2goal;
    //action client to navigate
    std::unique_ptr<NavigateClient> navigation_client_2d_ptr;
    upo_actions::NavigateGoal nav_goal;

    std::unique_ptr<Navigate3DClient> navigation3DClient;
    upo_actions::Navigate3DGoal goal3D;
    std::unique_ptr<actionlib::SimpleClientGoalState> state; */
    

    //!
    //Theta star algorithm parameters
    double ws_x_max; // 32.2
    double ws_y_max; // 32.2
    double ws_z_max;
    double ws_x_min;
    double ws_y_min;
    double ws_z_min;
    double map_resolution;
    double inflate_map;

    bool use3d;
    bool mapReceived;

    // JAC: Change to the right message of the goal. TODO
    // trajectory_msgs::msg::MultiDOFJointTrajectoryPoint currentGoal; TODO
    // std::vector<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint> goals_vector,goals_vector_bl_frame;
    int last;
    // std::unique_ptr<tf2_ros::TransformListener> tf_list_ptr;
    double timeout;

    //ADDING DUE TO ROS2
    std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_{nullptr};
    
    nav_msgs::msg::OccupancyGrid lcp;
};

} // namespace PathPlanners

#endif
