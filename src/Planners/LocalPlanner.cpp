/*
Local Planner using the LazyThetaStar Class 
José Antonio Cobano 
 */

#include <LocalPlanner.hpp>

namespace PathPlanners
{ 
    
LocalPlanner::LocalPlanner(tf2_ros::Buffer *tfBuffer_)
{
    nh.reset(new ros::NodeHandle("~"));
    // nh->param("mode3d", use3d, (bool)false); //JAC: It is not needed.

    // JAC: Parameters from planner_ros_node. Decide these parameters. They are set in the launch file.
    std::string algorithm_name;
        
    this->declare_parameter<std::string>("algorithm_name", "astar");
    this->declare_parameter<std::string>("heuristic_name", "euclidean");
    this->declare_parameter<float>("world_size_x", 5.0);
    this->declare_parameter<float>("world_size_y", 5.0);
    this->declare_parameter<float>("world_size_z", 5.0);
    this->declare_parameter<float>("resolution", 0.05);
    this->declare_parameter<bool>("inflate_map", true);
    this->declare_parameter<bool>("use3d", (bool)true);  //JAC: Check if this parameter is needed.
    this->declare_parameter<double>("cost_scaling_factor", 0.8);
    this->declare_parameter<double>("robot_radius", 0.4);
    this->declare_parameter<std::string>("frame_id", std::string("map"));
    this->declare_parameter<bool>("save_data_file", (bool)true);
    this->declare_parameter<std::string>("data_folder", std::string("planing_data.txt"));
    this->declare_parameter<float>("max_line_of_sight_distance", (float)1000.0);
    this->declare_parameter<float>("cost_weight", (float)0.0);
    this->declare_parameter<bool>("overlay_markers", (bool)false);
    this->declare_parameter<double>("inflation_size", 0.5);
    
    this->get_parameter("algorithm_name", algorithm_name); //JAC: The algorithm should be fixed.
    this->get_parameter("heuristic_name", heuristic_);    

    // configParams3D();
    configureAlgorithm(algorithm_name, heuristic_);
    
    tf_list_ptr.reset(new tf::TransformListener(ros::Duration(5)));
    tfBuffer = tfBuffer_;

    configTopics();
    // configTheta(); // JAC: This is configured in configureAlgorithm
    configServices(); 
}
void LocalPlanner::clearMarkers()
{
    waypointsMarker.action = RVizMarker::DELETEALL;
    lineMarker.action = RVizMarker::DELETEALL;

    visMarkersPublisher.publish(lineMarker);
    visMarkersPublisher.publish(waypointsMarker);

    lineMarker.points.clear();
    waypointsMarker.points.clear();

    lineMarker.action = RVizMarker::ADD;
    waypointsMarker.action = RVizMarker::ADD;
}
//Config standard services and action lib servers and clients
void LocalPlanner::configServices()
{
    // JAC: This should be update in Aerostack2.
    // execute_path_srv_ptr.reset(new ExecutePathServer(*nh, "/Execute_Plan", false));
    // //JAC: This service provides the goal. Change in AS2.
    // execute_path_srv_ptr->registerGoalCallback(boost::bind(&LocalPlanner::executePathGoalServerCB, this));
    // execute_path_srv_ptr->registerPreemptCallback(boost::bind(&LocalPlanner::executePathPreemptCB, this));
    // execute_path_srv_ptr->start();

    // // JAC: This should be update in Aerostack2.
    // navigation3DClient.reset(new Navigate3DClient("/Navigation3D", true));
    // navigation3DClient->waitForServer();

}
void LocalPlanner::resetFlags()
{
    // mapReceived = false;
    impossibleCnt = 0;
    occGoalCnt = 0;
    startIter = 1;
    timesCleaned = 0;
    badGoal = 0;
    last = 0;
}
void LocalPlanner::configParams3D() //JAC: UPDATE THE PARAMS TO ROS2 --> Consider configureAlgorithm as in planner_ros_node
{
    //Flags for flow control
    resetFlags();
    mapGeometryConfigured = false;

    robotPose.x = 0;
    robotPose.y = 0;
    robotPose.z = 0;

    nh.reset(new ros::NodeHandle("~"));
    tf_list.reset(new tf::TransformListener);

    //JAC: Consider all the params
    nh->param("timeout", timeout, (double)0.5);
    nh->param("arrived_thresh", arrivedThresh, (double)0.25);
    nh->param("ws_x_max", ws_x_max, (double)5);
    nh->param("ws_y_max", ws_y_max, (double)5);
    nh->param("ws_z_max", ws_z_max, (double)5);

    ws_x_min = -ws_x_max;
    ws_y_min = -ws_y_max;
    ws_z_min = -ws_z_max;

    // JAC: Take the parameters
    nh->param("map_resolution", map_resolution, (double)0.05);     // JAC map_resolution --> resolution
    nh->param("map_h_inflaction", map_h_inflaction, (double)0.5);  // JAC map_h_inflaction --> inflation_size
    nh->param("map_v_inflaction", map_v_inflaction, (double)0.5);  // JAC map_v_inflaction --> inflation_size
    nh->param("z_weight_cost", z_weight_cost, (double)1.2);        // JAC
    nh->param("z_not_inflate", z_not_inflate, (double)3);          // JAC
    
    
    nh->param("goal_weight", goal_weight, (double)1.1);            // JAC
    nh->param("traj_dxy_max", traj_dxy_max, (double)1);            
    nh->param("traj_pos_tol", traj_pos_tol, (double)1);            
    nh->param("traj_yaw_tol", traj_yaw_tol, (double)0.1);          
    nh->param("traj_dz_max", traj_dz_max, (double)1);              
    nh->param("traj_vxy_m", traj_vxy_m, (double)1);                
    nh->param("traj_vz_m", traj_vz_m, (double)1);                  
    nh->param("traj_vxy_m_1", traj_vxy_m_1, (double)1);
    nh->param("traj_vz_m_1", traj_vz_m_1, (double)1);
    nh->param("traj_wyaw_m", traj_wyaw_m, (double)1);
    nh->param("world_frame", world_frame, (string) "/map");
    nh->param("robot_base_frame", robot_base_frame, (string) "/base_link");

    nh->param("traj_dest_frame", traj_dest_frame, (string) "/base_link");

    nh->param("debug", debug, (bool)0);
    nh->param("show_config", showConfig, (bool)0);

    nh->param("initial_search_around", initialSearchAround, (double)0.5);
    nh->param("final_search_around", finalSearchAround, (double)0.3);

    // ROS_INFO_COND(showConfig, PRINTF_GREEN "Local Planner 3D Node Configuration:\n");
    // ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Lazy Theta* with optim.: goal_weight = [%.2f]", goal_weight);
    // ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Trajectory Position Increments = [%.2f], Tolerance: [%.2f]", traj_dxy_max, traj_pos_tol);
    // ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Robot base frame: %s, World frame: %s", robot_base_frame.c_str(), world_frame.c_str());
    // ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Workspace:\t X:[%.2f, %.2f]\t Y:[%.2f, %.2f]\t Z: [%.2f, %.2f]", ws_x_max, ws_x_min, ws_y_max, ws_y_min, ws_z_max, ws_z_min);
    // ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Map Resolution: %.2f\t Map H inflaction: %.2f\t Map V Inflaction: %.2f", map_resolution, map_h_inflaction, map_v_inflaction);
    // ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Z weight cost: %.2f\t Z not inflate: %.2f", z_weight_cost, z_not_inflate);

    RCLCPP_INFO(this->get_logger(),"Local Planner 3D Node Configuration:\n");
    RCLCPP_INFO(this->get_logger(),"\t Lazy Theta* with optim.: goal_weight = [%.2f]", goal_weight);
    RCLCPP_INFO(this->get_logger(),"\t Trajectory Position Increments = [%.2f], Tolerance: [%.2f]", traj_dxy_max, traj_pos_tol);
    RCLCPP_INFO(this->get_logger(),"\t Robot base frame: %s, World frame: %s", robot_base_frame.c_str(), world_frame.c_str());
    RCLCPP_INFO(this->get_logger(),"\t Workspace:\t X:[%.2f, %.2f]\t Y:[%.2f, %.2f]\t Z: [%.2f, %.2f]", ws_x_max, ws_x_min, ws_y_max, ws_y_min, ws_z_max, ws_z_min);
    RCLCPP_INFO(this->get_logger(),"\t Map Resolution: %.2f\t Map H inflaction: %.2f\t Map V Inflaction: %.2f", map_resolution, map_h_inflaction, map_v_inflaction);
    RCLCPP_INFO(this->get_logger(),"\t Z weight cost: %.2f\t Z not inflate: %.2f", z_weight_cost, z_not_inflate);

    configMarkers("local_path_3d", traj_dest_frame);
}
void LocalPlanner::configMarkers(std::string ns, std::string frame)
{
    //Line strip marker use member points, only scale.x is used to control linea width
    lineMarker.header.frame_id = frame;
    lineMarker.header.stamp = ros::Time::now();
    lineMarker.id = rand();
    lineMarker.ns = ns;
    lineMarker.lifetime = ros::Duration(500);
    lineMarker.type = RVizMarker::LINE_STRIP;
    lineMarker.action = RVizMarker::ADD;
    lineMarker.pose.orientation.w = 1;
    lineMarker.pose.position.z = 0.1;
    lineMarker.color.r = 1.0;
    lineMarker.color.g = 0.0;
    lineMarker.color.b = 0.0;
    lineMarker.color.a = 1.0;
    lineMarker.scale.x = 0.1;

    waypointsMarker.header.frame_id = frame;
    waypointsMarker.header.stamp = ros::Time::now();
    waypointsMarker.ns = ns;
    waypointsMarker.id = lineMarker.id + 12;
    waypointsMarker.lifetime = ros::Duration(500);
    waypointsMarker.type = RVizMarker::POINTS;
    waypointsMarker.action = RVizMarker::ADD;
    waypointsMarker.pose.orientation.w = 1;
    waypointsMarker.pose.position.z = 0.1;
    waypointsMarker.color.r = 0.0;
    waypointsMarker.color.g = 0.0;
    waypointsMarker.color.b = 1.0;
    waypointsMarker.color.a = 1.0;
    waypointsMarker.scale.x = 0.15;
    waypointsMarker.scale.y = 0.15;
    waypointsMarker.scale.z = 0.4;
}

// IT IS NOT NEEDED
// void LocalPlanner::collisionMapCallBack(const octomap_msgs::OctomapConstPtr &msg)
// {
//     mapReceived = true;
//     //map = msg;
//     theta3D.updateMap(msg);
//     // ROS_INFO_COND(debug, PRINTF_MAGENTA "Collision Map Received");
//     theta3D.publishOccupationMarkersMap();
// }

// void LocalPlanner::pointsSub(const PointCloud::ConstPtr &points)
void LocalPlanner::pointsSub(const sensor_msgs::msg::PointCloud2::SharedPtr points)
{

    // // ROS_INFO_COND(debug, PRINTF_MAGENTA "Collision Map Received");
    // mapReceived = true;
    // PointCloud out;
    // pcl_ros::transformPointCloud(robot_base_frame, *points, out, *tf_list_ptr);
    // theta3D.updateMap(out); //JAC: use octomap
    // theta3D.publishOccupationMarkersMap();
       
    // JAC: Here should be this? (It was in configureAlgorithm in planner_ros_node)
    
    mapReceived = true;
    // // Callback in planner_ros_node
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> _points_ptr = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*(points.get()), *(_points_ptr.get()));
    RCLCPP_INFO(this->get_logger(),"Loading map...");
    Planners::utils::configureWorldFromPointCloud(_points_ptr, *algorithm_, resolution_); //ROSInterfaces.cpp
    algorithm_->publishOccupationMarkersMap();
    // JAC: Grid shoulb be built here: computeGrid in grid3d.hpp??? TODO
    // JAC: configureWorldFromPointCloud and configureWorldCosts are run in configureAlgorithm in planner_ros_node 
    // JAC: It could be something like this but it would not be needed load the octomap and compute the point cloud (computePointCloud).
    // JAC: Directly computeGrid should be run.
    // m_grid3d_ = std::make_unique<Grid3d>(this); 
    // After computing the grid, the costs are added to each node with configureWorldCosts
    Planners::utils::configureWorldCosts(*m_grid3d_, *algorithm_); //ROSInterfaces.cpp
    RCLCPP_INFO(this->get_logger(),"Published occupation marker map");
    cloud_ = *_points_ptr;
    input_map_ = 2;
    pointcloud_sub_.reset();    

}
// JAC: Configure the parameters of the Lazy Theta Star --> configureAlgorithm
void LocalPlanner::dynRecCb(theta_star_2d::LocalPlannerConfig &config, uint32_t level)
{
    this->cost_weight = config.cost_weight;
    this->lof_distance = config.lof_distance;
    this->occ_threshold = config.occ_threshold;
    this->goal_weight = config.goal_weight;

    if (!use3d)
    {
        theta2D.setDynParams(goal_weight, cost_weight, lof_distance, occ_threshold);
        ROS_INFO_COND(debug, PRINTF_MAGENTA "Local Planner: 2D Dynamic reconfiguration required");
    }
}
void LocalPlanner::executePathPreemptCB()
{
    // ROS_INFO_COND(debug, "Goal Preempted");
    RCLCPP_INFO(this->get_logger(),"Goal Preempted");
    execute_path_srv_ptr->setPreempted(); // set the action state to preempted

    // JAC: This should be update in Aerostack2.
    navigation3DClient->cancelAllGoals();

    resetFlags();
    clearMarkers();
}
void LocalPlanner::executePathGoalServerCB() // Note: "Action" is not appended to exe here
{
    resetFlags();
    clearMarkers();
    start_time = ros::Time::now();

    // ROS_INFO_COND(debug, "Local Planner Goal received in action server mode");
    RCLCPP_INFO(this->get_logger(),"Local Planner Goal received in action server mode");
    //upo_actions::ExecutePathGoalConstPtr path_shared_ptr;
    auto path_shared_ptr = execute_path_srv_ptr->acceptNewGoal();
    globalTrajectory = path_shared_ptr->path;
    auto size = globalTrajectory.points.size();

    goals_vector = globalTrajectory.points;
    goal3D.global_goal.pose.position.x = globalTrajectory.points.at(globalTrajectory.points.size() - 1).transforms[0].translation.x;
    goal3D.global_goal.pose.position.y = globalTrajectory.points.at(globalTrajectory.points.size() - 1).transforms[0].translation.y;
    goal3D.global_goal.pose.position.z = globalTrajectory.points.at(globalTrajectory.points.size() - 1).transforms[0].translation.z;
        
    goal3D.global_goal.pose.orientation.x = globalTrajectory.points.at(globalTrajectory.points.size() - 1).transforms[0].rotation.x;
    goal3D.global_goal.pose.orientation.y = globalTrajectory.points.at(globalTrajectory.points.size() - 1).transforms[0].rotation.y;
    goal3D.global_goal.pose.orientation.z = globalTrajectory.points.at(globalTrajectory.points.size() - 1).transforms[0].rotation.z;
    goal3D.global_goal.pose.orientation.w = globalTrajectory.points.at(globalTrajectory.points.size() - 1).transforms[0].rotation.w;
    // JAC: Update
    navigation3DClient->sendGoal(goal3D);

    ROS_INFO_COND(debug, "Local Planner: Processed goal message");
}
void LocalPlanner::configTheta()
{
    string node_name = "local_planner_node";

    // JAC: I have to change theta3D --> Lazy3D
    theta3D.init(node_name, robot_base_frame, ws_x_max, ws_y_max, ws_z_max, ws_x_min, ws_y_min, ws_z_min, map_resolution, map_h_inflaction, map_v_inflaction, goal_weight, z_weight_cost, z_not_inflate, nh);
    theta3D.setTimeOut(timeout);
    // JAC: These parameters are not needed.
    // theta3D.setTrajectoryParams(traj_dxy_max, traj_dz_max, traj_pos_tol, traj_vxy_m, traj_vz_m, traj_vxy_m_1, traj_vz_m_1, traj_wyaw_m, traj_yaw_tol);
    theta3D.confPrintRosWarn(true);
    double min_r;
    nh->param("min_r_obstacle", min_r, 0.8);
    theta3D.setMinObstacleRadius(min_r);

    mapGeometryConfigured = true;

    //ROS_INFO_COND(debug, PRINTF_MAGENTA "Local Planner: Theta Star Configured");
}
void LocalPlanner::configTopics()
{
    // JAC: Subscription to the Point Cloud.
    local_map_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/points", 1, std::bind(&LocalPlanner::pointsSub, this, std::placeholders::_1));
    // local_map_sub = nh->subscribe<PointCloud>("/points", 1, &LocalPlanner::pointsSub, this);
    // JAC: Subscription to the grid generated from of the Point Cloud. Decide if use planner_ros_node o include this function in LocalPlanner.
    occupancy_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/grid", 1, std::bind(&HeuristicPlannerROS::occupancyGridCallback, this, std::placeholders::_1));

    point_markers_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("path_points_markers", 1);
    // visMarkersPublisher = nh->advertise<visualization_msgs::Marker>("markers", 1, true);
    line_markers_pub_  = this->create_publisher<visualization_msgs::msg::Marker>("path_line_markers", 1);
    // JAC: I have to change trajPub. It has to publish a TrajectoryWaypoints.msg
    trajPub = this->create_publisher<trajectory_msgs::MultiDOFJointTrajectory>("local_path", 1);
    // trajPub = nh->advertise<trajectory_msgs::MultiDOFJointTrajectory>("local_path", 1);
}

// JAC: IT IS NOT NEEDED
// //Calbacks and publication functions
// void LocalPlanner::localCostMapCb(const nav_msgs::msg::OccupancyGrid::ConstPtr &lcp)
// void LocalPlanner::localCostMapCb(const nav_msgs::msg::OccupancyGrid::SharePtr lcp)
// {
//     localCostMap = *lcp;
//     mapReceived = true;

//     //First time the map is received, configure the geometric params
//     if (!mapGeometryConfigured)
//     {
//         map_resolution = round(1000 * (lcp->info.resolution)) / 1000; //Because sometimes the map server shows not exact long numbers as 0.0500003212

//         ws_x_max = lcp->info.width * map_resolution + 2 * localCostMapInflationX;
//         ws_y_max = lcp->info.height * map_resolution + 2 * localCostMapInflationY;

//         local_costmap_center.x = ws_x_max / 2;
//         local_costmap_center.y = ws_y_max / 2;
//         //Initiaize the inflated costmap occupancy grid with the data and metadata from the original costmap

//         inflateCostMap();
//         mapGeometryConfigured = true;

//         theta2D.loadMapParams(ws_x_max, ws_y_max, map_resolution);
//         theta2D.getMap(&localCostMapInflated);
//         ROS_INFO_COND(showConfig, PRINTF_GREEN "\t WorkSpace: X:[%.2f], Y:[%.2f]", ws_x_max, ws_y_max);
//         ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Local Costmap Origin: [%.2f, %.2f]", local_costmap_center.x, local_costmap_center.y);
//         ROS_INFO_COND(showConfig, PRINTF_GREEN "\t Map: resol.= [%.2f]", map_resolution);
// }
// }
void LocalPlanner::plan()
{
    number_of_points = 0; //Reset variable

    // JAC: Review and modify in AS2
    if (!execute_path_srv_ptr->isActive())
    {
        clearMarkers();
        return;
    }

    calculatePath3D();
    // JAC: In this moment the solution local path has been published.
    // JAC: Related to actionlib should be update in Aerostack2
    state.reset(new actionlib::SimpleClientGoalState(navigation3DClient->getState()));

    seconds = finishT.time - startT.time - 1;
    milliseconds = (1000 - startT.millitm) + finishT.millitm;
    publishExecutePathFeedback();

    // JAC: Review and modify in AS2
    if (*state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        clearMarkers();
        action_result.arrived = true;
        execute_path_srv_ptr->setSucceeded(action_result);
        ROS_ERROR("LocalPlanner: Goal Succed");
        return;
    }
    // JAC: Review and modify in AS2
    else if (*state == actionlib::SimpleClientGoalState::ABORTED)
    {
        ROS_INFO_COND(debug, "Goal aborted by path tracker");
        resetFlags();
    }
    // JAC: Review and modify in AS2
    else if (*state == actionlib::SimpleClientGoalState::PREEMPTED)
    {
        ROS_INFO_COND(debug, "Goal preempted by path tracker");
        resetFlags();
    }
    //ROS_INFO_COND(debug, "Before start loop calculation");
}

// JAC: IT IS NOT NEEDED
// void LocalPlanner::calculatePath2D()
// {
//     ROS_INFO("Calculating");

//     ftime(&startT);
//     if (mapReceived && mapGeometryConfigured)
//     {
//         ROS_INFO_COND(debug, PRINTF_BLUE "Local Planner: Global trj received and local costmap received");
//         mapReceived = false;
//         inflateCostMap(); //TODO Gordo arreglar esta chapuza de funcion
//         theta2D.getMap(&localCostMapInflated);
//         ROS_INFO("Initial Point: %.2f, %.2f", local_costmap_center.x, local_costmap_center.y);
//         if (theta2D.setValidInitialPosition(local_costmap_center) || theta2D.searchInitialPosition2d(initialSearchAround))
//         {
//             ROS_INFO_COND(debug, PRINTF_MAGENTA "Start ok, calculating local goal");

//             if (calculateLocalGoal2D())
//             {
               
//                 ROS_INFO_COND(debug, PRINTF_MAGENTA "Local Goal calculated");
//                 freeLocalGoal();
//                 theta2D.getMap(&localCostMapInflated);
//                 costmap_inflated_pub_.publish(localCostMapInflated);

//                 if (theta2D.setValidFinalPosition(localGoal) || theta2D.searchFinalPosition2d(finalSearchAround))
//                 {
//                     ROS_INFO_COND(debug, PRINTF_BLUE "Local Planner: Computing Local Path(2)");

//                     number_of_points = theta2D.computePath();
//                     ROS_INFO_COND(debug, PRINTF_BLUE "Local Planner: Path computed, number %d", number_of_points);
//                     occGoalCnt = 0;
//                     timesCleaned = 0;

//                     if (number_of_points > 0)
//                     {
//                         buildAndPubTrayectory2D();
//                         planningStatus.data = "OK";
//                         if (impossibleCnt > 0) //If previously the local planner couldn t find solution, reset
//                             impossibleCnt = 0;
//                     }
//                     else if (number_of_points == 0) //!Esto es lo que devuelve el algoritmo cuando NO HAY SOLUCION
//                     {
                        
//                         impossibleCnt++;
//                         //ROS_INFO_COND(debug,"Local: +1 impossible");
//                         if (impossibleCnt > 2)
//                         {
//                             double dist2goal = euclideanDistance(nav_goal.global_goal.position.x, nav_goal.global_goal.position.y, getTfMapToRobot().transform.translation.x, getTfMapToRobot().transform.translation.y);

//                             /*if (dist2goal < arrivedThresh)
//                             {

//                                 clearMarkers();
//                                 action_result.arrived = true;
//                                 execute_path_srv_ptr->setSucceeded(action_result);
//                                 ROS_ERROR("LocalPlanner: Goal Succed");
//                                 navigation_client_2d_ptr->cancelGoal();
//                             }*/
//                             //else
//                             if( dist2goal > arrivedThresh ){
//                                 navigation_client_2d_ptr->cancelGoal();
//                                 planningStatus.data = "Requesting new global path, navigation cancelled";
//                                 execute_path_srv_ptr->setAborted();
//                                 impossibleCnt = 0;
//                             }
//                         }
//                     }
//                 }
//                 else if (occGoalCnt > 2) //!Caso GOAL OCUPADO
//                 {                        //If it cant find a free position near local goal, it means that there is something there.

//                     ROS_INFO_COND(debug, PRINTF_BLUE "Pausing planning, final position busy");
//                     planningStatus.data = "Final position Busy, Cancelling goal";
//                     //TODO What to tell to the path tracker
//                     navigation_client_2d_ptr->cancelGoal();

//                     execute_path_srv_ptr->setAborted();
//                     //In order to resume planning, someone must call the pause/resume planning Service that will change the flag to true
//                     occGoalCnt = 0;
//                 }
//                 else
//                 {
//                     ++occGoalCnt;
//                 }
//             }
//             else if (badGoal < 3)
//             {
//                 ++badGoal;
//             }
//             else
//             {
//                 navigation_client_2d_ptr->cancelGoal();
//                 ROS_INFO("Bad goal, aborting");
//                 execute_path_srv_ptr->setAborted();
//                 badGoal = 0;
//             }
//         }
//         else if (timesCleaned < 3)
//         {
//             ++timesCleaned;
//             ROS_INFO_COND(debug, "Local:Couldn't find a free point near start point, trying to clean the local costmap");
//             std_srvs::Trigger trg;
//             costmap_clean_srv.call(trg);
//         }
//         else
//         {
//             planningStatus.data = "Tried to clean costmap but no initial position found...";
//             navigation_client_2d_ptr->cancelGoal();
//             ROS_INFO("Tried to clean costmap but no intiial position found...");
//             execute_path_srv_ptr->setAborted();
//             clearMarkers();
//         }
//     }
//     ftime(&finishT);
// }
void LocalPlanner::calculatePath3D()
{
    if (mapReceived) // It is true once a pointcloud is received.
    {
        ROS_INFO_COND(debug, PRINTF_BLUE "Local Planner 3D: Global trj received and pointcloud received");
        mapReceived = false;

        //TODO : Set to robot pose to the center of the workspace?
        // if (theta3D.setValidInitialPosition(robotPose) || theta3D.searchInitialPosition3d(initialSearchAround))
        // JAC: Decide if implement "Symetric search in nearest XY-Zup neighbours" with searchInitialPosition3d
        if (!algorithm_->detectCollision(robotPose))
        {
            ROS_INFO_COND(debug, PRINTF_MAGENTA "Local Planner 3D: Calculating local goal");

            if (calculateLocalGoal3D()) //TODO: VER CUAL SERIA EL LOCAL GOAL EN ESTE CASO
            {
                ROS_INFO_COND(debug, PRINTF_MAGENTA "Local Planner 3D: Local Goal calculated");

                // ROS_INFO("%.6f,%.6f, %.6f", localGoal.x, localGoal.y,localGoal.z);
                RCLCPP_INFO(this->get_logger(),"%.6f,%.6f, %.6f", localGoal.x, localGoal.y,localGoal.z);

                // JAC: El local_goal debe ser de este tipo
                // const auto discrete_goal =  Planners::utils::discretePoint(_req.goal, resolution_);

                // if (!theta3D.isInside(localGoal))
                // JAC: Check how do it TODO
                // if (!theta3D.isInside(localGoal)) --> checkValid (Planners::utils::)
                if (!checkValid(localGoal))
                {
                    ROS_INFO("Returning, not inside :(");
                    // JAC: Modify in AS2
                    // action_result.arrived=false;
                    // execute_path_srv_ptr->setAborted(action_result, "Not inside after second check");
                    // navigation3DClient->cancelAllGoals();
                    return;
                }

                // if (theta3D.setValidFinalPosition(localGoal) || theta3D.searchFinalPosition3dAheadHorizontalPrior(finalSearchAround))
                // JAC: Decide if implement "// Asymetric search in nearest XinFront-Y-Zup neighbours" with searchFinalPosition3dAheadHorizontalPrior
                if (!algorithm_->detectCollision(localGoal))
                {
                    ROS_INFO_COND(debug, PRINTF_BLUE "Local Planner 3D: Computing Local Path");

                    // JAC: We calculate the path with findpath
                    // number_of_points = theta3D.computePath();
                    // JAC: robotPose or start should be like in planner_ros_node
                    // const auto discrete_start =  Planners::utils::discretePoint(_req.start, resolution_);
                    auto path_data = algorithm_->findPath(discrete_start, discrete_goal);

                    // ROS_INFO_COND(debug, PRINTF_BLUE "Local Planner 3D: Path computed, %d points", number_of_points);
                    occGoalCnt = 0;

                    if( std::get<bool>(path_data["solved"]) ){
                        Planners::utils::CoordinateList path;
                        try{
                            _rep.time_spent.data           = std::get<double>(path_data["time_spent"] );
                            _rep.time_spent.data /= 1000;
                            times.push_back(_rep.time_spent.data);

                            if(_req.tries.data < 2 || i == ( _req.tries.data - 1) ){
                                _rep.path_length.data          = std::get<double>(path_data["path_length"] );
                                _rep.explored_nodes.data       = std::get<size_t>(path_data["explored_nodes"] );
                                _rep.line_of_sight_checks.data = std::get<unsigned int>(path_data["line_of_sight_checks"] );

                                _rep.total_cost1.data           = std::get<unsigned int>(path_data["total_cost1"] );
                                _rep.total_cost2.data           = std::get<unsigned int>(path_data["total_cost2"] );
                                _rep.h_cost.data               = std::get<unsigned int>(path_data["h_cost"]);
                                _rep.g_cost1.data               = std::get<unsigned int>(path_data["g_cost1"]);
                                _rep.g_cost2.data               = std::get<unsigned int>(path_data["g_cost2"]);
                                _rep.c_cost.data               = std::get<unsigned int>(path_data["c_cost"]);

                                _rep.cost_weight.data          = std::get<double>(path_data["cost_weight"]);
                                _rep.max_los.data              = std::get<unsigned int>(path_data["max_line_of_sight_cells"]);
                            }
                            path = std::get<Planners::utils::CoordinateList>(path_data["path"]);

                        }catch(std::bad_variant_access const& ex){
                            std::cerr << "Bad variant error: " << ex.what() << std::endl;
                        }
                    }else{
                        RCLCPP_INFO(this->get_logger(),"Could not calculate path between request points");
                    }                    

                    // JAC: We do not need to compute trayectory. Just the path return by findPath
                    // JAC: path computed has to be converted to TrajectoryWaypoints.msg TODO

                    // if (number_of_points > 0)
                    // {
                    //     buildAndPubTrayectory3D();
                    //     planningStatus.data = "OK";
                    //     if (impossibleCnt > 0) //If previously the local planner couldn t find solution, reset
                    //         impossibleCnt = 0;
                    // }
                    // else if (number_of_points == 0) //!Esto es lo que devuelve el algoritmo cuando NO HAY SOLUCION
                    // {

                    //     impossibleCnt++;
                    //     //ROS_INFO_COND(debug,"Local: +1 impossible");
                    //     if (impossibleCnt > 2)
                    //     {

                    //         clearMarkers();
                    //         action_result.arrived=false;
                    //         execute_path_srv_ptr->setAborted(action_result, "Requesting new global path, navigation cancelled");
                    //         navigation3DClient->cancelAllGoals();
                    //         planningStatus.data = "Requesting new global path, navigation cancelled";
                    //         impossibleCnt = 0;
                    //     }
                    // }
                }
                else if (occGoalCnt > 2) //!Caso GOAL OCUPADO
                {                        //If it cant find a free position near local goal, it means that there is something there.
                    ROS_INFO_COND(debug, PRINTF_BLUE "Local Planner 3D: Pausing planning, final position busy");
                    planningStatus.data = "Final position Busy, Cancelling goal";
                    //TODO What to tell to the path tracker
                    action_result.arrived=false;
                    execute_path_srv_ptr->setAborted(action_result, "Local goal occupied");
                    //In order to resume planning, someone must call the pause/resume planning Service that will change the flag to true
                    occGoalCnt = 0;
                }
                else
                {
                    ++occGoalCnt;
                }
                // JAC: Here we have to convert the path to TrajectoryWaypoints.msg and publish
                // JAC: Check if publisher should be changed because 
                // trajPub.publish(localTrajectory);
                // publishTrajMarker3D();
            }
            else if (badGoal < 3)
            {
                ROS_INFO_COND(debug, "Local Planner 3D: Bad Goal Calculated: [%.2f, %.2f]", localGoal.x, localGoal.y);

                ++badGoal;
            }
            else
            {
                navigation3DClient->cancelAllGoals();
                action_result.arrived=false;
                execute_path_srv_ptr->setAborted(action_result,"Bad goal calculated 3 times");
                badGoal = 0;
                ROS_INFO("Bad goal calculated 3 times");
            }
        }
        else
        {
            planningStatus.data = "No initial position found...";
            action_result.arrived=false;
            execute_path_srv_ptr->setAborted(action_result,"No initial position found");
            navigation3DClient->cancelAllGoals();

            clearMarkers();
            ROS_INFO_COND(debug, "Local Planner 3D: No initial free position found");
        }
    }
}
void LocalPlanner::publishExecutePathFeedback()
{

    if (planningStatus.data.find("OK") > 0)
    {
        planningRate.data = 1000 / milliseconds;
    }
    else
    {
        planningRate.data = 0;
    }

    exec_path_fb.global_waypoint = waypointGoingTo;
    exec_path_fb.planning_rate = planningRate;
    exec_path_fb.status = planningStatus;
    execute_path_srv_ptr->publishFeedback(exec_path_fb);
}
bool LocalPlanner::calculateLocalGoal2D()
{

    ROS_INFO_COND(debug, "Local goal calculation");
    geometry_msgs::Vector3Stamped A, B, C;

    trajectory_msgs::MultiDOFJointTrajectory globalTrajBLFrame = globalTrajectory;
    //First we transform the trajectory published by global planner to base_link frame

    geometry_msgs::TransformStamped tr = getTfMapToRobot();

    globalTrajBLFrame.header.frame_id = robot_base_frame;

    for (size_t i = 0; i < globalTrajectory.points.size(); ++i)
    {
        globalTrajBLFrame.points[i].transforms[0].translation.x -= tr.transform.translation.x;
        globalTrajBLFrame.points[i].transforms[0].translation.y -= tr.transform.translation.y;
    }
    //Check that at least one point is inside the local workspace
    bool ok = false;
    for (size_t i = 0; i < globalTrajBLFrame.points.size(); ++i)
    {
        if (globalTrajBLFrame.points[i].transforms[0].translation.x < (ws_x_max / 2 - localCostMapInflationX) && globalTrajBLFrame.points[i].transforms[0].translation.y < (ws_y_max / 2 - localCostMapInflationX) &&
            globalTrajBLFrame.points[i].transforms[0].translation.x > (-ws_x_max / 2 + localCostMapInflationX) && globalTrajBLFrame.points[i].transforms[0].translation.y > (-ws_y_max / 2 + localCostMapInflationY))
        {
            ok = true;
        }
    }
    if (!ok)
        return ok;

    //Ya esta referida al base_link. Ahora la recorro desde i=1(porque i=0 es siempre la pos del base_link que al pasarla al sistema base_link sera (0,0))

    for (size_t i = startIter; i < globalTrajectory.points.size(); i++, startIter++)
    {
        B.vector.x = globalTrajBLFrame.points[i].transforms[0].translation.x;
        B.vector.y = globalTrajBLFrame.points[i].transforms[0].translation.y;

        C.vector.x = B.vector.x + (ws_x_max) / 2;
        C.vector.y = B.vector.y + (ws_y_max) / 2;

        if (std::abs(B.vector.x) > (ws_x_max / 2 - localCostMapInflationX) || std::abs(B.vector.y) > (ws_y_max / 2 - localCostMapInflationY) || (i == globalTrajectory.points.size() - (size_t)1))
        {
            A.vector.x = globalTrajBLFrame.points[i - 1].transforms[0].translation.x;
            A.vector.y = globalTrajBLFrame.points[i - 1].transforms[0].translation.y;
            if (std::abs(B.vector.x) > ws_x_max / 2 || std::abs(B.vector.y) > ws_y_max / 2)
            {

                C.vector.x = A.vector.x + (ws_x_max) / 2;
                C.vector.y = A.vector.y + (ws_y_max) / 2;

                while (C.vector.x < (ws_x_max - localCostMapInflationX + map_resolution) && C.vector.x > localCostMapInflationX - 2 * map_resolution &&
                       C.vector.y < (ws_y_max - localCostMapInflationY + map_resolution) && C.vector.y > localCostMapInflationY - 2 * map_resolution)
                { //Put the point between i-1 and i
                    C.vector.x += map_resolution * (B.vector.x - A.vector.x) / sqrtf(pow(B.vector.x - A.vector.x, 2) + pow(B.vector.y - A.vector.y, 2));
                    C.vector.y += map_resolution * (B.vector.y - A.vector.y) / sqrtf(pow(B.vector.x - A.vector.x, 2) + pow(B.vector.y - A.vector.y, 2));
                }
                startIter = i - 1;
            }

            break;
        }
    }
    ROS_INFO("lOCAL GOAL Prev round: %.2f \t %.2f", C.vector.x, C.vector.y);
    //TODO: Mejorar esta chapuza
    while (C.vector.x > (ws_x_max - map_resolution) || C.vector.y > (ws_y_max - map_resolution))
    {
        C.vector.x -= map_resolution * (B.vector.x - A.vector.x) / sqrtf(pow(B.vector.x - A.vector.x, 2) + pow(B.vector.y - A.vector.y, 2));
        C.vector.y -= map_resolution * (B.vector.y - A.vector.y) / sqrtf(pow(B.vector.x - A.vector.x, 2) + pow(B.vector.y - A.vector.y, 2));
    }
    if (C.vector.x == 0)
        C.vector.x += 2 * map_resolution;
    if (C.vector.y == 0)
        C.vector.y += 2 * map_resolution;

    C.vector.x = floor(C.vector.x * 10 + 10 * map_resolution) / 10;

    C.vector.y = floor(C.vector.y * 10 + 10 * map_resolution) / 10;

    if (C.vector.x == 0)
        C.vector.x += map_resolution;
    if (C.vector.y == 0)
        C.vector.y += map_resolution;

    localGoal = C.vector;
    return ok;
}
// Compute the local goal from the goals_vector (global path)
// JAC: Now we can test the local planner with global goal as input from terminal.
// JAC: localgoal should be like in planner_ros_node
// JAC: const auto discrete_goal =  Planners::utils::discretePoint(_req.goal, resolution_);
bool LocalPlanner::calculateLocalGoal3D()
{

    //Okey we have our std queu with the global waypoints
    //? 1. Take front and calculate if it inside the workspace
    //? 2. if it
    geometry_msgs::Vector3 currentGoalVec;
    //geometry_msgs::TransformStamped robot = getTfMapToRobot();

    goals_vector_bl_frame = goals_vector;

    geometry_msgs::PoseStamped pose, poseout;
    pose.header.frame_id = world_frame;
    poseout.header.frame_id = robot_base_frame;
    pose.header.stamp = ros::Time::now();
    pose.header.seq = rand();

    try
    {
        tf_list_ptr->waitForTransform(robot_base_frame, world_frame, ros::Time::now(), ros::Duration(1));
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("Transform exception : %s", ex.what());
        return false;
    }

    //Transform the global trajectory to base link
    ROS_INFO(PRINTF_CYAN "Calculating local goal 3D");
    for (auto &it : goals_vector_bl_frame)
    {
        pose.pose.position.x = it.transforms[0].translation.x;
        pose.pose.position.y = it.transforms[0].translation.y;
        pose.pose.position.z = it.transforms[0].translation.z;

        pose.pose.orientation.x = it.transforms[0].rotation.x;
        pose.pose.orientation.y = it.transforms[0].rotation.y;
        pose.pose.orientation.z = it.transforms[0].rotation.z;
        pose.pose.orientation.w = it.transforms[0].rotation.w;

        double norm = pose.pose.orientation.x * pose.pose.orientation.x +
                      pose.pose.orientation.y * pose.pose.orientation.y +
                      pose.pose.orientation.z * pose.pose.orientation.z +
                      pose.pose.orientation.w * pose.pose.orientation.w;
        norm = sqrt(norm);
        if (norm < 1e-6)
        {
            ROS_ERROR("Error, norm to small ");
            return false;
        }

        pose.pose.orientation.x /= norm;
        pose.pose.orientation.y /= norm;
        pose.pose.orientation.z /= norm;
        pose.pose.orientation.w /= norm;

        try
        {
            tf_list_ptr->transformPose(robot_base_frame, pose, poseout);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_ERROR("Tf error: %s", ex.what());
            return false;
        }

        it.transforms[0].translation.x = poseout.pose.position.x;
        it.transforms[0].translation.y = poseout.pose.position.y;
        it.transforms[0].translation.z = poseout.pose.position.z;

        it.transforms[0].rotation.x = poseout.pose.orientation.x;
        it.transforms[0].rotation.y = poseout.pose.orientation.y;
        it.transforms[0].rotation.z = poseout.pose.orientation.z;
        it.transforms[0].rotation.w = poseout.pose.orientation.w;

        //ROS_INFO_COND(debug, PRINTF_RED "Point Transformed: [%.2f,%.2f,%.2f]", poseout.pose.position.x, poseout.pose.position.y, poseout.pose.position.z);
    }
    int i = 0;

    if (!goals_vector_bl_frame.empty())
    {
        for (auto it = goals_vector_bl_frame.begin(); it != goals_vector_bl_frame.end(); it++)
        {
            ++i;
            if (i < last)
                continue;

            if (it == goals_vector_bl_frame.end() - 1)
            {
                currentGoal = *it;
                currentGoalVec.x = currentGoal.transforms[0].translation.x;
                currentGoalVec.y = currentGoal.transforms[0].translation.y;
                currentGoalVec.z = currentGoal.transforms[0].translation.z;
                if (!theta3D.isInside(currentGoalVec))
                {
                    last = 0;
                    action_result.arrived = false;
                    execute_path_srv_ptr->setAborted(action_result, "Preempted goal because global path does not fit into local workspace");
                    navigation3DClient->cancelAllGoals();
                    
                    /*if(goals_vector_bl_frame.size()==1)
                        return false;

                    currentGoal = *(it-1);//TODO COMPROBAR QUE NO ES UNA TRAYECTORIA DE UN UNICO PUNTO
                    last = i-1;

                }else{
                    last=i;
                }*/
                }
                //ROS_INFO_COND(debug, PRINTF_CYAN "Local Planner 3D: End of global trajectory queu");
                break;
            }
            else
            {

                currentGoal = *(it + 1);
                currentGoalVec.x = currentGoal.transforms[0].translation.x;
                currentGoalVec.y = currentGoal.transforms[0].translation.y;
                currentGoalVec.z = currentGoal.transforms[0].translation.z;
                ROS_INFO_COND(debug, "Local Planner 3D: i: Current goal bl frame: i: %d, last: %d [%.6f, %.6f, %.6f]", i, last, currentGoalVec.x, currentGoalVec.y, currentGoalVec.z);
                //ROS_INFO_COND(debug, "Local Planner 3D: i: %d Local goal map frame: [%.2f, %.2f, %.2f]", i, currentGoal.transforms[0].translation.x, currentGoal.transforms[0].translation.y, currentGoal.transforms[0].translation.z);
                if (!theta3D.isInside(currentGoalVec))
                {
                    currentGoal = *it;
                    ROS_INFO_COND(debug, PRINTF_CYAN "Local Planner 3D: Passing Local Goal: [%.6f, %.6f, %.6f]", currentGoal.transforms[0].translation.x, currentGoal.transforms[0].translation.y, currentGoal.transforms[0].translation.z);
                    last = i;
                    break;
                }
            }
        }

        localGoal.x = currentGoal.transforms[0].translation.x;
        localGoal.y = currentGoal.transforms[0].translation.y;
        localGoal.z = currentGoal.transforms[0].translation.z;

        //ROS_INFO_COND(debug, "i: %d Local goal bl frame: [%.2f, %.2f, %.2f]", i, localGoal.x, localGoal.y, localGoal.z);
        //ROS_INFO_COND(debug, "Local goalmap frame: [%.2f, %.2f, %.2f]", currentGoal.transforms[0].translation.x, currentGoal.transforms[0].translation.y, currentGoal.transforms[0].translation.z);

        return true;
    }
    else
    {
        return false;
    }
}
bool LocalPlanner::pointInside(geometry_msgs::Vector3 p)
{
    geometry_msgs::Vector3 robot_pose = getTfMapToRobot().transform.translation;
    // ROS_INFO_COND(debug, "Robot pose: [%.2f, %.2f, %.2f]", robot_pose.x, robot_pose.y, robot_pose.z);
    if (p.x > ws_x_max || p.x < ws_x_min ||
        p.y > ws_y_max || p.y < ws_y_min ||
        p.z > ws_z_max || p.z < ws_z_min)
    {
        return false;
    }
    else
    {
        return true;
    }
}
geometry_msgs::Point LocalPlanner::makePoint(const geometry_msgs::Vector3 &vec)
{

    geometry_msgs::Point p;
    p.x = vec.x;
    p.y = vec.y;
    p.z = vec.z;

    return p;
}
geometry_msgs::TransformStamped LocalPlanner::getTfMapToRobot()
{
    geometry_msgs::TransformStamped ret;

    try
    {
        ret = tfBuffer->lookupTransform(world_frame, robot_base_frame, ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
    }

    return ret;
}

void LocalPlanner::publishTrajMarker2D()
{
    //!This is done to clear out the previous markers
    waypointsMarker.action = RVizMarker::DELETEALL;
    lineMarker.action = RVizMarker::DELETEALL;

    visMarkersPublisher.publish(lineMarker);
    visMarkersPublisher.publish(waypointsMarker);

    lineMarker.points.clear();
    waypointsMarker.points.clear();

    lineMarker.action = RVizMarker::ADD;
    waypointsMarker.action = RVizMarker::ADD;

    lineMarker.header.stamp = ros::Time::now();
    waypointsMarker.header.stamp = ros::Time::now();

    geometry_msgs::Point p;

    for (int i = 0; i < localTrajectory.points.size(); i++)
    {
        p.x = localTrajectory.points[i].transforms[0].translation.x;
        p.y = localTrajectory.points[i].transforms[0].translation.y;

        lineMarker.points.push_back(p);
        waypointsMarker.points.push_back(p);
    }

    visMarkersPublisher.publish(lineMarker);
    visMarkersPublisher.publish(waypointsMarker);
}
void LocalPlanner::publishTrajMarker3D() //? DONE 3D
{
    //!This is done to clear out the previous markers
    waypointsMarker.action = RVizMarker::DELETEALL;
    lineMarker.action = RVizMarker::DELETEALL;

    visMarkersPublisher.publish(lineMarker);
    visMarkersPublisher.publish(waypointsMarker);

    lineMarker.points.clear();
    waypointsMarker.points.clear();

    lineMarker.action = RVizMarker::ADD;
    waypointsMarker.action = RVizMarker::ADD;

    lineMarker.header.stamp = ros::Time::now();
    waypointsMarker.header.stamp = ros::Time::now();

    geometry_msgs::Transform robot_pos; // = getTfMapToRobot().transform;
    robot_pos.translation.x = 0;
    robot_pos.translation.y = 0;
    robot_pos.translation.z = 0;

    if (traj_dest_frame != robot_base_frame)
    {
        try
        {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = robot_base_frame;
            pose.pose.orientation.w = 1;
            tf_list_ptr->waitForTransform(traj_dest_frame, robot_base_frame, ros::Time::now(), ros::Duration(1));
            tf_list_ptr->transformPose(traj_dest_frame, pose, pose);
            robot_pos.translation.x = pose.pose.position.x;
            robot_pos.translation.y = pose.pose.position.y;
            robot_pos.translation.z = pose.pose.position.z;
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("Couldn't transform points %s", ex.what());
        }
    }

    lineMarker.points.push_back(makePoint(robot_pos.translation));
    waypointsMarker.points.push_back(makePoint(robot_pos.translation));

    for (auto it : localTrajectory.points)
    {
        lineMarker.points.push_back(makePoint(it.transforms[0].translation));
        waypointsMarker.points.push_back(makePoint(it.transforms[0].translation));
    }

    visMarkersPublisher.publish(lineMarker);
    visMarkersPublisher.publish(waypointsMarker);
}

// JAC: IT IS NOT NEEDED
// void LocalPlanner::buildAndPubTrayectory2D()
// {
//     trajectory_msgs::MultiDOFJointTrajectoryPoint goal_temp;
//     geometry_msgs::Transform temp1;
//     //La trayectoria se obtiene en el frame del local costmap, que tiene la misma orientacion que el map pero esta centrado en el base_link

//     ROS_INFO_COND(debug, "Clearing local trajectory");
//     localTrajectory.points.clear();

//     if (number_of_points > 1)
//     {
//         theta2D. InAdvance(localTrajectory, getTfMapToRobot().transform);
//     }
//     else
//     {
//         getTrajectoryYawFixed(localTrajectory, 0);
//     }

//     ROS_INFO_COND(debug, "Got traj");

//     for (size_t i = 0; i < localTrajectory.points.size(); i++)
//     {
//         localTrajectory.points[i].transforms[0].translation.x += localCostMapInflated.info.origin.position.x;
//         localTrajectory.points[i].transforms[0].translation.y += localCostMapInflated.info.origin.position.y;
//     }
//     ROS_INFO_COND(debug, "After for loop");
//     localGoal.x += localCostMapInflated.info.origin.position.x;
//     localGoal.y += localCostMapInflated.info.origin.position.y;

//     temp1.translation.x = localGoal.x;
//     temp1.translation.y = localGoal.y;

//     temp1.rotation = globalTrajectory.points[startIter].transforms[0].rotation;

//     goal_temp.transforms.resize(1, temp1);

//     localTrajectory.points.push_back(goal_temp);

//     localTrajectory.header.stamp = ros::Time::now();
//     trajPub.publish(localTrajectory);
//     //ROS_WARN_THROTTLE(1, "Average dist 2 obstacles: %.2f", theta2D.getAvDist2Obs());
//     publishTrajMarker2D();
// }
void LocalPlanner::buildAndPubTrayectory3D()
{
    ROS_INFO_COND(debug, "Clearing local trajectory");
    localTrajectory.points.clear();
    double yaw = atan2(localGoal.y, localGoal.x);
    ROS_INFO_COND(debug, "Yaw fixed");

    theta3D.getTrajectoryYawFixed(localTrajectory, yaw);

    //if (number_of_points > 1)
    //{
    //    ROS_INFO_COND(debug, "Yaw in Advance");
    //    geometry_msgs::Transform tf;
    //    tf.rotation.w = 1;
    //    tf.translation.x = 0;
    //    tf.translation.y = 0;
    //    tf.translation.z = 0;
    //
    //    theta3D.getTrajectoryYawInAdvance(localTrajectory, tf);
    //}
    //else
    //{
    //
    //}
    //
    localTrajectory.header.stamp = ros::Time::now();

    if (traj_dest_frame != robot_base_frame)
    {
        if (!transformTrajectoryToFrame(traj_dest_frame))
        {
            ROS_ERROR("Local Planner 3D: Impossible to transform trajectory to frame %s", traj_dest_frame.c_str());
            return;
        }
    }

    trajPub.publish(localTrajectory);
    publishTrajMarker3D();
}
bool LocalPlanner::transformTrajectoryToFrame(std::string dest_frame)
{
    geometry_msgs::PoseStamped in, out;
    trajectory_msgs::MultiDOFJointTrajectory odom_traj;
    odom_traj.header = localTrajectory.header;

    odom_traj.points.clear();
    trajectory_msgs::MultiDOFJointTrajectoryPoint point;
    point.transforms.resize(1);
    try
    {
        for (auto it : localTrajectory.points)
        {

            in.header.frame_id = robot_base_frame;
            in.header.stamp = ros::Time::now();
            in.pose.position.x = it.transforms[0].translation.x;
            in.pose.position.y = it.transforms[0].translation.y;
            in.pose.position.z = it.transforms[0].translation.z;

            in.pose.orientation.x = it.transforms[0].rotation.x;
            in.pose.orientation.y = it.transforms[0].rotation.y;
            in.pose.orientation.z = it.transforms[0].rotation.z;
            in.pose.orientation.w = it.transforms[0].rotation.w;
            ROS_INFO("IN: [%.2f, %.2f, %.2f]", it.transforms[0].translation.x, it.transforms[0].translation.y, it.transforms[0].translation.z);
            tf_list_ptr->transformPose(dest_frame, in, out);

            point.transforms[0].translation.x = out.pose.position.x;
            point.transforms[0].translation.y = out.pose.position.y;
            point.transforms[0].translation.z = out.pose.position.z;

            point.transforms[0].rotation.x = out.pose.orientation.x;
            point.transforms[0].rotation.y = out.pose.orientation.y;
            point.transforms[0].rotation.z = out.pose.orientation.z;
            point.transforms[0].rotation.w = out.pose.orientation.w;
            odom_traj.points.push_back(point);
            ROS_INFO("IN: [%.2f, %.2f, %.2f]", point.transforms[0].translation.x, point.transforms[0].translation.y, point.transforms[0].translation.z);
        }
        localTrajectory = odom_traj;
        localTrajectory.header.frame_id = traj_dest_frame;
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("Impossible to transfrom trajectory from %s to %s: %s", robot_base_frame.c_str(), dest_frame.c_str(), ex.what());
        return false;
    }

    return true;
}
//Auxiliar functions
void LocalPlanner::showTime(string message, struct timeb st, struct timeb ft)
{
    float seconds, milliseconds;

    seconds = ft.time - st.time - 1;

    milliseconds = (1000 - st.millitm) + ft.millitm;
    cout << message << (milliseconds + seconds * 1000) << " ms" << endl;
}
//Add the occupied borders around received costmap
void LocalPlanner::inflateCostMap()
{
    //Free the data field and populate the header and info with the corresponding information
    localCostMapInflated.data.resize(0);

    localCostMapInflated.header.frame_id = localCostMap.header.frame_id;
    localCostMapInflated.header.seq = localCostMap.header.seq;
    localCostMapInflated.header.stamp = ros::Time(0);

    localCostMapInflated.info.height = localCostMap.info.height + std::round(2 * localCostMapInflationY / map_resolution);
    localCostMapInflated.info.width = localCostMap.info.width + std::round(2 * localCostMapInflationX / map_resolution);

    localCostMapInflated.info.resolution = map_resolution;

    localCostMapInflated.info.origin.position.x = localCostMap.info.origin.position.x - localCostMapInflationX;
    localCostMapInflated.info.origin.position.y = localCostMap.info.origin.position.y - localCostMapInflationY;

    float l = localCostMapInflated.info.width * localCostMapInflationY / map_resolution;
    //iterator to know where we are
    int iter = 0;
    //Upper border
    for (int i = 0; i < l; i++)
    {
        localCostMapInflated.data.push_back(100);
    } //Costmap from left border to right border, from upper original limit to lower original limit
    for (int i = 0; i < localCostMap.info.height; i++)
    {
        for (int j = 0; j < localCostMapInflationX / map_resolution; j++)
            localCostMapInflated.data.push_back(100);

        for (int k = 0; k < localCostMap.info.width; k++)
        {
            localCostMapInflated.data.push_back(localCostMap.data[iter]);
            iter++;
        }
        for (int l = 0; l < localCostMapInflationX / map_resolution; l++)
            localCostMapInflated.data.push_back(100);
    }
    //Lower boder
    for (int i = 0; i < l; i++)
        localCostMapInflated.data.push_back(100);
}
//Set to 0 the positions near the local goal in the border
void LocalPlanner::freeLocalGoal()
{
    //localGoal is a Vector3
    //This function free space around localGoal in the local costmap border

    /**
     * En esta primera version se consideran los 8 subcasos a pelo
     * * Sea w la anchura sin el borde inflado y h la altura sin el borde inflado
     * * Sea W la anchura con el borde inflado y H la altura con el borde inflado (W = w+2*infladoHorizontal, H = h+2*infladoVertical)
     * ! _ _ _ _ __ _ _ _ _
     * ! |                 |
     * ! | 1      2      3 |
     * ! |  |-----------|  |
     * ! |  |           |  |
     * ! |  |           |  |
     * ! | 8|           |4 |
     * ! |  |           |  |
     * ! |  |           |  |
     * ! |  |-----------|  |
     * ! |7       6      5 |
     * ! |_ _ _ _ _ _ _ _ _|
     * 
     * El dibujito deberia estar centrado y ser simetrico
     * ?1,3,5 y 7: Esquinas: En estos casos se libera espeacio horizontal y vertical
     * ?2,4,6 y 8: Bordes laterales: En estos casos liberamos la franja vertical/horizontal correspondiente segun sean los casos 8,4 o 2,6
     * 
    **/

    //First we do is to detect in which case we are
    //Primero: Esquinas: modulo de las componentes x e y mayor que el w y h
    //Luego si no es una esquina
    // TODO: Lo mas eficiente no es esto, sino comprobar primero los casos mas probables, que serian las 4 paredes, y luego las 4 esquinas)

    //Para todos los bucles siempre es igual
    // ! i: Numero de fila
    // ! j: columna
    int st, end;
    ROS_INFO("Local Goal: [%.2f, %.2f]", localGoal.x, localGoal.y);
    // ROS_INFO_COND(debug, PRINTF_GREEN "1");
    if (localGoal.y > localCostMapInflated.info.height * map_resolution - localCostMapInflationY)
    {
        //Esquina 1 o 3 o borde 2
        if (localGoal.x < localCostMapInflationX) //1
        {
            // ROS_INFO_COND(debug, PRINTF_GREEN "2");
            for (int i = localCostMapInflated.info.height - 2 * localCostMapInflationY / map_resolution; i < localCostMapInflated.info.height - localCostMapInflationY / map_resolution; i++)
                for (int j = 0; j < localCostMapInflationX / map_resolution; j++)
                    localCostMapInflated.data[localCostMapInflated.info.width * i + j] = 0;
            // ROS_INFO_COND(debug, PRINTF_GREEN "3");
            for (int i = localCostMapInflated.info.height - localCostMapInflationY / map_resolution; i < localCostMapInflated.info.height; i++)
                for (int j = 0; j < 2 * localCostMapInflationX / map_resolution; j++)
                    localCostMapInflated.data[localCostMapInflated.info.width * i + j] = 0;
        }
        else if (localGoal.x > localCostMapInflated.info.width * map_resolution - localCostMapInflationX) //5
        {
            // ROS_INFO_COND(debug, PRINTF_GREEN "4");
            for (int i = localCostMapInflated.info.height - 2 * localCostMapInflationY / map_resolution; i < localCostMapInflated.info.height - localCostMapInflationY / map_resolution; i++)
                for (int j = localCostMapInflated.info.width - localCostMapInflationX / map_resolution; j < localCostMapInflated.info.width; j++)
                    localCostMapInflated.data[localCostMapInflated.info.width * i + j] = 0;
            // ROS_INFO_COND(debug, PRINTF_GREEN "5");
            for (int i = localCostMapInflated.info.height - localCostMapInflationY / map_resolution; i < localCostMapInflated.info.height; i++)
                for (int j = localCostMapInflated.info.width - 2 * localCostMapInflationX / map_resolution; j < localCostMapInflated.info.width; j++)
                    localCostMapInflated.data[localCostMapInflated.info.width * i + j] = 0;
        }
        else //Borde 2
        {

            st = (localGoal.x - border_space) / map_resolution;
            if (st < 0)
                st = 0;

            end = (localGoal.x + border_space) / map_resolution;
            if (end > ws_x_max / map_resolution)
                end = ws_x_max / map_resolution;

            // ROS_INFO_COND(debug, PRINTF_GREEN "6");
            for (int i = localCostMapInflated.info.height - localCostMapInflationY / map_resolution; i < localCostMapInflated.info.height; i++)
                for (int j = st; j < end; j++)
                    localCostMapInflated.data[i * localCostMapInflated.info.width + j] = 0;
            // ROS_INFO_COND(debug, PRINTF_GREEN "7");
        }
    }
    else if (localGoal.y < localCostMapInflationY)
    {
        // ROS_INFO_COND(debug, PRINTF_GREEN "8");                                                      //Esquina 3 o 1 o borde 6
        if (localGoal.x > localCostMapInflated.info.width * map_resolution - localCostMapInflationX) //3
        {
            // ROS_INFO_COND(debug, PRINTF_GREEN "9");
            for (int i = 0; i < localCostMapInflationY / map_resolution; i++)
                for (int j = localCostMapInflated.info.width - 2 * localCostMapInflationX / map_resolution; j < localCostMapInflated.info.width; j++)
                    localCostMapInflated.data[localCostMapInflated.info.width * i + j] = 0;
            // ROS_INFO_COND(debug, PRINTF_GREEN "10");
            for (int i = localCostMapInflationY / map_resolution; i < 2 * localCostMapInflationY / map_resolution; i++)
                for (int j = localCostMapInflated.info.width - localCostMapInflationX / map_resolution; j < localCostMapInflated.info.width; j++)
                    localCostMapInflated.data[localCostMapInflated.info.width * i + j] = 0;
            // ROS_INFO_COND(debug, PRINTF_GREEN "11");
        }
        else if (localGoal.x < localCostMapInflationX) //1
        {
            // ROS_INFO_COND(debug, PRINTF_GREEN "12");
            for (int i = 0; i < localCostMapInflationY / map_resolution; i++) //Filas
                for (int j = 0; j < 2 * localCostMapInflationX / map_resolution; j++)
                    localCostMapInflated.data[localCostMapInflated.info.width * i + j] = 0;
            // ROS_INFO_COND(debug, PRINTF_GREEN "13");
            for (int i = localCostMapInflationY / map_resolution; i < 2 * localCostMapInflationY / map_resolution; i++)
                for (int j = 0; j < localCostMapInflationX / map_resolution; j++)
                    localCostMapInflated.data[localCostMapInflated.info.width * i + j] = 0;
        }
        else //Borde 6
        {
            // ROS_INFO_COND(debug, PRINTF_GREEN "14");

            st = (localGoal.x - border_space) / map_resolution;
            if (st < 0)
                st = 0;

            end = (localGoal.x + border_space) / map_resolution;
            if (end > ws_x_max / map_resolution)
                end = ws_x_max / map_resolution;

            for (int i = 0; i < localCostMapInflationY / map_resolution; i++)
                for (int j = st; j < end; j++)
                    localCostMapInflated.data[i * localCostMapInflated.info.width + j] = 0;
            // ROS_INFO_COND(debug, PRINTF_GREEN "15");
        }
    } //Si hemos llegado hasta aqui sabemos que la y esta dentro del costmap original
    else if (localGoal.x < localCostMapInflationX)
    { //Borde 8
        //Se recorren las filas desde la primera hasta la ultima y se ponen a 0 los N primeros valores de cada fila(numero de columnas infladas)
        // ROS_INFO_COND(debug, PRINTF_GREEN "16");

        st = (localGoal.y - border_space) / map_resolution;
        if (st < 0)
            st = 0;

        end = (localGoal.y + border_space) / map_resolution;
        if (end > ws_y_max / map_resolution)
            end = ws_y_max / map_resolution;

        for (int i = st; i < end; i++)
            for (int j = 0; j < localCostMapInflationX / map_resolution; j++)
                localCostMapInflated.data[localCostMapInflated.info.width * i + j] = 0;
        // ROS_INFO_COND(debug, PRINTF_GREEN "17");
    }
    else if (localGoal.x > localCostMapInflated.info.width * map_resolution - localCostMapInflationX)
    { //Borde 4
        //Se recorren las filas desde la primera hasta la ultima y se ponen a 0 los N ultimos valores de cada fila (numero de columnas infladas)
        // ROS_INFO_COND(debug, PRINTF_GREEN "18");
        st = (localGoal.y - border_space) / map_resolution;
        end = (localGoal.y + border_space) / map_resolution;
        if (st < 0)
            st = 0;
        if (end > ws_y_max / map_resolution)
            end = ws_y_max / map_resolution;

        for (int i = st; i < end; i++)
            for (int j = localCostMapInflated.info.width - localCostMapInflationX / map_resolution; j < localCostMapInflated.info.width; j++)
                localCostMapInflated.data[localCostMapInflated.info.width * i + j] = 0;

        // ROS_INFO_COND(debug, PRINTF_GREEN "19");
    }
    else
    {
        ROS_INFO_COND(debug, PRINTF_BLUE "Local Planner: No need to free space in the border, local goal inside original local costmap");
    }
}

void LocalPlanner::configureAlgorithm(const std::string &algorithm_name, const std::string &_heuristic){

        float ws_x, ws_y, ws_z;

        this->get_parameter("world_size_x", ws_x);
        this->get_parameter("world_size_y", ws_y);
        this->get_parameter("world_size_z", ws_z);
        this->get_parameter("resolution", resolution_);
        this->get_parameter("inflate_map", inflate_);
        this->get_parameter("use3d", use3d_);

        world_size_.x = std::floor(ws_x / resolution_);
        world_size_.y = std::floor(ws_y / resolution_);
        world_size_.z = std::floor(ws_z / resolution_);


        if( algorithm_name == "astar" ){
            RCLCPP_INFO(this->get_logger(),"Using A*");
            algorithm_.reset(new Planners::AStar(use3d_));
        }else if( algorithm_name == "costastar" ){
            RCLCPP_INFO(this->get_logger(),"Using Cost Aware A*");
            algorithm_.reset(new Planners::AStarM1(use3d_));
        }else if( algorithm_name == "astarsafetycost" ){
            RCLCPP_INFO(this->get_logger(),"Using A* Safety Cost");
            algorithm_.reset(new Planners::AStarM2(use3d_));    
        }else if ( algorithm_name == "thetastar" ){
            RCLCPP_INFO(this->get_logger(),"Using Theta*");
            algorithm_.reset(new Planners::ThetaStar(use3d_));
        }else if ( algorithm_name == "costhetastar" ){
            RCLCPP_INFO(this->get_logger(),"Using Cost Aware Theta* ");
            algorithm_.reset(new Planners::ThetaStarM1(use3d_));
        }else if ( algorithm_name == "thetastarsafetycost" ){
            RCLCPP_INFO(this->get_logger(),"Using Theta* Safety Cost");
            algorithm_.reset(new Planners::ThetaStarM2(use3d_));
        }else if( algorithm_name == "lazythetastar" ){
            RCLCPP_INFO(this->get_logger(),"Using LazyTheta*");
            algorithm_.reset(new Planners::LazyThetaStar(use3d_));
        }else if( algorithm_name == "costlazythetastar"){
            RCLCPP_INFO(this->get_logger(),"Using Cost Aware LazyTheta*");
            algorithm_.reset(new Planners::LazyThetaStarM1(use3d_));
        }else if( algorithm_name == "costlazythetastarmodified"){
            RCLCPP_INFO(this->get_logger(),"Using Cost Aware LazyTheta*");
            algorithm_.reset(new Planners::LazyThetaStarM1Mod(use3d_));
        }else if( algorithm_name == "lazythetastarsafetycost"){
            RCLCPP_INFO(this->get_logger(),"Using LazyTheta* Safety Cost");
            algorithm_.reset(new Planners::LazyThetaStarM2(use3d_));
        }else{
            RCLCPP_WARN(this->get_logger(),"Wrong algorithm name parameter. Using ASTAR by default");
            algorithm_.reset(new Planners::AStar(use3d_));
        }

        algorithm_->setWorldSize(world_size_, resolution_);

        configureHeuristic(_heuristic);

        RCLCPP_INFO(this->get_logger(),"Using discrete world size: [%d, %d, %d]", world_size_.x, world_size_.y, world_size_.z);
        RCLCPP_INFO(this->get_logger(),"Using resolution: [%f]", resolution_);

        if(inflate_){
            double inflation_size;
            this->get_parameter("inflation_size", inflation_size);
            inflation_steps_ = std::round(inflation_size / resolution_);
            RCLCPP_INFO(this->get_logger(),"Inflation size %.2f, using inflation step %d", inflation_size, inflation_steps_);
        }
        algorithm_->setInflationConfig(inflate_, inflation_steps_);


        double cost_scaling_factor, robot_radius;

        this->get_parameter("cost_scaling_factor", cost_scaling_factor);
        this->get_parameter("robot_radius", robot_radius);
        
        m_grid3d_->setCostParams(cost_scaling_factor, robot_radius);
        
        std::string frame_id;
        // lnh_.param("frame_id", frame_id, std::string("map"));		
        this->get_parameter("frame_id", frame_id);

        configMarkers(algorithm_name, frame_id, resolution_);

        // JAC: This is not needed
        // this->get_parameter("save_data_file", save_data_);
        // this->get_parameter("data_folder", data_folder_);

        // if(save_data_)
        //     RCLCPP_INFO_STREAM(this->get_logger(),"Saving path planning data results to " << data_folder_);

        // JAC: This should be done when the point cloud is received becuase input_map_=2 as we use point cloud. 
        //0: no map yet
        //1: using occupancy
        //2: using cloud
        if( input_map_ == 1 ){
            Planners::utils::configureWorldFromOccupancyWithCosts(occupancy_grid_, *algorithm_);
        }else if( input_map_ == 2 ){
            Planners::utils::configureWorldFromPointCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(cloud_), *algorithm_, resolution_);
            Planners::utils::configureWorldCosts(*m_grid3d_, *algorithm_);
        }

        //Algorithm specific parameters. Its important to set line of sight after configuring world size(it depends on the resolution)
        float sight_dist, cost_weight;
                                                                             //
        this->get_parameter("max_line_of_sight_distance", sight_dist);
        this->get_parameter("cost_weight", cost_weight);

        algorithm_->setMaxLineOfSight(sight_dist);
        algorithm_->setCostFactor(cost_weight);

        this->get_parameter("overlay_markers", overlay_markers_);

    }

} // namespace PathPlanners
