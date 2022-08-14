/*
Local Planner using the LazyThetaStar Class 
José Antonio Cobano 
 */

#include <LocalPlanner.hpp>

namespace PathPlanners
{ 
    
LocalPlanner::LocalPlanner(): rclcpp::Node("local_planner")
{
    // nh.reset(new ros::NodeHandle("~"));
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
    this->declare_parameter<std::string>("data_folder", std::string("planing_data.txt"));
    this->declare_parameter<float>("max_line_of_sight_distance", (float)1000.0);
    this->declare_parameter<float>("cost_weight", (float)0.0);
    this->declare_parameter<bool>("overlay_markers", (bool)false);
    this->declare_parameter<double>("inflation_size", 0.5);
    
    this->get_parameter("algorithm_name", algorithm_name); //JAC: The algorithm should be fixed.
    this->get_parameter("heuristic_name", heuristic_);    

    configureAlgorithm(algorithm_name, heuristic_);
    
    tf_list_ptr.reset(new tf::TransformListener(ros::Duration(5)));
    // tfBuffer = tfBuffer_;
    tfBuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

    configTopics();
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
    occGoalCnt = 0;
    badGoal = 0;
    last = 0;
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

// void LocalPlanner::pointsSub(const PointCloud::ConstPtr &points)
void LocalPlanner::pointsSub(const sensor_msgs::msg::PointCloud2::SharedPtr points)
{
    
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
    local_map_sub_.reset();    

}
// JAC: Removed when topics and/or services are confirmed about the reception of the global goal.
// void LocalPlanner::executePathPreemptCB()
// {
//     // ROS_INFO_COND(debug, "Goal Preempted");
//     RCLCPP_INFO(this->get_logger(),"Goal Preempted");
//     execute_path_srv_ptr->setPreempted(); // set the action state to preempted

//     // JAC: This should be update in Aerostack2.
//     navigation3DClient->cancelAllGoals();

//     resetFlags();
//     clearMarkers();
// }
// JAC: Removed when topics and/or services are confirmed about the reception of the global goal.
// void LocalPlanner::executePathGoalServerCB() // Note: "Action" is not appended to exe here
// {
//     resetFlags();
//     clearMarkers();
//     start_time = ros::Time::now();

//     // ROS_INFO_COND(debug, "Local Planner Goal received in action server mode");
//     RCLCPP_INFO(this->get_logger(),"Local Planner Goal received in action server mode");
//     //upo_actions::ExecutePathGoalConstPtr path_shared_ptr;
//     auto path_shared_ptr = execute_path_srv_ptr->acceptNewGoal();
//     globalTrajectory = path_shared_ptr->path;
//     auto size = globalTrajectory.points.size();

//     goals_vector = globalTrajectory.points;
//     goal3D.global_goal.pose.position.x = globalTrajectory.points.at(globalTrajectory.points.size() - 1).transforms[0].translation.x;
//     goal3D.global_goal.pose.position.y = globalTrajectory.points.at(globalTrajectory.points.size() - 1).transforms[0].translation.y;
//     goal3D.global_goal.pose.position.z = globalTrajectory.points.at(globalTrajectory.points.size() - 1).transforms[0].translation.z;
        
//     goal3D.global_goal.pose.orientation.x = globalTrajectory.points.at(globalTrajectory.points.size() - 1).transforms[0].rotation.x;
//     goal3D.global_goal.pose.orientation.y = globalTrajectory.points.at(globalTrajectory.points.size() - 1).transforms[0].rotation.y;
//     goal3D.global_goal.pose.orientation.z = globalTrajectory.points.at(globalTrajectory.points.size() - 1).transforms[0].rotation.z;
//     goal3D.global_goal.pose.orientation.w = globalTrajectory.points.at(globalTrajectory.points.size() - 1).transforms[0].rotation.w;
//     // JAC: Update --> This is done by publishing the local path.
//     // navigation3DClient->sendGoal(goal3D);

//     // ROS_INFO_COND(debug, "Local Planner: Processed goal message");
// }
void LocalPlanner::configTopics()
{
    // JAC: Subscription to the Point Cloud.
    local_map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/points", 1, std::bind(&LocalPlanner::pointsSub, this, std::placeholders::_1));
    // JAC: This node is not published.
    // occupancy_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/grid", 1, std::bind(&HeuristicPlannerROS::occupancyGridCallback, this, std::placeholders::_1));

    point_markers_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("path_points_markers", 1);
    line_markers_pub_  = this->create_publisher<visualization_msgs::msg::Marker>("path_line_markers", 1);
    // JAC: This has to be change. It has to publish to the corresponding message in AS2 (FollowPath) TODO
    // trajPub = this->create_publisher<trajectory_msgs::MultiDOFJointTrajectory>("local_path", 1);
}

void LocalPlanner::plan()
{
    number_of_points = 0; //Reset variable

    // // JAC: Review and modify in AS2. How do we know if execute paht is active?
    // if (!execute_path_srv_ptr->isActive())
    // {
    //     clearMarkers();
    //     return;
    // }

    // JAC: Here the solution local path is published.
    calculatePath3D();
    // JAC: This has to be decided to update. TODO
    // seconds = finishT.time - startT.time - 1;
    // milliseconds = (1000 - startT.millitm) + finishT.millitm;
    // publishExecutePathFeedback();

    // JAC: Review and modify in AS2
    // if (*state == actionlib::SimpleClientGoalState::SUCCEEDED)
    // {
    //     clearMarkers();
    //     action_result.arrived = true;
    //     execute_path_srv_ptr->setSucceeded(action_result);
    //     ROS_ERROR("LocalPlanner: Goal Succed");
    //     return;
    // }
    // // JAC: Review and modify in AS2
    // else if (*state == actionlib::SimpleClientGoalState::ABORTED)
    // {
    //     ROS_INFO_COND(debug, "Goal aborted by path tracker");
    //     resetFlags();
    // }
    // // JAC: Review and modify in AS2
    // else if (*state == actionlib::SimpleClientGoalState::PREEMPTED)
    // {
    //     ROS_INFO_COND(debug, "Goal preempted by path tracker");
    //     resetFlags();
    // }
    //ROS_INFO_COND(debug, "Before start loop calculation");
}

void LocalPlanner::calculatePath3D()
{
    if (mapReceived) // It is true once a pointcloud is received.
    {
        ROS_INFO_COND(debug, PRINTF_BLUE "Local Planner 3D: Global trj received and pointcloud received");
        mapReceived = false;

        //TODO : Set to robot pose to the center of the workspace?
        // if (theta3D.setValidInitialPosition(robotPose) || theta3D.searchInitialPosition3d(initialSearchAround))
        // JAC: Decide if we will implement "Symetric search in nearest XY-Zup neighbours" with searchInitialPosition3d
        if (!algorithm_->detectCollision(robotPose))
        {
            ROS_INFO_COND(debug, PRINTF_MAGENTA "Local Planner 3D: Calculating local goal");

            if (calculateLocalGoal3D()) //TODO: VER CUAL SERIA EL LOCAL GOAL EN ESTE CASO
            // geometry_msgs::msg::Vector3 localGoal, robotPose; --> from old localplanner
            {
                ROS_INFO_COND(debug, PRINTF_MAGENTA "Local Planner 3D: Local Goal calculated");

                // ROS_INFO("%.6f,%.6f, %.6f", localGoal.x, localGoal.y,localGoal.z);
                // JAC: geometry_msgs::msg::Vector3 localGoal
                RCLCPP_INFO(this->get_logger(),"%.6f,%.6f, %.6f", localGoal.x, localGoal.y,localGoal.z);

                // JAC: El local_goal debe ser de este tipo para pasarlo al findpath
                // const auto discrete_goal =  Planners::utils::discretePoint(_req.goal, resolution_);

                // if (!theta3D.isInside(localGoal)) --> checkValid (Planners::utils::)
                if (!checkValid(localGoal))
                {
                    ROS_INFO("Returning, not inside :(");
                    // JAC: Decide how communicate it. TODO

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
                    // JAC: robotPose or start should be like in planner_ros_node. robotPose is the position (0,0,0)
                    // const auto discrete_start =  Planners::utils::discretePoint(_req.start, resolution_);
                    // JAC: discrete_start --> robotPose, discrete_goal --> localgoal
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
                // JAC: How publish when local goal cannot be computed three times.
                // action_result.arrived=false;
                // execute_path_srv_ptr->setAborted(action_result,"Bad goal calculated 3 times");
                badGoal = 0;
                ROS_INFO("Bad goal calculated 3 times");
            }
        }
        // JAC: No initial free position found
        else
        {
            // planningStatus.data = "No initial position found...";
            // action_result.arrived=false;
            // execute_path_srv_ptr->setAborted(action_result,"No initial position found");

            clearMarkers();
            ROS_INFO_COND(debug, "Local Planner 3D: No initial free position found");
        }
    }
}
// JAC: Decide if this is considered from AS2.
// void LocalPlanner::publishExecutePathFeedback()
// {

//     if (planningStatus.data.find("OK") > 0)
//     {
//         planningRate.data = 1000 / milliseconds;
//     }
//     else
//     {
//         planningRate.data = 0;
//     }

//     exec_path_fb.global_waypoint = waypointGoingTo;
//     exec_path_fb.planning_rate = planningRate;
//     exec_path_fb.status = planningStatus;
//     execute_path_srv_ptr->publishFeedback(exec_path_fb);
// }

// Compute the local goal from the goals_vector (global path)
// JAC: Now we can test the local planner with global goal as input from terminal.
// JAC: localgoal should be like in planner_ros_node
// JAC: const auto discrete_goal =  Planners::utils::discretePoint(_req.goal, resolution_);
bool LocalPlanner::calculateLocalGoal3D()
{

    //Okey we have our std queu with the global waypoints
    //? 1. Take front and calculate if it inside the workspace
    //? 2. if it
    geometry_msgs::msg::Vector3 currentGoalVec;
    //geometry_msgs::TransformStamped robot = getTfMapToRobot();

    // JAC: We have to initialize the global goal --> We have to know the type of message
    // JAC: Once defined the type of message, we can compute the local goal.
    // goals_vector_bl_frame = goals_vector;

    geometry_msgs::msg::PoseStamped pose, poseout;
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
    for (auto &it : goals_vector_bl_frame) // JAC: The bucle has to be updated from the type of message of the gloabl goal
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
    // JAC: In our case, do we receive only one waypoint? I mean, a global path to compute the global goal or a global goal?
    // JAC: Update after deciding on the previous sentence.
    if (!goals_vector_bl_frame.empty()) // JAC: It has to be updated from the type of message of the gloabl goal
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
                // if (!theta3D.isInside(currentGoalVec))
                if (!checkValid(currentGoalVec))
                {
                    last = 0;
                    // JAC: This depends on the interfaces. TODO
                    // action_result.arrived = false;
                    // execute_path_srv_ptr->setAborted(action_result, "Preempted goal because global path does not fit into local workspace");
                    
                    // JAC: This is of the old localplanner
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
                // ROS_INFO_COND(debug, "Local Planner 3D: i: Current goal bl frame: i: %d, last: %d [%.6f, %.6f, %.6f]", i, last, currentGoalVec.x, currentGoalVec.y, currentGoalVec.z);
                //ROS_INFO_COND(debug, "Local Planner 3D: i: %d Local goal map frame: [%.2f, %.2f, %.2f]", i, currentGoal.transforms[0].translation.x, currentGoal.transforms[0].translation.y, currentGoal.transforms[0].translation.z);
                // if (!theta3D.isInside(currentGoalVec))
                if (!checkValid(currentGoalVec))
                {
                    currentGoal = *it;
                    ROS_INFO_COND(debug, PRINTF_CYAN "Local Planner 3D: Passing Local Goal: [%.6f, %.6f, %.6f]", currentGoal.transforms[0].translation.x, currentGoal.transforms[0].translation.y, currentGoal.transforms[0].translation.z);
                    last = i;
                    break;
                }
            }
        }

        // JAC: geometry_msgs::msg::Vector3 localGoal;
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
// JAC: This is used in publishTrajMarker3D. Do it later.
// geometry_msgs::Point LocalPlanner::makePoint(const geometry_msgs::Vector3 &vec)
// {

//     geometry_msgs::Point p;
//     p.x = vec.x;
//     p.y = vec.y;
//     p.z = vec.z;

//     return p;
// }

// JAC: Used in calculateLocalGoal3D
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

// JAC: This shoud be done to show the computed local path, but after deciding in calculatePath3D.
// void LocalPlanner::publishTrajMarker3D() //? DONE 3D
// {
//     //!This is done to clear out the previous markers
//     waypointsMarker.action = RVizMarker::DELETEALL;
//     lineMarker.action = RVizMarker::DELETEALL;

//     visMarkersPublisher.publish(lineMarker);
//     visMarkersPublisher.publish(waypointsMarker);

//     lineMarker.points.clear();
//     waypointsMarker.points.clear();

//     lineMarker.action = RVizMarker::ADD;
//     waypointsMarker.action = RVizMarker::ADD;

//     lineMarker.header.stamp = ros::Time::now();
//     waypointsMarker.header.stamp = ros::Time::now();

//     geometry_msgs::Transform robot_pos; // = getTfMapToRobot().transform;
//     robot_pos.translation.x = 0;
//     robot_pos.translation.y = 0;
//     robot_pos.translation.z = 0;

//     if (traj_dest_frame != robot_base_frame)
//     {
//         try
//         {
//             geometry_msgs::PoseStamped pose;
//             pose.header.frame_id = robot_base_frame;
//             pose.pose.orientation.w = 1;
//             tf_list_ptr->waitForTransform(traj_dest_frame, robot_base_frame, ros::Time::now(), ros::Duration(1));
//             tf_list_ptr->transformPose(traj_dest_frame, pose, pose);
//             robot_pos.translation.x = pose.pose.position.x;
//             robot_pos.translation.y = pose.pose.position.y;
//             robot_pos.translation.z = pose.pose.position.z;
//         }
//         catch (tf::TransformException &ex)
//         {
//             ROS_ERROR("Couldn't transform points %s", ex.what());
//         }
//     }

//     lineMarker.points.push_back(makePoint(robot_pos.translation));
//     waypointsMarker.points.push_back(makePoint(robot_pos.translation));

//     for (auto it : localTrajectory.points)
//     {
//         lineMarker.points.push_back(makePoint(it.transforms[0].translation));
//         waypointsMarker.points.push_back(makePoint(it.transforms[0].translation));
//     }

//     visMarkersPublisher.publish(lineMarker);
//     visMarkersPublisher.publish(waypointsMarker);
// }

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

        // JAC: This was in planne_ros_node but it should be done when the point cloud is received () because input_map_=2 as we use point cloud. 
        //0: no map yet
        //1: using occupancy
        //2: using cloud
        // if( input_map_ == 1 ){
        //     Planners::utils::configureWorldFromOccupancyWithCosts(occupancy_grid_, *algorithm_);
        // }else if( input_map_ == 2 ){
        //     Planners::utils::configureWorldFromPointCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(cloud_), *algorithm_, resolution_);
        //     Planners::utils::configureWorldCosts(*m_grid3d_, *algorithm_);
        // }

        //Algorithm specific parameters. Its important to set line of sight after configuring world size(it depends on the resolution)
        float sight_dist, cost_weight;
                                                                             //
        this->get_parameter("max_line_of_sight_distance", sight_dist);
        this->get_parameter("cost_weight", cost_weight);

        algorithm_->setMaxLineOfSight(sight_dist);
        algorithm_->setCostFactor(cost_weight);

        this->get_parameter("overlay_markers", overlay_markers_);

    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_sub_;
    // rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_sub_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr line_markers_pub_, point_markers_pub_;
    // JAC: Add trajPub
    // rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajPub;

    // JAC: Decide if topics or service will be used.
    // rclcpp::Service<heuristic_planners::srv::GetPath>::SharedPtr request_path_server_;
    // rclcpp::Service<heuristic_planners::srv::SetAlgorithm>::SharedPtr change_planner_server_;

    std::unique_ptr<Grid3d> m_grid3d_;

    std::unique_ptr<Planners::AlgorithmBase> algorithm_;
        
    visualization_msgs::msg::Marker path_line_markers_, path_points_markers_;
    
    //Parameters
    Planners::utils::Vec3i world_size_; // Discrete
    //float resolution_ = 0.2;
    float resolution_;

    bool use3d_{true};

    bool inflate_{false};
    unsigned int inflation_steps_{0};
    std::string data_folder_;
    bool overlay_markers_{0};
    unsigned int color_id_{0};
    // nav_msgs::msg::OccupancyGrid occupancy_grid_;
    // TODO?: Change type of message --> sensor_msgs::msg::PointCloud2
    pcl::PointCloud<pcl::PointXYZ> cloud_;
    //sensor_msgs::msg::PointCloud2 cloud_; //JAC

    //0: no map yet
    //1: using occupancy
    //2: using cloud
    int input_map_{0};
    std::string heuristic_;    

} // namespace PathPlanners
