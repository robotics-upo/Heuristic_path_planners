#include <iostream>

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
#include "utils/CeresOpt.hpp"

#include "Grid3D/local_grid3d.hpp"

#include <ros/ros.h>

#include <visualization_msgs/Marker.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <octomap_msgs/Octomap.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>

#include <nav_msgs/OccupancyGrid.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <heuristic_planners/GetPath.h>
#include <heuristic_planners/SetAlgorithm.h>
#include <heuristic_planners/Vec3i.h>
#include <heuristic_planners/CoordinateList.h>



/**
 * @brief Demo Class that demonstrate how to use the algorithms classes and utils 
 * with ROS 
 * 
 */
class HeuristicLocalPlannerROS
{

public:
    HeuristicLocalPlannerROS()
    {

        std::string algorithm_name;
        lnh_.param("algorithm", algorithm_name, (std::string)"astar");
        lnh_.param("heuristic", heuristic_, (std::string)"euclidean");
        
        configureAlgorithm(algorithm_name, heuristic_);

        // pointcloud_local_sub_     = lnh_.subscribe<pcl::PointCloud<pcl::PointXYZ>>("/points", 1, &HeuristicLocalPlannerROS::pointCloudCallback, this);
        // pointcloud_local_sub_     = lnh_.subscribe<sensor_msgs::PointCloud2>("/points", 1, &HeuristicLocalPlannerROS::pointCloudCallback, this);

        path_local_sub_     = lnh_.subscribe<heuristic_planners::CoordinateList>("/planner_ros_node/global_path", 1, &HeuristicLocalPlannerROS::globalPathCallback, this);
        
        //GLOBAL POSITIONING SUBSCRIBER - for SDF query
        globalposition_local_sub_ = lnh_.subscribe<geometry_msgs::PoseStamped>("/ground_truth_to_tf/pose", 1, &HeuristicLocalPlannerROS::globalPositionCallback, this);
        // pointcloud_local_sub_     = lnh_.subscribe("/points", 1, &HeuristicLocalPlannerROS::pointCloudCallback, this); //compile
        occupancy_grid_local_sub_ = lnh_.subscribe<nav_msgs::OccupancyGrid>("/grid", 1, &HeuristicLocalPlannerROS::occupancyGridCallback, this);
        network_update_sub_ = lnh_.subscribe<std_msgs::Empty>("/net_update", 1, &HeuristicLocalPlannerROS::networkUpdateCallback, this);

        // request_path_server_   = lnh_.advertiseService("request_path",  &HeuristicLocalPlannerROS::requestPathService, this); // This is in planner_ros_node.cpp and the corresponding service defined.
        change_planner_server_ = lnh_.advertiseService("set_algorithm", &HeuristicLocalPlannerROS::setAlgorithm, this);

        local_line_markers_pub_  = lnh_.advertise<visualization_msgs::Marker>("local_path_line_markers", 1);
        local_point_markers_pub_ = lnh_.advertise<visualization_msgs::Marker>("local_path_points_markers", 1);
        cloud_test  = lnh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("/cloud_PCL", 1, true);  // Defined by me to show the point cloud as pcl::PointCloud<pcl::PointXYZ
        
        networkReceivedFlag_ = 1;
        globalPathReceived_ = 0;
        timed_local_path_ = lnh_.createTimer(ros::Duration(1), &HeuristicLocalPlannerROS::localtimedCallback, this);
    }

    void plan(){
        // ROS_INFO("Call to Local Plan");
        number_of_points = 0; //Reset variable

    }


private:

    void occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr &_grid){
        ROS_INFO("Loading OccupancyGrid map...");
        Planners::utils::configureWorldFromOccupancyWithCosts(*_grid, *algorithm_);
        algorithm_->publishOccupationMarkersMap();
        occupancy_grid_local_sub_.shutdown();
        ROS_INFO("Occupancy Grid Loaded");
        occupancy_grid_ = *_grid;
        input_map_ = 1;
    }

    void globalPathCallback(const heuristic_planners::CoordinateList::ConstPtr& msg)
    {
        std::cout << "Entered global path retrieving callback" << std::endl;
        //Vaciamos la variable del path global anteriormente
        global_path_.clear();

        //Pasamos el mensaje a la variable global_path
        for (const auto& vec : msg->coordinates) {
            // Convert each your_package::Vec3i to Planners::utils::Vec3i
            Planners::utils::Vec3i vec_intermedio;
            vec_intermedio.x = vec.x;
            vec_intermedio.y = vec.y;
            vec_intermedio.z = vec.z;

            // Add the converted Vec3i to the path
            global_path_.push_back(vec_intermedio);
        }
        //Reverse waypoints order!! (GLOBAL PLANNER SENDS GOAL FIRST AND START LAST)
        std::reverse(global_path_.begin(), global_path_.end());

        //Ponemos el flag de recepción a 1
        globalPathReceived_ = 1;
        std::cout << "Global path successfully received" << std::endl;
        std::cout << global_path_ << std::endl;

    }

    void networkUpdateCallback(const std_msgs::Empty::ConstPtr& msg){
        ROS_INFO("Received new network callback");
        networkReceivedFlag_ = 1;
    }

    void globalPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
        double drone_x = resolution_ * std::round(msg->pose.position.x / resolution_);
        double drone_y = resolution_ * std::round(msg->pose.position.y / resolution_);
        double drone_z = resolution_ * std::round(msg->pose.position.z / resolution_);

        // Store as class members
        this->drone_x_ = drone_x;
        this->drone_y_ = drone_y;
        this->drone_z_ = drone_z;
    }

    void localtimedCallback(const ros::TimerEvent& event)
    {
        printf("-----TIMED CALLBACK------\n");

        // Only perform local planning if global path was received
        if(globalPathReceived_ == 1){

            auto loop_start = std::chrono::high_resolution_clock::now();
        


            // 1. Update Neural Network State (if new state available)
            if(networkReceivedFlag_ == 1)
            {
                printf("Importing new neural network state\n");
                loaded_sdf_ = torch::jit::load("/home/ros/exchange/weight_data/model.pt", c10::kCPU); 
                networkReceivedFlag_ = 0;
            }

            // 2. Update local map with the neural network information around the drone
            Planners::utils::configureLocalWorldCosts(*m_local_grid3d_, *algorithm_, drone_x_, drone_y_, drone_z_, loaded_sdf_);
            // printf("-- Exiting callback --\n");

            // 3. Calculate Local Path
            calculatePath3D();

            auto loop_end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> loop_duration = loop_end - loop_start;
            printf("TIEMPO DE LOOP: %.2f ms\n", loop_duration.count());

        }
    }

    // From lazy_theta_star_planners
    // void pointCloudCallback(const PointCloud::ConstPtr &points)
    // void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &_points)
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud)
    {
        // // ROS_INFO_COND(debug, PRINTF_MAGENTA "Collision Map Received");
        mapReceived = true; // At the end to run the calculate3Dpath after generating PointCloud?
        sensor_msgs::PointCloud2 base_cloud;

        // laser0 in bag
        // std::cout << "frame cloud: "      << cloud->header.frame_id  << std::endl;

        // Pre-cache transform for point-cloud to base frame and transform the pc
		if(!m_tfCache)
		{	
			try
			{
                // Providing ros::Time(0) will just get us the latest available transform. 
                // ros::Duration(2.0) --> Duration before timeout
                
                m_tfListener.waitForTransform(baseFrameId, cloud->header.frame_id, ros::Time(0), ros::Duration(2.0));
                // Get the transform between two frames by frame ID. 
                // lookupTransform (const std::string &target_frame, const std::string &source_frame, const ros::Time &time, StampedTransform &transform) const 
                m_tfListener.lookupTransform(baseFrameId, cloud->header.frame_id, ros::Time(0), pclTf);
				m_tfCache = true;
			}
			catch (tf::TransformException &ex)
			{
				ROS_ERROR("%s",ex.what());
				return;
			}
		}

        // base_cloud.header.frame_id is laser0 (like cloud)
        // base_cloud=*cloud;
        // base_cloud.header.frame_id = 

        // transformPointCloud (const std::string &target_frame, const tf::Transform &net_transform, const sensor_msgs::PointCloud2 &in, sensor_msgs::PointCloud2 &out)

        // Transform a sensor_msgs::PointCloud2 dataset from its frame to a given TF target frame. 
        pcl_ros::transformPointCloud(baseFrameId, pclTf, *cloud, base_cloud); // Transform pointcloud to our TF --> baseCloud
        // pcl_ros::transformPointCloud(globalFrameId, pclTf, *cloud, base_cloud); // Transform pointcloud to map/world (globalFrameId) --> baseCloud JAC
        
        // std::cout << "frame base_cloud: "      << base_cloud.header.frame_id  << std::endl;
        // std::cout << "Height: "      << base_cloud.height  << std::endl;
        // std::cout << "Width: "      << base_cloud.width  << std::endl;  //width is the length of the point cloud
        // std::cout << "Size: "      << base_cloud.size()  << std::endl;
        // ROS_INFO("PointCloud received");

        // PointCloud2 to PointXYZ conversion (base_cloud --> downCloud), with range limits [0,5000]: 5000 is defined in PointCloud2_to_PointXYZ
		// std::vector<pcl::PointXYZ> downCloud;
        // PointCloud2_to_PointXYZ(base_cloud, downCloud);

        // JAC: Decide if convert std::vector<pcl::PointXYZ> to pcl::PointCloud<pcl::PointXYZ> after obtaining downCloud or use PointCloud2_to_PointXYZ_pcl. Now, the latter because in ROSInterfaces.cpp is used.


        // Convert std::vector<pcl::PointXYZ> to pcl::PointCloud<pcl::PointXYZ>
        // std::vector<pcl::PointXYZ> downCloud;
        // pcl::PointCloud<pcl::PointXYZ> local_cloud_;
        // for(unsigned int i=0; i<downCloud.size(); i++) 
		// {
		// 	local_cloud_[i].x = downCloud[i].x;
		// 	local_cloud_[i].y = downCloud[i].y;
		// 	local_cloud_[i].z = downCloud[i].z;			
		// }

        // Local Point Cloud from Velodyne by considering only the size of local map. This is done in PointCloud2_to_PointXYZ_pcl
        pcl::PointCloud<pcl::PointXYZ> local_cloud_;
        local_cloud_.header.frame_id = baseFrameId;

        // sensor_msgs::PointCloud2 --> pcl::PointCloud<pcl::PointXYZ> to configure the local map. Only points inside local map are considered.
		PointCloud2_to_PointXYZ_pcl(base_cloud,local_cloud_);
        // std::cout << "Size: "      << local_cloud_.size()  << std::endl;        
        // std::cout << "frame local_cloud: "      << local_cloud_.header.frame_id  << std::endl;

        // Publisher to show that local_cloud_ is correctly generated and it corresponds to the point cloud from velodyne.
        cloud_test.publish(local_cloud_);

        // Clean the world before generating it for each iteration
        algorithm_->cleanLocalWorld();
        // Configure the local world from the PointCloud (for the local map)
        Planners::utils::configureWorldFromPointCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(local_cloud_), *algorithm_, resolution_); 
        // Publish the Occupancy Map
        algorithm_->publishLocalOccupationMarkersMap();

        // Configure the distance grid and the cost grid
        //!!!!!Planners::utils::configureLocalWorldCosts(boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(local_cloud_), *m_local_grid3d_, *algorithm_); // JAC: Is this correctly generated?

        // ROS_INFO("Published occupation marker local map");


        // cloud_ = *_points;
        // input_map_ = 2;
        // pointcloud_local_sub_.shutdown();        

        // std::cout << "Size: "      << downCloud.size()  << std::endl;
        // for(unsigned int i=0; i<downCloud.size(); i++) 
		// {
			// float x = downCloud[i].x, y = downCloud[i].y, z = downCloud[i].z;
			// points[i].x = x*r00 + y*r01 + z*r02;
			// points[i].y = x*r10 + y*r11 + z*r12;
			// points[i].z = x*r20 + y*r21 + z*r22;			
		// }

    }

	bool PointCloud2_to_PointXYZ_pcl(sensor_msgs::PointCloud2 &in, pcl::PointCloud<pcl::PointXYZ> &out)
	{	
		sensor_msgs::PointCloud2Iterator<float> iterX(in, "x");
		sensor_msgs::PointCloud2Iterator<float> iterY(in, "y");
		sensor_msgs::PointCloud2Iterator<float> iterZ(in, "z");
		out.clear();
		for(unsigned int i=0; i<in.width*in.height; i++, ++iterX, ++iterY, ++iterZ) 
		{
			pcl::PointXYZ p(*iterX, *iterY, *iterZ);
            // These parameters are obtained from input parameters of the size of the local world: local_world_size_x, local_world_size_y and local_world_size_z in the launch
            int size_x, size_y, size_z;
            
            size_x=size_y=local_world_size_meters.x;
            size_z=local_world_size_meters.z;

			// float d2 = p.x*p.x + p.y*p.y + p.z*p.z;
            if (p.x<=size_x && p.x>=-(size_x) && p.y<=size_y && p.y>=-(size_y) && p.z<=size_z && p.z>=-(size_z)) 
				out.push_back(p);			
		} 

		return true;
	}

	bool PointCloud2_to_PointXYZ(sensor_msgs::PointCloud2 &in, std::vector<pcl::PointXYZ> &out)
	{	
		sensor_msgs::PointCloud2Iterator<float> iterX(in, "x");
		sensor_msgs::PointCloud2Iterator<float> iterY(in, "y");
		sensor_msgs::PointCloud2Iterator<float> iterZ(in, "z");
		out.clear();
		for(unsigned int i=0; i<in.width*in.height; i++, ++iterX, ++iterY, ++iterZ) 
		{
			pcl::PointXYZ p(*iterX, *iterY, *iterZ);
			float d2 = p.x*p.x + p.y*p.y + p.z*p.z;
			if(d2 > 1 && d2 < 5000)
				out.push_back(p);			
		} 

		return true;
	}

    void calculatePath3D()
    {
        std::cout << "------ENTERED CALCULATEPATH3D-------" << std::endl;

        // 1. Check if starting point is free
        if(m_local_grid3d_->m_grid[(m_local_grid3d_->m_gridSize-1)/2].dist > 0)
        {
            ROS_INFO("Starting point is FREE");
            std::cout << "Point index queried: " << (m_local_grid3d_->m_gridSize-1)/2 <<" |  Value of dist: " << m_local_grid3d_->m_grid[(m_local_grid3d_->m_gridSize-1)/2].dist << std::endl;
        }
        else
        {
            ROS_INFO("Starting point is NOT FREE -> ABORTING");
            std::cout << "Point index queried: " << (m_local_grid3d_->m_gridSize-1)/2 <<" |  Value of dist: " << m_local_grid3d_->m_grid[(m_local_grid3d_->m_gridSize-1)/2].dist << std::endl;
            exit(EXIT_FAILURE);
        }

        // 2. Calculate local goal from global goal -> Check furthest global goal in the local goal
        Planners::utils::CoordinateList global_path_local;

        // 2.1 - Convert waypoints to local reference

        //Cálculo del origen del sistema local (a nivel de celda)

        int origen_local_x, origen_local_y, origen_local_z, drone_local_x, drone_local_y, drone_local_z;
        drone_local_x = (m_local_grid3d_->m_gridSizeX - 1)/2;
        drone_local_y = (m_local_grid3d_->m_gridSizeY - 1)/2;
        drone_local_z = (m_local_grid3d_->m_gridSizeZ - 1)/2;
        origen_local_x = round(drone_x_/resolution_) - drone_local_x;
        origen_local_y = round(drone_y_/resolution_) - drone_local_y;
        origen_local_z = round(drone_z_/resolution_) - drone_local_z;

        for (const auto& vec : global_path_) 
        {
            Planners::utils::Vec3i newpoint;
            newpoint.x = vec.x - origen_local_x;
            newpoint.y = vec.y - origen_local_y;
            newpoint.z = vec.z - origen_local_z;
        
            // Agregar el punto desplazado al nuevo vector
            global_path_local.push_back(newpoint);
        }
        // std::cout << "Global path (local reference): " << global_path_local << std::endl;

        // 2.2 - Find closest waypoint to the drone (which is supposed to be the "current" waypoint)
        double min_dist = std::numeric_limits<double>::infinity();
        int closest_index = -1;

        for (size_t i = 0; i < global_path_local.size(); ++i) 
        {
            const Planners::utils::Vec3i& act_waypoint = global_path_local[i];

            double dist_to_wayp = std::sqrt(
                std::pow(act_waypoint.x - drone_local_x, 2) +
                std::pow(act_waypoint.y - drone_local_y, 2) +
                std::pow(act_waypoint.z - drone_local_z, 2)
            );

            if (dist_to_wayp < min_dist) 
            {
                min_dist = dist_to_wayp;
                closest_index = i;
            }
        }
        // std::cout << "Closest waypoint: " << global_path_local[closest_index] << std::endl;

        // 2.3 - Find furthest next waypoint that is still inside the local map (the drone will treat this waypoint as the local goal
        bool points_in_range = true;
        int it = closest_index;
        Planners::utils::Vec3i local_goal;
        
        while (it <= (global_path_local.size() - 1) && points_in_range)
        {
            if (0 <= global_path_local[it].x && global_path_local[it].x < m_local_grid3d_->m_gridSizeX && 
                0 <= global_path_local[it].y && global_path_local[it].y < m_local_grid3d_->m_gridSizeY &&
                0 <= global_path_local[it].z && global_path_local[it].z < m_local_grid3d_->m_gridSizeZ) 
            {
                local_goal.x = global_path_local[it].x;
                local_goal.y = global_path_local[it].y;
                local_goal.z = global_path_local[it].z;
                it++;
            }
            else
            {
                local_goal.x = global_path_local[it - 1].x;
                local_goal.y = global_path_local[it - 1].y;
                local_goal.z = global_path_local[it - 1].z;
                points_in_range = false;
            }
            
        }
        // std::cout << "Local goal found: " << local_goal << std::endl;

        // 3 - Check if local goal accessible. If not, find closest point
        
        // 4 - Use path planner to find local waypoints
        Planners::utils::Vec3i discrete_start, discrete_goal;
        discrete_start.x = drone_local_x;
        discrete_start.y = drone_local_y;
        discrete_start.z = drone_local_z;
        discrete_goal.x = local_goal.x;
        discrete_goal.y = local_goal.y;
        discrete_goal.z = local_goal.z;

        std::cout << "Discrete local start: " << discrete_start << "  | Discrete local goal: " << discrete_goal << std::endl;

        if( algorithm_->detectCollision(discrete_start) ){
            std::cout << discrete_start << ": Start not valid" << std::endl;
        }
        if( algorithm_->detectCollision(discrete_goal) ){
            std::cout << discrete_goal << ": Goal not valid" << std::endl;
        }

        //std::cout << "LOCAL PATH CALCULATED SUCCESSFULLY" << std::endl;
        //Planners::utils::CoordinateList local_path = std::get<Planners::utils::CoordinateList>(local_path_data["path"]);
        //std::cout << "Local path: " << local_path << std::endl;

        
        auto local_path_data = algorithm_->findPath(discrete_start, discrete_goal, loaded_sdf_);
        if( std::get<bool>(local_path_data["solved"]) ){
            Planners::utils::CoordinateList local_path;
            local_path = std::get<Planners::utils::CoordinateList>(local_path_data["path"]);

            // Trilinear params test
            for(const auto& point: local_path){
                double x_test = static_cast<double>(point.x) * m_local_grid3d_->m_resolution;
                double y_test = static_cast<double>(point.y) * m_local_grid3d_->m_resolution;
                double z_test = static_cast<double>(point.z) * m_local_grid3d_->m_resolution;
                TrilinearParams p_test = m_local_grid3d_->computeDistInterpolation(x_test, y_test, z_test);
                std::cout << "Params for x = " << x_test/m_local_grid3d_->m_resolution << ", y = " << y_test/m_local_grid3d_->m_resolution << ", z = " << z_test/m_local_grid3d_->m_resolution << ": a0=" << p_test.a0 << ", a1=" << p_test.a1 << ", a2=" << p_test.a2 << ", a3=" << p_test.a3 << ", a4=" << p_test.a4 << ", a5=" << p_test.a5 << ", a6=" << p_test.a6 << ", a7=" << p_test.a7 << std::endl;

                float test_dist = p_test.a0 + p_test.a1*x_test + p_test.a2*y_test + p_test.a3*z_test + p_test.a4*x_test*y_test + p_test.a5*x_test*z_test + p_test.a6*y_test*z_test + p_test.a7*x_test*y_test*z_test;
                uint64_t test_index = point.x + point.y*m_local_grid3d_->m_gridStepY + point.z*m_local_grid3d_->m_gridStepZ;
                std::cout << "Interpolated distance : " << test_dist << "Real distance : " << m_local_grid3d_->m_grid[test_index].dist << std::endl;
            }

            // -----------------CERES OPTIMIZATION OF THE PATH-----------------
            Planners::utils::CoordinateList opt_local_path = Ceresopt::ceresOptimizer(local_path, *m_local_grid3d_);


            //Convert the local path to GLOBAL COORDINATES and push them into the markers

            Planners::utils::Vec3i local_origin = {origen_local_x, origen_local_y, origen_local_z};
            std::cout << "Local origin: " << local_origin;

            for(const auto &it: opt_local_path){
                Planners::utils::Vec3i global_wp_point = it + local_origin;
                local_path_line_markers_.points.push_back(Planners::utils::continousPoint(global_wp_point, resolution_));
                local_path_points_markers_.points.push_back(Planners::utils::continousPoint(global_wp_point, resolution_));
            }

            // std::cout << "Local_path_line_markers: " << local_path_line_markers_.points << std::endl;
            // std::cout << "Local_path_points_markers: " << local_path_points_markers_.points << std::endl;

            publishMarker(local_path_line_markers_, local_line_markers_pub_);
            publishMarker(local_path_points_markers_, local_point_markers_pub_);

            local_path_line_markers_.points.clear();
            local_path_points_markers_.points.clear();

            ROS_INFO("Path calculated succesfully");
        }
        else
            ROS_INFO("Couldn't calculate path");
    
    }

    bool setAlgorithm(heuristic_planners::SetAlgorithmRequest &_req, heuristic_planners::SetAlgorithmResponse &rep){
        
        configureAlgorithm(_req.algorithm.data, _req.heuristic.data);
        rep.result.data = true;
        return true;
    }

    void configureAlgorithm(const std::string &algorithm_name, const std::string &_heuristic){

        std::cout << "Entered configureAlgorithm" << std::endl;
        float ws_x, ws_y, ws_z;

        lnh_.param("local_world_size_x", ws_x, (float)2.0); // In meters
        lnh_.param("local_world_size_y", ws_y, (float)2.0); // In meters
        lnh_.param("local_world_size_z", ws_z, (float)1.0); // In meters
        lnh_.param("resolution", resolution_, (float)0.1);
        lnh_.param("inflate_map", inflate_, (bool)true);

        // world_size_.x = std::floor(ws_x / resolution_);
        // world_size_.y = std::floor(ws_y / resolution_);
        // world_size_.z = std::floor(ws_z / resolution_);

        local_world_size_meters.x=ws_x;
        local_world_size_meters.y=ws_y;
        local_world_size_meters.z=ws_z;

        local_world_size_.x = std::floor((2*ws_x) / resolution_) + 1;
        local_world_size_.y = std::floor((2*ws_y) / resolution_) + 1;
        local_world_size_.z = std::floor((2*ws_z) / resolution_) + 1;

        lnh_.param("use3d", use3d_, (bool)true);

        if( algorithm_name == "astar" ){
            ROS_INFO("Using A*");
            algorithm_.reset(new Planners::AStar(use3d_));
        }else if( algorithm_name == "costastar" ){
            ROS_INFO("Using Cost Aware A*");
            algorithm_.reset(new Planners::AStarM1(use3d_));
        }else if( algorithm_name == "astarsafetycost" ){
            ROS_INFO("Using A* Safety Cost");
            algorithm_.reset(new Planners::AStarM2(use3d_));    
        }else if ( algorithm_name == "thetastar" ){
            ROS_INFO("Using Theta*");
            algorithm_.reset(new Planners::ThetaStar(use3d_));
        }else if ( algorithm_name == "costhetastar" ){
            ROS_INFO("Using Cost Aware Theta* ");
            algorithm_.reset(new Planners::ThetaStarM1(use3d_));
        }else if ( algorithm_name == "thetastarsafetycost" ){
            ROS_INFO("Using Theta* Safety Cost");
            algorithm_.reset(new Planners::ThetaStarM2(use3d_));
        }else if( algorithm_name == "lazythetastar" ){
            ROS_INFO("Using LazyTheta*");
            algorithm_.reset(new Planners::LazyThetaStar(use3d_));
        }else if( algorithm_name == "costlazythetastar"){
            ROS_INFO("Using Cost Aware LazyTheta*");
            algorithm_.reset(new Planners::LazyThetaStarM1(use3d_));
        }else if( algorithm_name == "costlazythetastarmodified"){
            ROS_INFO("Using Cost Aware LazyTheta*");
            algorithm_.reset(new Planners::LazyThetaStarM1Mod(use3d_));
        }else if( algorithm_name == "lazythetastarsafetycost"){
            ROS_INFO("Using LazyTheta* Safety Cost");
            algorithm_.reset(new Planners::LazyThetaStarM2(use3d_));
        }else{
            ROS_WARN("Wrong algorithm name parameter. Using ASTAR by default");
            algorithm_.reset(new Planners::AStar(use3d_));
        }

        // algorithm_->setWorldSize(local_world_size_, resolution_);
        // JAC: QUITAR?
        algorithm_->setLocalWorldSize(local_world_size_, resolution_);

        configureHeuristic(_heuristic);

        ROS_INFO("Using discrete local world size: [%d, %d, %d]", local_world_size_.x, local_world_size_.y, local_world_size_.z);
        ROS_INFO("Using resolution: [%f]", resolution_);

        if(inflate_){
            double inflation_size;
            lnh_.param("inflation_size", inflation_size, 0.05);
            inflation_steps_ = std::round(inflation_size / resolution_);
            ROS_INFO("Inflation size %.2f, using inflation step %d", inflation_size, inflation_steps_);
        }
        algorithm_->setInflationConfig(inflate_, inflation_steps_);

        m_local_grid3d_.reset(new Local_Grid3d); //TODO Costs not implement yet  // Is this necessary in the Local Planner?
        double cost_scaling_factor, robot_radius;
        lnh_.param("cost_scaling_factor", cost_scaling_factor, 0.8);		
		lnh_.param("robot_radius", robot_radius, 0.4);		
        
        m_local_grid3d_->setCostParams(cost_scaling_factor, robot_radius);

        // Read node parameters
		// if(!lnh.getParam("in_cloud", m_inCloudTopic))
		// 	m_inCloudTopic = "/pointcloud";	
		if(!lnh_.getParam("base_frame_id", baseFrameId))
			baseFrameId = "base_link";	
		if(!lnh_.getParam("odom_frame_id", odomFrameId))
			odomFrameId = "odom";	
		if(!lnh_.getParam("global_frame_id", globalFrameId))
			globalFrameId = "map";	

        configMarkers(algorithm_name, globalFrameId, resolution_);  // Not influence by baseFrameId
        // configMarkers(algorithm_name, baseFrameId, resolution_);

        // From planner_ros_node
        // std::string frame_id;
        // lnh_.param("frame_id", frame_id, std::string("map"));		
        // configMarkers(algorithm_name, frame_id, resolution_);

        lnh_.param("save_data_file", save_data_, (bool)true);		
        lnh_.param("data_folder", data_folder_, std::string("planing_data.txt"));		
        if(save_data_)
            ROS_INFO_STREAM("Saving path planning data results to " << data_folder_);

        // JAC: This is not neccesary in local planner.
        if( input_map_ == 1 ){
            Planners::utils::configureWorldFromOccupancyWithCosts(occupancy_grid_, *algorithm_);
        }else if( input_map_ == 2 ){
            Planners::utils::configureWorldFromPointCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(cloud_), *algorithm_, resolution_);
            // Planners::utils::configureLocalWorldCosts(*m_local_grid3d_, *algorithm_); //JAC: Discomment
            ROS_INFO("CONFIGURED WORLD2");

        }
        //Algorithm specific parameters. Its important to set line of sight after configuring world size(it depends on the resolution)
        float sight_dist, cost_weight;
        lnh_.param("max_line_of_sight_distance", sight_dist, (float)1000.0); // In meters
        lnh_.param("cost_weight", cost_weight, (float)0.0);
        algorithm_->setMaxLineOfSight(sight_dist);
        algorithm_->setCostFactor(cost_weight);

        lnh_.param("overlay_markers", overlay_markers_, (bool)false);

        // Init internal variables: TF transform 
        m_tfCache = false;
        ROS_INFO("CONFIGURE ALGORITHM COMPLETED");
    }

    void configureHeuristic(const std::string &_heuristic){
        
        if( _heuristic == "euclidean" ){
            algorithm_->setHeuristic(Planners::Heuristic::euclidean);
            ROS_INFO("Using Euclidean Heuristics");
        }else if( _heuristic == "euclidean_optimized" ){
            algorithm_->setHeuristic(Planners::Heuristic::euclideanOptimized);
            ROS_INFO("Using Optimized Euclidean Heuristics");
        }else if( _heuristic == "manhattan" ){
            algorithm_->setHeuristic(Planners::Heuristic::manhattan);
            ROS_INFO("Using Manhattan Heuristics");
        }else if( _heuristic == "octogonal" ){
            algorithm_->setHeuristic(Planners::Heuristic::octagonal);
            ROS_INFO("Using Octogonal Heuristics");
        }else if( _heuristic == "dijkstra" ){
            algorithm_->setHeuristic(Planners::Heuristic::dijkstra);     
            ROS_INFO("Using Dijkstra Heuristics");
        }else{
            algorithm_->setHeuristic(Planners::Heuristic::euclidean);
            ROS_WARN("Wrong Heuristic param. Using Euclidean Heuristics by default");
        }
    }

    std::vector<std::pair<Planners::utils::Vec3i, double>> getClosestObstaclesToPathPoints(const Planners::utils::CoordinateList &_path){
        
        std::vector<std::pair<Planners::utils::Vec3i, double>> result;
        if ( use3d_ ){
            //TODO grid3d distances does not take into account the inflation added internally by the algorithm

            for(const auto &it: _path)
                result.push_back( m_local_grid3d_->getClosestObstacle(it) );
            }

        else{//TODO IMplement for 2d
            result.push_back(std::make_pair<Planners::utils::Vec3i, double>(Planners::utils::Vec3i{0,0,0}, 0.0));
        }
        return result;
    }

    void configMarkers(const std::string &_ns, const std::string &_frame, const double &_scale){

        local_path_line_markers_.ns = _ns;
        local_path_line_markers_.header.frame_id = _frame;
        local_path_line_markers_.header.stamp = ros::Time::now();
        local_path_line_markers_.id = rand();
        local_path_line_markers_.lifetime = ros::Duration(500);
        local_path_line_markers_.type = visualization_msgs::Marker::LINE_STRIP;
        local_path_line_markers_.action = visualization_msgs::Marker::ADD;
        local_path_line_markers_.pose.orientation.w = 1;

        local_path_line_markers_.color.r = 0.0;
        local_path_line_markers_.color.g = 1.0;
        local_path_line_markers_.color.b = 0.0;

        local_path_line_markers_.color.a = 1.0;
        local_path_line_markers_.scale.x = _scale;

        local_path_points_markers_.ns = _ns;
        local_path_points_markers_.header.frame_id = _frame;
        local_path_points_markers_.header.stamp = ros::Time::now();
        local_path_points_markers_.id = rand();
        local_path_points_markers_.lifetime = ros::Duration(500);
        local_path_points_markers_.type = visualization_msgs::Marker::POINTS;
        local_path_points_markers_.action = visualization_msgs::Marker::ADD;
        local_path_points_markers_.pose.orientation.w = 1;
        local_path_points_markers_.color.r = 0.0;
        local_path_points_markers_.color.g = 1.0;
        local_path_points_markers_.color.b = 1.0;
        local_path_points_markers_.color.a = 1.0;
        local_path_points_markers_.scale.x = _scale;
        local_path_points_markers_.scale.y = _scale;
        local_path_points_markers_.scale.z = _scale;

    }

    void publishMarker(visualization_msgs::Marker &_marker, const ros::Publisher &_pub){
        
        //Clear previous marker
        if( !overlay_markers_ ){
            _marker.action = visualization_msgs::Marker::DELETEALL;
            _pub.publish(_marker);
        }else{
            local_path_points_markers_.id           = rand();
            local_path_points_markers_.header.stamp = ros::Time::now();
            setRandomColor(local_path_points_markers_.color);

            local_path_line_markers_.id             = rand();
            local_path_line_markers_.header.stamp   = ros::Time::now();
            setRandomColor(local_path_line_markers_.color);
        }
        _marker.action = visualization_msgs::Marker::ADD;
        _pub.publish(_marker);
    }
    void setRandomColor(std_msgs::ColorRGBA &_color, unsigned int _n_div = 20){
        //Using golden angle approximation
        const double golden_angle = 180 * (3 - sqrt(5));
        double hue = color_id_ * golden_angle + 60;
        color_id_++;
        if(color_id_ == _n_div)
            color_id_ = 1;

        auto random_color = Planners::Misc::HSVtoRGB(hue, 100, 100);

        _color.r = random_color.x;
        _color.g = random_color.y;
        _color.b = random_color.z;
    }


    ros::NodeHandle lnh_{"~"};
    ros::ServiceServer request_path_server_, change_planner_server_;
    ros::Subscriber pointcloud_local_sub_, occupancy_grid_local_sub_, path_local_sub_;
    //TODO Fix point markers
    ros::Publisher local_line_markers_pub_, local_point_markers_pub_, cloud_test;

    tf::TransformListener m_tfListener;

    std::unique_ptr<Local_Grid3d> m_local_grid3d_;

    std::unique_ptr<Planners::AlgorithmBase> algorithm_;
        
    visualization_msgs::Marker local_path_line_markers_, local_path_points_markers_;
    
    //Parameters
    Planners::utils::Vec3i world_size_; // Discrete
    // JAC: local_world_size_meters: size of the local world provided by the launch
    // JAC: local_world_size_: size of the discrete local world from local_world_size_meters (of the launch) and resolution
    Planners::utils::Vec3i local_world_size_, local_world_size_meters; // Discrete
    float resolution_;

    bool save_data_;
    bool use3d_{true};

    bool inflate_{false};
    unsigned int inflation_steps_{0};
    std::string data_folder_;
    bool overlay_markers_{0};
    unsigned int color_id_{0};
    nav_msgs::OccupancyGrid occupancy_grid_;
    pcl::PointCloud<pcl::PointXYZ> cloud_;

    //! Indicates that the local transfrom for the pointcloud is cached
	bool m_tfCache;
	tf::StampedTransform pclTf;

    //! Node parameters
	// std::string m_inCloudTopic;
	std::string baseFrameId;
	std::string odomFrameId;
	std::string globalFrameId;
    // std::string m_baseFrameId;

    // Visualization of the map as pointcloud: measurements of 3D LIDAR
    // sensor_msgs::PointCloud2 local_cloud;
    //0: no map yet
    //1: using occupancy
    //2: using cloud
    int input_map_{0};
    std::string heuristic_;

    int number_of_points;
    bool mapReceived;

    // --------NEW VARIABLES -- SIREN LOCAL PLANNER
    //drone global position
    double drone_x_ = 0.0;
    double drone_y_ = 0.0;
    double drone_z_ = 0.0;

    //global path variable and flag
    Planners::utils::CoordinateList global_path_;
    int globalPathReceived_;

    //Network, network flag, subscriber
    torch::jit::script::Module loaded_sdf_;
    int networkReceivedFlag_;
    ros::Subscriber network_update_sub_;

    //Global position subscriber
    ros::Subscriber globalposition_local_sub_;


    // local pathplanner loop timer
    ros::Timer timed_local_path_;

};
// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "heuristic_local_planner_ros_node");

//     HeuristicLocalPlannerROS heuristic_local_planner_ros;
//     ros::spin();

// return 0;
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "heuristic_local_planner_ros_node");

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
    
    HeuristicLocalPlannerROS heuristic_local_planner_ros;

  	// f = boost::bind(&LocalPlanner::dynRecCb,&lcPlanner,  _1, _2);
  	// server.setCallback(f);

	ros::Rate loop_rate(30);
    while(ros::ok()){
        ros::spinOnce();
        // Call to Local Plan
        // heuristic_local_planner_ros.plan();
                
        loop_rate.sleep();
    }
    return 0;
}