#include <iostream>

#include "Planners/AStarGenerator.hpp"
#include "Planners/CostAwareAStarGenerator.hpp"
#include "Planners/ThetaStarGenerator.hpp"
#include "Planners/LazyThetaStarGenerator.hpp"
#include "Planners/CostAwareLazyThetaStarGenerator.hpp"
#include "utils/ros/ROSInterfaces.hpp"
#include "utils/SaveDataVariantToFile.hpp"
#include "Grid3D/grid3d.hpp"
#include "Grid3D/VoronoiGrid.hpp"

#include <ros/ros.h>

#include <visualization_msgs/Marker.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <nav_msgs/OccupancyGrid.h>

#include <heuristic_planners/GetPath.h>
#include <heuristic_planners/SetAlgorithm.h>

using namespace Planners;

/**
 * @brief Demo Class that demonstrate how to use the algorithms classes and utils 
 * with ROS 
 * 
 */
class HeuristicPlannerROS
{

public:
    HeuristicPlannerROS()
    {

        std::string algorithm_name;
        lnh_.param("algorithm", algorithm_name, (std::string)"astar");

        configureAlgorithm(algorithm_name);

        pointcloud_sub_     = lnh_.subscribe<pcl::PointCloud<pcl::PointXYZ>>("/points", 1, &HeuristicPlannerROS::pointCloudCallback, this);
        occupancy_grid_sub_ = lnh_.subscribe<nav_msgs::OccupancyGrid>("/grid", 1, &HeuristicPlannerROS::occupancyGridCallback, this);

        request_path_server_   = lnh_.advertiseService("request_path",  &HeuristicPlannerROS::requestPathService, this);
        change_planner_server_ = lnh_.advertiseService("set_algorithm", &HeuristicPlannerROS::setAlgorithm, this);

        line_markers_pub_  = lnh_.advertise<visualization_msgs::Marker>("path_line_markers", 1);
        point_markers_pub_ = lnh_.advertise<visualization_msgs::Marker>("path_points_markers", 1);

    }

private:

    void occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr &_grid){
        ROS_INFO("Loading OccupancyGrid map...");
        utils::configureWorldFromOccupancyWithCosts(*_grid, *algorithm_);
        algorithm_->publishOccupationMarkersMap();
        occupancy_grid_sub_.shutdown();
        ROS_INFO("Occupancy Grid Loaded");
        occupancy_grid_ = *_grid;
        input_map_ = 1;
    }

    void pointCloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &_points)
    {

        ROS_INFO("Loading map...");
        utils::configureWorldFromPointCloud(_points, *algorithm_, resolution_);
        algorithm_->publishOccupationMarkersMap();
        utils::configureWorldCosts(*m_grid3d_, *algorithm_);
        ROS_INFO("Published occupation marker map");
        cloud_ = *_points;
        input_map_ = 2;
        pointcloud_sub_.shutdown();
    }   
    bool setAlgorithm(heuristic_planners::SetAlgorithmRequest &_req, heuristic_planners::SetAlgorithmResponse &rep){
        
        configureAlgorithm(_req.algorithm.data);
        rep.result.data = true;
        return true;
    }
    bool requestPathService(heuristic_planners::GetPathRequest &_req, heuristic_planners::GetPathResponse &_rep){

        ROS_INFO("Path requested, computing path");
        //delete previous markers
        publishMarker(path_line_markers_, line_markers_pub_);
        publishMarker(path_points_markers_, point_markers_pub_);

        //Astar coordinate list is std::vector<vec3i>
        const auto discrete_goal =  discretePoint(_req.goal, resolution_);
        const auto discrete_start = discretePoint(_req.start, resolution_);

        if( algorithm_->detectCollision(discrete_start) ){
            std::cout << discrete_start << ": Start not valid" << std::endl;
            return false;
        }

        if( algorithm_->detectCollision(discrete_goal) ){
            std::cout << discrete_goal << ": Goal not valid" << std::endl;
            return false;
        }

        auto path_data = algorithm_->findPath(discrete_start, discrete_goal);
        
        if( std::get<bool>(path_data["solved"]) ){
            
            try{
                
                _rep.time_spent.data           = std::get<double>(path_data["time_spent"] );
                _rep.path_length.data          = std::get<double>(path_data["path_length"] );
                _rep.explored_nodes.data       = std::get<size_t>(path_data["explored_nodes"] );
                _rep.line_of_sight_checks.data = std::get<int>(   path_data["line_of_sight_checks"] );
            }catch(std::bad_variant_access const& ex){
                std::cerr << "Bad variant error: " << ex.what() << std::endl;
            }

            for(const auto &it: std::get<CoordinateList>(path_data["path"])){
                _rep.path_points.push_back(continousPoint(it, resolution_));
                path_line_markers_.points.push_back(continousPoint(it, resolution_));
                path_points_markers_.points.push_back(continousPoint(it, resolution_));
            }
            
            publishMarker(path_line_markers_, line_markers_pub_);
            publishMarker(path_points_markers_, point_markers_pub_);
            
            path_line_markers_.points.clear();
            path_points_markers_.points.clear();

            ROS_INFO("Path calculated succesfully");

            if(save_data_){
                utils::DataVariantSaver saver(file_data_path_);
                if(saver.savePathDataToFile(path_data))
                    ROS_INFO("Data saved succesfully");
            }

        }else{
            ROS_INFO("Could not calculate path between request points");
            
        }

        return true;
    }
    void configureAlgorithm(const std::string &algorithm_name){

        float ws_x, ws_y, ws_z;

        lnh_.param("world_size_x", ws_x, (float)100.0); // In meters
        lnh_.param("world_size_y", ws_y, (float)100.0); // In meters
        lnh_.param("world_size_z", ws_z, (float)100.0); // In meters
        lnh_.param("resolution", resolution_, (float)0.2);
        lnh_.param("inflate_map", inflate_, (bool)true);

        world_size_.x = std::floor(ws_x / resolution_);
        world_size_.y = std::floor(ws_y / resolution_);
        world_size_.z = std::floor(ws_z / resolution_);
        
        bool use3d{true};
        lnh_.param("use3d", use3d, (bool)true);

        if( algorithm_name == "astar" ){
            ROS_INFO("Using A*");
            algorithm_.reset(new AStarGenerator(use3d));
        }else if( algorithm_name == "costastar" ){
            ROS_INFO("Using Cost Aware A*");
            algorithm_.reset(new CostAwareAStarGenerator(use3d));
        }else if ( algorithm_name == "thetastar" ){
            ROS_INFO("Using Theta*");
            algorithm_.reset(new ThetaStarGenerator(use3d));
        }else if( algorithm_name == "lazythetastar" ){
            ROS_INFO("Using LazyTheta*");
            algorithm_.reset(new LazyThetaStarGenerator(use3d));
        }else if( algorithm_name == "costlazythetastar"){
            ROS_INFO("Using Cost Aware LazyTheta*");
            algorithm_.reset(new CostAwareLazyThetaStarGenerator(use3d));
        }else{
            ROS_WARN("Wrong algorithm name parameter. Using ASTAR by default");
            algorithm_.reset(new AStarGenerator(use3d));
        }

        algorithm_->setWorldSize(world_size_, resolution_);
        algorithm_->setHeuristic(Heuristic::euclidean);

        ROS_INFO("Using discrete world size: [%d, %d, %d]", world_size_.x, world_size_.y, world_size_.z);
        ROS_INFO("Using resolution: [%f]", resolution_);

        if(inflate_){
            double inflation_size;
            lnh_.param("inflation_size", inflation_size, 0.5);
            inflation_steps_ = std::round(inflation_size / resolution_);
            ROS_INFO("Inflation size %.2f, using inflation step %d", inflation_size, inflation_steps_);
        }
        algorithm_->setInflationConfig(inflate_, inflation_steps_);

        std::string grid_type;//standard or voronoi
        lnh_.param("grid_type", grid_type, (std::string)"standard");
        if( grid_type == "standard" ){
            ROS_INFO("Using %s grid_type", grid_type.c_str());
            m_grid3d_.reset(new Grid3d); //TODO Costs not implement yet
            double cost_scaling_factor, robot_radius;
            lnh_.param("cost_scaling_factor", cost_scaling_factor, 0.8);
		    lnh_.param("robot_radius", robot_radius, 0.4);
            m_grid3d_->setCostParams(cost_scaling_factor, robot_radius);
        }else if ( grid_type == "voronoi" ){
            ROS_INFO("Using %s grid_type", grid_type.c_str());
            m_grid3d_.reset(new VoronoiGrid); 

        }else{
            ROS_WARN("%s is an invalidad grid type, using standard by default", grid_type.c_str());
        }
        
        
        std::string frame_id;
        lnh_.param("frame_id", frame_id, std::string("map"));		
        configMarkers("astar", frame_id, resolution_);

        lnh_.param("save_data_file", save_data_, (bool)true);		
        lnh_.param("file_path", file_data_path_, std::string("planing_data.txt"));		
        if(save_data_)
            ROS_INFO("Saving path planning data results to %s", file_data_path_.c_str());

        //
        if( input_map_ == 1 ){
            utils::configureWorldFromOccupancyWithCosts(occupancy_grid_, *algorithm_);
        }else if( input_map_ == 2 ){
            utils::configureWorldFromPointCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(cloud_), *algorithm_, resolution_);
        }
        //Algorithm specific parameters. Its important to set line of sight after configuring world size(it depends on the resolution)
        float sight_dist, cost_weight;
        lnh_.param("max_line_of_sight_distance", sight_dist, (float)1000.0); // In meters
        lnh_.param("cost_weight", cost_weight, (float)0.0);
        algorithm_->setMaxLineOfSight(sight_dist);
        algorithm_->setCostFactor(cost_weight);
    }
    void configMarkers(const std::string &_ns, const std::string &_frame, const double &_scale){

        path_line_markers_.ns = _ns;
        path_line_markers_.header.frame_id = _frame;
        path_line_markers_.header.stamp = ros::Time::now();
        path_line_markers_.id = rand();
        path_line_markers_.lifetime = ros::Duration(500);
        path_line_markers_.type = visualization_msgs::Marker::LINE_STRIP;
        path_line_markers_.action = visualization_msgs::Marker::ADD;
        path_line_markers_.pose.orientation.w = 1;

        path_line_markers_.color.r = 0.0;
        path_line_markers_.color.g = 1.0;
        path_line_markers_.color.b = 0.0;

        path_line_markers_.color.a = 1.0;
        path_line_markers_.scale.x = _scale;

        path_points_markers_.ns = _ns;
        path_points_markers_.header.frame_id = _frame;
        path_points_markers_.header.stamp = ros::Time::now();
        path_points_markers_.id = rand();
        path_points_markers_.lifetime = ros::Duration(500);
        path_points_markers_.type = visualization_msgs::Marker::POINTS;
        path_points_markers_.action = visualization_msgs::Marker::ADD;
        path_points_markers_.pose.orientation.w = 1;
        path_points_markers_.color.r = 0.0;
        path_points_markers_.color.g = 1.0;
        path_points_markers_.color.b = 1.0;
        path_points_markers_.color.a = 1.0;
        path_points_markers_.scale.x = _scale;
        path_points_markers_.scale.y = _scale;
        path_points_markers_.scale.z = _scale;

    }
    void publishMarker(visualization_msgs::Marker &_marker, const ros::Publisher &_pub){
        
        //Clear previous marker
        _marker.action = visualization_msgs::Marker::DELETEALL;
        _pub.publish(_marker);
        _marker.action = visualization_msgs::Marker::ADD;
        _pub.publish(_marker);
    }


    ros::NodeHandle lnh_{"~"};
    ros::ServiceServer request_path_server_, change_planner_server_;
    ros::Subscriber pointcloud_sub_, occupancy_grid_sub_;
    //TODO Fix point markers
    ros::Publisher line_markers_pub_, point_markers_pub_;

    std::unique_ptr<Grid3d> m_grid3d_;

    std::unique_ptr<PathGenerator> algorithm_;
        
    visualization_msgs::Marker path_line_markers_, path_points_markers_;
    
    //Parameters
    Vec3i world_size_; // Discrete
    float resolution_;

    bool save_data_;
    bool inflate_{false};
    unsigned int inflation_steps_{0};
    std::string file_data_path_;
    
    nav_msgs::OccupancyGrid occupancy_grid_;
    pcl::PointCloud<pcl::PointXYZ> cloud_;
    //0: no map yet
    //1: using occupancy
    //2: using cloud
    int input_map_{0};
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "heuristic_planner_ros_node");

    HeuristicPlannerROS heuristic_planner_ros;
    ros::spin();

return 0;
}