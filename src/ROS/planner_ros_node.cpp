#include <iostream>

#include "Planners/AStarGenerator.hpp"
#include "Planners/ThetaStarGenerator.hpp"
#include "Planners/LazyThetaStarGenerator.hpp"
#include "utils/ros/ROSInterfaces.hpp"
#include "utils/SaveDataToFile.hpp"
#include "Grid3D/grid3d.hpp"

#include <ros/ros.h>

#include <visualization_msgs/Marker.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <heuristic_planners/GetPath.h>
#include <heuristic_planners/SetAlgorithm.h>

using namespace Planners;
class HeuristicPlannerROS
{

public:
    HeuristicPlannerROS()
    {

        map_sub_ = lnh_.subscribe<pcl::PointCloud<pcl::PointXYZ>>("/points", 1, &HeuristicPlannerROS::pointCloudCallback, this);

        request_path_server_   = lnh_.advertiseService("request_path",  &HeuristicPlannerROS::requestPathService, this);
        change_planner_server_ = lnh_.advertiseService("set_algorithm", &HeuristicPlannerROS::setAlgorithm, this);

        line_markers_pub_ = lnh_.advertise<visualization_msgs::Marker>("path_line_markers", 1);
        point_markers_pub_ = lnh_.advertise<visualization_msgs::Marker>("path_points_markers", 1);

        std::string algorithm_name;
        lnh_.param("algorithm", algorithm_name, (std::string)"astar");

        configureAlgorithm(algorithm_name);
    }

private:

    void pointCloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &_points)
    {

        ROS_INFO("Loading map...");
        int n_obs = 0;
        for(auto &it: *_points){
            algorithm_->addCollision(discretePoint(it, resolution_));
            n_obs++;
        }
        ROS_INFO("Map loaded. Added %d", n_obs);
        algorithm_->publishOccupationMarkersMap();
        ROS_INFO("Published occupation marker map");

        map_sub_.shutdown();
        bool map_ready_ = true;
    }   
    bool setAlgorithm(heuristic_planners::SetAlgorithmRequest &_req, heuristic_planners::SetAlgorithmResponse &rep){
        
        configureAlgorithm(_req.algorithm.data);
        map_sub_ = lnh_.subscribe<pcl::PointCloud<pcl::PointXYZ>>("/points", 1, &HeuristicPlannerROS::pointCloudCallback, this);

        return true;
    }
    bool requestPathService(heuristic_planners::GetPathRequest &_req, heuristic_planners::GetPathResponse &_rep){

        ROS_INFO("Path requested, computing path");
        //Astar coordinate list is std::vector<vec3i>
        const auto discrete_goal =  discretePoint(_req.goal, resolution_);
        const auto discrete_start = discretePoint(_req.start, resolution_);

        if( algorithm_->detectCollision(discrete_start) || 
            algorithm_->detectCollision(discrete_goal) ){
            ROS_ERROR("Goal or start point not valid");
            return false;
        }

        auto path_data = algorithm_->findPath(discrete_start, discrete_goal);
        if(static_cast<bool>(std::any_cast<bool>(path_data["solved"]))){
             
            try{
                _rep.path_length.data = static_cast<float>(std::any_cast<float>(path_data["path_length"]));
                _rep.time_spent.data =  static_cast<int>(std::floor(std::any_cast<double>(path_data["time_spent"])));
                _rep.explored_nodes.data = static_cast<int>(std::any_cast<size_t>(path_data["explored_nodes"]));
                _rep.line_of_sight_checks.data = static_cast<int>(std::any_cast<int>(path_data["line_of_sight_checks"]));

            }catch(const std::bad_any_cast& e){
                std::cerr << "Any cast error: " << e.what() << std::endl;
            }

            path_line_markers_.points.clear();
            path_points_markers_.points.clear();
            for(const auto &it: std::any_cast<CoordinateList>(path_data["path"])){
                _rep.path_points.push_back(continousPoint(it, resolution_));
                path_line_markers_.points.push_back(continousPoint(it, resolution_));
                path_points_markers_.points.push_back(continousPoint(it, resolution_));
            }

            publishMarker(path_line_markers_, line_markers_pub_);
            publishMarker(path_points_markers_, point_markers_pub_);

            ROS_INFO("Path calculated succesfully");

            if(save_data_){
                utils::DataSaver saver(file_data_path_);
                if(saver.savePathDataToFile(path_data));
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

        if( algorithm_name == "astar" ){
            ROS_INFO("Using A*");
            algorithm_.reset(new AStarGenerator);
        }else if ( algorithm_name == "thetastar" ){
            ROS_INFO("Using Theta*");
            algorithm_.reset(new ThetaStarGenerator);

        }else if( algorithm_name == "lazythetastar" ){
            ROS_INFO("Using LazyTheta*");
            algorithm_.reset(new LazyThetaStarGenerator);
        }else{
            ROS_WARN("Wrong algorithm name parameter. Using ASTAR by default");
            algorithm_.reset(new AStarGenerator);
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

        m_grid3d_.reset(new Grid3d); //TODO Costs not implement yet
        double cost_scaling_factor, robot_radius;
        lnh_.param("cost_scaling_factor", cost_scaling_factor, 0.8);		
		lnh_.param("robot_radius", robot_radius, 0.4);		
        
        m_grid3d_->setCostParams(cost_scaling_factor, robot_radius);
        
        std::string frame_id;
        lnh_.param("frame_id", frame_id, std::string("map"));		
        configMarkers("astar", frame_id, resolution_);

        lnh_.param("save_data_file", save_data_, (bool)true);		
        lnh_.param("file_path", file_data_path_, std::string("planing_data.txt"));		
        if(save_data_)
            ROS_INFO("Saving path planning data results to %s", file_data_path_.c_str());

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
    ros::Subscriber map_sub_;
    //TODO Fix point markers
    ros::Publisher line_markers_pub_, point_markers_pub_;

    std::unique_ptr<Grid3d> m_grid3d_;

    std::unique_ptr<PathGenerator> algorithm_;
        
    visualization_msgs::Marker path_line_markers_, path_points_markers_;
    
    bool map_ready_{false};
    //Parameters
    Vec3i world_size_; // Discrete
    float resolution_;

    bool save_data_;
    bool inflate_{false};
    unsigned int inflation_steps_{0};
    std::string file_data_path_;
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "astar_ros_node");

    HeuristicPlannerROS astar_ros_node;
    ros::spin();

return 0;
}