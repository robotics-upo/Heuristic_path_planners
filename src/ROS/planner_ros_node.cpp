#include <iostream>

#include "Planners/AStarGenerator.hpp"
#include "Planners/AStarGeneratorSafetyCost.hpp"
#include "Planners/CostAwareAStarGenerator.hpp"
#include "Planners/ThetaStarGenerator.hpp"
#include "Planners/CostAwareThetaStarGenerator.hpp"
#include "Planners/ThetaStarGeneratorSafetyCost.hpp"
#include "Planners/LazyThetaStarGenerator.hpp"
#include "Planners/CostAwareLazyThetaStarGenerator.hpp"
#include "Planners/LazyThetaStarGeneratorSafetyCost.hpp"
#include "utils/ros/ROSInterfaces.hpp"
#include "utils/SaveDataVariantToFile.hpp"
#include "utils/misc.hpp"
#include "utils/geometry_utils.hpp"

#include "Grid3D/grid3d.hpp"

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

        if(!_req.algorithm.data.empty())
            configureAlgorithm(_req.algorithm.data);

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
            utils::CoordinateList path;
            try{
                _rep.time_spent.data           = std::get<double>(path_data["time_spent"] );
                _rep.path_length.data          = std::get<double>(path_data["path_length"] );
                _rep.explored_nodes.data       = std::get<size_t>(path_data["explored_nodes"] );
                _rep.line_of_sight_checks.data = std::get<unsigned int>(path_data["line_of_sight_checks"] );

                _rep.total_cost.data           = std::get<unsigned int>(path_data["total_cost"] );
                _rep.h_cost.data               = std::get<unsigned int>(path_data["h_cost"]);
                _rep.g_cost.data               = std::get<unsigned int>(path_data["g_cost"]);
                _rep.c_cost.data               = std::get<unsigned int>(path_data["c_cost"]);

                _rep.cost_weight.data          = std::get<double>(path_data["cost_weight"]);
                _rep.max_los.data              = std::get<unsigned int>(path_data["max_line_of_sight_cells"]);
                path = std::get<CoordinateList>(path_data["path"]);

            }catch(std::bad_variant_access const& ex){
                std::cerr << "Bad variant error: " << ex.what() << std::endl;
            }
            std::vector<double> curvatures, angles;
            double av_curvature{0}, av_angles{0};
            double curv_sigma{0}, angles_sigma{0};
            for(size_t i = 1; i < path.size() - 2; ++i){
                double R = utils::geometry::getCircunferenceRadius(path[i-1], path[i], path[i+1]);
                if ( R != std::numeric_limits<double>::infinity() ){
                    curvatures.push_back(1/R);
                }
                else{
                    curvatures.push_back(0);
                }
            }

            if ( curvatures.size() > 0 ){
                av_curvature = std::accumulate(curvatures.begin(), curvatures.end(), 0.0)/curvatures.size();

                for(const auto &it: curvatures)
                    curv_sigma += pow(it - av_curvature,2);
                curv_sigma = sqrt(curv_sigma/curvatures.size());

            }
            const auto [curv_min, curv_max] = std::minmax_element(begin(curvatures), end(curvatures));
            // std::cout << "Average curvature: " << av_curvature << " 1/m"<< std::endl;
            path_data["av_curv"]  = av_curvature;
            path_data["std_dev_curv"]   = curv_sigma;
            path_data["min_curv"]  = *curv_min;
            path_data["max_curv"]  = *curv_max;

            // Angles between three points
            int changes=0;
            for(size_t i = 1; i < path.size() - 2; ++i){
                double alpha = (M_PI - utils::geometry::angleBetweenThreePoints(path[i-1], path[i], path[i+1]));

                if ( alpha > 2/180.0*M_PI || 
                     alpha < -2/180*M_PI  ){ // Two degreess
                    angles.push_back(alpha);
                    changes++;
                }
            }

            // Angles between three points
            if ( angles.size() > 0 ){
                av_angles = std::accumulate(angles.begin(), angles.end(), 0.0)/angles.size();

                for(const auto &it: angles)
                    angles_sigma += pow(it - av_angles,2);
                angles_sigma = sqrt(angles_sigma/angles.size());

            }
            const auto [angles_min, angles_max] = std::minmax_element(begin(angles), end(angles));
            std::cout << "Average angles: " << av_angles << " 1/m"<< std::endl;
            path_data["av_angles"]  = av_angles;
            path_data["std_dev_angles"]   = angles_sigma;
            path_data["min_angle"]  = *angles_min;
            path_data["max_angle"]  = *angles_max;         
            path_data["angle_changes"]  = changes;
            ///////////////////////////////////////////////////////////////////////////////////////////////  

            auto adjacent_path    = utils::geometry::getAdjacentPath(path, *algorithm_->getInnerWorld());
            auto result_distances = getClosestObstaclesToPathPoints(adjacent_path);

            std::vector<double> distances; 
            double sigma = 0;
            for(auto &it: result_distances)
                distances.push_back(it.second);

            const auto [min, max] = std::minmax_element(begin(distances), end(distances));
            double mean = std::accumulate(distances.begin(), distances.end(), 0.0)/distances.size();

            for(const auto &it: distances)
                sigma += pow(it - mean,2);
            sigma = sqrt(sigma/distances.size());

            _rep.n_points.data   = adjacent_path.size();
            _rep.min_distance_to_obstacle.data   = *min;
            _rep.max_distance_to_obstacle.data   = *max;
            _rep.mean_distance_to_obstacle.data  = mean;
            _rep.mean_std_dev_to_obstacle.data   = sigma;

            path_data["min_dist"]  = *min;
            path_data["max_dist"]  = *max;
            path_data["mean_dist"] = mean;
            path_data["std_dev"]   = sigma;
            
            if(save_data_){
                utils::DataVariantSaver saver1(data_folder_ + "/planning.txt");
                utils::DataVariantSaver saver2(data_folder_ + "/path_metrics.txt");
                utils::DataVariantSaver angles_saver(data_folder_ + "/angles.txt");

                if(saver1.savePathDataToFile(path_data) && 
                   saver2.savePathDistancesToFile(adjacent_path, result_distances) &&
                   angles_saver.saveAnglesToFile(angles) ){
                    ROS_INFO("Path data metrics saved");
                }else{
                    ROS_ERROR("Couldn't save path data metrics. Path and results does not have same size");
                }
            }

            for(const auto &it: std::get<CoordinateList>(path_data["path"])){
                // _rep.path_points.push_back(continousPoint(it, resolution_));
                path_line_markers_.points.push_back(continousPoint(it, resolution_));
                path_points_markers_.points.push_back(continousPoint(it, resolution_));
            }
            
            publishMarker(path_line_markers_, line_markers_pub_);
            publishMarker(path_points_markers_, point_markers_pub_);
            
            path_line_markers_.points.clear();
            path_points_markers_.points.clear();

            ROS_INFO("Path calculated succesfully");
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
        
        lnh_.param("use3d", use3d_, (bool)true);

        if( algorithm_name == "astar" ){
            ROS_INFO("Using A*");
            algorithm_.reset(new AStarGenerator(use3d_));
        }else if( algorithm_name == "costastar" ){
            ROS_INFO("Using Cost Aware A*");
            algorithm_.reset(new CostAwareAStarGenerator(use3d_));
        }else if( algorithm_name == "astarsafetycost" ){
            ROS_INFO("Using A* Safety Cost");
            algorithm_.reset(new AStarGeneratorSafetyCost(use3d_));    
        }else if ( algorithm_name == "thetastar" ){
            ROS_INFO("Using Theta*");
            algorithm_.reset(new ThetaStarGenerator(use3d_));
        }else if ( algorithm_name == "costhetastar" ){
            ROS_INFO("Using Cost Aware Theta* ");
            algorithm_.reset(new CostAwareThetaStarGenerator(use3d_));
        }else if ( algorithm_name == "thetastarsafetycost" ){
            ROS_INFO("Using Theta* Safety Cost");
            algorithm_.reset(new ThetaStarGeneratorSafetyCost(use3d_));
        }else if( algorithm_name == "lazythetastar" ){
            ROS_INFO("Using LazyTheta*");
            algorithm_.reset(new LazyThetaStarGenerator(use3d_));
        }else if( algorithm_name == "costlazythetastar"){
            ROS_INFO("Using Cost Aware LazyTheta*");
            algorithm_.reset(new CostAwareLazyThetaStarGenerator(use3d_));
        }else if( algorithm_name == "lazythetastarsafetycost"){
            ROS_INFO("Using LazyTheta* Safety Cost");
            algorithm_.reset(new LazyThetaStarGeneratorSafetyCost(use3d_));
        }else{
            ROS_WARN("Wrong algorithm name parameter. Using ASTAR by default");
            algorithm_.reset(new AStarGenerator(use3d_));
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
        configMarkers(algorithm_name, frame_id, resolution_);

        lnh_.param("save_data_file", save_data_, (bool)true);		
        lnh_.param("data_folder", data_folder_, std::string("planing_data.txt"));		
        if(save_data_)
            ROS_INFO("Saving path planning data results to %s", data_folder_.c_str());

        //
        if( input_map_ == 1 ){
            utils::configureWorldFromOccupancyWithCosts(occupancy_grid_, *algorithm_);
        }else if( input_map_ == 2 ){
            utils::configureWorldFromPointCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(cloud_), *algorithm_, resolution_);
            utils::configureWorldCosts(*m_grid3d_, *algorithm_);
        }
        //Algorithm specific parameters. Its important to set line of sight after configuring world size(it depends on the resolution)
        float sight_dist, cost_weight;
        lnh_.param("max_line_of_sight_distance", sight_dist, (float)1000.0); // In meters
        lnh_.param("cost_weight", cost_weight, (float)0.0);
        algorithm_->setMaxLineOfSight(sight_dist);
        algorithm_->setCostFactor(cost_weight);

        lnh_.param("overlay_markers", overlay_markers_, (bool)false);
    }
    std::vector<std::pair<utils::Vec3i, double>> getClosestObstaclesToPathPoints(const utils::CoordinateList &_path){
        
        std::vector<std::pair<utils::Vec3i, double>> result;
        if ( use3d_ ){
            //TODO grid3d distances does not take into account the inflation added internally by the algorithm

            for(const auto &it: _path)
                result.push_back( m_grid3d_->getClosestObstacle(it) );
            }

        else{//TODO IMplement for 2d
            result.push_back(std::make_pair<utils::Vec3i, double>(Vec3i{0,0,0}, 0.0));
        }
        return result;
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
        if( !overlay_markers_ ){
            _marker.action = visualization_msgs::Marker::DELETEALL;
            _pub.publish(_marker);
        }else{
            path_points_markers_.id           = rand();
            path_points_markers_.header.stamp = ros::Time::now();
            setRandomColor(path_points_markers_.color);

            path_line_markers_.id             = rand();
            path_line_markers_.header.stamp   = ros::Time::now();
            setRandomColor(path_line_markers_.color);
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

        auto random_color = Misc::HSVtoRGB(hue, 100, 100);

        _color.r = random_color.x;
        _color.g = random_color.y;
        _color.b = random_color.z;
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
    bool use3d_{true};

    bool inflate_{false};
    unsigned int inflation_steps_{0};
    std::string data_folder_;
    bool overlay_markers_{0};
    unsigned int color_id_{0};
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