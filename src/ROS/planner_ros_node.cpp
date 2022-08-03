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

#include "Grid3D/grid3d.hpp"

#include "rclcpp/rclcpp.hpp"

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

#include <heuristic_planners/srv/get_path.hpp>
#include <heuristic_planners/srv/set_algorithm.hpp>

/**
 * @brief Demo Class that demonstrate how to use the algorithms classes and utils 
 * with ROS 
 * 
 */
class HeuristicPlannerROS : public rclcpp::Node {

public:
    HeuristicPlannerROS()
      :rclcpp::Node("heuristic_planner_ros")
    {

        std::string algorithm_name;
        // lnh_.param("algorithm", algorithm_name, (std::string)"astar");
        // lnh_.param("heuristic", heuristic_, (std::string)"euclidean");
        
        this->declare_parameter<std::string>("algorithm_name", "astar");
        this->declare_parameter<std::string>("heuristic_name", "euclidean");
        this->get_parameter("algorithm_name", algorithm_name);
        this->get_parameter("heuristic_name", heuristic_);

        
        configureAlgorithm(algorithm_name, heuristic_);


        point_markers_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("path_points_markers", 1);
        line_markers_pub_  = this->create_publisher<visualization_msgs::msg::Marker>("path_line_markers", 1);

        request_path_server_ = this->create_service<heuristic_planners::srv::GetPath>("request_path",std::bind(
                                                                      &HeuristicPlannerROS::requestPathService, this,
                                                                      std::placeholders::_1, // Corresponds to the 'request'  input
                                                                      std::placeholders::_2  // Corresponds to the 'response' input
                                                                      ));

        change_planner_server_ = this->create_service<heuristic_planners::srv::SetAlgorithm>("set_algorithm",std::bind(
                                                                      &HeuristicPlannerROS::setAlgorithm, this,
                                                                      std::placeholders::_1, // Corresponds to the 'request'  input
                                                                      std::placeholders::_2  // Corresponds to the 'response' input
                                                                      ));


        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/points", 1, std::bind(&HeuristicPlannerROS::pointCloudCallback, this, std::placeholders::_1));
        occupancy_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/grid", 1, std::bind(&HeuristicPlannerROS::occupancyGridCallback, this, std::placeholders::_1));
    }

private:

    void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr _grid){
        RCLCPP_INFO(this->get_logger(),"Loading OccupancyGrid map...");
        Planners::utils::configureWorldFromOccupancyWithCosts(*_grid, *algorithm_);
        algorithm_->publishOccupationMarkersMap();
        occupancy_grid_sub_.reset();
        RCLCPP_INFO(this->get_logger(),"Occupancy Grid Loaded");
        occupancy_grid_ = *_grid;
        input_map_ = 1;
    }

    // void pointCloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &_points)
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr points)
    {
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> _points_ptr = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromROSMsg(*(points.get()), *(_points_ptr.get()));

        RCLCPP_INFO(this->get_logger(),"Loading map...");
        Planners::utils::configureWorldFromPointCloud(_points_ptr, *algorithm_, resolution_);
        algorithm_->publishOccupationMarkersMap();
        Planners::utils::configureWorldCosts(*m_grid3d_, *algorithm_);
        RCLCPP_INFO(this->get_logger(),"Published occupation marker map");
        cloud_ = *_points_ptr;
        input_map_ = 2;
        pointcloud_sub_.reset();
    }   
    bool setAlgorithm(const std::shared_ptr<heuristic_planners::srv::SetAlgorithm::Request> _req, 
        std::shared_ptr<heuristic_planners::srv::SetAlgorithm::Response> rep){
        
        configureAlgorithm(_req->algorithm.data, _req->heuristic.data);
        rep->result.data = true;
        return true;
    }
    bool requestPathService(const std::shared_ptr<heuristic_planners::srv::GetPath::Request> req, std::shared_ptr<heuristic_planners::srv::GetPath::Response> rep){
      auto _req = *(req.get());
      auto _rep = *(rep.get());

        if( !_req.algorithm.data.empty() ){
            if( !_req.heuristic.data.empty() ){
                configureAlgorithm(_req.algorithm.data, _req.heuristic.data);
            }else{
                configureAlgorithm(_req.algorithm.data, heuristic_);
            }
        }else if( !_req.heuristic.data.empty() ){
            configureHeuristic(_req.heuristic.data);
        }

        RCLCPP_INFO(this->get_logger(),"Path requested, computing path");
        //delete previous markers
        publishMarker(path_line_markers_, line_markers_pub_);
        publishMarker(path_points_markers_, point_markers_pub_);

        //Astar coordinate list is std::vector<vec3i>
        const auto discrete_goal =  Planners::utils::discretePoint(_req.goal, resolution_);
        const auto discrete_start = Planners::utils::discretePoint(_req.start, resolution_);

        if( algorithm_->detectCollision(discrete_start) ){
            std::cout << discrete_start << ": Start not valid" << std::endl;
            return false;
        }

        if( algorithm_->detectCollision(discrete_goal) ){
            std::cout << discrete_goal << ": Goal not valid" << std::endl;
            return false;
        }
        std::vector<double> times;
        times.reserve(_req.tries.data);
        int real_tries = _req.tries.data;
        if(real_tries == 0) real_tries = 1;

        for(int i = 0; i < real_tries; ++i){

            auto path_data = algorithm_->findPath(discrete_start, discrete_goal);

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

                if(save_data_){
                
                    const auto [av_curvature, curv_sigma, curv_min, curv_max] = Planners::utils::metrics::calculatePathCurvature(path);

                    const auto [av_angles, angles_sigma, angles_min, angles_max, changes, angles] = Planners::utils::metrics::calculatePathAnglesMetrics(path, 2);

                    const auto adjacent_path    = Planners::utils::geometry::getAdjacentPath(path, *algorithm_->getInnerWorld());
                    const auto result_distances = getClosestObstaclesToPathPoints(adjacent_path);
                    const auto [mean_dist, dist_stddev, min_dist, max_dist] = Planners::utils::metrics::calculateDistancesMetrics(result_distances );

                    path_data["av_curv"]        = av_curvature;
                    path_data["std_dev_curv"]   = curv_sigma;
                    path_data["min_curv"]       = curv_min;
                    path_data["max_curv"]       = curv_max;

                    path_data["av_angles"]      = av_angles;
                    path_data["std_dev_angles"] = angles_sigma;
                    path_data["min_angle"]      = angles_min;
                    path_data["max_angle"]      = angles_max;         
                    path_data["angle_changes"]  = changes;

                    path_data["mean_dist"]      = mean_dist;
                    path_data["std_dev"]        = dist_stddev;
                    path_data["min_dist"]       = min_dist;
                    path_data["max_dist"]       = max_dist;

                    _rep.n_points.data                   = adjacent_path.size();
                    _rep.mean_distance_to_obstacle.data  = mean_dist;
                    _rep.mean_std_dev_to_obstacle.data   = dist_stddev;
                    _rep.min_distance_to_obstacle.data   = min_dist;
                    _rep.max_distance_to_obstacle.data   = max_dist;

                    Planners::utils::DataVariantSaver saver;

                    if(saver.savePathDataToFile(path_data, data_folder_ + "/planning.txt") && 
                       saver.savePathDistancesToFile(adjacent_path, result_distances, data_folder_ + "/path_metrics.txt") &&
                       saver.saveAnglesToFile(angles, data_folder_ + "/angles.txt") ){
                        RCLCPP_INFO(this->get_logger(),"Path data metrics saved");
                    }else{
                        RCLCPP_ERROR(this->get_logger(),"Couldn't save path data metrics. Path and results does not have same size");
                    }
                }

                if(_req.tries.data < 2 || i == ( _req.tries.data - 1) ){

                    for(const auto &it: std::get<Planners::utils::CoordinateList>(path_data["path"])){
                        path_line_markers_.points.push_back(Planners::utils::continousPoint(it, resolution_));
                        path_points_markers_.points.push_back(Planners::utils::continousPoint(it, resolution_));
                    }

                    publishMarker(path_line_markers_, line_markers_pub_);
                    publishMarker(path_points_markers_, point_markers_pub_);

                    path_line_markers_.points.clear();
                    path_points_markers_.points.clear();

                    RCLCPP_INFO(this->get_logger(),"Path calculated succesfully");
                }
            }else{
                RCLCPP_INFO(this->get_logger(),"Could not calculate path between request points");
            }
        }  
        if(_req.tries.data > 2){
            auto av_time = std::accumulate(times.begin(), times.end(), 0.0) / times.size(); 
            std::cout << "Average Time: "      << av_time  << " milisecs" << std::endl;
            std::cout << "Average Frequency: " << 1000/av_time << std::endl;
        }
        return true;
    }
    void configureAlgorithm(const std::string &algorithm_name, const std::string &_heuristic){

        float ws_x, ws_y, ws_z;

        this->declare_parameter<float>("world_size_x", 100.0);
        this->declare_parameter<float>("world_size_y", 100.0);
        this->declare_parameter<float>("world_size_z", 0.0);
        this->declare_parameter<float>("resolution", 0.2);
        this->declare_parameter<bool>("inflate_map", true);
        this->declare_parameter<bool>("use3d", (bool)true);

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
            this->declare_parameter<double>("inflation_size", 0.5);
            this->get_parameter("inflation_size", inflation_size);
            inflation_steps_ = std::round(inflation_size / resolution_);
            RCLCPP_INFO(this->get_logger(),"Inflation size %.2f, using inflation step %d", inflation_size, inflation_steps_);
        }
        algorithm_->setInflationConfig(inflate_, inflation_steps_);

        m_grid3d_.reset(new Grid3d); //TODO Costs not implement yet

        double cost_scaling_factor, robot_radius;
        this->declare_parameter<double>("cost_scaling_factor", 0.8);
        this->get_parameter("cost_scaling_factor", cost_scaling_factor);

        this->declare_parameter<double>("robot_radius", 0.4);
        this->get_parameter("robot_radius", robot_radius);
        
        m_grid3d_->setCostParams(cost_scaling_factor, robot_radius);
        
        std::string frame_id;
        // lnh_.param("frame_id", frame_id, std::string("map"));		
        this->declare_parameter<std::string>("frame_id", std::string("map"));
        this->get_parameter("frame_id", frame_id);

        configMarkers(algorithm_name, frame_id, resolution_);

        this->declare_parameter<bool>("save_data_file", (bool)true);
        this->get_parameter("save_data_file", save_data_);

        this->declare_parameter<std::string>("data_folder", std::string("planing_data.txt"));
        this->get_parameter("data_folder", data_folder_);

        if(save_data_)
            RCLCPP_INFO_STREAM(this->get_logger(),"Saving path planning data results to " << data_folder_);

        //
        if( input_map_ == 1 ){
            Planners::utils::configureWorldFromOccupancyWithCosts(occupancy_grid_, *algorithm_);
        }else if( input_map_ == 2 ){
            Planners::utils::configureWorldFromPointCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(cloud_), *algorithm_, resolution_);
            Planners::utils::configureWorldCosts(*m_grid3d_, *algorithm_);
        }
        //Algorithm specific parameters. Its important to set line of sight after configuring world size(it depends on the resolution)
        float sight_dist, cost_weight;
                                                                             //
        this->declare_parameter<float>("max_line_of_sight_distance", (float)1000.0);
        this->get_parameter("max_line_of_sight_distance", sight_dist);

                                                                             
        this->declare_parameter<float>("cost_weight", (float)0.0);
        this->get_parameter("cost_weight", cost_weight);

        algorithm_->setMaxLineOfSight(sight_dist);
        algorithm_->setCostFactor(cost_weight);

        this->declare_parameter<bool>("overlay_markers", (bool)false);
        this->get_parameter("overlay_markers", overlay_markers_);

    }
    void configureHeuristic(const std::string &_heuristic){
        
        if( _heuristic == "euclidean" ){
            algorithm_->setHeuristic(Planners::Heuristic::euclidean);
            RCLCPP_INFO(this->get_logger(),"Using Euclidean Heuristics");
        }else if( _heuristic == "euclidean_optimized" ){
            algorithm_->setHeuristic(Planners::Heuristic::euclideanOptimized);
            RCLCPP_INFO(this->get_logger(),"Using Optimized Euclidean Heuristics");
        }else if( _heuristic == "manhattan" ){
            algorithm_->setHeuristic(Planners::Heuristic::manhattan);
            RCLCPP_INFO(this->get_logger(),"Using Manhattan Heuristics");
        }else if( _heuristic == "octogonal" ){
            algorithm_->setHeuristic(Planners::Heuristic::octagonal);
            RCLCPP_INFO(this->get_logger(),"Using Octogonal Heuristics");
        }else if( _heuristic == "dijkstra" ){
            algorithm_->setHeuristic(Planners::Heuristic::dijkstra);     
            RCLCPP_INFO(this->get_logger(),"Using Dijkstra Heuristics");
        }else{
            algorithm_->setHeuristic(Planners::Heuristic::euclidean);
            RCLCPP_WARN(this->get_logger(),"Wrong Heuristic param. Using Euclidean Heuristics by default");
        }
    }
    std::vector<std::pair<Planners::utils::Vec3i, double>> getClosestObstaclesToPathPoints(const Planners::utils::CoordinateList &_path){
        
        std::vector<std::pair<Planners::utils::Vec3i, double>> result;
        if ( use3d_ ){
            //TODO grid3d distances does not take into account the inflation added internally by the algorithm

            for(const auto &it: _path)
                result.push_back( m_grid3d_->getClosestObstacle(it) );
            }

        else{//TODO IMplement for 2d
            result.push_back(std::make_pair<Planners::utils::Vec3i, double>(Planners::utils::Vec3i{0,0,0}, 0.0));
        }
        return result;
    }
    void configMarkers(const std::string &_ns, const std::string &_frame, const double &_scale){

        path_line_markers_.ns = _ns;
        path_line_markers_.header.frame_id = _frame;
        path_line_markers_.header.stamp = this->now();
        path_line_markers_.id = rand();
        path_line_markers_.lifetime = rclcpp::Duration::from_seconds(500);
        path_line_markers_.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path_line_markers_.action = visualization_msgs::msg::Marker::ADD;
        path_line_markers_.pose.orientation.w = 1;

        path_line_markers_.color.r = 0.0;
        path_line_markers_.color.g = 1.0;
        path_line_markers_.color.b = 0.0;

        path_line_markers_.color.a = 1.0;
        path_line_markers_.scale.x = _scale;

        path_points_markers_.ns = _ns;
        path_points_markers_.header.frame_id = _frame;
        path_points_markers_.header.stamp = this->now();
        path_points_markers_.id = rand();
        path_points_markers_.lifetime = rclcpp::Duration::from_seconds(500);
        path_points_markers_.type = visualization_msgs::msg::Marker::POINTS;
        path_points_markers_.action = visualization_msgs::msg::Marker::ADD;
        path_points_markers_.pose.orientation.w = 1;
        path_points_markers_.color.r = 0.0;
        path_points_markers_.color.g = 1.0;
        path_points_markers_.color.b = 1.0;
        path_points_markers_.color.a = 1.0;
        path_points_markers_.scale.x = _scale;
        path_points_markers_.scale.y = _scale;
        path_points_markers_.scale.z = _scale;

    }
    template <typename T>
    void publishMarker(visualization_msgs::msg::Marker &_marker, const T &_pub ){
        
        //Clear previous marker
        if( !overlay_markers_ ){
            _marker.action = visualization_msgs::msg::Marker::DELETEALL;
            _pub->publish(_marker);
        }else{
            path_points_markers_.id           = rand();
            path_points_markers_.header.stamp = this->now();
            setRandomColor(path_points_markers_.color);

            path_line_markers_.id             = rand();
            path_line_markers_.header.stamp   = this->now();
            setRandomColor(path_line_markers_.color);
        }
        _marker.action = visualization_msgs::msg::Marker::ADD;
        _pub->publish(_marker);
    }
    void setRandomColor(std_msgs::msg::ColorRGBA &_color, unsigned int _n_div = 20){
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


    // ros::NodeHandle lnh_{"~"};
    // ros::ServiceServer request_path_server_, change_planner_server_;

    // ros::Subscriber pointcloud_sub_, occupancy_grid_sub_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_sub_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr line_markers_pub_, point_markers_pub_;

    rclcpp::Service<heuristic_planners::srv::GetPath>::SharedPtr request_path_server_;
    rclcpp::Service<heuristic_planners::srv::SetAlgorithm>::SharedPtr change_planner_server_;

    std::unique_ptr<Grid3d> m_grid3d_;

    std::unique_ptr<Planners::AlgorithmBase> algorithm_;
        
    visualization_msgs::msg::Marker path_line_markers_, path_points_markers_;
    
    //Parameters
    Planners::utils::Vec3i world_size_; // Discrete
    //float resolution_ = 0.2;
    float resolution_;

    bool save_data_;
    bool use3d_{true};

    bool inflate_{false};
    unsigned int inflation_steps_{0};
    std::string data_folder_;
    bool overlay_markers_{0};
    unsigned int color_id_{0};
    nav_msgs::msg::OccupancyGrid occupancy_grid_;
    // TODO?: Change type of message --> sensor_msgs::msg::PointCloud2
    pcl::PointCloud<pcl::PointXYZ> cloud_;
    //sensor_msgs::msg::PointCloud2 cloud_; //JAC

    //0: no map yet
    //1: using occupancy
    //2: using cloud
    int input_map_{0};
    std::string heuristic_;

};
int main(int argc, char **argv)
{
    // ros::init(argc, argv, "heuristic_planner_ros_node");
    rclcpp::init(argc, argv);

    auto node = std::make_shared<HeuristicPlannerROS>();
    rclcpp::spin(node);

    // HeuristicPlannerROS heuristic_planner_ros;
    // ros::spin();

return 0;
}
