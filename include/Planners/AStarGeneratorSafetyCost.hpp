#ifndef ASTARSATETY_HPP
#define ASTARSAFETY_HPP
/**
 * @file AStarGenerator.hpp
 * @author Rafael Rey (rreyarc@upo.es)
 * @brief 
 * @version 0.1
 * @date 2021-06-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */
// #include <Planners/PathGenerator.hpp>
#include <Planners/AStarGenerator.hpp>

#ifdef ROS
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include "utils/ros/ROSInterfaces.hpp"
#endif

namespace Planners{

    
    class AStarGeneratorSafetyCost : public AStarGenerator
    {
        
    public:
        /**
         * @brief Construct a new AStarGenerator object
         * @param _use_3d This parameter allows the user to choose between planning on a plane (8 directions possibles) or in the 3D full space (26 directions)
         */
        AStarGeneratorSafetyCost(bool _use_3d ):AStarGenerator(_use_3d) {}
        // AStarGenerator(bool _use_3d);
        /**
         * @brief Main function of the algorithm
         * 
         * @param _source Start discrete coordinates
         * @param _target Goal discrete coordinates
         * @return PathData PathData Results stored as PathData object
         */
        virtual PathData findPath(const Vec3i &_source, const Vec3i &_target) override;
        
        /**
         * @brief Published occupation markers map to visualize the loaded map in RVIZ
         * if the package is compiled without the ROS definition in the CMakeLists, this function is empty
         */
        void publishOccupationMarkersMap() override;
        
        /**
         * @brief 
         * 
         * @param _node 
         * @param _open_set 
         * @param _closed_set 
         */
        void publishROSDebugData(const Node* _node, const NodeSet &_open_set, const NodeSet &_closed_set);
        
    protected:

#ifdef ROS
        ros::NodeHandle lnh_{"~"};
        ros::Publisher explored_nodes_marker_pub_, occupancy_marker_pub_, 
                       openset_marker_pub_, closedset_marker_pub_,
                       best_node_marker_pub_, aux_text_marker_pub_;
        visualization_msgs::Marker explored_node_marker_, openset_markers_, 
                                   closed_set_markers_, best_node_marker_, aux_text_marker_;
        float resolution_;
    	pcl::PointCloud<pcl::PointXYZ>  occupancy_marker_; // Occupancy Map as PointCloud markers

#endif
    };

}

#endif 
