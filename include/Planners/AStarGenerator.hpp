#ifndef ASTARGENERATOR_HPP
#define ASTARGENERATOR_HPP
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
#include <Planners/PathGenerator.hpp>

#ifdef ROS
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include "utils/ros/ROSInterfaces.hpp"
#endif

namespace Planners{

    
    class AStarGenerator : public PathGenerator
    {
        
    public:
        /**
         * @brief Construct a new AStarGenerator object
         * @param _use_3d This parameter allows the user to choose between planning on a plane (8 directions possibles) or in the 3D full space (26 directions)
         */
        AStarGenerator(bool _use_3d, std::string _name);
        AStarGenerator(bool _use_3d);
        /**
         * @brief Main function of the algorithm
         * 
         * @param _source Start discrete coordinates
         * @param _target Goal discrete coordinates
         * @return PathData PathData Results stored as PathData object
         */
        PathData findPath(const Vec3i &_source, const Vec3i &_target) override;
        
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
        template<typename T, typename U>
        void publishROSDebugData(const Node* _node, const T &_open_set, const U &_closed_set);
        
    protected:

        void configAlgorithm();
        /**
         * @brief 
         * 
         * @param _current 
         * @param _openset 
         */
        virtual void exploreNeighbours(Node* _current, const Vec3i &_target,node_by_position &_index_by_pos);

        /**
         * @brief 
         * 
         * @param _current 
         * @param _suc 
         * @param _n_i 
         * @param _dirs 
         * @return unsigned int 
         */
        virtual unsigned int computeG(const Node* _current, Node* _suc, unsigned int _n_i, unsigned int _dirs);

        unsigned int line_of_sight_checks_{0}; 
        std::vector<Node*> closedSet_;
        
#ifdef ROS
        ros::NodeHandle lnh_{"~"};
        ros::Publisher explored_nodes_marker_pub_, occupancy_marker_pub_, 
                       openset_marker_pub_, closedset_marker_pub_,
                       best_node_marker_pub_, aux_text_marker_pub_;
        visualization_msgs::Marker explored_node_marker_, openset_markers_, 
                                   closed_set_markers_, best_node_marker_, aux_text_marker_;
        ros::Duration duration_pub_{0.001};
        ros::Time last_publish_tamp_;
        float resolution_;
    	pcl::PointCloud<pcl::PointXYZ>  occupancy_marker_; // Occupancy Map as PointCloud markers

#endif
    };

}

#endif 
