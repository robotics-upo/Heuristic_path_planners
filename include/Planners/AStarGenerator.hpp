#ifndef ASTARGENERATOR_HPP
#define ASTARGENERATOR_HPP

#include <Planners/PathGenerator.hpp>

#ifdef ROS
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#endif

namespace Planners{

    class AStarGenerator : public PathGenerator
    {
        
    public:
        AStarGenerator();

        PathData findPath(const Vec3i &source_, const Vec3i &target_);
                
        void publishOccupationMarkersMap();
        
    protected:

#ifdef ROS
        ros::NodeHandle lnh_{"~"};
        ros::Publisher explored_nodes_marker_pub_, occupancy_marker_pub_;
        visualization_msgs::Marker explored_node_marker_;
        float resolution_;
    	pcl::PointCloud<pcl::PointXYZ>  occupancy_marker_; // Occupancy Map as PointCloud markers

#endif
    };

}

#endif 
