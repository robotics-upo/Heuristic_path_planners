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

        PathData findPath(Vec3i source_, Vec3i target_);
        
        void addCollision(Vec3i coordinates_);
        
        void publishOccupationMarkersMap();
        
        void setInflationConfig(bool _inflate, unsigned int _inflation_steps) 
        { do_inflate_ = _inflate; inflate_steps_ = _inflation_steps;}
        
    private:
       
        unsigned int inflate_steps_{5};
        bool do_inflate_{true};

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
