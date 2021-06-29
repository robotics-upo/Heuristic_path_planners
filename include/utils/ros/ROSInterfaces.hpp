#ifndef ROSINTERFACES_HPP
#define ROSINTERFACES_HPP

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_2d/cost_values.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include "utils/utils.hpp"
#include "Planners/PathGenerator.hpp"

#include <math.h>

namespace Planners
{
    namespace utils
    {
        /**
         * @brief 
         *   
         * @param _point 
         * @param _res 
         * @return Vec3i 
         */
        Vec3i discretePoint(const pcl::PointXYZ &_point, const double &_res);

        /**
         * @brief 
         * 
         * @param _msg 
         * @param _res 
         * @return Vec3i 
         */
        Vec3i discretePoint(const geometry_msgs::Point &_msg, const double &_res);

        /**
         * @brief 
         * 
         * @param _msg 
         * @param _res 
         * @return Vec3i 
         */
        Vec3i discretePose(const geometry_msgs::Pose &_msg, const double &_res);
        /**
         * @brief 
         * 
         * @param _vec 
         * @param _res 
         * @return geometry_msgs::Point 
         */
        geometry_msgs::Point continousPoint(const Vec3i &_vec, const double &_res);
        /**
         * @brief 
         * 
         * @param _index 
         * @param _grid_width 
         * @return Vec3i 
         */
        inline Vec3i indexToXY(const unsigned int &_index, const unsigned int &_grid_width);
        /**
         * @brief 
         * 
         * @param _grid 
         * @param _algorithm 
         * @return true 
         * @return false 
         */
        bool configureWorldFromOccupancy(const nav_msgs::OccupancyGrid &_grid, PathGenerator &_algorithm, bool _set_size = false);

        /**
         * @brief 
         * 
         * @param _points 
         * @param _algorithm 
         * @return true 
         * @return false 
         */
        bool configureWorldFromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &_points, PathGenerator &_algorithm, const double &_resolution);
    }
}

#endif