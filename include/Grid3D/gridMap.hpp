#ifndef _GRIDMAP_HPP_
#define _GRIDMAP_HPP_

#include <iostream>
#include <filesystem>
#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <nav_msgs/OccupancyGrid.h>


#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include <heuristic_planners/CellCostValue.h>

namespace Planners
{
    namespace utils
    {
        class GridMap
        {
        public:
            GridMap();

        private:

            bool readOctomap(const std::string &_path);

            bool computeGrid();

            double octree_resolution_{0.1};
        };
    } // namespace utils

} // namespace Planners

#endif