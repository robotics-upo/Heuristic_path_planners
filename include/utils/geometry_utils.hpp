#ifndef GEOMETRY_UTILS_HPP
#define GEOMETRY_UTILS_HPP
/**
 * @file geometry_utils.hpp
 * @author Rafael Rey (reyarcenegui@gmail.com)
* @author Jose Antonio Cobano (jacobsua@upo.es)
 * @brief A set of geometry utilities functions
 * @version 0.1
 * @date 2021-06-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <iostream>
#include <vector>
#include <cmath>
#ifdef ROS
#include <Eigen/Dense>
#endif
#include "utils/utils.hpp"
#include "utils/LineOfSight.hpp"
#include "utils/world.hpp"


namespace Planners
{
    namespace utils
    {
        namespace geometry
        {
            /**
             * @brief Return the integrated distance alongside the _path object with the _resolution given
             * 
             * @param _path CoordinateList object of points (discrete)
             * @param _resolution 
             * @return float continous path length
             */
            float calculatePathLength(const CoordinateList &_path, const double &_resolution);

            /**
             * @brief Get the Adjacent Path object
             * 
             * @param _path 
             * @param _world 
             * @return utils::CoordinateList 
             */
            utils::CoordinateList getAdjacentPath(const utils::CoordinateList &_path, const utils::DiscreteWorld &_world);
            /**
             * @brief Discrete distance multiplied by dist_scale_factor_
             * 
             * @param _n1 
             * @param _n2 
             * @return unsigned int 
             */
            unsigned int distanceBetween2Nodes(const Node &_n1, const Node &_n2);
            /**
             * @brief  Discrete distance multiplied by dist_scale_factor_
             * 
             * @param _n1 
             * @param _n2 
             * @return unsigned int 
             */
            unsigned int distanceBetween2Nodes(const Node *_n1, const Node *_n2);
            
            /**
             * @brief 
             * 
             * @param _v1 
             * @param _v2 
             * @return unsigned int 
             */
            unsigned int distanceBetween2Nodes(const Vec3i &_v1, const Vec3i &_v2);
            /**
             * @brief Discrete distance 
             * 
             * @param _n1 
             * @param _n2 
             * @return unsigned int 
             */
            unsigned int NodesBetween2Nodes(const Node &_n1, const Node &_n2);
            /**
             * @brief  Discrete distance             * 
             * @param _n1 
             * @param _n2 
             * @return unsigned int 
             */
            unsigned int NodesBetween2Nodes(const Node *_n1, const Node *_n2);
            
            /**
             * @brief 
             * 
             * @param _v1 
             * @param _v2 
             * @return unsigned int 
             */
            unsigned int NodesBetween2Nodes(const Vec3i &_v1, const Vec3i &_v2);

            /**
             * @brief Returns the absolute value vector 
             * 
             * @param _vec 
             * @return Vec3i 
             */
            Vec3i abs(const Vec3i &_vec);

            /**
             * @brief 
             * 
             * @param _v1 
             * @param _v2 
             * @return int 
             */
            int dotProduct(const Vec3i &_v1, const Vec3i &_v2);

            /**
             * @brief 
             * 
             * @param _v 
             * @return double 
             */
            double moduleVector(const Vec3i &_v);

            /**
             * @brief 
             * 
             * @param _v1 
             * @param _v2 
             * @param _v3 
             * @return double 
             */
            double angleBetweenThreePoints(const Vec3i &_v1, const Vec3i &_v2, const Vec3i &_v3);

#ifdef ROS
            /**
             * @brief 
             * 
             * @param _v1 
             * @param _v2 
             * @param _v3 
             * @return double 
             */
            double angleBetweenThreePoints(const Eigen::Vector3d &_v1, const Eigen::Vector3d &_v2, const Eigen::Vector3d &_v3);

            /**
             * @brief Get the Circunference Radius object
             * 
             * @param _v1 
             * @param _v2 
             * @param _v3 
             * @return double 
             */
            double getCircunferenceRadius(const Vec3i &_v1, const Vec3i &_v2, const Vec3i &_v3);

            /**
             * @brief Get the Circunference Radius object
             * 
             * @param _v1 
             * @param _v2 
             * @param _v3 
             * @return double 
             */
            double getCircunferenceRadius(const Eigen::Vector3d &_v1, const Eigen::Vector3d &_v2, const Eigen::Vector3d &_v3);
#endif            
        }//namespace geometry
    }//namespace utils
}

#endif