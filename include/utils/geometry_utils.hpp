#ifndef GEOMETRY_UTILS_HPP
#define GEOMETRY_UTILS_HPP
/**
 * @file geometry_utils.hpp
 * @author Rafael Rey (rreyarc@upo.es)
 * @brief A set of geometry utilities functions
 * @version 0.1
 * @date 2021-06-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <iostream>
#include <vector>
#include <math.h>

#include "utils/utils.hpp"

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
             * @brief Discrete distance multiplied by dist_scale_factor_
             * 
             * @param n1 
             * @param n2 
             * @return unsigned int 
             */
            unsigned int distanceBetween2Nodes(const Node &n1, const Node &n2);
            /**
             * @brief  Discrete distance multiplied by dist_scale_factor_
             * 
             * @param n1 
             * @param n2 
             * @return unsigned int 
             */
            unsigned int distanceBetween2Nodes(const Node *n1, const Node *n2);
            
            /**
             * @brief Returns the absolute value vector 
             * 
             * @param _vec 
             * @return Vec3i 
             */
            Vec3i abs(const Vec3i &_vec);
            
        }//namespace geometry
    }//namespace utils
}

#endif