#ifndef GEOMETRY_UTILS_HPP
#define GEOMETRY_UTILS_HPP

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
             * @brief 
             * 
             * @param _path 
             * @param _resolution 
             * @return float 
             */
            float calculatePathLength(const CoordinateList &_path, const double &_resolution);
            /**
             * @brief 
             * 
             * @param n1 
             * @param n2 
             * @return unsigned int 
             */
            unsigned int distanceBetween2Nodes(const Node &n1, const Node &n2);
            /**
             * @brief 
             * 
             * @param n1 
             * @param n2 
             * @return unsigned int 
             */
            unsigned int distanceBetween2Nodes(const Node *n1, const Node *n2);
            
            /**
             * @brief 
             * 
             * @param _vec 
             * @return Vec3i 
             */
            Vec3i abs(const Vec3i &_vec);
            
        }//namespace geometry
    }//namespace utils
}

#endif