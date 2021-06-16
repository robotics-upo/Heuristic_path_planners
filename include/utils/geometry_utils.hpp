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

            float calculatePathLength(const CoordinateList &_path, const double &_resolution);
            float distanceBetween2Nodes(const Node &n1, const Node &n2);
            float distanceBetween2Nodes(const Node *n1, const Node *n2);
            
            
        }//namespace geometry
    }//namespace utils
}

#endif