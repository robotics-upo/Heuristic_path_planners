#ifndef GEOMETRY_UTILS_HPP
#define GEOMETRY_UTILS_HPP

#include <iostream>
#include <vector>
#include "utils/utils.hpp"
#include <math.h>

namespace Planners
{
    namespace utils
    {
        namespace geometry
        {

            float calculatePathLength(const CoordinateList &_path, const double &_resolution);
            float distanceBetween2nodes(Node &n1, Node &n2);
            float distanceBetween2nodesOTRO(Node *n1, Node *n2);
            
            
        }//namespace geometry
    }//namespace utils
}

#endif