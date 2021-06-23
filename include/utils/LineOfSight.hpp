#ifndef BRESENHAM_HPP
#define BRESENHAM_HPP

#include "utils/utils.hpp"
#include "utils/world.hpp"
#include "utils/geometry_utils.hpp"

namespace Planners
{
    namespace utils
    {
        namespace LineOfSight
        {
            /**
             * @brief 
             * 
             * @param _lnode 
             * @param _rnode 
             * @param _world 
             * @return true 
             * @return false 
             */
            bool bresenham3D(const Node *_lnode, const Node *_rnode, DiscreteWorld &_world);
        }
    }
}

#endif