#ifndef BRESENHAM_HPP
#define BRESENHAM_HPP
/**
 * @file LineOfSight.hpp
 * @author Rafael Rey (rreyarc@upo.es)
 * @brief Implementation of line of sight related algorithms
 * @version 0.1
 * @date 2021-06-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */
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
             * @param _visited_nodes 
             * @return true 
             * @return false 
             */
            bool bresenham3D(const Vec3i _lnode, const Vec3i _rnode, const DiscreteWorld &_world, CoordinateListPtr _visited_nodes);

            /**
             * @brief Implementation of bressenham 3D line of sight algorithm
             * 
             * @param _lnode First node
             * @param _rnode Second node
             * @param _world discrete world in which the nodes are stored. It is used
             * to check if the coordinates that the algorithm iterates are occupied/valids
             * @return true If there exists line of sight between both nodes
             * @return false If there is no line of sight between both nodes
             */
            bool bresenham3D(const Node *_lnode, const Node *_rnode, const DiscreteWorld &_world, CoordinateListPtr _visited_nodes = nullptr);


            /**
             * @brief 
             * 
             * @param _lnode 
             * @param _rnode 
             * @param _world 
             * @param _threshold 
             * @return true 
             * @return false 
             */
            bool bresenham3DWithMaxThreshold(const Node *_lnode, const Node *_rnode, const DiscreteWorld &_world, const unsigned int _threshold);
        

            int nodesInLineBetweenTwoNodes(const Node *_lnode, const Node *_rnode, const DiscreteWorld &_world, const unsigned int _threshold);
        }
    }
}

#endif