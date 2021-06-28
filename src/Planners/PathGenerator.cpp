#include "Planners/PathGenerator.hpp"

namespace Planners
{

    PathGenerator::PathGenerator(bool _use_3d = true){
        setHeuristic(&Heuristic::euclidean);
        CoordinateList directions2d, directions3d;
        directions2d = {
            { 0, 1, 0 }, {0, -1, 0}, { 1, 0, 0 }, { -1, 0, 0 }, //4 straight elements
            { 1, -1, 0 }, { -1, 1, 0 }, { -1, -1, 0 }, { 1, 1, 0 } //4 diagonal elements
        };
        directions3d = {

            { 0, 1, 0 }, {0, -1, 0}, { 1, 0, 0 }, { -1, 0, 0 }, { 0, 0, 1}, { 0, 0, -1}, //6 first elements

            { 1, -1, 0 }, { -1, 1, 0 }, { -1, -1, 0 }, { 1, 1, 0 },  { -1, 0, -1 }, //7-18 inclusive
            { 1, 0, 1 }, { 1, 0, -1 }, {-1, 0, 1}, { 0, -1, 1 }, { 0, 1, 1 }, { 0, 1, -1 },  { 0, -1, -1 }, 

            { -1, -1, 1 }, { 1, 1, 1 },  { -1, 1, 1 }, { 1, -1, 1 }, { -1, -1, -1 }, { 1, 1, -1 }, { -1, 1, -1 }, { 1, -1, -1 }, 
        };
        if(_use_3d){
            std::cout << "Using 3D Directions" << std::endl;
            direction = directions3d;
        }else{
            std::cout << "Using 2D Directions" << std::endl;
            direction = directions2d;
        }
    }
    void PathGenerator::setWorldSize(const Vec3i &_worldSize,const double _resolution)
    {
        discrete_world_.resizeWorld(_worldSize, _resolution);
    }

    void PathGenerator::setHeuristic(HeuristicFunction heuristic_)
    {
        heuristic = std::bind(heuristic_, std::placeholders::_1, std::placeholders::_2);
    }
    bool PathGenerator::configureCellCost(const Vec3i &coordinates_, const unsigned int &_cost){

        return discrete_world_.setNodeCost(coordinates_, _cost);
    }
    void PathGenerator::addCollision(const Vec3i &coordinates_, bool do_inflate, unsigned int steps)
    {
        if (do_inflate)
        {
            inflateNodeAsCube(coordinates_, direction, steps);
        }
        else
        {
            discrete_world_.setOccupied(coordinates_);
        }
    }
    void PathGenerator::addCollision(const Vec3i &coordinates_)
    {
        addCollision(coordinates_, do_inflate_, inflate_steps_);
    }
    bool PathGenerator::detectCollision(const Vec3i &coordinates_)
    {
        if (discrete_world_.isOccupied(coordinates_))
        {
            return true;
        }
        return false;
    }
    void PathGenerator::inflateNodeAsCube(const Vec3i &_ref, const CoordinateList &_directions, const unsigned int &_inflate_steps)
    {
        std::cout << "Inflating " << _ref  << "with " << _inflate_steps << " steps" << std::endl;
        for (const auto &it : _directions)
        {
            for (unsigned int i = 0; i < _inflate_steps; ++i)
            {
                auto new_vec = _ref + (i + 1) * it;
                // std::cout << "\tAdding " << new_vec << std::endl;
                discrete_world_.setOccupied(new_vec);
            }
        }
    }

}
