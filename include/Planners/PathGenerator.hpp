#ifndef PATHGENERATOR_HPP
#define PATHGENERATOR_HPP

#include <iostream>
#include <vector>
#include <set>
#include <functional>

#include <math.h>

#include "utils/world.hpp"
#include "utils/heuristic.hpp"
#include "utils/utils.hpp"
#include "utils/time.hpp"
#include "utils/geometry_utils.hpp"
#include "utils/LineOfSight.hpp"

namespace Planners
{
    using namespace utils;
    using HeuristicFunction = std::function<unsigned int(Vec3i, Vec3i)>;

    class Heuristic;
    class Clock;

    class PathGenerator
    {

    public:
        PathGenerator();

        void setWorldSize(Vec3i worldSize_, double _resolution);
        void setHeuristic(HeuristicFunction heuristic_);
        void addCollision(Vec3i coordinates_, bool do_inflate, bool steps);
        bool detectCollision(Vec3i coordinates_);

        PathData findPath(Vec3i source_, Vec3i target_);

    protected:
        void inflateNodeAsCube(const Vec3i &_ref,
                               const CoordinateList &_directions,
                               const unsigned int &_inflate_steps);
        
        HeuristicFunction heuristic;
        CoordinateList direction;

        utils::DiscreteWorld discrete_world_;

    private:
    };
}
#endif