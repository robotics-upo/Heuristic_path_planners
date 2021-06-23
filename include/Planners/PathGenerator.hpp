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

        void setWorldSize(const Vec3i &worldSize_,const double _resolution);
        void setHeuristic(HeuristicFunction heuristic_);
        
        void addCollision(const Vec3i &coordinates_, bool do_inflate, bool steps);
        void addCollision(const Vec3i &coordinates_);
        bool configureCellCost(const Vec3i &coordinates_, const unsigned int &_cost);

        bool detectCollision(const Vec3i &coordinates_);

        virtual PathData findPath(const Vec3i &source_, const Vec3i &target_) = 0;

        void setInflationConfig(const bool _inflate, const unsigned int _inflation_steps) 
        { do_inflate_ = _inflate; inflate_steps_ = _inflation_steps;}

        virtual void publishOccupationMarkersMap() = 0;

    protected:
        void inflateNodeAsCube(const Vec3i &_ref,
                               const CoordinateList &_directions,
                               const unsigned int &_inflate_steps);
        
        HeuristicFunction heuristic;
        CoordinateList direction;

        utils::DiscreteWorld discrete_world_;
        unsigned int inflate_steps_{1};
        bool do_inflate_{true};


    private:
    };
}
#endif