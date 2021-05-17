#ifndef THETASTAR_HPP
#define THETASTAR_HPP

#include <Planners/AStarGenerator.hpp>

namespace Planners
{

    class ThetaStarGenerator : public AStarGenerator
    {

    public:
        ThetaStarGenerator() {}

        PathData findPath(Vec3i source_, Vec3i target_);
        
    };

}

#endif
