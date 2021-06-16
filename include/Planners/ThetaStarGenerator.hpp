#ifndef THETASTAR_HPP
#define THETASTAR_HPP

#include <Planners/AStarGenerator.hpp>

namespace Planners
{

    class ThetaStarGenerator : public AStarGenerator
    {

    public:
        
        ThetaStarGenerator() {}

        PathData findPath(const Vec3i &source_, const Vec3i &target_);

    protected:

        void UpdateVertex(Node *s, Node *s2, NodeSet &openset);

        void ComputeCost(Node *s_aux, Node *s2_aux);
    };

}

#endif
