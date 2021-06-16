#ifndef THETASTAR_HPP
#define THETASTAR_HPP

#include <Planners/AStarGenerator.hpp>

namespace Planners
{

    class LazyThetaStarGenerator : public AStarGenerator
    {

    public:
        
        LazyThetaStarGenerator() {}

        PathData findPath(Vec3i source_, Vec3i target_);

        void UpdateVertex(Node *s, Node *s2, NodeSet &openset);
        void ComputeCost(Node *s_aux, Node *s2_aux);
        //void UpdateVertex(Node &s, Node &s2, NodeSet &openset);
        //void ComputeCost(Node &s, Node &s2);
                
    };

}

#endif
