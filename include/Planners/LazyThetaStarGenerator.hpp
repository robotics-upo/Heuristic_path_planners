#ifndef LAZYTHETASTAR_HPP
#define LAZYTHETASTAR_HPP

#include <Planners/ThetaStarGenerator.hpp>

namespace Planners
{

    class LazyThetaStarGenerator : public ThetaStarGenerator
    {

    public:
        
        LazyThetaStarGenerator(bool _use_3d ):ThetaStarGenerator(_use_3d) {}

        PathData findPath(const Vec3i &source_, const Vec3i &target_);

    protected:

        void ComputeCost(Node *s_aux, Node *s2_aux);

        void UpdateVertex(Node *s, Node *s2, NodeSet &openset);

        void SetVertex(Node *s_aux);
                
    };

}

#endif
