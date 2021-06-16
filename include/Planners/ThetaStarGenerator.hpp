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

        void UpdateVertex(Node *s, Node *s2, NodeSet &openset);
        void ComputeCost(Node *s_aux, Node *s2_aux);
        //void UpdateVertex(Node &s, Node &s2, NodeSet &openset);
        //void ComputeCost(Node &s, Node &s2);
        //Lazy: Check if exist line of sight.
		void SetVertex(Node *s_aux, NodeSet &openset);
        void ComputeCostLazy(Node *s_aux, Node *s2_aux);
    };

}

#endif
