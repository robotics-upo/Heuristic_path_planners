#include "Planners/CostAwareLazyThetaStarGeneratorModified.hpp"

namespace Planners
{
    CostAwareLazyThetaStarGeneratorModified::CostAwareLazyThetaStarGeneratorModified(bool _use_3d):CostAwareLazyThetaStarGenerator(_use_3d, "costlazythetastarmodified") {}
    CostAwareLazyThetaStarGeneratorModified::CostAwareLazyThetaStarGeneratorModified(bool _use_3d, std::string _name = "costlazythetastarmodified" ):CostAwareLazyThetaStarGenerator(_use_3d, _name) {}
    
    inline void CostAwareLazyThetaStarGeneratorModified::ComputeCost(Node *_s_aux, Node *_s2_aux)
    {
        auto distanceParent2 = geometry::distanceBetween2Nodes(_s_aux->parent, _s2_aux);

        auto cost_term = static_cast<unsigned int>(cost_weight_ * _s2_aux->cost * dist_scale_factor_reduced_);
        if ( ( _s_aux->parent->G + distanceParent2 + cost_term ) < _s2_aux->G )
        {
            _s2_aux->parent = _s_aux->parent;
            _s2_aux->G      = _s2_aux->parent->G + geometry::distanceBetween2Nodes(_s2_aux->parent, _s2_aux) +  cost_term;
            _s2_aux->C      = cost_term;       
        }
    }
    
}