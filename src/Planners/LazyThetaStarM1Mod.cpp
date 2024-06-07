#include "Planners/LazyThetaStarM1Mod.hpp"

namespace Planners
{
    LazyThetaStarM1Mod::LazyThetaStarM1Mod(bool _use_3d):LazyThetaStarM1(_use_3d, "lazythetastarm1mod") {}
    LazyThetaStarM1Mod::LazyThetaStarM1Mod(bool _use_3d, std::string _name = "lazythetastarm1mod" ):LazyThetaStarM1(_use_3d, _name) {}
    
    void LazyThetaStarM1Mod::ComputeCost(Node *_s_aux, Node *_s2_aux)
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