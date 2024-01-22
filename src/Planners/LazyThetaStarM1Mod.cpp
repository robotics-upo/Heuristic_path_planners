#include "Planners/LazyThetaStarM1Mod.hpp"

namespace Planners
{
    LazyThetaStarM1Mod::LazyThetaStarM1Mod(bool _use_3d):LazyThetaStarM1(_use_3d, "lazythetastarm1mod") {}
    LazyThetaStarM1Mod::LazyThetaStarM1Mod(bool _use_3d, std::string _name = "lazythetastarm1mod" ):LazyThetaStarM1(_use_3d, _name) {}
    
    void LazyThetaStarM1Mod::ComputeCost(Node *_s_aux, Node *_s2_aux)
    {
        auto distanceParent2 = geometry::distanceBetween2Nodes(_s_aux->parent, _s2_aux);
        // JAC Prueba: Quitar para original
        auto distanceParent2_nodes = LineOfSight::nodesInLineBetweenTwoNodes(_s_aux->parent, _s2_aux, discrete_world_, max_line_of_sight_cells_);  //REVISAR _s_aux->parent o _s_aux

        // No line of sight or distance greater than max_line_of_sight_cells
        if ( distanceParent2_nodes == 0 ){
            distanceParent2_nodes = 1;
        }
        // Line of sight
        else{
            auto cost_term = static_cast<unsigned int>(cost_weight_ * _s2_aux->cost * dist_scale_factor_reduced_);
            // auto cost_term = static_cast<unsigned int>((_s2_aux->cost/100) * dist_scale_factor_reduced_);  //NUEVO
            // auto cost_term = static_cast<unsigned int>(cost_weight_ * ((_s2_aux->cost+_s_aux->parent->cost)/2) * dist_scale_factor_reduced_) * distanceParent2_nodes; // JAC: Quitar para original
            if ( ( _s_aux->parent->G + distanceParent2 + cost_term ) < _s2_aux->G )
            {
                _s2_aux->parent = _s_aux->parent;
                // _s2_aux->G      = _s2_aux->parent->G + geometry::distanceBetween2Nodes(_s2_aux->parent, _s2_aux) +  cost_term;
                _s2_aux->G      = _s2_aux->parent->G + distanceParent2 +  cost_term;
                // _s2_aux->G      = _s2_aux->parent->G + distanceParent2 * cost_term; //NUEVO
                _s2_aux->C      = cost_term;      
                _s2_aux->gplush = _s2_aux->G + _s2_aux->H; 
            }
        }

        // Before else was not so the cost_term and the rest were computed although there were not line of sight or the distance were greater than max_los
        // CHANGED 30-Mar-2023
        // auto cost_term = static_cast<unsigned int>(cost_weight_ * _s2_aux->cost * dist_scale_factor_reduced_);
        // // auto cost_term = static_cast<unsigned int>(cost_weight_ * ((_s2_aux->cost+_s_aux->parent->cost)/2) * dist_scale_factor_reduced_) * distanceParent2_nodes; // JAC: Quitar para original
        // if ( ( _s_aux->parent->G + distanceParent2 + cost_term ) < _s2_aux->G )
        // {
        //     _s2_aux->parent = _s_aux->parent;
        //     _s2_aux->G      = _s2_aux->parent->G + geometry::distanceBetween2Nodes(_s2_aux->parent, _s2_aux) +  cost_term;
        //     _s2_aux->C      = cost_term;       
        // }
    }
    
}