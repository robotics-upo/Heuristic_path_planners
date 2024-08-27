#include "Planners/ThetaStarM1.hpp"

namespace Planners
{
    ThetaStarM1::ThetaStarM1(bool _use_3d):ThetaStar(_use_3d, "thetastarm1") {}
    ThetaStarM1::ThetaStarM1(bool _use_3d, std::string _name = "thetastarm1" ):ThetaStar(_use_3d, _name) {}

    inline void ThetaStarM1::ComputeCost(Node *_s_aux, Node *_s2_aux, torch::jit::script::Module& loaded_sdf)
    {
        auto distanceParent2 = geometry::distanceBetween2Nodes(_s_aux->parent, _s2_aux);

        line_of_sight_checks_++;
        if (LineOfSight::bresenham3D((_s_aux->parent), _s2_aux, discrete_world_, checked_nodes))
        {
            auto n_checked_nodes = checked_nodes->size();
            if (n_checked_nodes==0)
                    n_checked_nodes = 1;

            auto cost_term = static_cast<unsigned int>(cost_weight_ / (_s2_aux->cost * dist_scale_factor_reduced_))* n_checked_nodes;
            if ( (_s_aux->parent->G + distanceParent2 + cost_term) < _s2_aux->G )  // Conmensurable
            {
                _s2_aux->parent = _s_aux->parent;
                _s2_aux->G      = _s_aux->parent->G + distanceParent2 + cost_term;
                _s2_aux->C      = cost_term;
                _s2_aux->gplush = _s2_aux->H + _s2_aux->G; 
            }

        } else {
            auto distance2 = geometry::distanceBetween2Nodes(_s_aux, _s2_aux);
            
            auto cost_term = static_cast<unsigned int>(cost_weight_ / (_s2_aux->cost * dist_scale_factor_reduced_));
            unsigned int G_new = _s_aux->G + distance2 + cost_term;

            if ( G_new < _s2_aux->G){
                _s2_aux->parent = _s_aux;
                _s2_aux->G      = _s_aux->G + distance2 + cost_term;
                _s2_aux->C      = cost_term; 
                _s2_aux->gplush = _s2_aux->H + _s2_aux->G; 
            }
        }
        checked_nodes->clear();
    }
    inline unsigned int ThetaStarM1::computeG(const Node* _current, Node* _suc,  unsigned int _n_i, unsigned int _dirs, torch::jit::script::Module& loaded_sdf){
        
        unsigned int cost = _current->G;

        if(_dirs  == 8){
            cost += (_n_i < 4 ? dist_scale_factor_ : dd_2D_); //This is more efficient
        }else{
            cost += (_n_i < 6 ? dist_scale_factor_ : (_n_i < 18 ? dd_2D_ : dd_3D_)); //This is more efficient
        }

        auto cost_term = static_cast<unsigned int>(cost_weight_ / (_suc->cost * dist_scale_factor_reduced_));
        cost   += cost_term;
        _suc->C = cost_term;
        
        return cost;
    }
}
