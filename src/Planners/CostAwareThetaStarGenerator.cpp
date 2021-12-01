#include "Planners/CostAwareThetaStarGenerator.hpp"

namespace Planners
{
    CostAwareThetaStarGenerator::CostAwareThetaStarGenerator(bool _use_3d):ThetaStarGenerator(_use_3d, "costthetastar") {}
    CostAwareThetaStarGenerator::CostAwareThetaStarGenerator(bool _use_3d, std::string _name = "costthetastar" ):ThetaStarGenerator(_use_3d, _name) {}

    void CostAwareThetaStarGenerator::ComputeCost(Node *_s_aux, Node *_s2_aux)
    {
        utils::CoordinateListPtr checked_nodes, checked_nodes_current;
        checked_nodes.reset(new CoordinateList);
        checked_nodes_current.reset(new CoordinateList);
        auto distanceParent2 = geometry::distanceBetween2Nodes(_s_aux->parent, _s2_aux);

        line_of_sight_checks_++;
        // if (LineOfSight::bresenham3DWithMaxThreshold((_s_aux->parent), _s2_aux, discrete_world_, max_line_of_sight_cells_))
        if (LineOfSight::bresenham3D((_s_aux->parent), _s2_aux, discrete_world_, checked_nodes))
        {
            auto n_checked_nodes = checked_nodes->size();
            if (n_checked_nodes==0)
                    n_checked_nodes = 1;

            // if ((_s_aux->parent->G + distanceParent2 + static_cast<unsigned int>(cost_weight_ * _s2_aux->cost)) < (_s2_aux->G))
            if (_s_aux->parent->G + distanceParent2 + ((static_cast<unsigned int>(cost_weight_ * _s2_aux->cost * (dist_scale_factor_/100)))*(n_checked_nodes)) < (_s2_aux->G))  // Conmensurable
            {
                _s2_aux->parent = _s_aux->parent;
                _s2_aux->G = _s_aux->parent->G + distanceParent2 +  (static_cast<unsigned int>(cost_weight_ * _s2_aux->cost * (dist_scale_factor_/100))*(n_checked_nodes));
                _s2_aux->C = static_cast<int>(cost_weight_ * _s2_aux->cost * (dist_scale_factor_/100)) * (n_checked_nodes);
            }

        } else {
            auto distance2 = geometry::distanceBetween2Nodes(_s_aux, _s2_aux);
            
            unsigned int G_new = _s_aux->G + distance2 + static_cast<unsigned int>(cost_weight_ * _s2_aux->cost * (dist_scale_factor_/100));

            if ( G_new < _s2_aux->G){
                _s2_aux->parent = _s_aux;
                _s2_aux->G = _s_aux->G + distance2 +  (static_cast<unsigned int>(cost_weight_ * _s2_aux->cost * (dist_scale_factor_/100)));
                _s2_aux->C = static_cast<int>(cost_weight_ * _s2_aux->cost * (dist_scale_factor_/100));                
            }
        }
    }
    unsigned int CostAwareThetaStarGenerator::computeG(const Node* _current, Node* _suc,  unsigned int _n_i, unsigned int _dirs){
        
        unsigned int cost = _current->G;

        if(_dirs  == 8){
            cost += (_n_i < 4 ? dist_scale_factor_ : dd_2D_); //This is more efficient
        }else{
            cost += (_n_i < 6 ? dist_scale_factor_ : (_n_i < 18 ? dd_2D_ : dd_3D_)); //This is more efficient
        }

        cost += static_cast<int>(cost_weight_ * _suc->cost * (dist_scale_factor_/100));

        _suc->C = static_cast<int>(cost_weight_ * _suc->cost * (dist_scale_factor_/100));
        
        return cost;
    }
}
