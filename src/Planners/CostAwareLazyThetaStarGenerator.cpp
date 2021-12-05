#include "Planners/CostAwareLazyThetaStarGenerator.hpp"

namespace Planners
{
    CostAwareLazyThetaStarGenerator::CostAwareLazyThetaStarGenerator(bool _use_3d):LazyThetaStarGenerator(_use_3d, "costlazythetastar") {}
    CostAwareLazyThetaStarGenerator::CostAwareLazyThetaStarGenerator(bool _use_3d, std::string _name = "costlazythetastar" ):LazyThetaStarGenerator(_use_3d, _name) {}
    
    void CostAwareLazyThetaStarGenerator::SetVertex(Node *_s_aux)
    {   
        line_of_sight_checks_++;
        if (!LineOfSight::bresenham3DWithMaxThreshold(_s_aux->parent, _s_aux, discrete_world_, max_line_of_sight_cells_ ))
        {
            unsigned int G_max = std::numeric_limits<unsigned int>::max(); 
            unsigned int G_new;

            for (const auto &i: direction)
            {
                Vec3i newCoordinates(_s_aux->coordinates + i);

                if ( discrete_world_.isOccupied(newCoordinates) ) continue;

                if ( discrete_world_.isInClosedList(newCoordinates) )
                {
                    Node *successor2 = discrete_world_.getNodePtr(newCoordinates);
                    if (successor2 == nullptr) continue;
                    
                    auto cost_term = static_cast<unsigned int>(cost_weight_ * successor2->cost * dist_scale_factor_reduced_);
                    G_new = successor2->G +  geometry::distanceBetween2Nodes(successor2, _s_aux) + cost_term; 
                    if (G_new < G_max)
                    {
                        _s_aux->parent = successor2;
                        _s_aux->G      = G_new;
                        _s_aux->C      = cost_term;
                        _s_aux->gplush = _s_aux->G + _s_aux->H;
                    }
                }
            }
        }
    }
    void CostAwareLazyThetaStarGenerator::ComputeCost(Node *_s_aux, Node *_s2_aux)
    {
        auto distanceParent2 = geometry::distanceBetween2Nodes(_s_aux->parent, _s2_aux);

        auto distanceParent2_nodes = LineOfSight::nodesInLineBetweenTwoNodes(_s_aux->parent, _s2_aux, discrete_world_, max_line_of_sight_cells_);  //REVISAR _s_aux->parent o _s_aux

        if ( distanceParent2_nodes == 0 )
            distanceParent2_nodes = 1;

        auto cost_term = static_cast<unsigned int>(cost_weight_ * _s2_aux->cost * dist_scale_factor_reduced_) * distanceParent2_nodes;
        if ( ( _s_aux->parent->G + distanceParent2 + cost_term ) < _s2_aux->G )
        {
            _s2_aux->parent = _s_aux->parent;
            _s2_aux->G      = _s2_aux->parent->G + geometry::distanceBetween2Nodes(_s2_aux->parent, _s2_aux) +  cost_term;
            _s2_aux->C      = cost_term;       
        }
    }


    unsigned int CostAwareLazyThetaStarGenerator::computeG(const Node* _current, Node* _suc,  unsigned int _n_i, unsigned int _dirs){

        unsigned int cost = _current->G;

        if(_dirs  == 8){
            cost += (_n_i < 4 ? dist_scale_factor_ : dd_2D_); //This is more efficient
        }else{
            cost += (_n_i < 6 ? dist_scale_factor_ : (_n_i < 18 ? dd_2D_ : dd_3D_)); //This is more efficient
        }

        auto cost_term = static_cast<unsigned int>(cost_weight_ * _suc->cost * dist_scale_factor_reduced_);
        cost += cost_term;
        _suc->C = cost_term;

        return cost;
    }
    
}