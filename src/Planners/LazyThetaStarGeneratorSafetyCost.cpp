#include "Planners/LazyThetaStarGeneratorSafetyCost.hpp"

namespace Planners
{
    LazyThetaStarGeneratorSafetyCost::LazyThetaStarGeneratorSafetyCost(bool _use_3d):ThetaStarGeneratorSafetyCost(_use_3d, "lazythetastarsafetycost") {}
    LazyThetaStarGeneratorSafetyCost::LazyThetaStarGeneratorSafetyCost(bool _use_3d, std::string _name = "lazythetastarsafetycost" ):ThetaStarGeneratorSafetyCost(_use_3d, _name) {}
    
    void LazyThetaStarGeneratorSafetyCost::SetVertex(Node *_s_aux)
    {   
        if( !los_neighbour_ ){

            unsigned int G_max = std::numeric_limits<unsigned int>::max(); 
            unsigned int G_new;

            for (const auto &i: direction)
            {
                Vec3i newCoordinates(_s_aux->coordinates + i);
                Node *successor2 = discrete_world_.getNodePtr(newCoordinates);
                if (successor2 == nullptr || successor2->occuppied ) continue;

                if ( successor2->isInClosedList ) 
                {
                    auto dist = geometry::distanceBetween2Nodes(successor2, _s_aux);

                    G_new  = static_cast<unsigned int>(  successor2-> G + dist +  
                    ( static_cast<double>(_s_aux->cost) + static_cast<double>(successor2->cost) ) / 2);

                    if (G_new < G_max)
                    {
                        _s_aux->parent = successor2;
                        _s_aux->G      = G_new;
                        _s_aux->C      = (static_cast<double>(_s_aux->cost) + static_cast<double>(successor2->cost)) / 2;
                        _s_aux->gplush = _s_aux->G + _s_aux->H;
                    }
                }
            }
        }
        los_neighbour_ = false;
    }

    inline void LazyThetaStarGeneratorSafetyCost::ComputeCost(Node *_s_aux, Node *_s2_aux)
    {
        utils::CoordinateListPtr checked_nodes;
        checked_nodes.reset(new CoordinateList);

        line_of_sight_checks_++;
        if (LineOfSight::bresenham3D(_s_aux->parent, _s2_aux, discrete_world_, checked_nodes)) {
            
            los_neighbour_ = true;

            auto dist2   = geometry::distanceBetween2Nodes(_s_aux->parent, _s2_aux);
            auto edge2   = ComputeEdgeCost(checked_nodes, _s_aux->parent, _s2_aux);

            if ( ( _s_aux->parent->G + dist2 + edge2 ) < _s2_aux->G ) 
            {
                _s2_aux->parent = _s_aux->parent;
                _s2_aux->G      = _s2_aux->parent->G + dist2 + edge2;
                _s2_aux->C      = edge2;
                _s2_aux->gplush = _s2_aux->G + _s2_aux->H;
            }            
        } 
    }

    inline unsigned int LazyThetaStarGeneratorSafetyCost::computeG(const Node* _current, Node* _suc,  unsigned int _n_i, unsigned int _dirs){

        unsigned int cost = 0;

        if(_dirs == 8){
            cost = (_n_i < 4 ? dist_scale_factor_ : dd_2D_); //This is more efficient
        }else{
            cost = (_n_i < 6 ? dist_scale_factor_ : (_n_i < 18 ? dd_2D_ : dd_3D_)); //This is more efficient
        }

        double cc = ( _current->cost + _suc->cost ) / 2;
        auto edge_neighbour = static_cast<unsigned int>( cc *  cost_weight_ * dist_scale_factor_reduced_); 
    
        cost += ( _current->G + edge_neighbour );

        _suc->C = edge_neighbour;
        
        return cost;
    }
}
