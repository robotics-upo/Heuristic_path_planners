#include "Planners/CostAwareAStarGenerator.hpp"

namespace Planners{
    
CostAwareAStarGenerator::CostAwareAStarGenerator(bool _use_3d):AStarGenerator(_use_3d, "costastar") {}
CostAwareAStarGenerator::CostAwareAStarGenerator(bool _use_3d, std::string _name = "costastar" ):AStarGenerator(_use_3d, _name) {}

unsigned int CostAwareAStarGenerator::computeG(const Node* _current, Node* _suc, unsigned int _n_i, unsigned int _dirs){
    
    unsigned int cost = _current->G;

    if(_dirs == 8){
        cost += (_n_i < 4 ? dist_scale_factor_ : dd_2D_); //This is more efficient
    }else{
        cost += (_n_i < 6 ? dist_scale_factor_ : (_n_i < 18 ? dd_2D_ : dd_3D_)); //This is more efficient
    }
 
    cost += static_cast<unsigned int>(cost_weight_ * _suc->cost * (dist_scale_factor_/100));

    _suc->C = static_cast<unsigned int>(cost_weight_ * _suc->cost * (dist_scale_factor_/100));
    
    return cost;
}
}
