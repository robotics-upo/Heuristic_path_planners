#include "Planners/AStarGeneratorSafetyCost.hpp"

namespace Planners{
    
AStarGeneratorSafetyCost::AStarGeneratorSafetyCost(bool _use_3d):AStarGenerator(_use_3d, "astarsafety") {}
AStarGeneratorSafetyCost::AStarGeneratorSafetyCost(bool _use_3d, std::string _name = "astarsafety" ):AStarGenerator(_use_3d, _name) {}


unsigned int AStarGeneratorSafetyCost::computeG(const Node* _current, Node* _suc, unsigned int _n_i, unsigned int _dirs){
    
    unsigned int cost = 0;

    if(_dirs == 8){
        cost = (_n_i < 4 ? dist_scale_factor_ : dd_2D_); //This is more efficient
    }else{
        cost = (_n_i < 6 ? dist_scale_factor_ : (_n_i < 18 ? dd_2D_ : dd_3D_)); //This is more efficient
    }

    //TODO Give bb a more precise name
    // double bb = static_cast<double>( static_cast<double>(_suc->cost) / (static_cast<double>(cost) / static_cast<double>(dist_scale_factor_)) );
    // auto edge_neighbour = static_cast<unsigned int>( ( ( ( _current->cost + bb ) / ( 2 * 100 ) ) * cost ) *  cost_weight_);

    // CONMENSURABLE
    double bb = static_cast<double>( static_cast<double>(_suc->cost) );
    auto edge_neighbour = static_cast<unsigned int>( ((_current->cost + bb )/2) *  cost_weight_); 
    
    cost += ( _current->G + edge_neighbour );

    _suc->C = edge_neighbour;
    
    return cost;
}
}
