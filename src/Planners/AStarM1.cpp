#include "Planners/AStarM1.hpp"
#define map_res 0.2

namespace Planners{
    
AStarM1::AStarM1(bool _use_3d):AStar(_use_3d, "astarm1") {}
AStarM1::AStarM1(bool _use_3d, std::string _name = "astarm1" ):AStar(_use_3d, _name) {}

inline unsigned int AStarM1::computeG(const Node* _current, Node* _suc, unsigned int _n_i, unsigned int _dirs, torch::jit::script::Module& loaded_sdf){
    unsigned int cost = _current->G;
    
    if(_dirs == 8){
        cost += (_n_i < 4 ? dist_scale_factor_ : dd_2D_); //This is more efficient
    }else{
        cost += (_n_i < 6 ? dist_scale_factor_ : (_n_i < 18 ? dd_2D_ : dd_3D_)); //This is more efficient
    }

    auto cost_term = static_cast<unsigned int>(cost_weight_ * (1/(_suc->cost * dist_scale_factor_reduced_)));

     std::cout << "Previous cost: " << _suc->cost << std::endl;
    
    cost += cost_term;
    _suc->C = cost_term;
    
    return cost;
}
}