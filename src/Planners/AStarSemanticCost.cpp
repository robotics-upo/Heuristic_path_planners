#include "Planners/AStarSemanticCost.hpp"

namespace Planners{
    
AStarSemanticCost::AStarSemanticCost(bool _use_3d):AStar(_use_3d, "astarsemantic_cost") {}
AStarSemanticCost::AStarSemanticCost(bool _use_3d, std::string _name = "astarsemantic_cost" ):AStar(_use_3d, _name) {}

// // Cost Aware AStar
// inline unsigned int AStarSemantic::computeG(const Node* _current, Node* _suc, unsigned int _n_i, unsigned int _dirs){
    
//     unsigned int cost = _current->G;

//     if(_dirs == 8){
//         cost += (_n_i < 4 ? dist_scale_factor_ : dd_2D_); //This is more efficient
//     }else{
//         cost += (_n_i < 6 ? dist_scale_factor_ : (_n_i < 18 ? dd_2D_ : dd_3D_)); //This is more efficient
//     }
 
//     // auto cost_term = static_cast<unsigned int>(cost_weight_ * _suc->cost * dist_scale_factor_reduced_);
//     // IROS
//     // auto cost_term = static_cast<unsigned int>(cost_weight_ * ((_current->cost + _suc->cost)/2) * dist_scale_factor_reduced_); //+grad_suc 
//     // RA-L
//     auto cost_term = static_cast<unsigned int>(cost_weight_ * (1/(((static_cast<double>(_current->cost) + static_cast<double>(_suc->cost))/2) * dist_scale_factor_reduced_))); 
//     cost += cost_term;
//     _suc->C = cost_term;
    
//     return cost;
// }

inline unsigned int AStarSemanticCost::computeG(const Node* _current, Node* _suc,  unsigned int _n_i, unsigned int _dirs){
    unsigned int cost = _current->G;
    unsigned int cost2 = 0;

    float c_wall, c_door, c_colum, c_furnish, c_stair, c_panel, c_lamp, c_glass;
    c_wall=1;
    c_door=3;
    c_colum=2.5;
    c_furnish=1;
    c_stair=1.5;
    c_panel=1.5;
    c_lamp=2;
    c_glass=2;

    // c_wall=1;
    // c_door=1;
    // c_colum=1;
    // c_furnish=1;
    // c_stair=1;
    // c_panel=1;
    // c_lamp=1;
    // c_glass=1;


    if(_dirs  == 8){
        cost2 = (_n_i < 4 ? dist_scale_factor_ : dd_2D_); //This is more efficient
    }else{
        cost2 = (_n_i < 6 ? dist_scale_factor_ : (_n_i < 18 ? dd_2D_ : dd_3D_)); //This is more efficient
    }

    // if (_suc->semantic != 1){
        // std::cout << "cost: " << _suc->cost << std::endl;
        // std::cout << "semantic: " << _suc->semantic << std::endl;
        // usleep(1e4);
	    // std::cout << "Please a key to go to the next iteration..." << std::endl;
	    // getchar(); // Comentar para no usar tecla.
    // }
    
    // SEMANTIC COST
    // Wall
    if (_suc->semantic == 1){
        cost2=c_wall*cost2;
        // std::cout << "cost: " << _suc->cost << std::endl;
        // std::cout << "semantic: " << _suc->semantic << std::endl;
        // usleep(1e4);
	    // std::cout << "Please a key to go to the next iteration..." << std::endl;
	    // getchar(); // Comentar para no usar tecla.
    }
    // Door
    else if (_suc->semantic == 2){
        cost2=c_door*cost2;
        // std::cout << "cost: " << _suc->cost << std::endl;
        // std::cout << "semantic: " << _suc->semantic << std::endl;
        // usleep(1e4);
	    // std::cout << "Please a key to go to the next iteration..." << std::endl;
	    // getchar(); // Comentar para no usar tecla.
    }
    
    // Colum
    else if (_suc->semantic == 3){
        cost2=c_colum*cost2;
        // std::cout << "cost: " << _suc->cost << std::endl;
        // std::cout << "semantic: " << _suc->semantic << std::endl;
        // usleep(1e4);
	    // std::cout << "Please a key to go to the next iteration..." << std::endl;
	    // getchar(); // Comentar para no usar tecla.
    }
    // Furnishing
    else if (_suc->semantic == 4){
        cost2=c_furnish*cost2;
        // std::cout << "cost: " << _suc->cost << std::endl;
        // std::cout << "semantic: " << _suc->semantic << std::endl;
        // usleep(1e4);
	    // std::cout << "Please a key to go to the next iteration..." << std::endl;
	    // getchar(); // Comentar para no usar tecla.
    }
    // Stairs
    else if (_suc->semantic == 5){
        cost2=c_stair*cost2;
        // std::cout << "cost: " << _suc->cost << std::endl;
        // std::cout << "semantic: " << _suc->semantic << std::endl;
        // usleep(1e4);
	    // std::cout << "Please a key to go to the next iteration..." << std::endl;
	    // getchar(); // Comentar para no usar tecla.
    }
    // Panels --> Barandilla
    else if (_suc->semantic == 6){
        cost2=c_panel*cost2;
        // std::cout << "cost: " << _suc->cost << std::endl;
        // std::cout << "semantic: " << _suc->semantic << std::endl;
        // usleep(1e4);
	    // std::cout << "Please a key to go to the next iteration..." << std::endl;
	    // getchar(); // Comentar para no usar tecla.
    }
    // Lamp
    else if (_suc->semantic == 7){
        cost2=c_lamp*cost2;
        // std::cout << "cost: " << _suc->cost << std::endl;
        // std::cout << "semantic: " << _suc->semantic << std::endl;
        // usleep(1e4);
	    // std::cout << "Please a key to go to the next iteration..." << std::endl;
	    // getchar(); // Comentar para no usar tecla.
    }
    // Glass wall
    else if (_suc->semantic == 8){
        cost2=c_glass*cost2;
        // std::cout << "cost: " << _suc->cost << std::endl;
        // std::cout << "semantic: " << _suc->semantic << std::endl;
        // usleep(1e4);
	    // std::cout << "Please a key to go to the next iteration..." << std::endl;
	    // getchar(); // Comentar para no usar tecla.
    }

    cost = cost + cost2;
    
    _suc->C = _suc->cost;
    return cost;
}
}
