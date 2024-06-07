#include "Planners/AStarM1.hpp"
#define map_res 0.2

namespace Planners{
    
AStarM1::AStarM1(bool _use_3d):AStar(_use_3d, "astarm1") {}
AStarM1::AStarM1(bool _use_3d, std::string _name = "astarm1" ):AStar(_use_3d, _name) {}

inline unsigned int AStarM1::computeG(const Node* _current, Node* _suc, unsigned int _n_i, unsigned int _dirs, HIOSDFNet& sdf_net){
    unsigned int cost = _current->G;

    if(_dirs == 8){
        cost += (_n_i < 4 ? dist_scale_factor_ : dd_2D_); //This is more efficient
    }else{
        cost += (_n_i < 6 ? dist_scale_factor_ : (_n_i < 18 ? dd_2D_ : dd_3D_)); //This is more efficient
    }
    float real_x = _suc->coordinates.x * map_res;
    float real_y = _suc->coordinates.y * map_res;
    float real_z = _suc->coordinates.z * map_res;

    // Query the net
    torch::Tensor input_tensor = torch::tensor({{real_x, real_y, real_z}}, torch::kFloat32);
    // Set the model to evaluation mode
    //sdf_net.eval();

    // Query the model
    torch::Tensor output_tensor = sdf_net.forward(input_tensor);

    // Convert the output tensor to a scalar value (assuming the model outputs a single value per input)
    float model_output = output_tensor.item<float>() * map_res;
    // auto cost_term = static_cast<unsigned int>(cost_weight_ * _suc->cost * dist_scale_factor_reduced_);
    
    //auto cost_term = static_cast<unsigned int>(cost_weight_ * model_output * dist_scale_factor_reduced_);

    auto cost_term = static_cast<unsigned int>(cost_weight_ * (1/(model_output * dist_scale_factor_reduced_)));

    // auto cost_term = static_cast<unsigned int>(cost_weight_ * (1/(((static_cast<double>(_current->cost) + static_cast<double>(_suc->cost))/2) * dist_scale_factor_reduced_)));


    
    std::cout << "Previous value: " << _suc->cost << "|| Actual value: " << model_output << std::endl;

    cost += cost_term;
    _suc->C = cost_term;
    
    return cost;
}
}
