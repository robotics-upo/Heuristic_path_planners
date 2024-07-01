#include "Planners/LazyThetaStarSIREN.hpp"
#define map_res 0.2

namespace Planners
{
    LazyThetaStarSIREN::LazyThetaStarSIREN(bool _use_3d):LazyThetaStar(_use_3d, "lazythetastarsiren") {}
    LazyThetaStarSIREN::LazyThetaStarSIREN(bool _use_3d, std::string _name = "lazythetastarsiren" ):LazyThetaStar(_use_3d, _name) {}
    
    void LazyThetaStarSIREN::SetVertex(Node *_s_aux, torch::jit::script::Module& loaded_sdf)
    {   
        line_of_sight_checks_++;

        if (!LineOfSight::bresenham3DWithMaxThreshold(_s_aux->parent, _s_aux, discrete_world_, max_line_of_sight_cells_ ))
        {
            unsigned int G_max = std::numeric_limits<unsigned int>::max(); 
            unsigned int G_new;

            for (const auto &i: direction)
            {
                Vec3i newCoordinates(_s_aux->coordinates + i);
                Node *successor2 = discrete_world_.getNodePtr(newCoordinates);
                if (successor2 == nullptr || successor2->occuppied ) continue;

                if ( successor2->isInClosedList ) 
                {
                    // Query the net
                    float real_x = successor2->coordinates.x * map_res;
                    float real_y = successor2->coordinates.y * map_res;
                    float real_z = successor2->coordinates.z * map_res;
                    torch::Tensor input_tensor = torch::tensor({{real_x, real_y, real_z}}, torch::kFloat32).to(torch::kCUDA);
                    std::vector<torch::jit::IValue> inputs;
                    inputs.push_back(input_tensor);
                    auto start = std::chrono::high_resolution_clock::now();
                    at::Tensor output_tensor = loaded_sdf.forward(inputs).toTensor();
                    auto end = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<double, std::milli> duration = end - start;
                    float model_output = output_tensor.item<float>();

                    auto cost_term = static_cast<unsigned int>(cost_weight_ / (model_output * dist_scale_factor_reduced_));
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
    inline void LazyThetaStarSIREN::ComputeCost(Node *_s_aux, Node *_s2_aux, torch::jit::script::Module& loaded_sdf)
    {
        auto distanceParent2 = geometry::distanceBetween2Nodes(_s_aux->parent, _s2_aux);

        auto distanceParent2_nodes = LineOfSight::nodesInLineBetweenTwoNodes(_s_aux->parent, _s2_aux, discrete_world_, max_line_of_sight_cells_);  //REVISAR _s_aux->parent o _s_aux

        // Query the net
        float real_x = _s2_aux->coordinates.x * map_res;
        float real_y = _s2_aux->coordinates.y * map_res;
        float real_z = _s2_aux->coordinates.z * map_res;
        torch::Tensor input_tensor = torch::tensor({{real_x, real_y, real_z}}, torch::kFloat32).to(torch::kCUDA);
        std::vector<torch::jit::IValue> inputs;
        inputs.push_back(input_tensor);
        auto start = std::chrono::high_resolution_clock::now();
        at::Tensor output_tensor = loaded_sdf.forward(inputs).toTensor();
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration = end - start;
        float model_output = output_tensor.item<float>();

        if ( distanceParent2_nodes == 0 )
            distanceParent2_nodes = 1;

        auto cost_term = static_cast<unsigned int>(cost_weight_ / (model_output * dist_scale_factor_reduced_)) * distanceParent2_nodes;
        if ( ( _s_aux->parent->G + distanceParent2 + cost_term ) < _s2_aux->G )
        {
            _s2_aux->parent = _s_aux->parent;
            _s2_aux->G      = _s2_aux->parent->G + geometry::distanceBetween2Nodes(_s2_aux->parent, _s2_aux) +  cost_term;
            _s2_aux->C      = cost_term;       
        }
    }


    unsigned int LazyThetaStarSIREN::computeG(const Node* _current, Node* _suc,  unsigned int _n_i, unsigned int _dirs, torch::jit::script::Module& loaded_sdf){
    
        unsigned int cost = _current->G;

        if(_dirs  == 8){
            cost += (_n_i < 4 ? dist_scale_factor_ : dd_2D_); //This is more efficient
        }else{
            cost += (_n_i < 6 ? dist_scale_factor_ : (_n_i < 18 ? dd_2D_ : dd_3D_)); //This is more efficient
        }

        // Query the net
        float real_x = _suc->coordinates.x * map_res;
        float real_y = _suc->coordinates.y * map_res;
        float real_z = _suc->coordinates.z * map_res;
        torch::Tensor input_tensor = torch::tensor({{real_x, real_y, real_z}}, torch::kFloat32).to(torch::kCUDA);
        std::vector<torch::jit::IValue> inputs;
        inputs.push_back(input_tensor);
        auto start = std::chrono::high_resolution_clock::now();
        at::Tensor output_tensor = loaded_sdf.forward(inputs).toTensor();
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration = end - start;
        float model_output = output_tensor.item<float>();

        auto cost_term = static_cast<unsigned int>(cost_weight_ / (model_output * dist_scale_factor_reduced_));
        cost += cost_term;
        _suc->C = cost_term;

        return cost;
    }
    
}