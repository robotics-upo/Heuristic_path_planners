#include "Planners/LazyThetaStarM1.hpp"

namespace Planners
{
    LazyThetaStarM1::LazyThetaStarM1(bool _use_3d):LazyThetaStar(_use_3d, "lazythetastarm1") {}
    LazyThetaStarM1::LazyThetaStarM1(bool _use_3d, std::string _name = "lazythetastarm1" ):LazyThetaStar(_use_3d, _name) {}
    
    void LazyThetaStarM1::SetVertex(Node *_s_aux)
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
                    // auto cost_term = static_cast<unsigned int>(cost_weight_ * successor2->cost * dist_scale_factor_reduced_);
                    // USE this cost_term when EDF decrease as distance increases
                    // auto cost_term = static_cast<unsigned int>(cost_weight_ * ((static_cast<double>(_s_aux->cost) + static_cast<double>(successor2->cost))/2) * dist_scale_factor_reduced_); //+grad_suc // IROS Paper 
                    // COST CONSIDERING EDF INCREASING AS DISTANCE INCREASE
                    auto cost_term = static_cast<unsigned int>(cost_weight_ * (1/(((static_cast<double>(_s_aux->cost) + static_cast<double>(successor2->cost))/2) * dist_scale_factor_reduced_)));
                    // G_new = successor2->G +  geometry::distanceBetween2Nodes(successor2, _s_aux) + cost_term; 
                    // auto cost_term = static_cast<unsigned int>((successor2->cost/100) * dist_scale_factor_reduced_); //NUEVO  ¿Por qué se divide entre 100?
                    G_new = successor2->G +  geometry::distanceBetween2Nodes(successor2, _s_aux) + cost_term;  //NUEVO //Estaba multiplicando por cost_term!!!!!
                    if (G_new < G_max)
                    {
                        G_max = G_new;
                        _s_aux->parent = successor2;
                        _s_aux->G      = G_new;
                        _s_aux->C      = cost_term;
                        _s_aux->gplush = _s_aux->G + _s_aux->H;
                    }
                }
            }
        }
    }
    inline void LazyThetaStarM1::ComputeCost(Node *_s_aux, Node *_s2_aux)
    {
        auto distanceParent2 = geometry::distanceBetween2Nodes(_s_aux->parent, _s2_aux);
        // std::cout << "distanceParent2: " << distanceParent2 << std::endl;
        // ROS_INFO("Compute COST");

        auto distanceParent2_nodes = LineOfSight::nodesInLineBetweenTwoNodes(_s_aux->parent, _s2_aux, discrete_world_, max_line_of_sight_cells_);  //REVISAR _s_aux->parent o _s_aux
        // std::cout << "distanceParent2_nodes: " << distanceParent2_nodes << std::endl;

        // No line of sight or distance greater than max_line_of_sight_cells
        if ( distanceParent2_nodes == 0 ){
            distanceParent2_nodes = 1;
        }

        // Line of sight
        else{
            // auto cost_term = static_cast<unsigned int>(cost_weight_ * ((_s_aux->parent->cost + _s2_aux->cost)/2) * dist_scale_factor_reduced_) * distanceParent2_nodes; // IROS Paper 

            // auto cost_term = static_cast<unsigned int>(cost_weight_ * (( static_cast<double>(_s_aux->parent->cost) + static_cast<double>(_s2_aux->cost) ) /2) * dist_scale_factor_reduced_) * distanceParent2_nodes; // IROS Paper 
            // COST CONSIDERING EDF INCREASING AS DISTANCE INCREASE
            auto cost_term = static_cast<unsigned int>(cost_weight_ * (1/(((static_cast<double>(_s_aux->parent->cost) + static_cast<double>(_s2_aux->cost))/2) * dist_scale_factor_reduced_))) * distanceParent2_nodes; 
            // auto cost_term = static_cast<unsigned int>(cost_weight_ * _s2_aux->cost * dist_scale_factor_reduced_) * distanceParent2_nodes;
            // auto cost_term = static_cast<unsigned int>(cost_weight_ * _s2_aux->cost * dist_scale_factor_reduced_) * distanceParent2;
            if ( ( _s_aux->parent->G + distanceParent2 + cost_term ) < _s2_aux->G )
            {
                _s2_aux->parent = _s_aux->parent;
                // _s2_aux->G      = _s2_aux->parent->G + geometry::distanceBetween2Nodes(_s2_aux->parent, _s2_aux) +  cost_term;
                _s2_aux->G      = _s2_aux->parent->G + distanceParent2 +  cost_term;
                _s2_aux->C      = cost_term;     
                _s2_aux->gplush = _s2_aux->G + _s2_aux->H;   
            }
        }

        // ROS_INFO("Using resolution: [%lf]", _s2_aux->cost);

        // Before else was not so the cost_term and the rest were computed although there were not line of sight or the distance were greater than max_los
        // CHANGED 30-Mar-2023
        // auto cost_term = static_cast<unsigned int>(cost_weight_ * _s2_aux->cost * dist_scale_factor_reduced_) * distanceParent2_nodes;
        // if ( ( _s_aux->parent->G + distanceParent2 + cost_term ) < _s2_aux->G )
        // {
        //     _s2_aux->parent = _s_aux->parent;
        //     _s2_aux->G      = _s2_aux->parent->G + geometry::distanceBetween2Nodes(_s2_aux->parent, _s2_aux) +  cost_term;
        //     _s2_aux->C      = cost_term;       
        // }
    }


    unsigned int LazyThetaStarM1::computeG(const Node* _current, Node* _suc,  unsigned int _n_i, unsigned int _dirs){

        unsigned int cost = _current->G;
        // EXT: Compute here the gradient from dd_2D_, dd_3D_ and each _suc->cost
        // float grad_suc; // or _suc->grad
        float dist_current_suc;


        if(_dirs  == 8){
            // cost += (_n_i < 4 ? dist_scale_factor_ : dd_2D_); //This is more efficient
            dist_current_suc = (_n_i < 4 ? dist_scale_factor_ : dd_2D_); //This is more efficient
            cost += dist_current_suc;
            
        }else{
            // cost += (_n_i < 6 ? dist_scale_factor_ : (_n_i < 18 ? dd_2D_ : dd_3D_)); //This is more efficient
            dist_current_suc = (_n_i < 6 ? dist_scale_factor_ : (_n_i < 18 ? dd_2D_ : dd_3D_)); //This is more efficient
            cost += dist_current_suc;
        }

        // grad_suc=(_suc->cost - _current->cost)/(dist_current_suc);
        // grad_suc=(_suc->cost - _current->cost);
        // std::cout << "current: " << _current->cost << std::endl;
        // std::cout << "suc: " << _suc->cost << std::endl;
        // std::cout << "gradiente: " << grad_suc << std::endl;
        // auto cost_term = static_cast<unsigned int>(cost_weight_ * _suc->cost * dist_scale_factor_reduced_); //+grad_suc

        // COST CONSIDERING EDF INCREASING AS DISTANCE INCREASE
        auto cost_term = static_cast<unsigned int>(cost_weight_ * (1/(((static_cast<double>(_current->cost) + static_cast<double>(_suc->cost))/2) * dist_scale_factor_reduced_))); 
        cost += cost_term;
        _suc->C = cost_term;

        return cost;
    }
    // // NUEVO
    // unsigned int LazyThetaStarM1::computeG(const Node* _current, Node* _suc,  unsigned int _n_i, unsigned int _dirs){

    //     unsigned int cost = _current->G;

    //     auto cost_term = static_cast<unsigned int>((_suc->cost/100) * dist_scale_factor_reduced_);
    //     if(_dirs  == 8){
    //         cost += (_n_i < 4 ? dist_scale_factor_ : dd_2D_)*cost_term; //This is more efficient
    //     }else{
    //         cost += (_n_i < 6 ? dist_scale_factor_ : (_n_i < 18 ? dd_2D_ : dd_3D_))*cost_term; //This is more efficient
    //     }

    //     // cost += cost_term;
    //     _suc->C = cost_term;

    //     return cost;
    // }
}