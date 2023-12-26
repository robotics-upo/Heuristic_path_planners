#include "Planners/LazyThetaStar_EDF.hpp"

namespace Planners
{
    LazyThetaStarEDF::LazyThetaStarEDF(bool _use_3d):ThetaStar(_use_3d, "lazythetastaredf") {}
    LazyThetaStarEDF::LazyThetaStarEDF(bool _use_3d, std::string _name = "lazythetastaredf" ):ThetaStar(_use_3d, _name) {}
   
   // SetVertex original of the LazyThetaStar
    // void LazyThetaStarGradient::SetVertex(Node *_s_aux)
    // {
    //     line_of_sight_checks_++;

    //     if (!LineOfSight::bresenham3D((_s_aux->parent), _s_aux, discrete_world_))
    //     {
    //         unsigned int G_max = std::numeric_limits<unsigned int>::max(); 
    //         unsigned int G_new;

    //         for (const auto &i: direction)
    //         {
    //             Vec3i newCoordinates(_s_aux->coordinates + i);
    //             Node *successor2 = discrete_world_.getNodePtr(newCoordinates);

    //             if (successor2 == nullptr || successor2->occuppied ) continue;

    //             if ( successor2->isInClosedList ) 
    //             {
    //                 G_new = successor2->G + geometry::distanceBetween2Nodes(successor2, _s_aux);
    //                 if (G_new < G_max)
    //                 {
    //                     G_max = G_new;
    //                     _s_aux->parent = successor2;
    //                     _s_aux->G = G_new;
    //                     _s_aux->gplush = _s_aux->G + _s_aux->H;
    //                 }
    //             }
    //         }
    //     }
    // }

    // // This is the SetVertex of the IROS paper
    // void LazyThetaStarEDF::SetVertex(Node *_s_aux)
    // {   
    //     line_of_sight_checks_++;
    //     if (!LineOfSight::bresenham3DWithMaxThreshold(_s_aux->parent, _s_aux, discrete_world_, max_line_of_sight_cells_ ))
    //     {
    //         unsigned int G_max = std::numeric_limits<unsigned int>::max(); 
    //         unsigned int G_new;

    //         for (const auto &i: direction)
    //         {
    //             Vec3i newCoordinates(_s_aux->coordinates + i);
    //             Node *successor2 = discrete_world_.getNodePtr(newCoordinates);
    //             if (successor2 == nullptr || successor2->occuppied ) continue;

    //             if ( successor2->isInClosedList ) 
    //             {
    //                 cost_weight_ = (4*successor2->cost - _s_aux->cost);
    //                 auto cost_term = static_cast<unsigned int>(cost_weight_ * ((successor2->cost + _s_aux->cost)/2) * dist_scale_factor_reduced_);
    //                 // auto cost_term = static_cast<unsigned int>(cost_weight_ * successor2->cost * dist_scale_factor_reduced_);  // ESTABA ESTE 09-05-2023

    //                 // G_new = successor2->G +  geometry::distanceBetween2Nodes(successor2, _s_aux) + cost_term; 
    //                 // auto cost_term = static_cast<unsigned int>((successor2->cost/100) * dist_scale_factor_reduced_); //NUEVO
    //                 G_new = successor2->G +  geometry::distanceBetween2Nodes(successor2, _s_aux) + cost_term;  //NUEVO
    //                 if (G_new < G_max)
    //                 {
    //                     _s_aux->parent = successor2;
    //                     _s_aux->G      = G_new;
    //                     _s_aux->C      = cost_term;
    //                     _s_aux->gplush = _s_aux->G + _s_aux->H;
    //                 }
    //             }
    //         }
    //     }
    // }

// This is the SetVertex to test
    void LazyThetaStarEDF::SetVertex(Node *_s_aux)
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
                    // auto cost_term = static_cast<unsigned int>(successor2->cost * dist_scale_factor_reduced_);
                    // auto cost_term = static_cast<unsigned int>(cost_weight_ * successor2->cost * dist_scale_factor_reduced_);
                    // auto cost_term = static_cast<unsigned int>(cost_weight_ * ((static_cast<double>(_s_aux->cost) + static_cast<double>(successor2->cost))/2) * dist_scale_factor_reduced_); //+grad_suc // IROS Paper 
                    // COST CONSIDERING EDF INCREASING AS DISTANCE INCREASE
                    // auto cost_term = static_cast<unsigned int>(cost_weight_ * (1/(((static_cast<double>(_s_aux->cost) + static_cast<double>(successor2->cost))/2) * dist_scale_factor_reduced_)));
                    auto cost_term = static_cast<unsigned int>(cost_weight_ * (1/(((static_cast<double>(_s_aux->cost) + static_cast<double>(successor2->cost))*(0.5)) * dist_scale_factor_reduced_)));
                    G_new = successor2->G +  geometry::distanceBetween2Nodes(successor2, _s_aux) + cost_term; 
                    // auto cost_term = static_cast<unsigned int>((successor2->cost/100) * dist_scale_factor_reduced_); //NUEVO
                    // G_new = successor2->G +  (geometry::distanceBetween2Nodes(successor2, _s_aux)  cost_term);  //NUEVO
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

    // ComputeCost original of the LazyThetaStar
    // inline void LazyThetaStarEDF::ComputeCost(Node *_s_aux, Node *_s2_aux)
    // {
    //     auto distanceParent2 = geometry::distanceBetween2Nodes(_s_aux->parent, _s2_aux);

    //     if ( (_s_aux->parent->G + distanceParent2) < _s2_aux->G )
    //     {
    //         _s2_aux->parent = _s_aux->parent;
    //         _s2_aux->G      = _s2_aux->parent->G + distanceParent2;
    //         _s2_aux->gplush = _s2_aux->G + _s2_aux->H;
    //     }
    // }

    // // This is the computeCost of the IROS paper
    // inline void LazyThetaStarEDF::ComputeCost(Node *_s_aux, Node *_s2_aux)
    // {
    //     auto distanceParent2 = geometry::distanceBetween2Nodes(_s_aux->parent, _s2_aux);
    //     // std::cout << "distanceParent2: " << distanceParent2 << std::endl;
    //     // ROS_INFO("Compute COST");

    //     auto distanceParent2_nodes = LineOfSight::nodesInLineBetweenTwoNodes(_s_aux->parent, _s2_aux, discrete_world_, max_line_of_sight_cells_);  //REVISAR _s_aux->parent o _s_aux
    //     // std::cout << "distanceParent2_nodes: " << distanceParent2_nodes << std::endl;

    //     // No line of sight or distance greater than max_line_of_sight_cells
    //     if ( distanceParent2_nodes == 0 ){
    //         distanceParent2_nodes = 1;
    //     }

    //     // Line of sight
    //     else{
    //         // From LazyThetaSatarM1
    //         cost_weight_ = 4*(_s2_aux->cost -_s_aux->parent->cost);
    //         auto cost_term = static_cast<unsigned int>(cost_weight_ * ((_s_aux->parent->cost + _s2_aux->cost)/2) * dist_scale_factor_reduced_) * distanceParent2_nodes;
    //         // auto cost_term = static_cast<unsigned int>(cost_weight_ * _s2_aux->cost * dist_scale_factor_reduced_) * distanceParent2;
    //         if ( ( _s_aux->parent->G + distanceParent2 + cost_term ) < _s2_aux->G )
    //         {
    //             _s2_aux->parent = _s_aux->parent;
    //             // _s2_aux->G      = _s2_aux->parent->G + geometry::distanceBetween2Nodes(_s2_aux->parent, _s2_aux) +  cost_term;
    //             _s2_aux->G      = _s2_aux->parent->G + distanceParent2 +  cost_term;
    //             _s2_aux->C      = cost_term;       
    //         }
    //     }
    // }

    // This is the computeCost to test
    inline void LazyThetaStarEDF::ComputeCost(Node *_s_aux, Node *_s2_aux)
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
            // From LazyThetaSatarM1
            // auto cost_term = static_cast<unsigned int>(cost_weight_ * ((_s_aux->parent->cost + _s2_aux->cost)/2) * dist_scale_factor_reduced_) * distanceParent2_nodes;
            // auto cost_term = static_cast<unsigned int>(cost_weight_ * (( static_cast<double>(_s_aux->parent->cost) + static_cast<double>(_s2_aux->cost) ) /2) * dist_scale_factor_reduced_) * distanceParent2_nodes; // IROS Paper 
            // COST CONSIDERING EDF INCREASING AS DISTANCE INCREASE
            // auto cost_term = static_cast<unsigned int>(cost_weight_ * (1/(((static_cast<double>(_s_aux->parent->cost) + static_cast<double>(_s2_aux->cost))/2) * dist_scale_factor_reduced_))) * distanceParent2_nodes; 
            auto cost_term = static_cast<unsigned int>(cost_weight_ * (1/(((static_cast<double>(_s_aux->parent->cost) + static_cast<double>(_s2_aux->cost))*(0.5)) * dist_scale_factor_reduced_))) * distanceParent2_nodes; 

            // auto cost_term = static_cast<unsigned int>(cost_weight_ * _s2_aux->cost * dist_scale_factor_reduced_) * distanceParent2;
            if ( ( _s_aux->parent->G + (distanceParent2 + cost_term) ) < _s2_aux->G )
            {
                _s2_aux->parent = _s_aux->parent;
                // _s2_aux->G      = _s2_aux->parent->G + geometry::distanceBetween2Nodes(_s2_aux->parent, _s2_aux) +  cost_term;
                _s2_aux->G      = _s2_aux->parent->G + distanceParent2 + cost_term;
                _s2_aux->C      = cost_term;    
                _s2_aux->gplush = _s2_aux->G + _s2_aux->H;   
            }

            // ******************************************//
            // TRIANGLE INEQUALITY
            // ******************************************//
            // // Cost from current (_s_aux) to succesor (_s2_aux) because the triangle inequality is not fulfilled
            // auto cost_term2 = static_cast<unsigned int>(cost_weight_ * (1/(((static_cast<double>(_s_aux->cost) + static_cast<double>(_s2_aux->cost))*(0.5)) * dist_scale_factor_reduced_))); //dist_scale_factor_reduced_=1
            // unsigned int G_new = _s2_aux->G +  geometry::distanceBetween2Nodes(_s2_aux, _s_aux) + cost_term2;
            
            // // if ( (( _s_aux->parent->G + (distanceParent2 + cost_term) ) < _s2_aux->G) &&  (( _s_aux->parent->G + (distanceParent2 + cost_term) ) < G_new))
            // if (( _s_aux->parent->G + (distanceParent2 + cost_term) ) < _s2_aux->G) 
            // {
            //     // std::cout << "LoS" << std::endl; 
            //     if (( _s_aux->parent->G + (distanceParent2 + cost_term) ) < G_new)
            //     {
            //         _s2_aux->parent = _s_aux->parent;
            //         // _s2_aux->G      = _s2_aux->parent->G + geometry::distanceBetween2Nodes(_s2_aux->parent, _s2_aux) +  cost_term;
            //         _s2_aux->G      = _s2_aux->parent->G + distanceParent2 + cost_term;
            //         _s2_aux->C      = cost_term;    
            //         _s2_aux->gplush = _s2_aux->G + _s2_aux->H;  
            //         // std::cout << "PARENT" << std::endl; 
            //     }
            //     else{
            //         std::cout << "NOOOOO  PARENT" << std::endl; 
            //     }
                
            // }
            // else{
            //     // std::cout << "ENTRA1" << std::endl;
            //     if (G_new < _s2_aux->G){
            //             std::cout << "ENTRA2" << std::endl; // AQUI NO ENTRA NUNCA
            //             _s_aux->parent = _s2_aux;
            //             _s_aux->G      = G_new;
            //             _s_aux->C      = cost_term2;
            //             _s_aux->gplush = _s_aux->G + _s_aux->H;
            //     }
            // }
            // ******************************************//
        }
    }

    // This is the computeG to test: G*cost
    unsigned int LazyThetaStarEDF::computeG(const Node* _current, Node* _suc,  unsigned int _n_i, unsigned int _dirs){

        unsigned int cost = _current->G;
        // EXT: Compute here the gradient from dd_2D_, dd_3D_ and each _suc->cost
        // float grad_suc; // or _suc->grad
        // float dist_current_suc;


        if(_dirs  == 8){
            cost += (_n_i < 4 ? dist_scale_factor_ : dd_2D_); //This is more efficient
            // dist_current_suc = (_n_i < 4 ? dist_scale_factor_ : dd_2D_); //This is more efficient
            // cost += dist_current_suc;
            
        }else{
            cost += (_n_i < 6 ? dist_scale_factor_ : (_n_i < 18 ? dd_2D_ : dd_3D_)); //This is more efficient
            // dist_current_suc = (_n_i < 6 ? dist_scale_factor_ : (_n_i < 18 ? dd_2D_ : dd_3D_)); //This is more efficient
            // cost += dist_current_suc;
        }

        // grad_suc=(_suc->cost - _current->cost)/(dist_current_suc);
        // grad_suc=(_suc->cost - _current->cost);
        // std::cout << "current: " << _current->cost << std::endl;
        // std::cout << "suc: " << _suc->cost << std::endl;
        // std::cout << "gradiente: " << grad_suc << std::endl;
        // auto cost_term = static_cast<unsigned int>(((_current->cost + _suc->cost)/2) * dist_scale_factor_reduced_); //+grad_suc
        // auto cost_term = static_cast<unsigned int>(cost_weight_ * ((static_cast<double>(_current->cost) + static_cast<double>(_suc->cost))/2) * dist_scale_factor_reduced_); //+grad_suc // IROS Paper 
        // COST CONSIDERING EDF INCREASING AS DISTANCE INCREASE

        // auto cost_term = static_cast<unsigned int>(cost_weight_ * (1/(((static_cast<double>(_current->cost) + static_cast<double>(_suc->cost))/2) * dist_scale_factor_reduced_))); 
        auto cost_term = static_cast<unsigned int>(cost_weight_ * (1/(((static_cast<double>(_current->cost) + static_cast<double>(_suc->cost))*(0.5)) * dist_scale_factor_reduced_))); 
        cost += cost_term;
        // cost += dist_scale_factor_ * cost_term;
        _suc->C = cost_term;

        return cost;
    }

    PathData LazyThetaStarEDF::findPath(const Vec3i &_source, const Vec3i &_target)
    {
        Node *current = nullptr;

        bool solved{false};

        discrete_world_.getNodePtr(_source)->parent = new Node(_source);
        discrete_world_.setOpenValue(_source, true);

        utils::Clock main_timer;
        main_timer.tic();

        line_of_sight_checks_ = 0;

        MagicalMultiSet openSet;

        node_by_cost& indexByCost              = openSet.get<IndexByCost>();
        node_by_position& indexByWorldPosition = openSet.get<IndexByWorldPosition>();

        indexByCost.insert(discrete_world_.getNodePtr(_source));
        while (!indexByCost.empty())
        {

            auto it = indexByCost.begin();
            current = *it;
            indexByCost.erase(indexByCost.begin());
        
            if (current->coordinates == _target)
            {
                solved = true;
                break;
            }
            closedSet_.push_back(current);

            current->isInOpenList = false;
            current->isInClosedList = true;

            // std::cout << "current G: " << current->G << std::endl;
            // std::cout << "current cost: " << current->cost << std::endl;

            SetVertex(current);
#if defined(ROS) && defined(PUB_EXPLORED_NODES)
            publishROSDebugData(current, indexByCost, closedSet_);
#endif

            // exploreNeighbours(current, _target, indexByWorldPosition);
            // EXPLORING NEIGHBOURS CONSIDERING THE EDF GRADIENT
            exploreNeighbours_Gradient(current, _target, indexByWorldPosition);
            if (indexByCost.empty()){
                std::cout << "OPEN SET IS EMPTY" << std::endl;
                // Call the requestPathService service with M1.
            }
            

        }
        main_timer.toc();
    
        PathData result_data = createResultDataObject(current, main_timer, closedSet_.size(), 
                                                  solved, _source, line_of_sight_checks_);
   
#if defined(ROS) && defined(PUB_EXPLORED_NODES)
        explored_node_marker_.points.clear();
#endif
        closedSet_.clear();
        delete discrete_world_.getNodePtr(_source)->parent;

        discrete_world_.resetWorld();
        return result_data;
    }

}
