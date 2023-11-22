#include "Planners/LazyThetaStar_Gradient.hpp"

namespace Planners
{
    LazyThetaStarGradient::LazyThetaStarGradient(bool _use_3d):ThetaStar(_use_3d, "lazythetastargradient") {}
    LazyThetaStarGradient::LazyThetaStarGradient(bool _use_3d, std::string _name = "lazythetastargradient" ):ThetaStar(_use_3d, _name) {}
   
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

    void LazyThetaStarGradient::SetVertex(Node *_s_aux)
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
                    G_new = successor2->G +  geometry::distanceBetween2Nodes(successor2, _s_aux);
                    if (G_new < G_max)
                    {
                        G_max = G_new;
                        _s_aux->parent = successor2;
                        _s_aux->G      = G_new;
                        _s_aux->gplush = _s_aux->G + _s_aux->H;
                    }
                }
            }
        }
    }

    // ComputeCost original of the LazyThetaStar
    // inline void LazyThetaStarGradient::ComputeCost(Node *_s_aux, Node *_s2_aux)
    // {
    //     auto distanceParent2 = geometry::distanceBetween2Nodes(_s_aux->parent, _s2_aux);

    //     if ( (_s_aux->parent->G + distanceParent2) < _s2_aux->G )
    //     {
    //         _s2_aux->parent = _s_aux->parent;
    //         _s2_aux->G      = _s2_aux->parent->G + distanceParent2;
    //         _s2_aux->gplush = _s2_aux->G + _s2_aux->H;
    //     }
    // }

    inline void LazyThetaStarGradient::ComputeCost(Node *_s_aux, Node *_s2_aux)
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
            // auto cost_term = static_cast<unsigned int>(cost_weight_ * _s2_aux->cost * dist_scale_factor_reduced_) * distanceParent2_nodes;
            // // auto cost_term = static_cast<unsigned int>(cost_weight_ * _s2_aux->cost * dist_scale_factor_reduced_) * distanceParent2;
            // if ( ( _s_aux->parent->G + distanceParent2 + cost_term ) < _s2_aux->G )
            // {
            //     _s2_aux->parent = _s_aux->parent;
            //     // _s2_aux->G      = _s2_aux->parent->G + geometry::distanceBetween2Nodes(_s2_aux->parent, _s2_aux) +  cost_term;
            //     _s2_aux->G      = _s2_aux->parent->G + distanceParent2 +  cost_term;
            //     _s2_aux->C      = cost_term;       
            // }

            if ( (_s_aux->parent->G + distanceParent2) < _s2_aux->G )
            {
                _s2_aux->parent = _s_aux->parent;
                _s2_aux->G      = _s2_aux->parent->G + distanceParent2;
                _s2_aux->gplush = _s2_aux->G + _s2_aux->H;
            }
        }
    }

    PathData LazyThetaStarGradient::findPath(const Vec3i &_source, const Vec3i &_target)
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

            SetVertex(current);
#if defined(ROS) && defined(PUB_EXPLORED_NODES)
            publishROSDebugData(current, indexByCost, closedSet_);
#endif

            // exploreNeighbours(current, _target, indexByWorldPosition);
            // EXPLORING NEIGHBOURS CONSIDERING THE EDF GRADIENT
            exploreNeighbours_Gradient(current, _target, indexByWorldPosition);

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
