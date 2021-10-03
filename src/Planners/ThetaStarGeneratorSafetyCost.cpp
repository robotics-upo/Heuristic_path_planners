#include "Planners/ThetaStarGeneratorSafetyCost.hpp"

namespace Planners
{
    ThetaStarGeneratorSafetyCost::ThetaStarGeneratorSafetyCost(bool _use_3d):ThetaStarGenerator(_use_3d, "thetastarsafetycost") {}
    
    ThetaStarGeneratorSafetyCost::ThetaStarGeneratorSafetyCost(bool _use_3d, std::string _name = "thetastarsafetycost" ):ThetaStarGenerator(_use_3d, _name) {}
    
    void ThetaStarGeneratorSafetyCost::ComputeCost(Node *_s_aux, Node *_s2_aux)
    {
        utils::CoordinateListPtr checked_nodes, checked_nodes_current;
        checked_nodes.reset(new CoordinateList);
        checked_nodes_current.reset(new CoordinateList);

        line_of_sight_checks_++;
        if (LineOfSight::bresenham3D((_s_aux->parent), _s2_aux, discrete_world_, checked_nodes))  
        {
            auto dist2   = geometry::distanceBetween2Nodes(_s_aux->parent, _s2_aux);
            auto edge2   = ComputeEdgeCost(checked_nodes, _s_aux, _s2_aux, dist2);

            line_of_sight_checks_++;
            LineOfSight::bresenham3D(_s_aux, _s2_aux, discrete_world_, checked_nodes_current);

            auto dist1   = geometry::distanceBetween2Nodes(_s_aux, _s2_aux);  
            auto edge1   =  ComputeEdgeCost(checked_nodes_current, _s_aux, _s2_aux, dist1);

            if ( ( _s_aux->parent->G + dist2 + edge2 ) <= ( _s_aux->G + dist1 + edge1)) 
            {
                _s2_aux->parent = _s_aux->parent;
                _s2_aux->G      = _s_aux->parent->G + dist2 + edge2;  // This is the same than A*
            }
            else{
                _s2_aux->parent =_s_aux;
                _s2_aux->G      = _s_aux->G + dist1 + edge1;   // This is the same than A*             
            }
        } else {

            _s2_aux->parent=_s_aux;

            line_of_sight_checks_++;
            LineOfSight::bresenham3D(_s_aux, _s2_aux, discrete_world_, checked_nodes);
            
            auto dist1     = geometry::distanceBetween2Nodes(_s_aux, _s2_aux);  
            auto edge1   =  ComputeEdgeCost(checked_nodes, _s_aux, _s2_aux, dist1);

            _s2_aux->G     =  _s_aux->G + dist1 + edge1;  // This is the same than A*
        }
    }

    unsigned int ThetaStarGeneratorSafetyCost::ComputeEdgeCost(const utils::CoordinateListPtr _checked_nodes, const Node* _s, const Node* _s2, unsigned int _dist){ 
        
        //TODO Change this hardcoded values to parameters 
        float dist_max = 100; // This parameter should appear in mean_dist_cost2 instead of *100.
        float scale    = 1.0; // Increase the influence of the distance cost. Important change between 5 and 6.

        double dist_cost{0};
        double mean_dist_cost{0};
            
        auto n_checked_nodes = _checked_nodes->size();
        if( n_checked_nodes >= 1 )
            for(auto &it: *_checked_nodes)
                dist_cost += discrete_world_.getNodePtr(it)->cost;

        double cost_origin    = _s->parent->cost;
        double cost_goal      = _s2->cost;
            
        if( n_checked_nodes > 1){
            mean_dist_cost = ( ( ( cost_origin - cost_goal ) / 2 + dist_cost ) / ( n_checked_nodes * dist_max ) ); //A
        }
        else if (n_checked_nodes == 1){
            mean_dist_cost = ( ( ( cost_origin + cost_goal ) / 2 + dist_cost ) / ( n_checked_nodes * dist_max * dist_max ) ); //A
        }
        else{ 
            mean_dist_cost = ( cost_origin + cost_goal ) / ( 2 * dist_max);
        }

        // auto dist = geometry::distanceBetween2Nodes(_s->parent, _s2);

        return static_cast<unsigned int>( mean_dist_cost  * _dist * scale );        
    }

    PathData ThetaStarGeneratorSafetyCost::findPath(const Vec3i &_source, const Vec3i &_target)
    {
        Node *current = nullptr;
        NodeSet openSet, closedSet;
        bool solved{false};

        openSet.insert(discrete_world_.getNodePtr(_source));
        discrete_world_.getNodePtr(_source)->parent = new Node(_source);
        discrete_world_.setOpenValue(_source, true);
        
        utils::Clock main_timer;
        main_timer.tic();

        line_of_sight_checks_ = 0;

        while (!openSet.empty())
        {
            current = *openSet.begin();

            if (current->coordinates == _target)
            {
                solved = true;
                break;
            }

            openSet.erase(openSet.begin());
            closedSet.insert(current);

            discrete_world_.setOpenValue(*current, false);
            discrete_world_.setClosedValue(*current, true);

#if defined(ROS) && defined(PUB_EXPLORED_NODES)        
            publishROSDebugData(current, openSet, closedSet);
#endif

            for (unsigned int i = 0; i < direction.size(); ++i)
            {

                Vec3i newCoordinates(current->coordinates + direction[i]);

                if (discrete_world_.isOccupied(newCoordinates) ||
                    discrete_world_.isInClosedList(newCoordinates))
                    continue;

                Node *successor = discrete_world_.getNodePtr(newCoordinates);

                if (successor == nullptr) continue;

                if (!discrete_world_.isInOpenList(newCoordinates))
                {
                    unsigned int totalCost = 0;

                    if(direction.size()  == 8){
                        totalCost += (i < 4 ? dist_scale_factor_ : dd_2D_); //This is more efficient
                    }else{
                        totalCost += (i < 6 ? dist_scale_factor_ : (i < 18 ? dd_2D_ : dd_3D_)); //This is more efficient
                    }
                    double bb = static_cast<double>( static_cast<double>(successor->cost) / (static_cast<double>(totalCost) / static_cast<double>(dist_scale_factor_)) );

                    auto edge_neighbour = static_cast<unsigned int>( ( ( ( current->cost + bb ) / ( 2 * 100 ) ) * totalCost ) );
            
                    successor->G = current->G + totalCost + edge_neighbour; // This is the same than A*
                    successor->H = heuristic(successor->coordinates, _target);
                    openSet.insert(successor);
                    discrete_world_.setOpenValue(*successor, true);
                }
                
                UpdateVertex(current, successor, openSet); 
            }
        }
        main_timer.toc();

        PathData result_data = createResultDataObject(current, main_timer, closedSet.size(), 
                                                  solved, _source, line_of_sight_checks_);
   
#if defined(ROS) && defined(PUB_EXPLORED_NODES)
        explored_node_marker_.points.clear();
#endif

        discrete_world_.resetWorld();
        return result_data;
    }

}
