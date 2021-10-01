#include "Planners/AStarGeneratorSafetyCost.hpp"

namespace Planners{
    
PathData AStarGeneratorSafetyCost::findPath(const Vec3i &_source, const Vec3i &_target)
{
    Node *current = nullptr;
    NodeSet openSet, closedSet;
    bool solved{false};

    //TODO Change this scale variable to a parameter or something else
    double scale=1; // Increase the influence of the distance cost. Important change between 5 and 6.
    
    openSet.insert(discrete_world_.getNodePtr(_source));
    discrete_world_.setOpenValue(_source, true);
    
    utils::Clock main_timer;
    main_timer.tic();

    while (!openSet.empty()) {

        current = *openSet.begin();

        if (current->coordinates == _target) { solved = true; break; }
        
        openSet.erase(openSet.begin());
        closedSet.insert(current);

        discrete_world_.setOpenValue(*current, false);
        discrete_world_.setClosedValue(*current, true);

#if defined(ROS) && defined(PUB_EXPLORED_NODES)        
        publishROSDebugData(current, openSet, closedSet);
#endif

        for (unsigned int i = 0; i < direction.size(); ++i) {
            
            Vec3i newCoordinates(current->coordinates + direction[i]);

            if ( discrete_world_.isOccupied(newCoordinates) || 
                 discrete_world_.isInClosedList(newCoordinates) ) 
                continue;

            
            Node *successor = discrete_world_.getNodePtr(newCoordinates);

            if(successor == nullptr) continue;

            unsigned int totalCost;
            if(direction.size() == 8){
                totalCost = (i < 4 ? dist_scale_factor_ : dd_2D_); //This is more efficient
            }else{
                totalCost = (i < 6 ? dist_scale_factor_ : (i < 18 ? dd_2D_ : dd_3D_)); //This is more efficient
            }
            //TODO Give bb a more precise name
            double bb = static_cast<double>( static_cast<double>(successor->cost) / (static_cast<double>(totalCost) / static_cast<double>(dist_scale_factor_)) );

            auto edge_neighbour = static_cast<unsigned int>( ( ( ( current->cost + bb ) / ( 2 * 100 ) ) * totalCost ) * scale);
            
            if (!discrete_world_.isInOpenList(newCoordinates)) { 
                successor->parent = current;
                successor->G = current->G + totalCost + edge_neighbour; //Method B
                successor->H = heuristic(successor->coordinates, _target);
                openSet.insert(successor);
                discrete_world_.setOpenValue(successor->coordinates, true);
            }
            else if ((current->G + totalCost + edge_neighbour) < (successor->G)) { //B
                successor->parent = current;
                successor->G = current->G + totalCost + edge_neighbour; //B
            }
        }
    }
    main_timer.toc();
    
    PathData result_data;
    result_data["solved"] = solved;

    CoordinateList path;
    if(solved){
        while (current != nullptr) {
            path.push_back(current->coordinates);
            current = current->parent;
        }
    }else{
        std::cout<< "Error impossible to calcualte a solution" << std::endl;
    }
    result_data["algorithm"] = std::string("astarsafetycost");
    result_data["cost_weight"] = cost_weight_;
    result_data["path"] = path;
    result_data["time_spent"] = main_timer.getElapsedMillisecs();
    result_data["explored_nodes"] = closedSet.size();
    result_data["start_coords"] = _source;
    result_data["goal_coords"] = _target;
    result_data["path_length"] = geometry::calculatePathLength(path, discrete_world_.getResolution());
    result_data["line_of_sight_checks"] = static_cast<unsigned int>(0);
    

#if defined(ROS) && defined(PUB_EXPLORED_NODES)
    explored_node_marker_.points.clear();
#endif
    
    discrete_world_.resetWorld();
    return result_data;
}

}
