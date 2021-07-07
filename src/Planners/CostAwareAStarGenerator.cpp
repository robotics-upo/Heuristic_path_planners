#include "Planners/CostAwareAStarGenerator.hpp"

namespace Planners{
    
PathData CostAwareAStarGenerator::findPath(const Vec3i &source_, const Vec3i &target_)
{
    Node *current = nullptr;
    NodeSet openSet, closedSet;
    bool solved{false};

    openSet.insert(discrete_world_.getNodePtr(source_));
    discrete_world_.setOpenValue(source_, true);
    
    utils::Clock main_timer;
    main_timer.tic();
    while (!openSet.empty()) {

        current = *openSet.begin();

        if (current->coordinates == target_) { solved = true; break; }
        
        openSet.erase(openSet.begin());
        closedSet.insert(current);

        discrete_world_.setOpenValue(current->coordinates, false);
        discrete_world_.setClosedValue(current->coordinates, true);

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

            unsigned int totalCost = current->G;
            if(direction.size()  == 8){
                totalCost += (i < 4 ? dist_scale_factor_ : dd_2D_); //This is more efficient
            }else{
                totalCost += (i < 6 ? dist_scale_factor_ : (i < 18 ? dd_2D_ : dd_3D_)); //This is more efficient
            }
            
            if (!discrete_world_.isInOpenList(newCoordinates)) { 
                successor->parent = current;
                successor->G = totalCost + static_cast<int>(cost_weight_ * successor->cost);
                successor->H = heuristic(successor->coordinates, target_);
                openSet.insert(successor);
                discrete_world_.setOpenValue(successor->coordinates, true);
            }
            else if (totalCost < successor->G) {
                successor->parent = current;
                successor->G = totalCost + static_cast<int>(cost_weight_ * successor->cost);
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
    result_data["algorithm"] = std::string("cost_aware_astar");
    result_data["cost_weight"] = cost_weight_;
    result_data["path"] = path;
    result_data["time_spent"] = main_timer.getElapsedMillisecs();
    result_data["explored_nodes"] = closedSet.size();
    result_data["start_coords"] = source_;
    result_data["goal_coords"] = target_;
    result_data["path_length"] = geometry::calculatePathLength(path, discrete_world_.getResolution());
    result_data["line_of_sight_checks"] = 0;
    

#if defined(ROS) && defined(PUB_EXPLORED_NODES)
    explored_node_marker_.points.clear();
#endif
    
    discrete_world_.resetWorld();
    return result_data;
}

}
