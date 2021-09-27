#include "Planners/AStarGeneratorSafetyCost.hpp"

namespace Planners{
    
PathData AStarGeneratorSafetyCost::findPath(const Vec3i &_source, const Vec3i &_target)
{
    Node *current = nullptr;
    NodeSet openSet, closedSet;
    bool solved{false};

    float scale=1; // Increase the influence of the distance cost. Important change between 5 and 6.
    float factor_cost = 1.4142;
    float factor_cost2 = 1.73;
    // float factor_cost = 2;

    openSet.insert(discrete_world_.getNodePtr(_source));
    discrete_world_.setOpenValue(_source, true);
    
    utils::Clock main_timer;
    main_timer.tic();
    while (!openSet.empty()) {

        float aa, bb;
        aa=0;

        current = *openSet.begin();

        if (current->coordinates == _target) { solved = true; break; }
        
        openSet.erase(openSet.begin());
        closedSet.insert(current);

        discrete_world_.setOpenValue(*current, false);
        discrete_world_.setClosedValue(*current, true);

        aa=current->cost;
        // std::cout << "Current Cost " << current->cost << " : " << std::endl;
        // std::cout << "Current G " << current->G << " : " << std::endl;

#if defined(ROS) && defined(PUB_EXPLORED_NODES)        
        publishROSDebugData(current, openSet, closedSet);
#endif

        for (unsigned int i = 0; i < direction.size(); ++i) {
            
            Vec3i newCoordinates(current->coordinates + direction[i]);
            float edge_neighbour = 0;

            if ( discrete_world_.isOccupied(newCoordinates) || 
                 discrete_world_.isInClosedList(newCoordinates) ) 
                continue;

            
            Node *successor = discrete_world_.getNodePtr(newCoordinates);

            if(successor == nullptr) continue;

            // unsigned int totalCost_previuous = current->G; //A
            unsigned int totalCost = 0;
            if(direction.size()  == 8){
                totalCost += (i < 4 ? dist_scale_factor_ : dd_2D_); //This is more efficient
                // Method 1
                // bb=successor->cost;
                // Method 2
                if (totalCost > 100) {
                    bb=(successor->cost)/(factor_cost);
                    // std::cout << "Cost scaled " << bb << " : " << std::endl;
                }
                else {
                    bb=successor->cost;
                }
                
                edge_neighbour = (((aa+bb)/(2*100))*totalCost)*scale;
                // std::cout << "Successor Cost " << successor->cost << " : " << std::endl;
                // std::cout << "edge " << edge_neighbour << " : " << std::endl;
                // std::cout << "TotalCost " << totalCost << " : " << std::endl;

            }else{
                totalCost += (i < 6 ? dist_scale_factor_ : (i < 18 ? dd_2D_ : dd_3D_)); //This is more efficient
                // Method 1
                // bb=successor->cost;
                // Method 2
                if ((totalCost > 100) && (totalCost < 150)) {
                    bb=(successor->cost)/(factor_cost);
                    // std::cout << "Cost scaled " << bb << " : " << std::endl;
                }
                else if ((totalCost > 150) && (totalCost < 200)){
                    bb=(successor->cost)/(factor_cost2);
                }
                else {
                    bb=successor->cost;
                }
                // std::cout << "Successor Cost " << successor->cost << " : " << std::endl;
                edge_neighbour = (((aa+bb)/(2*100))*totalCost)*scale;
            }
            
            if (!discrete_world_.isInOpenList(newCoordinates)) { 
                successor->parent = current;
                successor->G = current->G + totalCost + edge_neighbour; //Method B
                // successor->G = current->G + edge_neighbour; //Method A
                // std::cout << "Successor Cost " << successor->cost << " : " << std::endl;
                successor->H = heuristic(successor->coordinates, _target);
                openSet.insert(successor);
                discrete_world_.setOpenValue(successor->coordinates, true);
            }
            else if ((current->G + totalCost + edge_neighbour) < (successor->G)) { //B
            // else if ((current->G + edge_neighbour) < (successor->G)) { //A
                successor->parent = current;
                successor->G = current->G + totalCost + edge_neighbour; //B
                // successor->G = current->G + edge_neighbour; //A
                // std::cout << "UPDATE G " << std::endl;
            }
            // std::cout << "Successor Heuristic " << successor->H << " : " << std::endl;
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
    result_data["line_of_sight_checks"] = 0;
    

#if defined(ROS) && defined(PUB_EXPLORED_NODES)
    explored_node_marker_.points.clear();
#endif
    
    discrete_world_.resetWorld();
    return result_data;
}

}
