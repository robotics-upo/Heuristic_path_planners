#include "Planners/CostAwareAStarGenerator.hpp"

namespace Planners{
    
CostAwareAStarGenerator::CostAwareAStarGenerator(bool _use_3d):AStarGenerator(_use_3d, "costastar") {}
CostAwareAStarGenerator::CostAwareAStarGenerator(bool _use_3d, std::string _name = "costastar" ):AStarGenerator(_use_3d, _name) {}

PathData CostAwareAStarGenerator::findPath(const Vec3i &_source, const Vec3i &_target)
{
    Node *current = nullptr;
    NodeSet openSet, closedSet;
    bool solved{false};

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

            unsigned int totalCost = current->G;
            if(direction.size()  == 8){
                totalCost += (i < 4 ? dist_scale_factor_ : dd_2D_); //This is more efficient
            }else{
                totalCost += (i < 6 ? dist_scale_factor_ : (i < 18 ? dd_2D_ : dd_3D_)); //This is more efficient
            }
            
            if (!discrete_world_.isInOpenList(newCoordinates)) { 
                successor->parent = current;
                successor->G = totalCost + static_cast<unsigned int>(cost_weight_ * successor->cost);
                successor->H = heuristic(successor->coordinates, _target);
                openSet.insert(successor);
                discrete_world_.setOpenValue(successor->coordinates, true);
            }
            else if (totalCost < successor->G) {
                successor->parent = current;
                successor->G = totalCost + static_cast<unsigned int>(cost_weight_ * successor->cost);
            }
        }
    }
    main_timer.toc();
    
    PathData result_data = createResultDataObject(current, main_timer, closedSet.size(), 
                                                  solved, _source, 0);
   
#if defined(ROS) && defined(PUB_EXPLORED_NODES)
    explored_node_marker_.points.clear();
#endif
    
    discrete_world_.resetWorld();
    return result_data;
}

}
