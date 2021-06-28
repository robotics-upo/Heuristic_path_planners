#include "Planners/AStarGenerator.hpp"

namespace Planners{
    
    
AStarGenerator::AStarGenerator(bool _use_3d = true): PathGenerator(_use_3d)
{

    //If compiled with ros and visualization
#ifdef ROS
    explored_nodes_marker_pub_ = lnh_.advertise<visualization_msgs::Marker>("astar_explored_nodes", 1);
	occupancy_marker_pub_ = lnh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("occupancy_markers", 1, true);

    std::string frame_id;
    lnh_.param("frame_id", frame_id, std::string("map"));	
    lnh_.param("resolution", resolution_, (float)0.2);
	occupancy_marker_.header.frame_id = frame_id; // "world";

    explored_node_marker_.header.frame_id = frame_id; //"world";
	explored_node_marker_.header.stamp = ros::Time();
	explored_node_marker_.ns = "debug";
	explored_node_marker_.id = 66;
	explored_node_marker_.type = visualization_msgs::Marker::CUBE_LIST;
	explored_node_marker_.action = visualization_msgs::Marker::ADD;
	explored_node_marker_.pose.orientation.w = 1.0;
	explored_node_marker_.scale.x = 1.0 * resolution_;
	explored_node_marker_.scale.y = 1.0 * resolution_;
	explored_node_marker_.scale.z = 1.0 * resolution_;
	explored_node_marker_.color.a = 0.7;
	explored_node_marker_.color.r = 0.0;
	explored_node_marker_.color.g = 1.0;
	explored_node_marker_.color.b = 0.0;
#endif

}
void AStarGenerator::publishOccupationMarkersMap()
{
#ifdef ROS
	occupancy_marker_.clear();
    for(const auto &it: discrete_world_.getElements()){
        if(!it.occuppied) continue;
        pcl::PointXYZ point;

		point.x = it.coordinates.x * resolution_;
		point.y = it.coordinates.y * resolution_;
		point.z = it.coordinates.z * resolution_;
		occupancy_marker_.push_back(point);
    }

	occupancy_marker_pub_.publish(occupancy_marker_);
#endif
}

PathData AStarGenerator::findPath(const Vec3i &source_, const Vec3i &target_)
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
    geometry_msgs::Point point;
	point.x = current->coordinates.x * resolution_;
	point.y = current->coordinates.y * resolution_;
	point.z = current->coordinates.z * resolution_;
    explored_node_marker_.header.stamp = ros::Time();
	explored_node_marker_.header.seq++;
	explored_node_marker_.points.push_back(point);
    explored_nodes_marker_pub_.publish(explored_node_marker_);
#endif

        for (unsigned int i = 0; i < direction.size(); ++i) {
            
            Vec3i newCoordinates(current->coordinates + direction[i]);
            unsigned int totalCost = current->G;
            if ( discrete_world_.isOccupied(newCoordinates) || 
                 discrete_world_.isInClosedList(newCoordinates) ) 
                continue;
            if(direction.size()  == 8){
                totalCost += (i < 4 ? 100 : 141); //This is more efficient
            }else{
                totalCost += (i < 6 ? 100 : (i < 18 ? 141 : 173)); //This is more efficient
            }
            
            Node *successor = discrete_world_.getNodePtr(newCoordinates);

            if(successor == nullptr) continue;

            if (!discrete_world_.isInOpenList(newCoordinates)) { 
                successor->parent = current;
                successor->G = totalCost;
                successor->H = heuristic(successor->coordinates, target_);
                openSet.insert(successor);
                discrete_world_.setOpenValue(successor->coordinates, true);
            }
            else if (totalCost < successor->G) {
                successor->parent = current;
                successor->G = totalCost;
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
    result_data["algorithm"] = std::string("astar");
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
