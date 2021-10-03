#include "Planners/AStarGenerator.hpp"

namespace Planners{
    
AStarGenerator::AStarGenerator(bool _use_3d = true, std::string _name = "astar" ): PathGenerator(_use_3d, _name){
    configROS();
}
    
AStarGenerator::AStarGenerator(bool _use_3d = true): PathGenerator(_use_3d, "astar")
{
    configROS();
}
void AStarGenerator::configROS(){
    //If compiled with ros and visualization
#ifdef ROS
    explored_nodes_marker_pub_ = lnh_.advertise<visualization_msgs::Marker>("explored_nodes",   1);
    openset_marker_pub_        = lnh_.advertise<visualization_msgs::Marker>("openset_nodes",    1);
    closedset_marker_pub_      = lnh_.advertise<visualization_msgs::Marker>("closed_set_nodes", 1);
    best_node_marker_pub_      = lnh_.advertise<visualization_msgs::Marker>("best_node_marker", 1);
    aux_text_marker_pub_       = lnh_.advertise<visualization_msgs::Marker>("aux_text_marker",  1);
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

    openset_markers_    = explored_node_marker_;
    openset_markers_.color.b = 1.0;
    openset_markers_.color.g = 0.0;
	openset_markers_.id      = 67;

    closed_set_markers_ = explored_node_marker_;
    closed_set_markers_.color.g = 0.0;
    closed_set_markers_.color.r = 1.0;
	explored_node_marker_.id = 68;

    best_node_marker_ = explored_node_marker_;
    best_node_marker_.color.g = 0.7;
    best_node_marker_.color.b = 0.7;
    best_node_marker_.id     = 69;
	best_node_marker_.type = visualization_msgs::Marker::SPHERE;

    aux_text_marker_   = explored_node_marker_;
	aux_text_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	aux_text_marker_.id = 70;
	aux_text_marker_.color.a = 0.7;
	aux_text_marker_.color.g = 0.0;
    aux_text_marker_.text = "";
	aux_text_marker_.scale.z = 3.0 * resolution_;

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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
void AStarGenerator::publishROSDebugData(const Node* _node, const NodeSet &_open_set, const NodeSet &_closed_set)
{
#if defined(ROS) && defined(PUB_EXPLORED_NODES)

    explored_node_marker_.header.stamp = ros::Time();
    explored_node_marker_.header.seq++;
    openset_markers_.header.stamp = ros::Time();
    openset_markers_.header.seq++;
    closed_set_markers_.header.stamp = ros::Time();
    closed_set_markers_.header.seq++;

    openset_markers_.points.clear();
    closed_set_markers_.points.clear();

    explored_node_marker_.points.push_back(continousPoint(_node->coordinates, resolution_));

    for(const auto &it: _open_set)
        openset_markers_.points.push_back(continousPoint(it->coordinates, resolution_));

    for(const auto &it: _closed_set)
        closed_set_markers_.points.push_back(continousPoint(it->coordinates, resolution_));

    best_node_marker_.pose.position = continousPoint(_node->coordinates, resolution_);
    best_node_marker_.pose.position.z += resolution_;

    aux_text_marker_.text = "Best node G = " + std::to_string(_node->G) + "Best node G+H = " + std::to_string(_node->G+_node->H) +
                 std::string("\nCost = ") + std::to_string(static_cast<int>(cost_weight_ * _node->cost));
	aux_text_marker_.pose = best_node_marker_.pose;
    aux_text_marker_.pose.position.z += 5 * resolution_;

    closedset_marker_pub_.publish(closed_set_markers_);
    openset_marker_pub_.publish(openset_markers_);
    best_node_marker_pub_.publish(best_node_marker_);
    explored_nodes_marker_pub_.publish(explored_node_marker_);
    aux_text_marker_pub_.publish(aux_text_marker_);
    // usleep(1e4);
    // std::cout << "Please a key to go to the next iteration..." << std::endl;
    // getchar(); // Comentar para no usar tecla.

#endif
#pragma GCC diagnostic pop

}

void AStarGenerator::exploreNeighbours(Node* _current, const Vec3i &_target,NodeSet &_openset){
    
    for (unsigned int i = 0; i < direction.size(); ++i) {
            
        Vec3i newCoordinates = _current->coordinates + direction[i];

        if ( discrete_world_.isOccupied(newCoordinates) || 
             discrete_world_.isInClosedList(newCoordinates) ) 
            continue;
 
        Node *successor = discrete_world_.getNodePtr(newCoordinates);

        if(successor == nullptr) continue;
        
        unsigned int totalCost = _current->G;

        if(direction.size()  == 8){
            totalCost += (i < 4 ? dist_scale_factor_ : dd_2D_); //This is more efficient
        }else{
            totalCost += (i < 6 ? dist_scale_factor_ : (i < 18 ? dd_2D_ : dd_3D_)); //This is more efficient
        }
            
        if (!discrete_world_.isInOpenList(newCoordinates)) { 
            successor->parent = _current;
            successor->G = totalCost;
            successor->H = heuristic(successor->coordinates, _target);
            _openset.insert(successor);
            discrete_world_.setOpenValue(successor->coordinates, true);
        }
        else if (totalCost < successor->G) {
            successor->parent = _current;
            successor->G = totalCost;
        }
    }
}
PathData AStarGenerator::findPath(const Vec3i &_source, const Vec3i &_target)
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

        exploreNeighbours(current, _target, openSet);     
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
