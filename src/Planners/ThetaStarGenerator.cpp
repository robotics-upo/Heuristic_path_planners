#include "Planners/ThetaStarGenerator.hpp"
//#include <algorithm>
#include <utils/LineOfSight.hpp>
#include <utils/geometry_utils.hpp>

namespace Planners
{
    void ThetaStarGenerator::UpdateVertex(Node *s, Node *s2, NodeSet &openset)
    //void ThetaStarGenerator::UpdateVertex(Node &s, Node &s2, NodeSet &openset)
    {
        //std::cout << "Updating vertex" << std::endl;
        // std::cerr << "S Parent ["<< s->parent->coordinates.x << ";"<< s->parent->coordinates.y <<";"<< s->parent->coordinates.z <<"]." << std::endl;
        // std::cerr << "S2 Parent ["<< s2->parent->coordinates.x << ";"<< s2->parent->coordinates.y <<";"<< s2->parent->coordinates.z <<"]." << std::endl;
        float g_old = s2->G;
        
        ThetaStarGenerator::ComputeCost(s, s2);
        // To test the Lazy
        //ThetaStarGenerator::ComputeCostLazy(s, s2);
        if (s2->G < g_old){
            if (discrete_world_.isInOpenList(s2->coordinates)){
                openset.erase(s2);
            }
            openset.insert(s2);
        }
	
    } 

    void ThetaStarGenerator::ComputeCost(Node *s_aux, Node *s2_aux){
    //void ThetaStarGenerator::ComputeCost(Node &s, Node &s2){
        
        //std::cout << "Compute cost" << std::endl;
        //std::cerr << "S Parent ["<< s_aux->parent->coordinates.x << ";"<< s_aux->parent->coordinates.y <<";"<< s_aux->parent->coordinates.z <<"] not valid." << std::endl;
        float distanceParent2 = Planners::utils::geometry::distanceBetween2nodesOTRO(s_aux->parent, s2_aux);

		if (Planners::utils::LineOfSight::bresenham3D((s_aux->parent), s2_aux, discrete_world_)) {
            if ((s_aux->parent->G + distanceParent2 + s2_aux->H) < (s2_aux->G +s2_aux->H)){
            //if ((s_aux->parent->G + distanceParent2) < (s2_aux->G)){
                s2_aux->parent = s_aux->parent;
                s2_aux->G = s2_aux->parent->G + Planners::utils::geometry::distanceBetween2nodesOTRO(s2_aux->parent, s2_aux);
                // s2->H??? --> Deberia estar calculado de antes porque está en la lista Open
            }
			//std::cout<<"Theta* LINE OF SIGHT" << std::endl;
		}
		else
		{
            double distanceParent = Planners::utils::geometry::distanceBetween2nodesOTRO(s_aux, s2_aux);
            if ((s_aux->G + distanceParent + s2_aux->H) < (s2_aux->G +s2_aux->H))
            //if ((s_aux->G + distanceParent) < (s2_aux->G))
            {
                s2_aux->parent = s_aux;
                s2_aux->G = s2_aux->parent->G + Planners::utils::geometry::distanceBetween2nodesOTRO(s2_aux->parent, s2_aux);
                // s2->H??? --> Deberia estar calculado de antes porque está en la lista Open
            }
		}
    }
    
    void ThetaStarGenerator::ComputeCostLazy(Node *s_aux, Node *s2_aux){
    //void ThetaStarGenerator::ComputeCost(Node &s, Node &s2){
        
        // //double distanceParent2 = weightedDistanceBetween2nodes((*s.parentNode), s2);
        //std::cout << "Compute cost" << std::endl;
        //std::cerr << "S Parent ["<< s_aux->parent->coordinates.x << ";"<< s_aux->parent->coordinates.y <<";"<< s_aux->parent->coordinates.z <<"] not valid." << std::endl;
        float distanceParent2 = Planners::utils::geometry::distanceBetween2nodesOTRO(s_aux->parent, s2_aux);
				
        if ((s_aux->parent->G + distanceParent2 + + s2_aux->H) < (s2_aux->G + + s2_aux->H)){
            s2_aux->parent = s_aux->parent;
            s2_aux->G = s2_aux->parent->G + Planners::utils::geometry::distanceBetween2nodesOTRO(s2_aux->parent, s2_aux);
            // s2->H??? --> Deberia estar calculado de antes porque está en la lista Open
        }
		//std::cout<<"Theta* LINE OF SIGHT" << std::endl;
		
    }

    void ThetaStarGenerator::SetVertex(Node *s_aux, NodeSet &openset){
        if (!Planners::utils::LineOfSight::bresenham3D((s_aux->parent), s_aux, discrete_world_)) {
            std::cout<<"NO LINE OF SIGHT" << std::endl;
             for (unsigned int i = 0; i < direction.size(); ++i) {
            
                Vec3i newCoordinates(s_aux->coordinates + direction[i]);
            
                if ( discrete_world_.isOccupied(newCoordinates)) 
                        continue;
                if (discrete_world_.isInClosedList(newCoordinates)){
                    //unsigned int G_new = s_aux->G + ( i < 6 ? 10 : ( i < 18 ? 14 : 17) ); //This is more efficient
                    float G_new;
                    float G_max = 1000; // Poner bien
                    Node *successor2 = discrete_world_.getNodePtr(newCoordinates);
                    if(successor2 == nullptr) continue;
                    G_new= successor2->G + Planners::utils::geometry::distanceBetween2nodesOTRO(successor2, s_aux);
                    if (G_new < G_max){
                        s_aux->parent = successor2;
                        s_aux->G = G_new;
                        //s_aux->H = heuristic(successor2->coordinates, target_);
                    }

                }

             }

        }
        
    }

    PathData ThetaStarGenerator::findPath(Vec3i source_, Vec3i target_)
    {
        Node *current = nullptr;
        NodeSet openSet, closedSet;
        bool solved{false};

        openSet.insert(discrete_world_.getNodePtr(source_));
        //Set the parent of the start node itself
        discrete_world_.getNodePtr(source_)->parent = new Node(source_);
        // discrete_world_.getNodePtr(source_)->parent->coordinates = source_;

        discrete_world_.setOpenValue(source_, true);
    
        Planners::utils::Clock main_timer;
        main_timer.tic();
        while (!openSet.empty()) {

            current = *openSet.begin();

            //If its the goal, FINISH
            if (current->coordinates == target_) { solved = true; break; }
        
            openSet.erase(openSet.begin());
            closedSet.insert(current);

            discrete_world_.setOpenValue(current->coordinates, false);  //flag que se consulta para saber si esta en el Open
            discrete_world_.setClosedValue(current->coordinates, true); //flag que se consulta para saber si esta en el Close

            // To test the Lazy: SetVertex
            //SetVertex(current, openSet);

    # if defined(ROS) && defined(PUB_EXPLORED_NODES)
        geometry_msgs::Point point;
	    point.x = current->coordinates.x * resolution_;
	    point.y = current->coordinates.y * resolution_;
	    point.z = current->coordinates.z * resolution_;
        explored_node_marker_.header.stamp = ros::Time();
	    explored_node_marker_.header.seq++;
	    explored_node_marker_.points.push_back(point);
        explored_nodes_marker_pub_.publish(explored_node_marker_);
    # endif

        for (unsigned int i = 0; i < direction.size(); ++i) {
            
            Vec3i newCoordinates(current->coordinates + direction[i]);
            
            if ( discrete_world_.isOccupied(newCoordinates) || 
                 discrete_world_.isInClosedList(newCoordinates) ) 
                continue;
            unsigned int totalCost = current->G + ( i < 6 ? 10 : ( i < 18 ? 14 : 17) ); //This is more efficient
            
            Node *successor = discrete_world_.getNodePtr(newCoordinates);

            if(successor == nullptr) continue;

            // If node is not in Open List
            if (!discrete_world_.isInOpenList(newCoordinates)) { 
                successor->parent = current;
                successor->G = totalCost;
                successor->H = heuristic(successor->coordinates, target_);
                openSet.insert(successor);
                discrete_world_.setOpenValue(successor->coordinates, true);
            }
            // If node is in Open List
            // A_star
            // else if (totalCost < successor->G) {
            //     successor->parent = current;
            //     successor->G = totalCost;
            //     //std::cerr << "S Parent ["<< successor->parent->coordinates.x << ";"<< successor->parent->coordinates.y <<";"<< successor->parent->coordinates.z <<"]." << std::endl;
            // }
            // Theta Star
            // std::cerr << "UPDATE" << std::endl;
			UpdateVertex(current, successor, openSet);
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
    result_data["path_length"] = Planners::utils::geometry::calculatePathLength(path, discrete_world_.getResolution());
    
// JAC: Tengo que comentar esto para que compile.
// #ifdef ROS
//     explored_node_marker_.points.clear();
// #endif
    
    discrete_world_.resetWorld();
    return result_data;

    }

}

