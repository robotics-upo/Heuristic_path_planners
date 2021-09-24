#include "Planners/LazyThetaStarGeneratorSafetyCost.hpp"

namespace Planners
{

    // void LazyThetaStarGeneratorSafetyCost::SetVertex(Node *_s_aux)
    // {   
    //     if (!LineOfSight::bresenham3DWithMaxThreshold((_s_aux->parent), _s_aux, discrete_world_, max_line_of_sight_cells_ ))
    //     {
    //         unsigned int G_max = std::numeric_limits<unsigned int>::max(); 
    //         unsigned int G_new;

    //         for (const auto &i: direction)
    //         {
    //             Vec3i newCoordinates(_s_aux->coordinates + i);

    //             if ( discrete_world_.isOccupied(newCoordinates) ) continue;

    //             if ( discrete_world_.isInClosedList(newCoordinates) )
    //             {
    //                 Node *successor2 = discrete_world_.getNodePtr(newCoordinates);
    //                 if (successor2 == nullptr) continue;

    //                 G_new = successor2->G +  geometry::distanceBetween2Nodes(successor2, _s_aux) + static_cast<int>(cost_weight_ * successor2->cost);
    //                 if (G_new < G_max)
    //                 {
    //                     G_max = G_new;
    //                     _s_aux->parent = successor2;
    //                     _s_aux->G = G_new;
    //                 }
    //             }
    //         }
    //     }
    // }

    void LazyThetaStarGeneratorSafetyCost::SetVertex(Node *_s_aux)
    {   
        // if (!LineOfSight::bresenham3DWithMaxThreshold((_s_aux->parent), _s_aux, discrete_world_, max_line_of_sight_cells_ ))
        unsigned int G_max = std::numeric_limits<unsigned int>::max(); 
        unsigned int G_new;
        float mean_dist_cost;
        float cost_current;
        float cost_successor;
        float dist;
        float edge;
        float dist_max = 100;

        // std::cout << "G max: " << G_max << std::endl;

        for (const auto &i: direction)
        {
            Vec3i newCoordinates(_s_aux->coordinates + i);
            if ( discrete_world_.isOccupied(newCoordinates) ) continue;
            if ( discrete_world_.isInClosedList(newCoordinates) )
            {
                Node *successor2 = discrete_world_.getNodePtr(newCoordinates);
                if (successor2 == nullptr) continue;

                cost_current = 0;
                cost_successor = 0;
                mean_dist_cost = 0;

                dist = geometry::distanceBetween2Nodes(successor2, _s_aux);
                cost_current = _s_aux->cost/dist_max;
                cost_successor =  successor2->cost/dist_max;
                mean_dist_cost = (cost_current + cost_successor)/2;
                edge=(mean_dist_cost)*dist;
                

                // G_new = successor2->G +  geometry::distanceBetween2Nodes(successor2, _s_aux) + static_cast<int>(cost_weight_ * successor2->cost);
                G_new = successor2-> G + dist + edge;
                if (G_new < G_max)
                {
                    // std::cout << "UPDATE G" << std::endl;
                    G_max = G_new;
                    _s_aux->parent = successor2;
                    _s_aux->G = G_new;
                }
            }
        }
    }

    void LazyThetaStarGeneratorSafetyCost::ComputeCost(Node *_s_aux, Node *_s2_aux)
    {
        // auto distanceParent2 = geometry::distanceBetween2Nodes(_s_aux->parent, _s2_aux);
         utils::CoordinateListPtr checked_nodes;
        checked_nodes.reset(new CoordinateList);

        float dist_max=100; // This parameter should appear in mean_dist_cost2 instead of *100.
        float scale=1.0; // Increase the influence of the distance cost. Important change between 5 and 6.

        if (LineOfSight::bresenham3D((_s_aux->parent), _s2_aux, discrete_world_, checked_nodes)) {
            
            float dist_cost2= 0;
            float mean_dist_cost2 = 0;
            float edge2 = 0;
            float cost_origin2;
            float cost_goal2;
            auto dist2 = geometry::distanceBetween2Nodes(_s_aux->parent, _s2_aux);

            los_neighbour_ = true;

            if (checked_nodes->size() > 1){
                // std::cout << "IF Parent" << std::endl;
                for(auto &it: *checked_nodes){
                    auto node = discrete_world_.getNodePtr(it);
                    // std::cout << "Node Coordinates: " << node->coordinates << std::endl;
                    // std::cout << "Node Cost: " << node->cost << std::endl;
                    // std::cout << "Node G: " << node->G << std::endl;
                    // std::cout << "Node Score (G+H): " << node->getScore() << std::endl;
                    // std::cout << "Node Score (G+D+H): " << node->getScoreWithSafetyCost() << std::endl;
                    // dist_cost2 += node->G;
                    dist_cost2 += node->cost;
                }
                // std::cout << "Node Coordinates of Parent: " << _s_aux->parent->coordinates << std::endl;
                // cost_origin2 = _s_aux->parent->cost/dist_max; // 17-09-2021
                cost_origin2 = _s_aux->parent->cost;

                // std::cout << "NODE ADDR: " << &_s_aux << std::endl;
                // std::cout << "PARENT NODE ADDR: " << &_s_aux->parent << std::endl;
                // std::cout << "cost_origin: " << _s_aux->parent->cost << std::endl;
                // std::cout << "cost_origin: " << cost_origin2 << std::endl;
                // cost_goal2=  _s2_aux->cost/dist_max; // 17-09-2021
                cost_goal2=  _s2_aux->cost;

                // std::cout << "cost_goal: " << _s2_aux->cost << std::endl;
                // std::cout << "cost_goal: " << cost_goal2 << std::endl;

                // meand_dist_cost2 should be between 0 and 1.
                float var1, var2, var3;
                var1=(checked_nodes->size());
                var2=var1-1;  // Because the checked_nodes also considers the origina and goal cost.
                var3=1/var2;
                // std::cout << "VAR2: " << var2 << std::endl;
                // std::cout << "VAR3: " << var3 << std::endl;

                // mean_dist_cost2 = ((dist_cost2/((var1)*dist_max))+(cost_origin2/2-cost_goal2/2))/var2;  // 17-09-2021
                mean_dist_cost2 = ((dist_cost2+(cost_origin2/2)-(cost_goal2/2))/(var1*dist_max)); //A
                // mean_dist_cost2 = ((dist_cost2+cost_origin2)/((var1+1)*dist_max));  // B Considering completely origin and goal. Goal is included in dist_cost2

                edge2=(mean_dist_cost2)*dist2*scale; 
                //edge2 = 0;  // It would behave like Theta*

                // std::cout << "dist_cost2: " << dist_cost2 << std::endl;
                // std::cout << "visited nodes: " << checked_nodes->size() << std::endl;
                // std::cout << "cost_origin2: " << cost_origin2 << std::endl;
                // std::cout << "cost_goal2: " << cost_goal2 << std::endl;
                // std::cout << "mean_dist_cost2: " << mean_dist_cost2 << std::endl;
                // std::cout << "dist2: " << dist2 << std::endl;
                // std::cout << "Edge Cost2: " << edge2 << std::endl;
            }
            else if (checked_nodes->size() == 1){
                for(auto &it: *checked_nodes){
                    auto node = discrete_world_.getNodePtr(it);
                    // std::cout << "Node Coordinates: " << node->coordinates << std::endl;
                    // std::cout << "Node Cost: " << node->cost << std::endl;
                    // std::cout << "Node G: " << node->G << std::endl;
                    // std::cout << "Node Score (G+H): " << node->getScore() << std::endl;
                    // std::cout << "Node Score (G+D+H): " << node->getScoreWithSafetyCost() << std::endl;
                    // dist_cost2 += node->G;
                    dist_cost2 += node->cost;
                }    
                // std::cout << "ELSE IF Parent" << std::endl;               
                cost_origin2 = _s_aux->parent->cost/dist_max;
                cost_goal2=  _s2_aux->cost/dist_max;
                mean_dist_cost2 = (((cost_origin2 + cost_goal2)/2 + ((dist_cost2)/dist_max)));
                edge2=(mean_dist_cost2)*dist2*scale;    //Parece que no mejora mucho respecto al mean_dist_cost2*dist2*scale
                // edge2 = 0; // It would behave like Theta*
                // G + normalized cost
                // edge2=dist2+dist_cost2;
                
                // std::cout << "dist_cost: " << dist_cost2 << std::endl;
                // std::cout << "visited nodes: " << checked_nodes->size() << std::endl;
                // std::cout << "cost_origin: " << cost_origin2 << std::endl;
                // std::cout << "cost_goal: " << cost_goal2 << std::endl;
                // std::cout << "mean_dist_cost2: " << mean_dist_cost2 << std::endl;
                // std::cout << "dist2: " << dist2 << std::endl;
                // std::cout << "Edge Cost2: " << edge2 << std::endl;                
            }
            else{
                // std::cout << "ELSE Parent" << std::endl;  
                // std::cout << "Origin Coordinates: " << _s_aux->parent->coordinates << std::endl;             
                // std::cout << "Goal Coordinates: " << _s2_aux->coordinates << std::endl;   
                // std::cout << "Node Coordinates of Parent: " << _s_aux->parent->coordinates << std::endl;          
                // std::cout << "NODE ADDR: " << &_s_aux << std::endl;
                // std::cout << "PARENT NODE ADDR: " << &_s_aux->parent << std::endl;
                cost_origin2 = _s_aux->parent->cost/dist_max;
                cost_goal2=  _s2_aux->cost/dist_max;
                // G + normalized cost
                // edge2= dist2 + ((cost_origin2 + cost_goal2)/2); 
                
                mean_dist_cost2 = (cost_origin2 + cost_goal2)/2;
                edge2=(mean_dist_cost2)*dist2*scale;
                // edge2 = 0; // It would behave like Theta*

                // std::cout << "visited nodes: " << checked_nodes->size() << std::endl;
                // std::cout << "cost_origin: " << _s_aux->parent->cost << std::endl;
                // std::cout << "cost_origin: " << cost_origin2 << std::endl;
                // std::cout << "cost_goal: " << _s2_aux->cost << std::endl;
                // std::cout << "cost_goal: " << cost_goal2 << std::endl;
                // std::cout << "dist2: " << dist2 << std::endl;
                // std::cout << "mean_dist_cost2: " << mean_dist_cost2 << std::endl;
                // std::cout << "Edge Cost2: " << edge2 << std::endl;                
            }
            // Compute cost considering the safety cost.
            if ((_s_aux->parent->G + dist2 + edge2) < (_s2_aux->G))
            {
                _s2_aux->parent = _s_aux->parent;
                // _s2_aux->G = _s2_aux->parent->G + geometry::distanceBetween2Nodes(_s2_aux->parent, _s2_aux) +  static_cast<int>(cost_weight_ * _s_aux->cost);
                _s2_aux->G = _s2_aux->parent->G + dist2 + edge2;
            }            
        } 


    }

    PathData LazyThetaStarGeneratorSafetyCost::findPath(const Vec3i &_source, const Vec3i &_target)
    {
        Node *current = nullptr;
        NodeSet openSet, closedSet;
        bool solved{false};

        float factor_cost = 1.4142;
        float factor_cost2 = 1.73;

        openSet.insert(discrete_world_.getNodePtr(_source));

        discrete_world_.getNodePtr(_source)->parent = new Node(_source);
        discrete_world_.setOpenValue(_source, true);

        utils::Clock main_timer;
        main_timer.tic();

        int line_of_sight_checks{0};

        while (!openSet.empty())
        {
            float aa, bb;
            aa=0;

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

            aa=current->cost;

            if (!los_neighbour_) 
                SetVertex(current); // Does this function make sense in the Lazy Safety Cost algorithm?

            los_neighbour_ = false;

            //in every setVertex the line of sight function is called 
            line_of_sight_checks++;
#if defined(ROS) && defined(PUB_EXPLORED_NODES)
            publishROSDebugData(current, openSet, closedSet);
#endif

            for (unsigned int i = 0; i < direction.size(); ++i)
            {

                Vec3i newCoordinates(current->coordinates + direction[i]);
                float edge_neighbour = 0;
                bb=0;

                if (discrete_world_.isOccupied(newCoordinates) ||
                    discrete_world_.isInClosedList(newCoordinates))
                    continue;
                Node *successor = discrete_world_.getNodePtr(newCoordinates);

                if (successor == nullptr)
                    continue;

                if (!discrete_world_.isInOpenList(newCoordinates))
                {
                    unsigned int totalCost = current->G;

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

                        edge_neighbour = (((aa+bb)/(2*100))*totalCost);

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

                        edge_neighbour = (((aa+bb)/(2*100))*totalCost);
                    }

                    successor->parent = current;
                    //if (successor->cost >0) std::cout << "Successor Cost " << successor->cost << " : " << std::endl;
                    //successor->G = totalCost + static_cast<int>(cost_weight_ * successor->cost);
                    //successor->G = totalCost + successor->parent->G + static_cast<int>(cost_weight_ * successor->cost);
                    successor->G = current->G + totalCost + edge_neighbour; // This is the same than A*
                    successor->H = heuristic(successor->coordinates, _target);
                    openSet.insert(successor);
                    //discrete_world_.setOpenValue(successor->coordinates, true);
                    discrete_world_.setOpenValue(*successor, true);
                }
                UpdateVertex(current, successor, openSet);
            }
        }
        main_timer.toc();

        PathData result_data;
        result_data["solved"] = solved;

        CoordinateList path;
        if (solved)
        {
            while (current != nullptr)
            {
                path.push_back(current->coordinates);
                current = current->parent;
            }
        }
        else
        {
            std::cout << "Error impossible to calcualte a solution" << std::endl;
        }
        result_data["algorithm"] = std::string("lazythetastar");
        result_data["path"] = path;
        result_data["time_spent"] = main_timer.getElapsedMillisecs();
        result_data["explored_nodes"] = closedSet.size();
        result_data["start_coords"] = _source;
        result_data["goal_coords"] = _target;
        result_data["path_length"] = geometry::calculatePathLength(path, discrete_world_.getResolution());
        result_data["line_of_sight_checks"] = line_of_sight_checks;

#if defined(ROS) && defined(PUB_EXPLORED_NODES)
        explored_node_marker_.points.clear();
#endif

        discrete_world_.resetWorld();
        return result_data;
    }

}
