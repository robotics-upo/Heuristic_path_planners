#include "Planners/CostAwareThetaStarGenerator.hpp"

namespace Planners
{
    void CostAwareThetaStarGenerator::UpdateVertex(Node *_s, Node *_s2, NodeSet &_openset)
    {
        float g_old = _s2->G;
        // std::cout << "s1 G 1: " << _s->G << std::endl;
        // std::cout << "s1 Parent G 1: " << _s->parent->G << std::endl;
        // std::cout << "s1 cost: " << _s->cost << std::endl;        
        // std::cout << "s2 G 1: " << _s2->G << std::endl;
        // std::cout << "s2 Parent G 1: " << _s->parent->G << std::endl;
        // std::cout << "s2 cost: " << _s2->cost << std::endl;
        // std::cout << "Update1 " << std::endl;
        ComputeCost(_s, _s2);
        // std::cout << "s2 G 2: " << _s2->G << std::endl;
        if (_s2->G < g_old)
        // if (_s2->non_uni < g_old)
        {
            /*
            The node is erased and after that inserted to simply 
            re-order the open list thus we can be sure that the node at
            the front of the list will be the one with the lowest cost
            */
            // std::cout << "Update2 " << std::endl;
            if (discrete_world_.isInOpenList(*_s2))
                _openset.erase(_s2);

            _openset.insert(_s2);
        }
    }

    void CostAwareThetaStarGenerator::ComputeCost(Node *_s_aux, Node *_s2_aux)
    {
        //auto distanceParent2 = geometry::distanceBetween2Nodes(_s_aux->parent, _s2_aux);
        utils::CoordinateListPtr checked_nodes, checked_nodes_current;
        checked_nodes.reset(new CoordinateList);
        checked_nodes_current.reset(new CoordinateList);

        float dist_max=100; // This parameter should appear in mean_dist_cost2 instead of *100.
        float scale=1.5; // Increase the influence of the distance cost. Important change between 5 and 6.

        if (LineOfSight::bresenham3D((_s_aux->parent), _s2_aux, discrete_world_, checked_nodes))  
        {
            // Computation of edge _s_aux->parent to s2_aux
            // std::cout << "LineOfSight" << std::endl;               
            float dist_cost2= 0;
            float mean_dist_cost2 = 0;
            float edge2 = 0;
            float cost_origin2;
            float cost_goal2;
            auto dist2 = geometry::distanceBetween2Nodes(_s_aux->parent, _s2_aux);

            // INTERESTING THE BEHAVIOUR WITH THIS CONDITION WHEN 
            // if ((_s_aux->parent->coordinates.x == _s_aux->coordinates.x) & (_s_aux->parent->coordinates.y == _s_aux->coordinates.y) & (_s_aux->parent->coordinates.z == _s_aux->coordinates.z))
            // {
            //     _s_aux->parent->cost=_s_aux->cost;
            //     std::cout << "IGUALES" << std::endl;
            // }

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

                // std::cout << "visited nodes: " << checked_nodes->size() << std::endl;
                // std::cout << "cost_origin: " << _s_aux->parent->cost << std::endl;
                // std::cout << "cost_origin: " << cost_origin2 << std::endl;
                // std::cout << "cost_goal: " << _s2_aux->cost << std::endl;
                // std::cout << "cost_goal: " << cost_goal2 << std::endl;
                // std::cout << "dist2: " << dist2 << std::endl;
                // std::cout << "mean_dist_cost2: " << mean_dist_cost2 << std::endl;
                // std::cout << "Edge Cost2: " << edge2 << std::endl;                
            }

            // JAC: Normalize the dist_cost2 depending on the number of cells and the maximum value of the cost.
            //dist_cost2= dist_cost2/(dist_max*checked_nodes->size());
            
            // float mean_dist_cost2 = 0;
            // float edge2 = 0;
            // int cost_origin2 = 0;
            // int cost_goal2 = 0;
            // auto dist2 = geometry::distanceBetween2Nodes(_s_aux->parent, _s2_aux);

            // ******************************************************************************               
            // Computation of edge _s_aux to _s2_aux
            // ******************************************************************************
            LineOfSight::bresenham3D(_s_aux, _s2_aux, discrete_world_, checked_nodes_current);
            
            float dist_cost= 0;
            float mean_dist_cost = 0;
            float edge1 = 0;
            float cost_origin = 0;
            float cost_goal = 0;
            auto dist1 = geometry::distanceBetween2Nodes(_s_aux, _s2_aux);  

            if (checked_nodes_current->size() > 1){
                // std::cout << "IF Current" << std::endl;
                for(auto &it: *checked_nodes_current){
                    auto node = discrete_world_.getNodePtr(it);
                    // std::cout << "Node Coordinates: " << node->coordinates << std::endl;
                    // std::cout << "Node Cost: " << node->cost << std::endl;
                    // std::cout << "Node G: " << node->G << std::endl;
                    // std::cout << "Node Score (G+H): " << node->getScore() << std::endl;
                    // std::cout << "Node Score (G+D+H): " << node->getScoreWithSafetyCost() << std::endl;
                    // dist_cost2 += node->G;
                    dist_cost += node->cost;
                }                
                // cost_origin = _s_aux->cost/dist_max; // 17-09-2021
                cost_origin = _s_aux->cost;
                
                // cost_goal=  _s2_aux->cost/dist_max; // 17-09-2021
                cost_goal=  _s2_aux->cost;
                
                float var11, var22, var33;
                var11=(checked_nodes_current->size());
                var22=var11-1;
                var33=1/var22;
                // std::cout << "VAR22: " << var22 << std::endl;
                // std::cout << "VAR33: " << var33 << std::endl;

                // mean_dist_cost = var33*((dist_cost/((var11)*dist_max))+(cost_origin/2)-(cost_goal/2));  // 17-09-2021
                mean_dist_cost = ((dist_cost+(cost_origin/2)-(cost_goal/2))/(var11*dist_max)); //A
                // mean_dist_cost = ((dist_cost+cost_origin)/((var11+1)*dist_max));  // B Considering completely origin and goal. Goal is included in dist_cost2
               
                edge1=(mean_dist_cost)*dist1*scale;
                // edge2=mean_dist_cost2;

                // std::cout << "dist_cost: " << dist_cost << std::endl;
                // std::cout << "visited nodes: " << checked_nodes_current->size() << std::endl;
                // std::cout << "cost_origin: " << _s_aux->cost << std::endl;
                // std::cout << "cost_origin: " << cost_origin << std::endl;
                // std::cout << "cost_goal: " << _s2_aux->cost << std::endl;
                // std::cout << "cost_goal: " << cost_goal << std::endl;
                // std::cout << "mean_dist_cost: " << mean_dist_cost << std::endl;
                // std::cout << "dist1: " << dist1 << std::endl;
                // std::cout << "Edge Cost1: " << edge1 << std::endl;
            }
            else if (checked_nodes_current->size() == 1){ 
                for(auto &it: *checked_nodes_current){
                    auto node = discrete_world_.getNodePtr(it);
                    // std::cout << "Node Coordinates: " << node->coordinates << std::endl;
                    // std::cout << "Node Cost: " << node->cost << std::endl;
                    // std::cout << "Node G: " << node->G << std::endl;
                    // std::cout << "Node Score (G+H): " << node->getScore() << std::endl;
                    // std::cout << "Node Score (G+D+H): " << node->getScoreWithSafetyCost() << std::endl;
                    // dist_cost2 += node->G;
                    dist_cost += node->cost;
                }    
                // std::cout << "ELSE IF Current" << std::endl;               
                cost_origin = _s_aux->cost/dist_max;
                cost_goal =  _s2_aux->cost/dist_max;
                // mean_dist_cost = ((((cost_origin + cost_goal)/2 + dist_cost))/(2*dist_max));
                mean_dist_cost = (((cost_origin + cost_goal)/2 + ((dist_cost)/dist_max)));
                edge1=(mean_dist_cost)*dist1*scale;

                // G + normalized cost
                // edge1=dist1 + dist_cost;
                
                // std::cout << "dist_cost: " << dist_cost2 << std::endl;
                // std::cout << "visited nodes: " << checked_nodes_current->size() << std::endl;
                // std::cout << "cost_origin: " << cost_origin << std::endl;
                // std::cout << "cost_goal: " << cost_goal << std::endl;
                // std::cout << "mean_dist_cost1: " << mean_dist_cost << std::endl;
                // std::cout << "dist1: " << dist1 << std::endl;
                // std::cout << "Edge Cost1: " << edge1 << std::endl;            
            }
            else{
                // std::cout << "ELSE Current" << std::endl;
                // std::cout << "Origin Coordinates: " << _s_aux->coordinates << std::endl;             
                // std::cout << "Goal Coordinates: " << _s2_aux->coordinates << std::endl;                 
                cost_origin = _s_aux->cost/dist_max;
                cost_goal=  _s2_aux->cost/dist_max;
                mean_dist_cost = (cost_origin + cost_goal)/2;
                edge1=(mean_dist_cost)*dist1*scale;
                // G + normalized cost
                // edge1 = dist1 + ((cost_origin + cost_goal)/2); 

                // std::cout << "visited nodes: " << checked_nodes_current->size() << std::endl;
                // std::cout << "cost_origin: " << _s_aux->cost << std::endl;
                // std::cout << "cost_origin: " <<  cost_origin  << std::endl;
                // std::cout << "cost_goal: " << _s2_aux->cost << std::endl;
                // std::cout << "cost_goal: " <<  cost_goal << std::endl;
                // std::cout << "mean_dist_cost: " << mean_dist_cost << std::endl;
                // std::cout << "dist1: " << dist1 << std::endl;
                // std::cout << "Edge Cost1: " << edge1 << std::endl;                
            }

            // std::cout << "s2_aux_parent G: " << _s_aux->parent->G << std::endl;
            // std::cout << "s2_aux G: " << _s_aux->G << std::endl;



            if ((_s_aux->parent->G + edge2) <= (_s_aux->G + edge1)) 
            {
                // std::cout << "IIIIFFFF Edge2" << std::endl;
                _s2_aux->parent = _s_aux->parent;
                //_s2_aux->G = _s_aux->parent->G + dist2; // It is not necessary
                // std::cout << "Previous Non_uni: " << _s_aux->parent->non_uni << std::endl;
                _s2_aux->G = _s_aux->parent->G + edge2;

                // if ((_s_aux->parent->G + distanceParent2) < (_s2_aux->G))
                // {
                //     _s2_aux->parent = _s_aux->parent;
                //     _s2_aux->G = _s_aux->parent->G + distanceParent2;
                // }
            }
            else{
                // std::cout << "EEELLLSSSEEE Edge1" << std::endl;
                //auto distance2 = geometry::distanceBetween2Nodes(_s_aux, _s2_aux);
                _s2_aux->parent=_s_aux;
                //_s2_aux->G=_s_aux->G + dist;  // It is not necessary

                // // Computation of edge _s_aux to _s2_aux
                // LineOfSight::bresenham3D(_s_aux, _s2_aux, discrete_world_, checked_nodes);
                // float dist_cost= 0;
                // for(auto &it: *checked_nodes){
                //     auto node = discrete_world_.getNodePtr(it);
                //     // std::cout << "Node Coordinates: " << node->coordinates << std::endl;
                //     // std::cout << "Node Cost: " << node->cost << std::endl;
                //     // std::cout << "Node G: " << node->G << std::endl;
                //     // std::cout << "Node Score (G+H): " << node->getScore() << std::endl;
                //     // std::cout << "Node Score (G+D+H): " << node->getScoreWithSafetyCost() << std::endl;
                //     dist_cost += node->G + node->cost;
                // }
                // float mean_dist_cost = 0;
                // float edge1 = 0;
                // int cost_origin = 0;
                // int cost_goal = 0;
                // //int tam = checked_nodes->size();
                // auto dist = geometry::distanceBetween2Nodes(_s_aux, _s2_aux);
                // cost_origin= _s_aux->G + _s_aux->cost;
                // cost_goal= _s2_aux->G + _s2_aux->cost;
                // mean_dist_cost = (1/(checked_nodes->size() - 1))*(dist_cost-(cost_origin/2)-(cost_goal/2)); // JAC: Include the v(c_p)" and v(c_c)"
                // edge1=mean_dist_cost*dist;
                // // std::cout << "dist_cost: " << dist_cost << std::endl;
                // // std::cout << "visited nodes: " << checked_nodes->size() << std::endl;
                // // std::cout << "cost_origin: " << cost_origin << std::endl;
                // // std::cout << "cost_goal: " << cost_goal << std::endl;
                // // std::cout << "mean_dist_cost: " << mean_dist_cost << std::endl;
                // // std::cout << "dist: " << dist << std::endl;
                // // std::cout << "Edge Cost1: " << edge1 << std::endl;            
                // std::cout << "Edge Cost1: " << edge1 << std::endl;  

                //_s2_aux->non_uni= edge1;
                // std::cout << "Previous Non_uni: " << _s_aux->parent->non_uni << std::endl;
                _s2_aux->G= _s_aux->G + edge1;               
            }

            
        }
        else {
            // std::cout << "NOOOO LineOfSight" << std::endl;
            //auto dist1 = geometry::distanceBetween2Nodes(_s_aux, _s2_aux);
            _s2_aux->parent=_s_aux;
            //_s2_aux->G=_s_aux->G + distance2;  // It is not necessary

            LineOfSight::bresenham3D(_s_aux, _s2_aux, discrete_world_, checked_nodes);
            
            float dist_cost= 0;
            float mean_dist_cost = 0;
            float edge1 = 0;
            float cost_origin = 0;
            float cost_goal = 0;
            auto dist1 = geometry::distanceBetween2Nodes(_s_aux, _s2_aux);  

            if (checked_nodes->size() > 1){
                // std::cout << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX" << std::endl;
                // std::cout << "IF" << std::endl;
                for(auto &it: *checked_nodes){
                    auto node = discrete_world_.getNodePtr(it);
                    // std::cout << "Node Coordinates: " << node->coordinates << std::endl;
                    // std::cout << "Node Cost: " << node->cost << std::endl;
                    // std::cout << "Node G: " << node->G << std::endl;
                    // std::cout << "Node Score (G+H): " << node->getScore() << std::endl;
                    // std::cout << "Node Score (G+D+H): " << node->getScoreWithSafetyCost() << std::endl;
                    // dist_cost2 += node->G;
                    dist_cost += node->cost;
                }             
                // std::cout << "Origin Coordinates: " << _s_aux->coordinates << std::endl;             
                // std::cout << "Goal Coordinates: " << _s2_aux->coordinates << std::endl;                    
                // cost_origin = _s_aux->cost/dist_max; // 17-09-2021
                cost_origin = _s_aux->cost;
                
                // cost_goal=  _s2_aux->cost/dist_max; // 17-09-2021
                cost_goal=  _s2_aux->cost;
                
                float var11, var22, var33;
                var11=(checked_nodes->size());
                var22=var11-1;
                var33=1/var22;
                // std::cout << "VAR22: " << var22 << std::endl;
                // std::cout << "VAR33: " << var33 << std::endl;

                // mean_dist_cost = var33*((dist_cost/((var11)*dist_max))+(cost_origin/2)-(cost_goal/2));  // 17-09-2021
                mean_dist_cost = ((dist_cost+(cost_origin/2)-(cost_goal/2))/(var11*dist_max));  //A
                // mean_dist_cost = ((dist_cost+cost_origin)/((var11+1)*dist_max));  // B Considering completely origin and goal. Goal is included in dist_cost2

                edge1=(mean_dist_cost)*dist1*scale;
                // edge1=mean_dist_cost;

                // std::cout << "dist_cost: " << dist_cost << std::endl;
                // std::cout << "visited nodes: " << checked_nodes->size() << std::endl;
                // std::cout << "cost_origin: " << cost_origin << std::endl;
                // std::cout << "cost_goal: " << cost_goal << std::endl;
                // std::cout << "mean_dist_cost: " << mean_dist_cost << std::endl;
                // std::cout << "dist1: " << dist1 << std::endl;
                // std::cout << "Edge Cost1: " << edge1 << std::endl;
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
                    dist_cost += node->cost;
                }
                // std::cout << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX" << std::endl;
                // std::cout << "ELSE IF" << std::endl;               
                cost_origin = _s_aux->cost/dist_max;
                cost_goal =  _s2_aux->cost/dist_max;
                mean_dist_cost = (((cost_origin + cost_goal)/2 + ((dist_cost)/dist_max)));
                // mean_dist_cost = ((((cost_origin + cost_goal)/2 + dist_cost))/(2*dist_max));
                edge1=(mean_dist_cost)*dist1*scale;

                // G + normalized cost
                // edge1=dist1 + dist_cost;
                
                // std::cout << "dist_cost: " << dist_cost2 << std::endl;
                // std::cout << "visited nodes: " << checked_nodes->size() << std::endl;
                // std::cout << "cost_origin: " << cost_origin << std::endl;
                // std::cout << "cost_goal: " << cost_goal << std::endl;
                // std::cout << "mean_dist_cost: " << mean_dist_cost << std::endl;
                // std::cout << "dist1: " << dist1 << std::endl;
                // std::cout << "Edge Cost1: " << edge1 << std::endl;            
            }
            else{
                // std::cout << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX" << std::endl;
                // std::cout << "ELSE" << std::endl;
                cost_origin = _s_aux->cost/dist_max;
                cost_goal=  _s2_aux->cost/dist_max;
                // mean_dist_cost = (((cost_origin + cost_goal))/(2*dist_max));
                mean_dist_cost = (cost_origin + cost_goal)/2;
                edge1=(mean_dist_cost)*dist1*scale;
                // G + normalized cost
                // edge1 = dist1 + ((cost_origin + cost_goal)/2); 

                // std::cout << "visited nodes: " << checked_nodes->size() << std::endl;
                // std::cout << "cost_origin: " << _s_aux->cost << std::endl;
                // std::cout << "cost_goal: " << _s2_aux->cost << std::endl;
                // std::cout << "mean_dist_cost: " << mean_dist_cost << std::endl;
                // std::cout << "dist1: " << dist1 << std::endl;
                // std::cout << "Edge Cost1: " << edge1 << std::endl;                
            }
            _s2_aux->G= _s_aux->G + edge1;

            
        }

        // To print the checked_nodes with Bresenham
        // if( !checked_nodes->empty() ){
        //     std::cout << "Theta Star cells checked in line of sight check between " << _s_aux->coordinates << " and " << _s2_aux->coordinates << " : " << std::endl;
        //     std::cout << *(checked_nodes.get()) << std::endl;
        //     std::cout << checked_nodes->size() << std::endl;
        // }
    }

    PathData CostAwareThetaStarGenerator::findPath(const Vec3i &_source, const Vec3i &_target)
    {
        Node *current = nullptr;
        NodeSet openSet, closedSet;
        bool solved{false};

        openSet.insert(discrete_world_.getNodePtr(_source));
        discrete_world_.getNodePtr(_source)->parent = new Node(_source);
        // discrete_world_.getNodePtr(_source)->parent = discrete_world_.getNodePtr(_source); // Para resolver inicialización de cost del parent pero peta el algoritmo al final
        discrete_world_.setOpenValue(_source, true);
        
        utils::Clock main_timer;
        main_timer.tic();

        int line_of_sight_checks{0};

        while (!openSet.empty())
        {
            //std::cout << "SAFETY COST" << std::endl;

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

            // std::cout << "Current G: " << current->G << std::endl;
            // std::cout << "Current Parent G: " << current->parent->G << std::endl;
            // std::cout << "Current Cost: " << aa << std::endl;
            
            
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

                if (successor == nullptr) continue;

                if (!discrete_world_.isInOpenList(newCoordinates))
                {
                    unsigned int totalCost = 0;
                    // unsigned int totalCost = current->G;
                    // unsigned int totalCost = current->non_uni;
                    // std::cout << "totalCost " << totalCost << " : " << std::endl;

                    if(direction.size()  == 8){
                        totalCost += (i < 4 ? dist_scale_factor_ : dd_2D_); //This is more efficient
                        // std::cout << "Current Cost: " << current->cost << std::endl;
                        // std::cout << "Successor Cost: " << successor->cost << std::endl;

                        bb=successor->cost;
                        edge_neighbour = (((aa+bb)/(2*100))*totalCost);
                        // edge_neighbour = totalCost + ((aa+bb)/2);  // G + normalized cost, 17-09-2021
                        // edge_neighbour = totalCost + ((aa+bb)/(2*100))*totalCost;
                        // std::cout << "edge_neighbour: " << edge_neighbour << std::endl;
                        // totalCost += edge_neighbour;
                        // totalCost= current->G + edge_neighbour;
                        // std::cout << "edge neighbour: " << edge_neighbour << std::endl;
                        
                        
                        // std::cout << "total Cost1: " << totalCost << std::endl;
                        // std::cout << "Successor Cost: " << successor->cost << std::endl;                        
                        // totalCost += (successor->cost)/100; //JAC: Include the cost of non_uni? That is, the cost to go from current to succesor.
                        // std::cout << "total Cost2: " << totalCost << std::endl;                        
                        

                    }else{
                        totalCost += (i < 6 ? dist_scale_factor_ : (i < 18 ? dd_2D_ : dd_3D_)); //This is more efficient
                        bb=successor->cost;
                        edge_neighbour = (((aa+bb)/(2*100))*totalCost);
                        //edge_neighbour = totalCost + ((aa+bb)/2);   // G + normalized cost, 17-09-2021
                        // edge_neighbour = totalCost + ((aa+bb)/(2*100))*totalCost;

                        // totalCost += edge_neighbour;
                        
                        // std::cout << "total Cost1: " << totalCost << std::endl;
                        // totalCost += (successor->cost)/100; //JAC: Include the cost of non_uni? That is, the cost to go from current to succesor.
                        // std::cout << "total Cost2: " << totalCost << std::endl;                        
                        // std::cout << "Successor Cost: " << successor->cost << std::endl;                        
                    }
                    //std::cout << "totalCost2 " << totalCost << " : " << std::endl;

                    // std::cout << "Current Coordinates: " << current->coordinates << std::endl;
                    // std::cout << "Parent Current Coordinates: " << current->parent->coordinates << std::endl;
                    // std::cout << "Successor Coordinates: " << successor->coordinates << std::endl;
                    // successor->parent = current;
                    // successor->G = edge_neighbour; 
                    successor->G = current->G + edge_neighbour; // This is the right
                    // successor->G = totalCost; 
                    // std::cout << "Successor G: " << successor->G << std::endl;                        
                    successor->H = heuristic(successor->coordinates, _target);
                    // std::cout << "Heuristic: " << successor->H << std::endl;
                    openSet.insert(successor);
                    //discrete_world_.setOpenValue(successor->coordinates, true);
                    discrete_world_.setOpenValue(*successor, true);
                }
                
                UpdateVertex(current, successor, openSet); 
                //Every time a vertex is updated, a line of sight check is 
                line_of_sight_checks++;
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
        result_data["algorithm"] = std::string("thtetastar");
        result_data["path"] = path;
        result_data["time_spent"] = main_timer.getElapsedMillisecs();
        result_data["explored_nodes"] = closedSet.size();
        result_data["start_coords"] = _source;
        result_data["goal_coords"] = _target;
        result_data["path_length"] = geometry::calculatePathLength(path, discrete_world_.getResolution());
        std::cout << "Line of sight checks: " << line_of_sight_checks << std::endl;
        result_data["line_of_sight_checks"] = line_of_sight_checks;

#if defined(ROS) && defined(PUB_EXPLORED_NODES)
        explored_node_marker_.points.clear();
#endif

        discrete_world_.resetWorld();
        return result_data;
    }

}
