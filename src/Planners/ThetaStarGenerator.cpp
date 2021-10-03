#include "Planners/ThetaStarGenerator.hpp"

namespace Planners
{
    ThetaStarGenerator::ThetaStarGenerator(bool _use_3d):AStarGenerator(_use_3d, "thetastar") {}
    
    ThetaStarGenerator::ThetaStarGenerator(bool _use_3d, std::string _name = "thetastar" ):AStarGenerator(_use_3d, _name) {}
    
    void ThetaStarGenerator::UpdateVertex(Node *_s, Node *_s2, NodeSet &_openset)
    {
        unsigned int g_old = _s2->G;

        ComputeCost(_s, _s2);
        if (_s2->G < g_old)
        {
            /*
            The node is erased and after that inserted to simply 
            re-order the open list thus we can be sure that the node at
            the front of the list will be the one with the lowest cost
            */
            if (discrete_world_.isInOpenList(*_s2))
                _openset.erase(_s2);

            _openset.insert(_s2);
        }
    }

    void ThetaStarGenerator::ComputeCost(Node *_s_aux, Node *_s2_aux)
    {
        auto distanceParent2 = geometry::distanceBetween2Nodes(_s_aux->parent, _s2_aux);
        line_of_sight_checks_++;
        if (LineOfSight::bresenham3D((_s_aux->parent), _s2_aux, discrete_world_))
        {
            if ((_s_aux->parent->G + distanceParent2) < (_s2_aux->G))
            {
                _s2_aux->parent = _s_aux->parent;
                _s2_aux->G = _s_aux->parent->G + distanceParent2;
            }
        }
        else {
            auto distance2 = geometry::distanceBetween2Nodes(_s_aux, _s2_aux);
            if ((_s_aux->G + distance2) < _s2_aux->G){
                _s2_aux->parent=_s_aux;
                _s2_aux->G=_s_aux->G + distance2;
            }
        }
    }

    PathData ThetaStarGenerator::findPath(const Vec3i &_source, const Vec3i &_target)
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
                    unsigned int totalCost = current->G;

                    if(direction.size()  == 8){
                        totalCost += (i < 4 ? dist_scale_factor_ : dd_2D_); //This is more efficient
                    }else{
                        totalCost += (i < 6 ? dist_scale_factor_ : (i < 18 ? dd_2D_ : dd_3D_)); //This is more efficient
                    }

                    successor->parent = current;
                    successor->G = totalCost + successor->parent->G; //also current->G like succesor->parent->G
                    successor->H = heuristic(successor->coordinates, _target);
                    openSet.insert(successor);
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
        result_data["algorithm"] = std::string("thetastar");
        result_data["path"] = path;
        result_data["time_spent"] = main_timer.getElapsedMillisecs();
        result_data["explored_nodes"] = closedSet.size();
        result_data["start_coords"] = _source;
        result_data["goal_coords"] = _target;
        result_data["path_length"] = geometry::calculatePathLength(path, discrete_world_.getResolution());
        result_data["line_of_sight_checks"] = line_of_sight_checks_;

#if defined(ROS) && defined(PUB_EXPLORED_NODES)
        explored_node_marker_.points.clear();
#endif

        discrete_world_.resetWorld();
        return result_data;
    }

}
