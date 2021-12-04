#include "Planners/ThetaStarGenerator.hpp"

namespace Planners
{
    ThetaStarGenerator::ThetaStarGenerator(bool _use_3d):AStarGenerator(_use_3d, "thetastar") {}
    
    ThetaStarGenerator::ThetaStarGenerator(bool _use_3d, std::string _name = "thetastar" ):AStarGenerator(_use_3d, _name) {}
    
    void ThetaStarGenerator::UpdateVertex(Node *_s, Node *_s2, node_by_position &_index_by_pos, node_by_cost &_index_by_cost)
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
            auto found = _index_by_pos.find(_s2->world_index);
            _index_by_pos.erase(found);
            _index_by_cost.insert(_s2);
        }
    }

    void ThetaStarGenerator::ComputeCost(Node *_s_aux, Node *_s2_aux)
    {
        auto distanceParent2 = geometry::distanceBetween2Nodes(_s_aux->parent, _s2_aux);
        line_of_sight_checks_++;
        if ( LineOfSight::bresenham3D(_s_aux->parent, _s2_aux, discrete_world_) )
        {
            if ( ( _s_aux->parent->G + distanceParent2 ) < _s2_aux->G )
            {
                _s2_aux->parent = _s_aux->parent;
                _s2_aux->G      = _s_aux->parent->G + distanceParent2;
                _s2_aux->gplush = _s2_aux->G + _s2_aux->H;
            }

        } else {

            auto distance2 = geometry::distanceBetween2Nodes(_s_aux, _s2_aux);
            if ( ( _s_aux->G + distance2 ) < _s2_aux->G )
            {
                _s2_aux->parent = _s_aux;
                _s2_aux->G      = _s_aux->G + distance2;
                _s2_aux->gplush = _s2_aux->G + _s2_aux->H;
            }
        }
    }

    void ThetaStarGenerator::exploreNeighbours(Node* _current, const Vec3i &_target,node_by_position &_index_by_pos, node_by_cost &_index_by_cost){

        for (unsigned int i = 0; i < direction.size(); ++i) {

            Vec3i newCoordinates = _current->coordinates + direction[i];

            if ( discrete_world_.isOccupied(newCoordinates) || 
                 discrete_world_.isInClosedList(newCoordinates) ) 
                continue;
    
            Node *successor = discrete_world_.getNodePtr(newCoordinates);

            if(successor == nullptr) continue;

            if (!discrete_world_.isInOpenList(newCoordinates)) { 

                successor->parent = _current;
                successor->G = computeG(_current, successor, i, direction.size());
                successor->H = heuristic(successor->coordinates, _target);
                successor->gplush = successor->G + successor->H;
                _index_by_cost.insert(successor);
                discrete_world_.setOpenValue(*successor, true);
            }
         
            UpdateVertex(_current, successor, _index_by_pos, _index_by_cost); 
        }
    }
}
