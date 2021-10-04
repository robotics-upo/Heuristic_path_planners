#include "Planners/LazyThetaStarGenerator.hpp"

namespace Planners
{
    LazyThetaStarGenerator::LazyThetaStarGenerator(bool _use_3d):ThetaStarGenerator(_use_3d, "lazythetastar") {}
    LazyThetaStarGenerator::LazyThetaStarGenerator(bool _use_3d, std::string _name = "lazythetastar" ):ThetaStarGenerator(_use_3d, _name) {}
   
    void LazyThetaStarGenerator::SetVertex(Node *_s_aux)
    {
        line_of_sight_checks_++;

        if (!LineOfSight::bresenham3D((_s_aux->parent), _s_aux, discrete_world_))
        {
            unsigned int G_max = std::numeric_limits<unsigned int>::max(); 
            unsigned int G_new;

            for (const auto &i: direction)
            {
                Vec3i newCoordinates(_s_aux->coordinates + i);

                if ( discrete_world_.isOccupied(newCoordinates) ) continue;

                if ( discrete_world_.isInClosedList(newCoordinates) )
                {
                    Node *successor2 = discrete_world_.getNodePtr(newCoordinates);
                    if (successor2 == nullptr) continue;

                    G_new = successor2->G +  geometry::distanceBetween2Nodes(successor2, _s_aux);
                    if (G_new < G_max)
                    {
                        G_max = G_new;
                        _s_aux->parent = successor2;
                        _s_aux->G = G_new;
                    }
                }
            }
        }
    }
    void LazyThetaStarGenerator::ComputeCost(Node *_s_aux, Node *_s2_aux)
    {
        auto distanceParent2 = geometry::distanceBetween2Nodes(_s_aux->parent, _s2_aux);

        if ((_s_aux->parent->G + distanceParent2) < (_s2_aux->G))
        {
            _s2_aux->parent = _s_aux->parent;
            _s2_aux->G = _s2_aux->parent->G + distanceParent2;
        }
    }
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

    unsigned int LazyThetaStarGenerator::computeG(const Node* _current, const Node* _suc,  unsigned int _n_i, unsigned int _dirs){

        unsigned int cost = _current->G;

        if(_dirs  == 8){
            cost += (_n_i < 4 ? dist_scale_factor_ : dd_2D_); //This is more efficient
        }else{
            cost += (_n_i < 6 ? dist_scale_factor_ : (_n_i < 18 ? dd_2D_ : dd_3D_)); //This is more efficient
        }
        // TODO In Theta* we add this to the cost, should it be the same here?? Its more efficient if we dont
        // add this term to the cost
        // cost += _suc->parent->G;

        return cost;
    }
#pragma GCC diagnostic pop

    
    PathData LazyThetaStarGenerator::findPath(const Vec3i &_source, const Vec3i &_target)
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

            SetVertex(current);
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
