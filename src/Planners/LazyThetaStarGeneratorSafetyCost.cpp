#include "Planners/LazyThetaStarGeneratorSafetyCost.hpp"

namespace Planners
{
    LazyThetaStarGeneratorSafetyCost::LazyThetaStarGeneratorSafetyCost(bool _use_3d):ThetaStarGeneratorSafetyCost(_use_3d, "lazythetastarsafetycost") {}
    LazyThetaStarGeneratorSafetyCost::LazyThetaStarGeneratorSafetyCost(bool _use_3d, std::string _name = "lazythetastarsafetycost" ):ThetaStarGeneratorSafetyCost(_use_3d, _name) {}
    
    void LazyThetaStarGeneratorSafetyCost::SetVertex(Node *_s_aux)
    {   
        unsigned int G_max = std::numeric_limits<unsigned int>::max(); 
        unsigned int G_new;

        //TODO WHat is this value? dist_scale_factor_ ?
        //TODO 2 "scale" does not appear here? 
        // unsigned int dist_max = 100;

        for (const auto &i: direction)
        {
            Vec3i newCoordinates(_s_aux->coordinates + i);

            if ( discrete_world_.isOccupied(newCoordinates) ) continue;
            
            if ( discrete_world_.isInClosedList(newCoordinates) )
            {
                Node *successor2 = discrete_world_.getNodePtr(newCoordinates);
                if (successor2 == nullptr) continue;

                auto dist = geometry::distanceBetween2Nodes(successor2, _s_aux);
                
                // Be careful with castings here. Its already checked before and after is the same result.
                //TODO Add comment explaining the "100" in the equation
                G_new  = static_cast<unsigned int>(  successor2-> G + dist +  
                ( static_cast<double>(_s_aux->cost) + static_cast<double>(successor2->cost) ) / ( 2 * 100 ) * dist);

                if (G_new < G_max)
                {
                    G_max = G_new;
                    _s_aux->parent = successor2;
                    _s_aux->G = G_new;
                    _s_aux->C = static_cast<double>(_s_aux->cost) + static_cast<double>(successor2->cost) / ( 2 * 100 ) * dist;
                }
            }
        }
    }

    void LazyThetaStarGeneratorSafetyCost::ComputeCost(Node *_s_aux, Node *_s2_aux)
    {
        utils::CoordinateListPtr checked_nodes;
        checked_nodes.reset(new CoordinateList);

        line_of_sight_checks_++;
        if (LineOfSight::bresenham3D((_s_aux->parent), _s2_aux, discrete_world_, checked_nodes)) {
            
            los_neighbour_ = true;

            auto dist2   = geometry::distanceBetween2Nodes(_s_aux->parent, _s2_aux);
            auto edge2   = ComputeEdgeCost(checked_nodes, _s_aux, _s2_aux, dist2);

            if ( ( _s_aux->parent->G + dist2 + edge2 ) < ( _s2_aux->G ) )
            {
                _s2_aux->parent = _s_aux->parent;
                _s2_aux->G      = _s2_aux->parent->G + dist2 + edge2;
                _s2_aux->C      = edge2;
            }            
        } 
    }

    unsigned int LazyThetaStarGeneratorSafetyCost::computeG(const Node* _current, Node* _suc,  unsigned int _n_i, unsigned int _dirs){

        unsigned int cost = 0;

        if(_dirs == 8){
            cost = (_n_i < 4 ? dist_scale_factor_ : dd_2D_); //This is more efficient
        }else{
            cost = (_n_i < 6 ? dist_scale_factor_ : (_n_i < 18 ? dd_2D_ : dd_3D_)); //This is more efficient
        }

        double bb = static_cast<double>( static_cast<double>(_suc->cost) / (static_cast<double>(cost) / static_cast<double>(dist_scale_factor_)) );
        auto edge_neighbour = static_cast<unsigned int>( ( ( ( _current->cost + bb ) / ( 2 * 100 ) ) * cost ) );
    
        cost += ( _current->G + edge_neighbour );

        _suc->C = edge_neighbour;
        
        return cost;
    }
    PathData LazyThetaStarGeneratorSafetyCost::findPath(const Vec3i &_source, const Vec3i &_target)
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

            if (!los_neighbour_) 
                SetVertex(current); // Does this function make sense in the Lazy Safety Cost algorithm?

            los_neighbour_ = false;

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
