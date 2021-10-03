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
        unsigned int dist_max = 100;

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
                G_new  = static_cast<unsigned int>(  successor2-> G + dist +  
                ( static_cast<double>(_s_aux->cost) + static_cast<double>(successor2->cost) ) / ( 2 * static_cast<double>(dist_max) ) * dist);

                if (G_new < G_max)
                {
                    G_max = G_new;
                    _s_aux->parent = successor2;
                    _s_aux->G = G_new;
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
            }            
        } 
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

            for (unsigned int i = 0; i < direction.size(); ++i)
            {

                Vec3i newCoordinates(current->coordinates + direction[i]);

                if (discrete_world_.isOccupied(newCoordinates) ||
                    discrete_world_.isInClosedList(newCoordinates))
                    continue;
                Node *successor = discrete_world_.getNodePtr(newCoordinates);

                if (successor == nullptr)
                    continue;

                if (!discrete_world_.isInOpenList(newCoordinates))
                {
                    //TODO : Check that this is OK. Before it was 
                    // unsigned int totalCost = current->G;
                    unsigned int totalCost = 0;

                    if(direction.size()  == 8){
                        totalCost += (i < 4 ? dist_scale_factor_ : dd_2D_); //This is more efficient
                    }else{
                        totalCost += (i < 6 ? dist_scale_factor_ : (i < 18 ? dd_2D_ : dd_3D_)); //This is more efficient
                    }

                    double bb = static_cast<double>( static_cast<double>(successor->cost) / (static_cast<double>(totalCost) / static_cast<double>(dist_scale_factor_)) );

                    auto edge_neighbour = static_cast<unsigned int>( ( ( ( current->cost + bb ) / ( 2 * 100 ) ) * totalCost ) );
            
                    successor->parent = current;
                    successor->G = current->G + totalCost + edge_neighbour; // This is the same than A*
                    successor->H = heuristic(successor->coordinates, _target);
                    openSet.insert(successor);
                    discrete_world_.setOpenValue(*successor, true);
                }
                UpdateVertex(current, successor, openSet);
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
