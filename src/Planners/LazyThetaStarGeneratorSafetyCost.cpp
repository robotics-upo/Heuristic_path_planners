#include "Planners/LazyThetaStarGeneratorSafetyCost.hpp"

namespace Planners
{
    LazyThetaStarGeneratorSafetyCost::LazyThetaStarGeneratorSafetyCost(bool _use_3d):ThetaStarGeneratorSafetyCost(_use_3d, "lazythetastarsafetycost") {}
    LazyThetaStarGeneratorSafetyCost::LazyThetaStarGeneratorSafetyCost(bool _use_3d, std::string _name = "lazythetastarsafetycost" ):ThetaStarGeneratorSafetyCost(_use_3d, _name) {}
    
    void LazyThetaStarGeneratorSafetyCost::SetVertex(Node *_s_aux)
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

                auto dist = geometry::distanceBetween2Nodes(successor2, _s_aux);
                
                G_new  = static_cast<unsigned int>(  successor2-> G + dist +  
                ( static_cast<double>(_s_aux->cost) + static_cast<double>(successor2->cost) ) / 2);

                if (G_new < G_max)
                {
                    _s_aux->parent = successor2;
                    _s_aux->G      = G_new;
                    _s_aux->C      = (static_cast<double>(_s_aux->cost) + static_cast<double>(successor2->cost)) / 2;
                    _s_aux->gplush = _s_aux->G + _s_aux->H;
                }
            }
        }
    }

    void LazyThetaStarGeneratorSafetyCost::ComputeCost(Node *_s_aux, Node *_s2_aux)
    {
        utils::CoordinateListPtr checked_nodes;
        checked_nodes.reset(new CoordinateList);

        line_of_sight_checks_++;
        if (LineOfSight::bresenham3D(_s_aux->parent, _s2_aux, discrete_world_, checked_nodes)) {
            
            los_neighbour_ = true;

            auto dist2   = geometry::distanceBetween2Nodes(_s_aux->parent, _s2_aux);
            auto edge2   = ComputeEdgeCost(checked_nodes, _s_aux->parent, _s2_aux);

            if ( ( _s_aux->parent->G + dist2 + edge2 ) < _s2_aux->G ) 
            {
                _s2_aux->parent = _s_aux->parent;
                _s2_aux->G      = _s2_aux->parent->G + dist2 + edge2;
                _s2_aux->C      = edge2;
                _s2_aux->gplush = _s2_aux->G + _s2_aux->H;
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

        double cc = ( _current->cost + _suc->cost ) / 2;
        auto edge_neighbour = static_cast<unsigned int>( cc *  cost_weight_ * (dist_scale_factor_/100)); 
    
        cost += ( _current->G + edge_neighbour );

        _suc->C = edge_neighbour;
        
        return cost;
    }
    PathData LazyThetaStarGeneratorSafetyCost::findPath(const Vec3i &_source, const Vec3i &_target)
    {
        Node *current = nullptr;
        std::vector<Node*> closedSet;
        bool solved{false};

        discrete_world_.getNodePtr(_source)->parent = new Node(_source);
        discrete_world_.setOpenValue(_source, true);

        utils::Clock main_timer;
        main_timer.tic();

        line_of_sight_checks_ = 0;
        MagicalMultiSet openSet;

        node_by_cost& indexByCost              = openSet.get<IndexByCost>();
        node_by_position& indexByWorldPosition = openSet.get<IndexByWorldPosition>();

        indexByCost.insert(discrete_world_.getNodePtr(_source));
        while (!openSet.empty())
        {
            auto it = indexByCost.begin();
            current = *it;
            indexByCost.erase(indexByCost.begin());

            if (current->coordinates == _target)
            {
                solved = true;
                break;
            }

            closedSet.push_back(current);

            discrete_world_.setOpenValue(*current, false);
            discrete_world_.setClosedValue(*current, true);

            if (!los_neighbour_) 
                SetVertex(current); // Does this function make sense in the Lazy Safety Cost algorithm?

            los_neighbour_ = false;

#if defined(ROS) && defined(PUB_EXPLORED_NODES)
            publishROSDebugData(current, indexByCost, closedSet);
#endif
            exploreNeighbours(current, _target, indexByWorldPosition);


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
