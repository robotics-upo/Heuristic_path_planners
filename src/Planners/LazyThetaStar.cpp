#include "Planners/LazyThetaStar.hpp"

namespace Planners
{
    LazyThetaStar::LazyThetaStar(bool _use_3d):ThetaStar(_use_3d, "lazythetastar") {}
    LazyThetaStar::LazyThetaStar(bool _use_3d, std::string _name = "lazythetastar" ):ThetaStar(_use_3d, _name) {}
   
    void LazyThetaStar::SetVertex(Node *_s_aux, torch::jit::script::Module& loaded_sdf)
    {
        line_of_sight_checks_++;

        if (!LineOfSight::bresenham3D((_s_aux->parent), _s_aux, discrete_world_))
        {
            unsigned int G_max = std::numeric_limits<unsigned int>::max(); 
            unsigned int G_new;

            for (const auto &i: direction)
            {
                Vec3i newCoordinates(_s_aux->coordinates + i);
                Node *successor2 = discrete_world_.getNodePtr(newCoordinates);

                if (successor2 == nullptr || successor2->occuppied ) continue;

                if ( successor2->isInClosedList ) 
                {
                    G_new = successor2->G + geometry::distanceBetween2Nodes(successor2, _s_aux);
                    if (G_new < G_max)
                    {
                        G_max = G_new;
                        _s_aux->parent = successor2;
                        _s_aux->G = G_new;
                        _s_aux->gplush = _s_aux->G + _s_aux->H;
                    }
                }
            }
        }
    }
    inline void LazyThetaStar::ComputeCost(Node *_s_aux, Node *_s2_aux, torch::jit::script::Module& loaded_sdf)
    {
        auto distanceParent2 = geometry::distanceBetween2Nodes(_s_aux->parent, _s2_aux);

        if ( (_s_aux->parent->G + distanceParent2) < _s2_aux->G )
        {
            _s2_aux->parent = _s_aux->parent;
            _s2_aux->G      = _s2_aux->parent->G + distanceParent2;
            _s2_aux->gplush = _s2_aux->G + _s2_aux->H;
        }
    }

    PathData LazyThetaStar::findPath(const Vec3i &_source, const Vec3i &_target, torch::jit::script::Module& loaded_sdf)
    {
        Node *current = nullptr;

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
        while (!indexByCost.empty())
        {

            auto it = indexByCost.begin();
            current = *it;
            indexByCost.erase(indexByCost.begin());
        
            if (current->coordinates == _target)
            {
                solved = true;
                break;
            }
            closedSet_.push_back(current);

            current->isInOpenList = false;
            current->isInClosedList = true;

            SetVertex(current, loaded_sdf);
#if defined(ROS) && defined(PUB_EXPLORED_NODES)
            publishROSDebugData(current, indexByCost, closedSet_);
#endif

            exploreNeighbours(current, _target, indexByWorldPosition, loaded_sdf);

        }
        main_timer.toc();
    
        PathData result_data = createResultDataObject(current, main_timer, closedSet_.size(), 
                                                  solved, _source, line_of_sight_checks_);
   
#if defined(ROS) && defined(PUB_EXPLORED_NODES)
        explored_node_marker_.points.clear();
#endif
        closedSet_.clear();
        delete discrete_world_.getNodePtr(_source)->parent;

        discrete_world_.resetWorld();
        return result_data;
    }

}
