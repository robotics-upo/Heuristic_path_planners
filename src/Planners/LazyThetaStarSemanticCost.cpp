#include "Planners/LazyThetaStarSemanticCost.hpp"

namespace Planners
{
    LazyThetaStarSemanticCost::LazyThetaStarSemanticCost(bool _use_3d):ThetaStarSemanticCost(_use_3d, "lazythetastar_semantic_cost") {}
    // LazyThetaStarSemanticCost::LazyThetaStarSemanticCost(bool _use_3d):ThetaStarSemantic(_use_3d, "lazythetastar_semantic_cost") {}
    LazyThetaStarSemanticCost::LazyThetaStarSemanticCost(bool _use_3d, std::string _name = "lazythetastar_semantic_cost" ):ThetaStarSemanticCost(_use_3d, _name) {}
    // LazyThetaStarSemanticCost::LazyThetaStarSemanticCost(bool _use_3d, std::string _name = "lazythetastar_semantic_cost" ):ThetaStarSemantic(_use_3d, _name) {}
    
    void LazyThetaStarSemanticCost::SetVertex(Node *_s_aux)
    {

        // line_of_sight_checks_++;
        // if (!LineOfSight::bresenham3DWithMaxThreshold(_s_aux->parent, _s_aux, discrete_world_, max_line_of_sight_cells_ ))
        // {
        //     unsigned int G_max = std::numeric_limits<unsigned int>::max(); 
        //     unsigned int G_new;

        if( !los_neighbour_ ){

            unsigned int G_max = std::numeric_limits<unsigned int>::max(); 
            unsigned int G_new;

            for (const auto &i: direction)
            {
                Vec3i newCoordinates(_s_aux->coordinates + i);
                Node *successor2 = discrete_world_.getNodePtr(newCoordinates);
                if (successor2 == nullptr || successor2->occuppied ) continue;

                if ( successor2->isInClosedList ) 
                {
                    // Semantic cost of the succesor

                    unsigned int cost_semantic = 0;

                    float c_wall, c_door, c_colum, c_furnish, c_stair, c_panel, c_lamp, c_glass;
                    float coste;
                    // float coef,c_wall, c_door, c_colum, c_furnish, c_stair, c_panel, c_lamp, c_glass;
                    // coef=1;
                    c_wall=coef;
                    c_door=3*coef;
                    c_colum=2.5*coef;
                    c_furnish=1*coef;
                    c_stair=1.5*coef;
                    c_panel=1.5*coef;
                    c_lamp=2*coef;
                    c_glass=2*coef;

                    // coste=1+static_cast<double>(successor2->cost);
                    coste=static_cast<double>(successor2->cost);

                    auto distance = geometry::distanceBetween2Nodes(successor2, _s_aux);

                    // Wall
                    if (successor2->semantic == 1){
                        // cost_semantic=(c_wall/(1+static_cast<double>(successor2->cost)))*distance; // + distance;
                        cost_semantic=(c_wall/coste)*distance;
                        // std::cout << "cost: " << _suc->cost << std::endl;
                        // std::cout << "semantic: " << _suc->semantic << std::endl;
                        // usleep(1e4);
	                    // std::cout << "Please a key to go to the next iteration..." << std::endl;
	                    // getchar(); // Comentar para no usar tecla.
                    }
                    // Door
                    else if (successor2->semantic == 2){
                        // cost_semantic=(c_door/(1+static_cast<double>(successor2->cost)))*distance; // + distance;
                        cost_semantic=(c_door/coste)*distance;
                    }

                    // Colum
                    else if (successor2->semantic == 3){
                        // cost_semantic=(c_colum/(1+static_cast<double>(successor2->cost)))*distance; // + distance;
                        cost_semantic=(c_colum/coste)*distance;
                    }
                    // Furnishing
                    else if (successor2->semantic == 4){
                        // cost_semantic=(c_furnish/(1+static_cast<double>(successor2->cost)))*distance; // + distance;
                        cost_semantic=(c_furnish/coste)*distance;
                    }
                    // Stairs
                    else if (successor2->semantic == 5){
                        // cost_semantic=(c_stair/(1+static_cast<double>(successor2->cost)))*distance; // + distance;
                        cost_semantic=(c_stair/coste)*distance;
                    }
                    // Panels --> Barandilla
                    else if (successor2->semantic == 6){
                        // cost_semantic=(c_panel/(1+static_cast<double>(successor2->cost)))*distance; // + distance;
                        cost_semantic=(c_panel/coste)*distance;
                    }
                    // Lamp
                    else if (successor2->semantic == 7){
                        // cost_semantic=(c_lamp/(1+static_cast<double>(successor2->cost)))*distance; // + distance;
                        cost_semantic=(c_lamp/coste)*distance;
                    }
                    // Glass wall
                    else if (successor2->semantic == 8){
                        // cost_semantic=(c_glass/(1+static_cast<double>(successor2->cost)))*distance; // + distance;
                        cost_semantic=(c_glass/coste)*distance;
                    }
                    else 
                        cost_semantic=distance;

                    G_new = successor2->G + cost_semantic;
                    if (G_new < G_max)
                    {
                        G_max = G_new;
                        _s_aux->parent = successor2;
                        _s_aux->G      = G_new;
                        _s_aux->C      = cost_semantic;
                        _s_aux->gplush = _s_aux->G + _s_aux->H;
                    }
                }
            }
        }
        los_neighbour_ = false;
    }

    inline void LazyThetaStarSemanticCost::ComputeCost(Node *_s_aux, Node *_s2_aux)
    {
        // // WITHOUT CONSIDERING A MAXIMUM LINE OF SIGHT
        // line_of_sight_checks_++;
        // if (LineOfSight::bresenham3D(_s_aux->parent, _s2_aux, discrete_world_, checked_nodes)) {
            
        //     los_neighbour_ = true;

        //     auto dist2   = geometry::distanceBetween2Nodes(_s_aux->parent, _s2_aux);
        //     auto edge2   = ComputeSemanticEdgeCost(checked_nodes, _s_aux->parent, _s2_aux);
        //     if (edge2 == 0){
        //         edge2 = 1;
        //     }

        //     if ( ( _s_aux->parent->G + (dist2 * edge2) ) < _s2_aux->G ) 
        //     {
        //         _s2_aux->parent = _s_aux->parent;
        //         _s2_aux->G      = _s2_aux->parent->G + (dist2 * edge2);
        //         _s2_aux->C      = dist2 * edge2;
        //         _s2_aux->gplush = _s2_aux->G + _s2_aux->H;
        //     }            
        // } 
        // checked_nodes->clear();

        // CONSIDERIN A MAXIMUM LINE OF SIGHT --> CORRECT
        line_of_sight_checks_++;
        auto distanceParent2_nodes = LineOfSight::nodesInLineBetweenTwoNodes(_s_aux->parent, _s2_aux, discrete_world_, max_line_of_sight_cells_);  //REVISAR _s_aux->parent o _s_aux
        // std::cout << "distanceParent2_nodes: " << distanceParent2_nodes << std::endl;

        // No line of sight or distance greater than max_line_of_sight_cells
        if ( distanceParent2_nodes == 0 ){
            distanceParent2_nodes = 1;
        }
        // Line of sight
        else{
            if (LineOfSight::bresenham3D(_s_aux->parent, _s2_aux, discrete_world_, checked_nodes)) {
            
                los_neighbour_ = true;

                auto dist2   = geometry::distanceBetween2Nodes(_s_aux->parent, _s2_aux);
                auto edge2   = ComputeSemanticEdgeCost(checked_nodes, _s_aux->parent, _s2_aux);
                if (edge2 == 0){
                    edge2 = 1;
                }

                if ( ( _s_aux->parent->G + (dist2 * edge2) ) < _s2_aux->G ) 
                {
                    _s2_aux->parent = _s_aux->parent;
                    _s2_aux->G      = _s2_aux->parent->G + (dist2 * edge2);
                    _s2_aux->C      = dist2 * edge2;
                    _s2_aux->gplush = _s2_aux->G + _s2_aux->H;
                }            
            } 
        }
        checked_nodes->clear();
    }

    inline unsigned int LazyThetaStarSemanticCost::computeG(const Node* _current, Node* _suc,  unsigned int _n_i, unsigned int _dirs){

        // unsigned int cost = 0;
        unsigned int cost = _current->G;
        unsigned int cost2 = 0;

        float c_wall, c_door, c_colum, c_furnish, c_stair, c_panel, c_lamp, c_glass;
        // float coef,c_wall, c_door, c_colum, c_furnish, c_stair, c_panel, c_lamp, c_glass;
        // coef=1;
        // std::cout << "coef: " << coef << std::endl;
        // usleep(1e4);
	    // std::cout << "Please a key to go to the next iteration..." << std::endl;
	    // getchar(); // Comentar para no usar tecla.

        c_wall=coef;
        c_door=3*coef;
        c_colum=2.5*coef;
        c_furnish=1*coef;
        c_stair=1.5*coef;
        c_panel=1.5*coef;
        c_lamp=2*coef;
        c_glass=2*coef;

        if(_dirs == 8){
            cost2 = (_n_i < 4 ? dist_scale_factor_ : dd_2D_); //This is more efficient
        }else{
            cost2 = (_n_i < 6 ? dist_scale_factor_ : (_n_i < 18 ? dd_2D_ : dd_3D_)); //This is more efficient
        }

        float coste;
        // coste=1+static_cast<double>(_suc->cost);
        coste=static_cast<double>(_suc->cost);

        // double cc = ( _current->cost + _suc->cost ) / 2;
        // auto edge_neighbour = static_cast<unsigned int>( cc *  cost_weight_ * dist_scale_factor_reduced_); 
    
        // cost += ( _current->G + edge_neighbour );

        // _suc->C = edge_neighbour;
        
        // SEMANTIC COST
        // std::cout << "c_colum: " << c_colum << std::endl;
        // std::cout << "cost: " << static_cast<double>(_suc->cost) << std::endl;
        // std::cout << "divisin: " << (c_colum/static_cast<double>(_suc->cost)) << std::endl;
        // usleep(1e4);
	    // std::cout << "Please a key to go to the next iteration..." << std::endl;
	    // getchar(); // Comentar para no usar tecla.
        // Wall
        if (_suc->semantic == 1){
            // cost2=(c_wall/(1+static_cast<double>(_suc->cost)))*cost2; // + cost2;
            cost2=(c_wall/coste)*cost2;

            cost2=(c_wall/static_cast<double>(_suc->cost))*cost2 + cost2;
            // std::cout << "cost: " << _suc->cost << std::endl;
            // std::cout << "semantic: " << _suc->semantic << std::endl;
            // usleep(1e4);
	        // std::cout << "Please a key to go to the next iteration..." << std::endl;
	        // getchar(); // Comentar para no usar tecla.
        }
        // Door
        else if (_suc->semantic == 2){
            // cost2=(c_door/(1+static_cast<double>(_suc->cost)))*cost2; // + cost2;
            cost2=(c_door/coste)*cost2;

            // std::cout << "cost: " << _suc->cost << std::endl;
            // std::cout << "semantic: " << _suc->semantic << std::endl;
            // usleep(1e4);
	        // std::cout << "Please a key to go to the next iteration..." << std::endl;
	        // getchar(); // Comentar para no usar tecla.
        }

        // Colum
        else if (_suc->semantic == 3){
            // cost2=(c_colum/(1+static_cast<double>(_suc->cost)))*cost2; // + cost2;
            cost2=(c_colum/coste)*cost2;

            // std::cout << "cost: " << _suc->cost << std::endl;
            // std::cout << "semantic: " << _suc->semantic << std::endl;
            // usleep(1e4);
	        // std::cout << "Please a key to go to the next iteration..." << std::endl;
	        // getchar(); // Comentar para no usar tecla.
        }
        // Furnishing
        else if (_suc->semantic == 4){
            // cost2=(c_furnish/(1+static_cast<double>(_suc->cost)))*cost2; // + cost2;
            cost2=(c_furnish/coste)*cost2;

            // std::cout << "cost: " << _suc->cost << std::endl;
            // std::cout << "semantic: " << _suc->semantic << std::endl;
            // usleep(1e4);
	        // std::cout << "Please a key to go to the next iteration..." << std::endl;
	        // getchar(); // Comentar para no usar tecla.
        }
        // Stairs
        else if (_suc->semantic == 5){
            // cost2=(c_stair/(1+static_cast<double>(_suc->cost)))*cost2; // + cost2;
            cost2=(c_stair/coste)*cost2;

            // std::cout << "cost: " << _suc->cost << std::endl;
            // std::cout << "semantic: " << _suc->semantic << std::endl;
            // usleep(1e4);
	        // std::cout << "Please a key to go to the next iteration..." << std::endl;
	        // getchar(); // Comentar para no usar tecla.
        }
        // Panels --> Barandilla
        else if (_suc->semantic == 6){
            // cost2=(c_panel/(1+static_cast<double>(_suc->cost)))*cost2; // + cost2;
            cost2=(c_panel/coste)*cost2;

            // std::cout << "cost: " << _suc->cost << std::endl;
            // std::cout << "semantic: " << _suc->semantic << std::endl;
            // usleep(1e4);
	        // std::cout << "Please a key to go to the next iteration..." << std::endl;
	        // getchar(); // Comentar para no usar tecla.
        }
        // Lamp
        else if (_suc->semantic == 7){
            // cost2=(c_lamp/(1+static_cast<double>(_suc->cost)))*cost2; // + cost2;
            cost2=(c_lamp/coste)*cost2;

            // std::cout << "cost: " << _suc->cost << std::endl;
            // std::cout << "semantic: " << _suc->semantic << std::endl;
            // usleep(1e4);
	        // std::cout << "Please a key to go to the next iteration..." << std::endl;
	        // getchar(); // Comentar para no usar tecla.std::cout << "cost: " << _suc->cost << std::endl;
            // std::cout << "semantic: " << _suc->semantic << std::endl;
            // usleep(1e4);
	        // std::cout << "Please a key to go to the next iteration..." << std::endl;
	        // getchar(); // Comentar para no usar tecla.
        }
        // Glass wall
        else if (_suc->semantic == 8){
            // cost2=(c_glass/(1+static_cast<double>(_suc->cost)))*cost2; // + cost2;
            cost2=(c_glass/coste)*cost2;

            // std::cout << "cost: " << _suc->cost << std::endl;
            // std::cout << "semantic: " << _suc->semantic << std::endl;
            // usleep(1e4);
	        // std::cout << "Please a key to go to the next iteration..." << std::endl;
	        // getchar(); // Comentar para no usar tecla.
        }

        cost = cost + cost2;

        _suc->C = _suc->cost;

        return cost;
    }

    PathData LazyThetaStarSemanticCost::findPath(const Vec3i &_source, const Vec3i &_target)
    {
        utils::Clock main_timer;
        main_timer.tic();

        MagicalMultiSet openSet;

        Node *current = nullptr;
        line_of_sight_checks_ = 0;

        bool solved{false};

        discrete_world_.getNodePtr(_source)->parent = new Node(_source);
        discrete_world_.setOpenValue(_source, true);

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

            SetVertex(current);
#if defined(ROS) && defined(PUB_EXPLORED_NODES)
            publishROSDebugData(current, indexByCost, closedSet_);
#endif
            exploreNeighbours(current, _target, indexByWorldPosition);
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
