#include "Planners/AlgorithmBase.hpp"

namespace Planners
{

    AlgorithmBase::AlgorithmBase(bool _use_3d = true, const std::string &_algorithm_name = "generic_3d_algorithm"): algorithm_name_(_algorithm_name){
        setHeuristic(&Heuristic::euclidean);
        CoordinateList directions2d, directions3d;
        directions2d = {
            { 0, 1, 0 }, {0, -1, 0}, { 1, 0, 0 }, { -1, 0, 0 }, //4 straight elements
            { 1, -1, 0 }, { -1, 1, 0 }, { -1, -1, 0 }, { 1, 1, 0 } //4 diagonal elements
        };
        directions3d = {

            { 0, 1, 0 }, {0, -1, 0}, { 1, 0, 0 }, { -1, 0, 0 }, { 0, 0, 1}, { 0, 0, -1}, //6 first elements

            { 1, -1, 0 }, { -1, 1, 0 }, { -1, -1, 0 }, { 1, 1, 0 },  { -1, 0, -1 }, //7-18 inclusive
            { 1, 0, 1 }, { 1, 0, -1 }, {-1, 0, 1}, { 0, -1, 1 }, { 0, 1, 1 }, { 0, 1, -1 },  { 0, -1, -1 }, 

            { -1, -1, 1 }, { 1, 1, 1 },  { -1, 1, 1 }, { 1, -1, 1 }, { -1, -1, -1 }, { 1, 1, -1 }, { -1, 1, -1 }, { 1, -1, -1 }, 
        };
        if(_use_3d){
            std::cout << "[Algorithm] Using 3D Directions" << std::endl;
            direction = directions3d;
        }else{
            std::cout << "[Algorithm] Using 2D Directions" << std::endl;
            direction = directions2d;
        }
    }
    void AlgorithmBase::setWorldSize(const Vec3i &_worldSize,const double _resolution)
    {
        discrete_world_.resizeWorld(_worldSize, _resolution);
    }

    // // JAC QUITAR setLocalWorldSize?
    void AlgorithmBase::setLocalWorldSize(const Vec3i &_worldSize,const double _resolution)
    {
        // el resizeLocalWorld lo hace bien (sept-2024)
        discrete_world_.resizeLocalWorld(_worldSize, _resolution); // Hay un error y parece que proviene del getWorldIndex en el world.hpp porque toma los valores del world_x_size y x_y_size en lugar de los "local"
    }

    // JAC
    void AlgorithmBase::cleanLocalWorld()
    {
        discrete_world_.cleanWorld();
    }    
    Vec3i AlgorithmBase::getWorldSize(){
        return discrete_world_.getWorldSize();
    }
    double AlgorithmBase::getWorldResolution(){
        return discrete_world_.getResolution();
    }
    utils::DiscreteWorld* AlgorithmBase::getInnerWorld(){
        return &discrete_world_;
    }

    void AlgorithmBase::setHeuristic(HeuristicFunction heuristic_)
    {
        heuristic = std::bind(heuristic_, std::placeholders::_1, std::placeholders::_2);
    }
    bool AlgorithmBase::configureCellCost(const Vec3i &coordinates_, const double &_cost){

        return discrete_world_.setNodeCost(coordinates_, _cost);
    }
    void AlgorithmBase::addCollision(const Vec3i &coordinates_, bool do_inflate, unsigned int steps)
    {
        if (do_inflate)
        {
            inflateNodeAsCube(coordinates_, direction, steps);
        }
        else
        {
            discrete_world_.setOccupied(coordinates_);
        }
    }
    void AlgorithmBase::addCollision(const Vec3i &coordinates_)
    {
        addCollision(coordinates_, do_inflate_, inflate_steps_);
    }
    bool AlgorithmBase::detectCollision(const Vec3i &coordinates_)
    {
        if (discrete_world_.isOccupied(coordinates_))
        {
            return true;
        }
        return false;
    }
    void AlgorithmBase::inflateNodeAsCube(const Vec3i &_ref, const CoordinateList &_directions, const unsigned int &_inflate_steps)
    {
        for (const auto &it : _directions)
        {
            for (unsigned int i = 0; i < _inflate_steps; ++i)
            {
                auto new_vec = _ref + (i + 1) * it;
                discrete_world_.setOccupied(new_vec);
            }
        }
    }

    PathData AlgorithmBase::createResultDataObject(const Node* _last, utils::Clock &_timer, 
                                                    const size_t _explored_nodes, bool _solved,
                                                    const Vec3i &_start, const unsigned int _sight_checks){
                                    
        PathData result_data;

        result_data["solved"]       = _solved;
        result_data["goal_coords"]  = _last->coordinates;
        result_data["g_final_node"] = _last->G;

        CoordinateList path;
        
        if(_solved){
            while (_last != nullptr) {
                
                path.push_back(_last->coordinates);
                _last = _last->parent;
            }
        }else{
            std::cout << "Error impossible to calculate a solution" << std::endl;
        }
        
        unsigned int total_cost1{0};
        unsigned int total_cost2{0};
        
        unsigned int total_H{0};
        unsigned int total_G1{0};
        unsigned int total_G2{0};
        unsigned int total_C{0};

        unsigned int total_grid_cost1{0};
        unsigned int total_grid_cost2{0};

#ifdef COMPUTE_STATISTICS
        
        auto adjacent_path = utils::geometry::getAdjacentPath(path, discrete_world_);
        
        for(size_t i = 0; i < adjacent_path.size() - 1; ++i){
            auto node_current = discrete_world_.getNodePtr(adjacent_path[i]);
            auto node         = discrete_world_.getNodePtr(adjacent_path[i+1]);
            if( node == nullptr )
                continue;
            
            total_H         += node->H;
            total_C         += node->C;

            total_grid_cost1 += static_cast<unsigned int>( cost_weight_ * node_current->cost * dist_scale_factor_reduced_ );  // CAA*+M1
            total_grid_cost2 += static_cast<unsigned int>( ( ( node->cost + node_current->cost ) / 2) *  cost_weight_ * dist_scale_factor_reduced_); ; //CAA*+M2         

            unsigned int g_real1 = utils::geometry::distanceBetween2Nodes(adjacent_path[i], adjacent_path[i+1]); 
            unsigned int g_real2 = utils::geometry::distanceBetween2Nodes(adjacent_path[i], adjacent_path[i+1]);  // Conmensurable

            total_G1 += g_real1; 
            total_G2 += g_real2;            
        }
        
        total_cost1 = total_G1 + total_grid_cost1; 
        total_cost2 = total_G2 + total_grid_cost2; 
#endif
        result_data["algorithm"]               = algorithm_name_;
        result_data["path"]                    = path;
        result_data["time_spent"]              = _timer.getElapsedMicroSeconds();
        result_data["explored_nodes"]          = _explored_nodes;
        result_data["start_coords"]            = _start;
        result_data["path_length"]             = geometry::calculatePathLength(path, discrete_world_.getResolution());

        result_data["total_cost1"]              = total_cost1;
        result_data["total_cost2"]              = total_cost2;
        result_data["h_cost"]                   = total_H;
        result_data["g_cost1"]                  = total_G1;
        result_data["g_cost2"]                  = total_G2;
        result_data["c_cost"]                   = total_C;
        result_data["grid_cost1"]               = total_grid_cost1;
        result_data["grid_cost2"]               = total_grid_cost2;

        result_data["line_of_sight_checks"]    = _sight_checks;
        result_data["max_line_of_sight_cells"] = max_line_of_sight_cells_;
        result_data["cost_weight"]             = cost_weight_;
    
        return result_data;
    }

}
