#include "Planners/PathGenerator.hpp"

namespace Planners
{

    PathGenerator::PathGenerator(bool _use_3d = true,  std::string _algorithm_name = "generic_3d_algorithm"): algorithm_name_(_algorithm_name){
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
    void PathGenerator::setWorldSize(const Vec3i &_worldSize,const double _resolution)
    {
        discrete_world_.resizeWorld(_worldSize, _resolution);
    }
    Vec3i PathGenerator::getWorldSize(){
        return discrete_world_.getWorldSize();
    }
    double PathGenerator::getWorldResolution(){
        return discrete_world_.getResolution();
    }
    utils::DiscreteWorld* PathGenerator::getInnerWorld(){
        return &discrete_world_;
    }

    void PathGenerator::setHeuristic(HeuristicFunction heuristic_)
    {
        heuristic = std::bind(heuristic_, std::placeholders::_1, std::placeholders::_2);
    }
    bool PathGenerator::configureCellCost(const Vec3i &coordinates_, const unsigned int &_cost){

        return discrete_world_.setNodeCost(coordinates_, _cost);
    }
    void PathGenerator::addCollision(const Vec3i &coordinates_, bool do_inflate, unsigned int steps)
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
    void PathGenerator::addCollision(const Vec3i &coordinates_)
    {
        addCollision(coordinates_, do_inflate_, inflate_steps_);
    }
    bool PathGenerator::detectCollision(const Vec3i &coordinates_)
    {
        if (discrete_world_.isOccupied(coordinates_))
        {
            return true;
        }
        return false;
    }
    void PathGenerator::inflateNodeAsCube(const Vec3i &_ref, const CoordinateList &_directions, const unsigned int &_inflate_steps)
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
    PathData PathGenerator::createResultDataObject(const Node* _last, utils::Clock &_timer, 
                                                    const size_t _explored_nodes, bool _solved,
                                                    const Vec3i &_start, const unsigned int _sight_checks){
                                    
        PathData result_data;

        result_data["solved"] = _solved;
        result_data["goal_coords"] = _last->coordinates;

        CoordinateList path;
        unsigned int total_cost{0};
        unsigned int total_H{0};
        unsigned int total_G{0};
        unsigned int total_C{0};

        if(_solved){
            while (_last != nullptr) {
                unsigned int g_real = _last->G - _last->C;
                total_H += _last->H;
                total_G += g_real;
                total_C +=  _last->C;

                total_cost += g_real + _last->H + _last->C;
                path.push_back(_last->coordinates);
                _last = _last->parent;
            }
        }else{
            std::cout << "Error impossible to calcualte a solution" << std::endl;
        }
        result_data["algorithm"]               = algorithm_name_;
        result_data["path"]                    = path;
        result_data["time_spent"]              = _timer.getElapsedMillisecs();
        result_data["explored_nodes"]          = _explored_nodes;
        result_data["start_coords"]            = _start;
        result_data["path_length"]             = geometry::calculatePathLength(path, discrete_world_.getResolution());

        result_data["total_cost"]              = total_cost;
        result_data["h_cost"]                  = total_H;
        result_data["g_cost"]                  = total_G;
        result_data["c_cost"]                  = total_C;

        result_data["line_of_sight_checks"]    = _sight_checks;
        result_data["max_line_of_sight_cells"] = max_line_of_sight_cells_;
        result_data["cost_weight"]             = cost_weight_;
    
        return result_data;
    }

}
