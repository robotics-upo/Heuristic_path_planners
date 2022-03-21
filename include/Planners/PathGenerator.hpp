#ifndef PATHGENERATOR_HPP
#define PATHGENERATOR_HPP
/**
 * @file PathGenerator.hpp
 * @author Rafael Rey (reyarcenegui@gmail.com)
 * @author Jose Antonio Cobano (jacobsua@upo.es)
 * 
 * @brief Generic PathGenerator Base Class. The algorithms should inherit from this class as far as possible.
 * It implements some generic functions used by all the algorithms
 * It could be extended to add functionalities required by other type of algorithms, not only heuristics ones
 * 
 * @version 0.1
 * @date 2021-06-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <iostream>
#include <vector>
#include <set>
#include <functional>

#include <math.h>

#include "utils/world.hpp"
#include "utils/heuristic.hpp"
#include "utils/utils.hpp"
#include "utils/time.hpp"
#include "utils/geometry_utils.hpp"
#include "utils/LineOfSight.hpp"

namespace Planners
{
    using namespace utils;
    using HeuristicFunction = std::function<unsigned int(Vec3i, Vec3i)>;

    class Heuristic;
    class Clock;

    /**
     * @brief Main base class that implements useful functions for children algorithm class 
     * and provides a guide to implement any new algorithm.
     */
    class PathGenerator
    {

    public:
        /**
         * @brief Construct a new Path Generator object
         * 
         * @param _use_3d: This params allows the algorithm to choose a set of 3D directions of explorations 
         * or a set or 2D directions of explorations. The 2D case is simply 3D but without the directions with Z!=0
         * directions2d = {
         *  { 0, 1, 0 }, {0, -1, 0}, { 1, 0, 0 }, { -1, 0, 0 }, //4 straight elements
         *  { 1, -1, 0 }, { -1, 1, 0 }, { -1, -1, 0 }, { 1, 1, 0 } //4 diagonal elements
         * };
         * directions3d = {
         *
         *  { 0, 1, 0 }, {0, -1, 0}, { 1, 0, 0 }, { -1, 0, 0 }, { 0, 0, 1}, { 0, 0, -1}, //6 first elements
         *
         *  { 1, -1, 0 }, { -1, 1, 0 }, { -1, -1, 0 }, { 1, 1, 0 },  { -1, 0, -1 }, //7-18 inclusive
         *  { 1, 0, 1 }, { 1, 0, -1 }, {-1, 0, 1}, { 0, -1, 1 }, { 0, 1, 1 }, { 0, 1, -1 },  { 0, -1, -1 }, 
         *
         *  { -1, -1, 1 }, { 1, 1, 1 },  { -1, 1, 1 }, { 1, -1, 1 }, { -1, -1, -1 }, { 1, 1, -1 }, { -1, 1, -1 }, { 1, -1, -1 }, 
         *};
         * Note that the the ordering is not trivial, the inner loop that explorates node take advantage of this order to directly use
         * pre-compiled distance depending if the squared norm of the vector is 1,2 or 3.
         * This pre compiled distances appears in the header utils.hpp
         * 
         * 
         * @param _algorithm_name Algorithm name to uniquely identify the type of algorithm. 
         */
        PathGenerator(bool _use_3d, std::string _algorithm_name);

        /**
         * @brief Set the World Size object. This method call the resizeWorld method 
         * from the internal discrete world object
         * 
         * @param worldSize_ Discrete world size vector in units of resolution.
         * @param _resolution resolution to save inside the world object
         */
        void setWorldSize(const Vec3i &worldSize_,const double _resolution);

        /**
         * @brief Get the World Size, it simply call the getWorldSize method from the
         * discrete world internal object
         * 
         * @return Vec3i discrete world bounds
         */
        Vec3i getWorldSize();
        /**
         * @brief Get the World Resolution that is been used by the internal 
         * discrete world object
         * @return double resolution 
         */
        double getWorldResolution();

        /**
         * @brief Get a pointer to the inner world object 
         * 
         * @return utils::DiscreteWorld* 
         */
        utils::DiscreteWorld* getInnerWorld();

        /**
         * @brief Configure the heuristic from the list of static functions in the heuristic.hpp
         * header
         * @param heuristic_ Should be one of the static functions of the Heuristic Class
         * for example Heuristic::Euclidean
         */
        void setHeuristic(HeuristicFunction heuristic_);
        
        /**
         * @brief Mark a set of coordinates of the map as occupied (blocked)
         * 
         * @param coordinates_ Discrete vector of coordinates
         * @param do_inflate enable inflation (mark surrounding coordinates as occupied)
         * @param steps inflation steps (in multiples of the resolution value)
         */
        void addCollision(const Vec3i &coordinates_, bool do_inflate, unsigned int steps);
        
        /**
         * @brief Calls the addCollision with the internal inflation configuration values
         * 
         * @param coordinates_ Discrete coordinates vector
         */
        void addCollision(const Vec3i &coordinates_);

        /**
         * @brief Function to use in the future to configure the cost of each node
         * 
         * @param coordinates_ Discrete coordinates
         * @param _cost cost value 
         * @return true 
         * @return false 
         */
        bool configureCellCost(const Vec3i &coordinates_, const double &_cost);

        /**
         * @brief Check if a set of discrete coordinates are marked as occupied
         * 
         * @param coordinates_ Discrete vector of coordinates
         * @return true Occupied
         * @return false not occupied
         */
        bool detectCollision(const Vec3i &coordinates_);

        /**
         * @brief Main function that should be inherit by each algorithm. 
         * This function should accept two VALID start and goal discrete coordinates and return
         * a PathData object containing the necessary information (path, time....)
         * 
         * @param _source Start discrete coordinates
         * @param _target Goal discrete coordinates
         * @return PathData Results stored as PathData object
         */
        virtual PathData findPath(const Vec3i &_source, const Vec3i &_target) = 0;

        /**
         * @brief Configure the simple inflation implementation
         * By default the inflation way is "As a Cube"
         * @param _inflate If inflate by default
         * @param _inflation_steps The number of adjacent cells to inflate
         */
        void setInflationConfig(const bool _inflate, const unsigned int _inflation_steps) 
        { do_inflate_ = _inflate; inflate_steps_ = _inflation_steps;}

        /**
         * @brief Set the Cost Factor object
         * 
         */
        virtual void setCostFactor(const float &_factor){ cost_weight_ = _factor; }

        /**
         * @brief Set the Max Line Of Sight object
         * 
         * @param _max_line_of_sight 
         */
        virtual void setMaxLineOfSight(const float &_max_line_of_sight){ max_line_of_sight_cells_ = std::floor(_max_line_of_sight/discrete_world_.getResolution()); }
        /**
         * @brief Deleted function to be inherit from
         * 
         */
        virtual void publishOccupationMarkersMap() = 0;

    protected:

        /**
         * @brief Basic inflation function 
         * 
         * @param _ref Discrete coordinates vector
         * @param _directions Directions vector to inflate
         * @param _inflate_steps number of cells to inflate in each direction
         */
        void inflateNodeAsCube(const Vec3i &_ref,
                               const CoordinateList &_directions,
                               const unsigned int &_inflate_steps);
        
        /**
         * @brief Create a Result Data Object object
         * 
         * @param _last 
         * @param _timer 
         * @param _explored_nodes 
         * @param _solved 
         * @param _start 
         * @param _sight_checks 
         * @return PathData 
         */
        virtual PathData createResultDataObject(const Node* _last, utils::Clock &_timer, 
                                                const size_t _explored_nodes, bool _solved, 
                                                const Vec3i &_start, const unsigned int _sight_checks);

                                                        
        HeuristicFunction heuristic; /*!< TODO Comment */
        CoordinateList direction; /*!< TODO Comment */

        utils::DiscreteWorld discrete_world_; /*!< TODO Comment */
        unsigned int inflate_steps_{1}; /*!< TODO Comment */
        bool do_inflate_{true}; /*!< TODO Comment */

        double cost_weight_{0}; /*!< TODO Comment */
        unsigned int max_line_of_sight_cells_{0}; /*!< TODO Comment */

        const std::string algorithm_name_{""}; /*!< TODO Comment */

    private:
    };
}
#endif