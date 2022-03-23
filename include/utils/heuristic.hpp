#ifndef HEURISTIC_HPP
#define HEURISTIC_HPP
/**
 * @file heuristic.hpp
 * @author Rafael Rey (reyarcenegui@gmail.com)
* @author Jose Antonio Cobano (jacobsua@upo.es)
 * @brief Heuristic functions as static members of Heuritic class
 * to easily change between heuristics inside the algorithms
 * @version 0.1
 * @date 2021-06-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "utils/utils.hpp"


namespace Planners
{
    using namespace utils;

    /**
     * @brief Heuristic class, collection of static functions
     * The heuristic is computed as a discrete value because the discrete coordinates are used
     * All these discrete values correspond to std::floor(dist_scale_factor_100 * real_distance). sqrt(2) in this discrete approach
     * correspond to 141 (1,41 * dist_scale_factor_=100).
     */
    class Heuristic
    {   
        /**
         * @brief Returns the absolute of value of the difference of each component 
         * 
         * @param _source 
         * @param _target 
         * @return Vec3i 
         */
        static Vec3i getDelta(const Vec3i &_source, const Vec3i &_target);

    public:
        /**
         * @brief Manhattan heuristic
         * 
         * @param _source 
         * @param _target 
         * @return unsigned int 
         */
        static unsigned int manhattan(const Vec3i &_source, const Vec3i &_target);
        /**
         * @brief Euclidean heuristic. This is the one used by the standard algorithms
         * 
         * @param _source 
         * @param _target 
         * @return unsigned int 
         */
        static unsigned int euclidean(const Vec3i &_source, const Vec3i &_target);

        /**
         * @brief 
         * 
         * @param _source 
         * @param _target 
         * @return unsigned int 
         */
        static unsigned int euclideanOptimized(const Vec3i &_source, const Vec3i &_target);

        /**
         * @brief 
         * 
         * @param _factor 
         * @param _source 
         * @param _target 
         * @return unsigned int 
         */
        static unsigned int euclideanAttractive(const float _factor, const Vec3i &_source, const Vec3i &_target);

        /**
         * @brief  Octogonal heuristic
         * 
         * @param _source 
         * @param _target 
         * @return unsigned int 
         */
        static unsigned int octagonal(const Vec3i &_source, const Vec3i &_target);
        /**
         * @brief Dijkstra heuristic. (Absence of heuristic, always returns 0)
         * 
         * @return unsigned int 
         */
        static unsigned int dijkstra(const Vec3i &_source, const Vec3i &_target);

    };

}

#endif