#ifndef HEURISTIC_HPP
#define HEURISTIC_HPP
/**
 * @file heuristic.hpp
 * @author Rafael Rey (rreyarc@upo.es)
 * @brief Heuristic functions as static members of Heuritic class
 * to easily change between heuristics inside the algorithms
 * @version 0.1
 * @date 2021-06-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "utils/utils.hpp"

using namespace Planners::utils;

namespace Planners
{
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
         * @param source_ 
         * @param target_ 
         * @return Vec3i 
         */
        static Vec3i getDelta(Vec3i source_, Vec3i target_);

    public:
        /**
         * @brief Manhattan heuristic
         * 
         * @param source_ 
         * @param target_ 
         * @return unsigned int 
         */
        static unsigned int manhattan(Vec3i source_, Vec3i target_);
        /**
         * @brief Euclidean heuristic. This is the one used by the standard algorithms
         * 
         * @param source_ 
         * @param target_ 
         * @return unsigned int 
         */
        static unsigned int euclidean(Vec3i source_, Vec3i target_);
        /**
         * @brief  Octogonal heuristic
         * 
         * @param source_ 
         * @param target_ 
         * @return unsigned int 
         */
        static unsigned int octagonal(Vec3i source_, Vec3i target_);
        /**
         * @brief Dikjstra heuristic. (Absence of heuristic, always returns 0)
         * 
         * @return unsigned int 
         */
        static unsigned int dikjstra(Vec3i source_, Vec3i target_);

    };

}

#endif