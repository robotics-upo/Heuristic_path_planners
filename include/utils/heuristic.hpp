#ifndef HEURISTIC_HPP
#define HEURISTIC_HPP

#include "utils/utils.hpp"

using namespace Planners::utils;

namespace Planners
{
    class Heuristic
    {   
        /**
         * @brief Get the Delta object
         * 
         * @param source_ 
         * @param target_ 
         * @return Vec3i 
         */
        static Vec3i getDelta(Vec3i source_, Vec3i target_);

    public:
        /**
         * @brief 
         * 
         * @param source_ 
         * @param target_ 
         * @return unsigned int 
         */
        static unsigned int manhattan(Vec3i source_, Vec3i target_);
        /**
         * @brief 
         * 
         * @param source_ 
         * @param target_ 
         * @return unsigned int 
         */
        static unsigned int euclidean(Vec3i source_, Vec3i target_);
        /**
         * @brief 
         * 
         * @param source_ 
         * @param target_ 
         * @return unsigned int 
         */
        static unsigned int octagonal(Vec3i source_, Vec3i target_);
        /**
         * @brief 
         * 
         * @return unsigned int 
         */
        static unsigned int dikjstra();

    };

}

#endif