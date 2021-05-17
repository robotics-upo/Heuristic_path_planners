#ifndef HEURISTIC_HPP
#define HEURISTIC_HPP

#include "utils/utils.hpp"

using namespace Planners::utils;

namespace Planners
{
    class Heuristic
    {
        static Vec3i getDelta(Vec3i source_, Vec3i target_);

    public:
        static unsigned int manhattan(Vec3i source_, Vec3i target_);
        static unsigned int euclidean(Vec3i source_, Vec3i target_);
        static unsigned int octagonal(Vec3i source_, Vec3i target_);
        static unsigned int dikjstra(Vec3i source_, Vec3i target_);

    };

}

#endif