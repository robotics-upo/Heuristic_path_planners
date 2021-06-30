#include "utils/heuristic.hpp"

namespace Planners
{
    using namespace utils;
    
    Vec3i Heuristic::getDelta(Vec3i source_, Vec3i target_)
    {
        return {abs(source_.x - target_.x), abs(source_.y - target_.y), abs(source_.z - target_.z)};
    }

    unsigned int Heuristic::manhattan(Vec3i source_, Vec3i target_)
    {
        auto delta = std::move(getDelta(source_, target_));
        return static_cast<unsigned int>(100 * (delta.x + delta.y + delta.z));
    }

    unsigned int Heuristic::euclidean(Vec3i source_, Vec3i target_)
    {
        auto delta = std::move(getDelta(source_, target_));
        return static_cast<unsigned int>(100 * sqrt(pow(delta.x, 2) + pow(delta.y, 2) + pow(delta.z, 2)));
    }
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
    unsigned int Heuristic::dikjstra(Vec3i source_, Vec3i target_)
    {
        return 0;
    }
#pragma GCC diagnostic pop
    //Does it make sense in 3D?
    unsigned int Heuristic::octagonal(Vec3i source_, Vec3i target_)
    {
        auto delta = std::move(getDelta(source_, target_));
        return 100 * (delta.x + delta.y + delta.z) + (-6) * std::min(delta.x, delta.y);
    }

}
