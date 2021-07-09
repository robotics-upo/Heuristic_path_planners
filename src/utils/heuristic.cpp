#include "utils/heuristic.hpp"

namespace Planners
{
    using namespace utils;
    
    Vec3i Heuristic::getDelta(const Vec3i &_source, const Vec3i &_target)
    {
        return {abs(_source.x - _target.x), abs(_source.y - _target.y), abs(_source.z - _target.z)};
    }

    unsigned int Heuristic::manhattan(const Vec3i &_source, const Vec3i &_target)
    {
        auto delta = std::move(getDelta(_source, _target));
        return static_cast<unsigned int>(dist_scale_factor_ * (delta.x + delta.y + delta.z));
    }

    unsigned int Heuristic::euclidean(const Vec3i &_source, const Vec3i &_target)
    {
        auto delta = std::move(getDelta(_source, _target));
        return static_cast<unsigned int>(dist_scale_factor_ * sqrt(pow(delta.x, 2) + pow(delta.y, 2) + pow(delta.z, 2)));
    }
    unsigned int Heuristic::euclideanAttractive(const float _factor, const Vec3i &_source, const Vec3i &_target){
        auto delta = std::move(getDelta(_source, _target));
        return static_cast<unsigned int>( _factor * dist_scale_factor_ * sqrt(pow(delta.x, 2) + pow(delta.y, 2) + pow(delta.z, 2)));
    }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
    unsigned int Heuristic::dijkstra(const Vec3i &_source, const Vec3i &_target)
    {
        return 0;
    }
#pragma GCC diagnostic pop
    //Does it make sense in 3D?
    unsigned int Heuristic::octagonal(const Vec3i &_source, const Vec3i &_target)
    {
        auto delta = std::move(getDelta(_source, _target));
        return dist_scale_factor_ * (delta.x + delta.y + delta.z) + (-6) * std::min(delta.x, delta.y);
    }

}
