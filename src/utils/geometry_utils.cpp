#include "utils/geometry_utils.hpp"

namespace Planners
{
    namespace utils
    {
        namespace geometry
        {

            float calculatePathLength(const CoordinateList &_path, const double &_resolution)
            {
                float len = 0;
                for (long unsigned int i = 1; i < _path.size(); i++)
                {
                    len += sqrtf(pow((_path[i].x - _path[i - 1].x) * _resolution, 2) +
                                 pow((_path[i].y - _path[i - 1].y) * _resolution, 2) +
                                 pow((_path[i].z - _path[i - 1].z) * _resolution, 2));
                }
                return len;
            }
            unsigned int distanceBetween2Nodes(const Node &_n1, const Node &_n2)
            {
                return distanceBetween2Nodes(_n1.coordinates, _n2.coordinates);
            }
            unsigned int distanceBetween2Nodes(const Node *_n1, const Node *_n2)
            {
                return distanceBetween2Nodes(*_n1, *_n2);
            }
            unsigned int distanceBetween2Nodes(const Vec3i &_v1, const Vec3i &_v2)
            {
                return static_cast<unsigned int>(dist_scale_factor_ * sqrt(pow(_v1.x - _v2.x, 2) +
                                                                           pow(_v1.y - _v2.y, 2) +
                                                                           pow(_v1.z - _v2.z, 2)));
            }
            Vec3i abs(const Vec3i &_vec)
            {
                return { std::abs(_vec.x), std::abs(_vec.y), std::abs(_vec.z) };
            }
        }
    }
}