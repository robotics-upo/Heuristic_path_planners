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
                return static_cast<unsigned int>(dist_scale_factor_ * sqrt(pow(_n1.coordinates.x - _n2.coordinates.x, 2) +
                                                                           pow(_n1.coordinates.y - _n2.coordinates.y, 2) +
                                                                           pow(_n1.coordinates.z - _n2.coordinates.z, 2)));
            }
            unsigned int distanceBetween2Nodes(const Node *_n1, const Node *_n2)
            {
                return distanceBetween2Nodes(*_n1, *_n2);
            }
            Vec3i abs(const Vec3i &_vec)
            {
                return { std::abs(_vec.x), std::abs(_vec.y), std::abs(_vec.z) };
            }
        }
    }
}