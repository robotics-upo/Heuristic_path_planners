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
                for (int i = 1; i < _path.size(); i++)
                {
                    len += sqrtf(pow((_path[i].x - _path[i - 1].x) * _resolution, 2) +
                                 pow((_path[i].y - _path[i - 1].y) * _resolution, 2) +
                                 pow((_path[i].z - _path[i - 1].z) * _resolution, 2));
                }
                return len;
            }
            float distanceBetween2nodes(Node &n1, Node &n2)
            {
	            return sqrt(pow(n1.coordinates.x - n2.coordinates.x, 2) +
			    pow(n1.coordinates.y - n2.coordinates.y, 2) +
			    pow(n1.coordinates.z - n2.coordinates.z, 2));
            } 
            float distanceBetween2nodesOTRO(Node *n1, Node *n2)
            {
                return sqrt(pow(n1->coordinates.x - n2->coordinates.x, 2) +
			    pow(n1->coordinates.y - n2->coordinates.y, 2) +
			    pow(n1->coordinates.z - n2->coordinates.z, 2));
            } 
        }
    }
}