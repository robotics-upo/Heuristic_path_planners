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
            utils::CoordinateList getAdjacentPath(const utils::CoordinateList &_path, const utils::DiscreteWorld &_world){
        
                if( _path.size() == 0)
                    return {};

                utils::CoordinateList adjacent_path;
                adjacent_path.push_back(_path[0]);
        
                utils::CoordinateListPtr visited_nodes;
                visited_nodes.reset(new CoordinateList);

                for(size_t i = 0; i < _path.size() -1 ; ++i){
                    utils::LineOfSight::bresenham3D(_path[i], _path[i+1], _world, visited_nodes);

                if(visited_nodes->size() > 0){
                    for(auto &it: *visited_nodes)
                        adjacent_path.push_back(it);
                
                }else if( i != 0) {
                    adjacent_path.push_back(_path[i]);
                }
            
                    visited_nodes.reset(new utils::CoordinateList);
                }
                return adjacent_path;
            }

            inline int dotProduct(const Vec3i &_v1, const Vec3i &_v2){
                return _v1.x * _v2.x + _v1.y * _v2.y + _v1.z * _v2.z;
            }
            inline double moduleVector(const Vec3i &_v){
                return sqrt( _v.x * _v.x + _v.y * _v.y + _v.z * _v.z);
            }

            double angleBetweenThreePoints(const Vec3i &_v1, const Vec3i &_v2, const Vec3i &_v3){

                auto z1 = ( _v1 - _v2 );
                auto z2 = ( _v3 - _v2 );

                return  std::acos(dotProduct(z1, z2) / (moduleVector(z1) * moduleVector(z2)) );
            }

        }
    }
}