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

            inline double moduleVector(const Vec3i &_v){
                return sqrt( _v.x * _v.x + _v.y * _v.y + _v.z * _v.z);
            }
            
            double angleBetweenThreePoints(const Vec3i &_v1, const Vec3i &_v2, const Vec3i &_v3){
                return angleBetweenThreePoints(_v1.toEigen(), _v2.toEigen(), _v3.toEigen());
            }
            double angleBetweenThreePoints(const Eigen::Vector3d &_v1, const Eigen::Vector3d &_v2, const Eigen::Vector3d &_v3){

                Eigen::Vector3d z1 = ( _v1 - _v2 );
                Eigen::Vector3d z2 = ( _v3 - _v2 );
                z1.normalize();
                z2.normalize();

                return  std::acos( z1.dot(z2) );
            }

            double getCircunferenceRadius(const Vec3i &_v1, const Vec3i &_v2, const Vec3i &_v3){
                return getCircunferenceRadius(_v1.toEigen(), _v2.toEigen(), _v3.toEigen());
            }
            double getCircunferenceRadius(const Eigen::Vector3d &_v1, const Eigen::Vector3d &_v2, const Eigen::Vector3d &_v3){
            
                Eigen::Vector3d PQ = _v2 - _v1;
                Eigen::Vector3d PR = _v3 - _v1;
                Eigen::Vector3d n = PQ.cross(PR);   

                if ( n.norm() == 0){
                    // std::cerr << "Error, given vector are colinears so the solution is not defined" << std::endl;
                    return std::numeric_limits<double>::infinity();;
                }

                PQ.normalize();
                PR.normalize();
                n.normalize();

                Eigen::Vector3d e1 = n.cross(PQ);
                Eigen::Vector3d e2 = PQ;
                Eigen::Vector3d e3 = n;
                e1.normalize();
                e2.normalize();
                e3.normalize();

                Eigen::Matrix3d m;
                m << e1 , e2, e3;

                Eigen::Vector3d Pn = m.inverse() * _v1;
                Eigen::Vector3d Qn = m.inverse() * _v2;
                Eigen::Vector3d Rn = m.inverse() * _v3;

                Eigen::MatrixXd M(3,3);

                M << Pn(0), Pn(1), 1,
                     Qn(0), Qn(1), 1,
                     Rn(0), Rn(1), 1;

                double d1 = Pn(0)*Pn(0) + Pn(1)*Pn(1);
                double d2 = Qn(0)*Qn(0) + Qn(1)*Qn(1);
                double d3 = Rn(0)*Rn(0) + Rn(1)*Rn(1);
                Eigen::Vector3d D(-1 * d1, -1*d2, -1*d3 );

                Eigen::FullPivLU<Eigen::Matrix3d> lu(M);
                lu.setThreshold(1e-4);
                Eigen::Vector3d result = lu.solve(D);

                double x0 = -result(0)/2;
                double y0 = -result(1)/2;
                double R  = sqrt(x0*x0 + y0*y0 - result(2));
                // std::cout << "X0,Y0: " << x0 << ", " << y0 << std::endl;
                // std::cout << "R: " << R << std::endl;

                return R;
            }

        }
    }
}