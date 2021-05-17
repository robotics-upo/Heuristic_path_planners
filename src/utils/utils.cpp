#include "utils/utils.hpp"

namespace Planners
{

    namespace utils
    {

        bool Planners::utils::Vec3i::operator==(const Vec3i &coordinates_)
        {
            return (x == coordinates_.x && y == coordinates_.y && z == coordinates_.z);
        }

        Vec3i &Vec3i::operator/=(float _divid)
        {
            x = x / _divid;
            y = y / _divid;
            z = z / _divid;

            return *this;
        }
        Planners::utils::Vec3i operator/(const Planners::utils::Vec3i &left_, const Planners::utils::Vec3i &right_)
        {
            return {left_.x + right_.x, left_.y + right_.y, left_.z + right_.z};
        }

        Node::Node(Planners::utils::Vec3i coordinates_, Node *parent_)
        {
            parent = parent_;
            coordinates = coordinates_;
            G = H = 0;
        }
        Node::Node(){

        }
        unsigned int Node::getScore()
        {
            return G + H;
        }
        float calcualtePathLength(const CoordinateList &_path, const double &_resolution)
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

    }
}