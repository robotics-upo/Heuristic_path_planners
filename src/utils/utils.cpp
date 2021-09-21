#include "utils/utils.hpp"

namespace Planners
{

    namespace utils
    {

        bool Planners::utils::Vec3i::operator==(const Vec3i &coordinates_)
        {
            return (x == coordinates_.x && y == coordinates_.y && z == coordinates_.z);
        }
        bool Planners::utils::Vec3i::operator>=(const Vec3i &coordinates_)
        {
            return (x >= coordinates_.x && y >= coordinates_.y && z >= coordinates_.z);
        }
        bool Planners::utils::Vec3i::operator<=(const Vec3i &coordinates_)
        {
            return (x <= coordinates_.x && y <= coordinates_.y && z <= coordinates_.z);
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
            // G = H = cost = non_uni =0;
            G = H =0;
        }
        Node::Node(){

        }
        unsigned int Node::getScore()
        {
            return G + H;
        }        
        unsigned int Node::getScoreWithSafetyCost()
        {
            return G + H + cost;  //Add the distance cost.
        }

    }
}