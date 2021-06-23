#ifndef ROSINTERFACES_HPP
#define ROSINTERFACES_HPP

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <math.h>

namespace Planners
{
    namespace utils
    {
        /**
         * @brief 
         * 
         * @tparam T 
         * @tparam U 
         * @param _point 
         * @param _res 
         * @return Vec3i 
         */
        template <typename T, typename U>
        Vec3i discretePoint(const T _point, const U &_res)
        { //Take care of negative values

            return {static_cast<int>(std::round(_point.x / _res)),
                    static_cast<int>(std::round(_point.y / _res)),
                    static_cast<int>(std::round(_point.z / _res))};
        }

        /**
         * @brief 
         * 
         * @param _msg 
         * @param _res 
         * @return Vec3i 
         */
        Vec3i discretePoint(const geometry_msgs::Point &_msg, const double &_res)
        {

            return {static_cast<int>(std::round(_msg.x / _res)),
                    static_cast<int>(std::round(_msg.y / _res)),
                    static_cast<int>(std::round(_msg.z / _res))};
        }

        /**
         * @brief 
         * 
         * @param _vec 
         * @param _res 
         * @return geometry_msgs::Point 
         */
        geometry_msgs::Point continousPoint(const Vec3i &_vec, const double &_res)
        {

            geometry_msgs::Point ret;

            ret.x = _vec.x * _res;
            ret.y = _vec.y * _res;
            ret.z = _vec.z * _res;

            return ret;
        }

    }
}

#endif