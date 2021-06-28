#ifndef ROSINTERFACES_HPP
#define ROSINTERFACES_HPP

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_2d/cost_values.h>
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
        Vec3i discretePose(const geometry_msgs::Pose &_msg, const double &_res)
        {

            return {static_cast<int>(std::round(_msg.position.x / _res)),
                    static_cast<int>(std::round(_msg.position.y / _res)),
                    static_cast<int>(std::round(_msg.position.z / _res))};
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
        /**
         * @brief 
         * 
         * @param _index 
         * @param _grid_width 
         * @return Vec3i 
         */
        inline Vec3i indexToXY(const unsigned int &_index, const unsigned int &_grid_width){

            return {static_cast<int>(std::floor( _index % _grid_width )),
                    static_cast<int>(std::floor( _index / _grid_width )),
                    0};
        }
        /**
         * @brief 
         * 
         * @param _grid 
         * @param _algorithm 
         * @return true 
         * @return false 
         */
        bool configureWorldFromOccupancy(const nav_msgs::OccupancyGrid &_grid, PathGenerator &_algorithm, bool set_size = false){
            
            if(set_size){
                Vec3i world_size;
                world_size.x = std::floor(_grid.info.width  / _grid.info.resolution);
                world_size.y = std::floor(_grid.info.height / _grid.info.resolution);
                world_size.z = 1;
                _algorithm.setWorldSize(world_size, _grid.info.resolution);
            }

            Vec3i cell;
            for(long unsigned int i = 0; i < _grid.data.size(); ++i){
                // 100 should be costmap_2d::LETHAL_OBSTACLE but by the values that map server publishes are between 0 and 100
                if ( _grid.data[i] == 100 ){
                    cell = indexToXY(i, _grid.info.width);
                    _algorithm.addCollision(cell);
                }
            } 

            return true;
        }
    }
}

#endif