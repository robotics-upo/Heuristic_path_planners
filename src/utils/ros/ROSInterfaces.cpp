#include "utils/ros/ROSInterfaces.hpp"

namespace Planners
{
    namespace utils
    {

        Vec3i discretePoint(const pcl::PointXYZ &_point, const double &_res)
        { //Take care of negative values

            return {static_cast<int>(std::round(_point.x / _res)),
                    static_cast<int>(std::round(_point.y / _res)),
                    static_cast<int>(std::round(_point.z / _res))};
        }
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
        geometry_msgs::Point continousPoint(const Vec3i &_vec, const double &_res)
        {
            geometry_msgs::Point ret;

            ret.x = _vec.x * _res;
            ret.y = _vec.y * _res;
            ret.z = _vec.z * _res;

            return ret;
        }
        inline Vec3i indexToXY(const unsigned int &_index, const unsigned int _grid_width)
        {

            return {static_cast<int>(std::floor(_index % _grid_width)),
                    static_cast<int>(std::floor(_index / _grid_width)),
                    0};
        }

        bool configureWorldFromOccupancy(const nav_msgs::OccupancyGrid &_grid, AlgorithmBase &_algorithm, bool _set_size)
        {

            if (_set_size)
            {
                Vec3i world_size;
                world_size.x = std::floor(_grid.info.width / _grid.info.resolution);
                world_size.y = std::floor(_grid.info.height / _grid.info.resolution);
                world_size.z = 1;
                _algorithm.setWorldSize(world_size, _grid.info.resolution);
            }

            Vec3i cell;
            for (long unsigned int i = 0; i < _grid.data.size(); ++i)
            {
                // 100 should be costmap_2d::LETHAL_OBSTACLE but by the values that map server publishes are between 0 and 100
                if (_grid.data[i] == 100)
                {
                    cell = indexToXY(i, _grid.info.width);
                    _algorithm.addCollision(cell);
                }
            }

            return true;
        }
        bool configureWorldFromOccupancyWithCosts(const nav_msgs::OccupancyGrid &_grid, AlgorithmBase &_algorithm, bool _set_size)
        {

            if (_set_size)
            {
                Vec3i world_size;
                world_size.x = std::floor(_grid.info.width / _grid.info.resolution);
                world_size.y = std::floor(_grid.info.height / _grid.info.resolution);
                world_size.z = 1;
                _algorithm.setWorldSize(world_size, _grid.info.resolution);
            }

            Vec3i cell;
            for (long unsigned int i = 0; i < _grid.data.size(); ++i)
            {
                // int data = _grid.data[i];
                // std::cout << "Cost: " << data << std::endl;
                // 100 should be costmap_2d::LETHAL_OBSTACLE but by the values that map server publishes are between 0 and 100
                cell = indexToXY(i, _grid.info.width);
                if (_grid.data[i] >= 99)
                    _algorithm.addCollision(cell);
                _algorithm.configureCellCost( cell, _grid.data[i] );
            }

            return true;
        }
        bool configureWorldFromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &_points, AlgorithmBase &_algorithm, const double &_resolution)
        {

            for (auto &it : *_points)
                _algorithm.addCollision(discretePoint(it, _resolution));

            return true;
        }

        bool configureWorldCosts(Grid3d &_grid, AlgorithmBase &_algorithm)
        {

            auto world_size = _algorithm.getWorldSize();
            auto resolution = _algorithm.getWorldResolution();

            for (int i = 0; i < world_size.x; i++)
            {
                for (int j = 0; j < world_size.y; j++)
                {
                    for (int k = 0; k < world_size.z; k++)
                    {
                        //JAC: Precision
                        // auto cost = _grid.getCellCost(i * resolution, j * resolution, k * resolution);
                        float cost = _grid.getCellCost(i * resolution, j * resolution, k * resolution);
                        // std::cout << "Cost: " << cost << std::endl;   
                        _algorithm.configureCellCost({i, j, k}, cost);
                }
                    }
            }

            return true;
        }

        bool configureLocalWorldCosts(Local_Grid3d &_grid, AlgorithmBase &_algorithm, float drone_x, float drone_y, float drone_z, torch::jit::script::Module& loaded_sdf)
        {
            // Computar el grid local
            _grid.computeLocalGrid(loaded_sdf, drone_x, drone_y, drone_z);

            auto world_size = _algorithm.getWorldSize();
            std::cout << "World size: " << world_size << std::endl;
            auto resolution = _algorithm.getWorldResolution();

            for (int i = 0; i < world_size.x; i++)
            {
                for (int j = 0; j < world_size.y; j++)
                {
                    for (int k = 0; k < world_size.z; k++)
                    {
                        float cost = _grid.getCellCost(i * resolution, j * resolution, k * resolution);
                        _algorithm.configureCellCost({i, j, k}, cost);
                        if(cost <= resolution)
                            _algorithm.addCollision({i, j, k});
                        else
                            _algorithm.removeCollision({i, j, k});
                    }
                }
            }


            return true;
        }

        
        // bool configureLocalWorldCosts(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &_points, Local_Grid3d &_grid, AlgorithmBase &_algorithm)
        // {
        //     //  JAC: Generate the local grid
        //     // std::cout << "no. of pts=" << _points->size() << std::endl;
        //     // unsigned t0, t1;
        //     // t0 = clock();
        //     _grid.computeLocalGrid(_points);
        //     // t1 = clock();
        //     // double time = (double(t1-t0)/CLOCKS_PER_SEC);
        //     // std::cout << "Execution Time: " << time << std::endl;

        //     auto world_size = _algorithm.getWorldSize();
        //     auto resolution = _algorithm.getWorldResolution();
        //     // std::cout << "world size X: " << world_size.x  << std::endl;  //50
        //     // std::cout << "world size Y: " << world_size.y  << std::endl;  //50
        //     // std::cout << "world size Z: " << world_size.z  << std::endl;  //20
        //     // std::cout << "resolution: " << resolution  << std::endl;  //0.2

        //     // JAC: 50-50 milliseconds --> CUDA
        //     // t0 = clock();
        //     for (int i = 0; i < world_size.x; i++)
        //     {
        //         for (int j = 0; j < world_size.y; j++)
        //         {
        //             for (int k = 0; k < world_size.z; k++)
        //             {
        //                 //JAC: Precision
        //                 // auto cost = _grid.getCellCost(i * resolution, j * resolution, k * resolution);
        //                 float cost = _grid.getCellCost(i * resolution, j * resolution, k * resolution);
        //                 // if (cost > 285.65) // JAC: cost is given in the discrete_world. 
        //                     // std::cout << "Cost: " << cost << std::endl;   
        //                 _algorithm.configureCellCost({i, j, k}, cost);
        //             }
        //         }
        //     }
        //     // t1 = clock();
        //     // double time = (double(t1-t0)/CLOCKS_PER_SEC);
        //     // std::cout << "Execution Time: " << time << std::endl;
        //     return true;
        // }

    } //ns utils
} //ns planners