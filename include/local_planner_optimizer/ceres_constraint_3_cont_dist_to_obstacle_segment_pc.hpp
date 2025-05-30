#ifndef CERES_CONSTRAINTS_3_PC_CONT_DIST_TO_OBSTACLE_SEGMENT
#define CERES_CONSTRAINTS_3_PC_CONT_DIST_TO_OBSTACLE_SEGMENT

#include <iostream>
#include <fstream>
#include <string>
#include "utils/ros/ROSInterfaces.hpp"
#include "utils/SaveDataVariantToFile.hpp"
#include "utils/misc.hpp"
#include "utils/geometry_utils.hpp"
#include "utils/metrics.hpp"
#include <ros/ros.h>
#include <Eigen/Dense>
#include <mutex>


#include <heuristic_planners/Vec3i.h>
#include <heuristic_planners/CoordinateList.h>

#include "Grid3D/local_grid3d.hpp"

#include <ceres/ceres.h>


using ceres::SizedCostFunction;
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

class Ceres3_PCObstacleDistanceCostContSegmentFunctor {

public:
    Ceres3_PCObstacleDistanceCostContSegmentFunctor(double t_act, int n_points, double max_local_distance, const std::vector<Eigen::Vector3d>& obstacle_point_cloud, double weight): weight_(weight), n_points_(n_points), t_act_(t_act), max_local_distance_(max_local_distance), obstacle_point_cloud_(obstacle_point_cloud) {}

    template <typename T>
    bool operator()(const T* const stateCoeff, const T* const stateCoeffConstant, T* residual) const{

        T p[3];
        T weight_T = T(weight_);
        T n_points_T = T(n_points_);
        p[0] = stateCoeff[0] * ceres::pow(t_act_, 3) + stateCoeff[1] * ceres::pow(t_act_, 2) + stateCoeff[2] * t_act_ + stateCoeffConstant[0];
        p[1] = stateCoeff[3] * ceres::pow(t_act_, 3) + stateCoeff[4] * ceres::pow(t_act_, 2) + stateCoeff[5] * t_act_ + stateCoeffConstant[1];
        p[2] = stateCoeff[6] * ceres::pow(t_act_, 3) + stateCoeff[7] * ceres::pow(t_act_, 2) + stateCoeff[8] * t_act_ + stateCoeffConstant[2];

        // Find point in pointcloud closest to our point

        T min_dist = T(max_local_distance_);
        for (const auto& obs : obstacle_point_cloud_) {
            T dx = p[0] - T(obs[0]);
            T dy = p[1] - T(obs[1]);
            T dz = p[2] - T(obs[2]);
            T dist = ceres::sqrt(dx*dx + dy*dy + dz*dz);

            if (ceres::IsFinite(dist) && dist < min_dist) {
                min_dist = dist;
            }
        }

        //residual[0] = (weight_T / n_points_T) * (max_local_distance_ - min_dist) ;
        residual[0] = weight_T / (n_points_T * min_dist) ;


        return true;
    }

    double weight_, t_act_, max_local_distance_;
    int n_points_;
    const std::vector<Eigen::Vector3d>& obstacle_point_cloud_;
    
private:


};

#endif