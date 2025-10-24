#ifndef CERES_CONSTRAINTS_CONT_INIT_DISTANCE_TO_WP_G5
#define CERES_CONSTRAINTS_CONT_INIT_DISTANCE_TO_WP_G5

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


#include <heuristic_planners/Vec3i.h>
#include <heuristic_planners/CoordinateList.h>

#include "Grid3D/local_grid3d.hpp"

#include <ceres/ceres.h>


using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

class DistanceToWPG5Functor {

public:
    DistanceToWPG5Functor (double weight, Planners::utils::Vec3i global_wp, double t_act): weight_(weight), global_wp_(global_wp), t_act_(t_act) {}

    template <typename T>
    bool operator()(const T* const stateCoeff, const T* const stateCoeffConstant, T* residual) const {

        T x_act = stateCoeff[0] * ceres::pow(t_act_, 5) + stateCoeff[1] * ceres::pow(t_act_, 4) + stateCoeff[2] * ceres::pow(t_act_, 3) + stateCoeff[3] * ceres::pow(t_act_, 2) + stateCoeff[4] * t_act_ + stateCoeffConstant[0];
        T y_act = stateCoeff[5] * ceres::pow(t_act_, 5) + stateCoeff[6] * ceres::pow(t_act_, 4) + stateCoeff[7] * ceres::pow(t_act_, 3) + stateCoeff[8] * ceres::pow(t_act_, 2) + stateCoeff[9] * t_act_ + stateCoeffConstant[1];
        T z_act = stateCoeff[10] * ceres::pow(t_act_, 5) + stateCoeff[11] * ceres::pow(t_act_, 4) + stateCoeff[12] * ceres::pow(t_act_, 3) + stateCoeff[13] * ceres::pow(t_act_, 2) + stateCoeff[14] * t_act_ + stateCoeffConstant[2];

        residual[0] = weight_ * (ceres::pow((x_act-T(global_wp_.x)),2) + ceres::pow((y_act-T(global_wp_.y)),2) + ceres::pow((z_act-T(global_wp_.z)),2));

        return true;
    }

    double weight_;
    double t_act_;
    Planners::utils::Vec3i global_wp_;
    
    
    
private:


};

// class DistanceToWPG5Functor {
// public:
//     DistanceToWPG5Functor(double weight, const Planners::utils::CoordinateList& global_path_local_section, int f_segments): weight_(weight), global_path_local_section_(global_path_local_section), f_segments_(f_segments) {}

//     template <typename T>
//     bool operator()(const T* const stateCoeff, const T* const stateCoeffConstant, T* residual) const {
//         // Generate full trajectory (t ∈ [0,1])
//         std::vector<T> traj_x(f_segments_), traj_y(f_segments_), traj_z(f_segments_);
//         traj_x.reserve(f_segments_);
//         traj_y.reserve(f_segments_);
//         traj_z.reserve(f_segments_);
//         for (int i = 0; i < f_segments_; ++i) {
//             T t = T(i) / T(f_segments_ - 1);
//             traj_x[i] = stateCoeff[0]*ceres::pow(t,5) + stateCoeff[1]*ceres::pow(t,4) + stateCoeff[2]*ceres::pow(t,3) + stateCoeff[3]*ceres::pow(t,2) + stateCoeff[4]*t + stateCoeffConstant[0];
//             traj_y[i] = stateCoeff[5]*ceres::pow(t,5) + stateCoeff[6]*ceres::pow(t,4) + stateCoeff[7]*ceres::pow(t,3) + stateCoeff[8]*ceres::pow(t,2) + stateCoeff[9]*t + stateCoeffConstant[1];
//             traj_z[i] = stateCoeff[10]*ceres::pow(t,5) + stateCoeff[11]*ceres::pow(t,4) + stateCoeff[12]*ceres::pow(t,3) + stateCoeff[13]*ceres::pow(t,2) + stateCoeff[14]*t + stateCoeffConstant[2];
//         }

//         // Find minimum distance to each wp and add to cost
//         T total_cost = T(0.0);
//         for (size_t wp_idx = 0; wp_idx < global_path_local_section_.size(); ++wp_idx) {
//             const auto& wp = global_path_local_section_[wp_idx];
//             T min_dist2 = T(1e12);

//             for (int i = 0; i < f_segments_; ++i) {
//                 T dx = traj_x[i] - T(wp.x);
//                 T dy = traj_y[i] - T(wp.y);
//                 T dz = traj_z[i] - T(wp.z);
//                 T dist2 = dx*dx + dy*dy + dz*dz;
//                 if (dist2 < min_dist2) 
//                     min_dist2 = dist2;
//             }

//             total_cost += min_dist2;
//         }

//         residual[0] = weight_ * total_cost;
//         return true;
//     }

// private:
//     double weight_;
//     int f_segments_;
//     const Planners::utils::CoordinateList& global_path_local_section_;
// };


#endif