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

#endif