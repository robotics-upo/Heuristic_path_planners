#ifndef CERES_CONSTRAINTS_CONT_FIX_GOAL
#define CERES_CONSTRAINTS_CONT_FIX_GOAL

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

#define t_final 10.0

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

class FixGoalContFunctor {

public:
    FixGoalContFunctor(double weight, Planners::utils::Vec3i local_goal): weight_(weight), local_goal_(local_goal) {}

    template <typename T>
    bool operator()(const T* const stateCoeff, const T* const stateCoeffConstant, T* residual) const {

        T t_fin = T(t_final);

        T x_fin = stateCoeff[0] * ceres::pow(t_fin, 5) + stateCoeff[1] * ceres::pow(t_fin, 4) + stateCoeff[2] * ceres::pow(t_fin, 3) + stateCoeff[3] * ceres::pow(t_fin, 2) + stateCoeff[4] * t_fin + stateCoeffConstant[0];
        T y_fin = stateCoeff[5] * ceres::pow(t_fin, 5) + stateCoeff[6] * ceres::pow(t_fin, 4) + stateCoeff[7] * ceres::pow(t_fin, 3) + stateCoeff[8] * ceres::pow(t_fin, 2) + stateCoeff[9] * t_fin + stateCoeffConstant[1];
        T z_fin = stateCoeff[10] * ceres::pow(t_fin, 5) + stateCoeff[11] * ceres::pow(t_fin, 4) + stateCoeff[12] * ceres::pow(t_fin, 3) + stateCoeff[13] * ceres::pow(t_fin, 2) + stateCoeff[14] * t_fin + stateCoeffConstant[2];

        residual[0] = weight_ * (ceres::pow((x_fin-T(local_goal_.x)),2) + ceres::pow((y_fin-T(local_goal_.y)),2) + ceres::pow((z_fin-T(local_goal_.z)),2));

        return true;
    }

    double weight_;
    Planners::utils::Vec3i local_goal_;
    
    
private:


};

#endif