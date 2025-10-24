#ifndef CERES_CONSTRAINTS_CONT_INIT_FIX_GOAL_G5
#define CERES_CONSTRAINTS_CONT_INIT_FIX_GOAL_G5

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
#include <vector>
#include <limits>


using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

class FixGoalInitG5Functor {

public:
    FixGoalInitG5Functor(double weight, Planners::utils::Vec3i local_goal): weight_(weight), local_goal_(local_goal) {}

    template <typename T>
    bool operator()(const T* const stateCoeff, const T* const stateCoeffConstant, T* residual) const {

        T t_fin = T(t_final);

        T x_fin = stateCoeff[0] + stateCoeff[1] + stateCoeff[2] + stateCoeff[3] + stateCoeff[4] + stateCoeffConstant[0];
        T y_fin = stateCoeff[5] + stateCoeff[6] + stateCoeff[7] + stateCoeff[8] + stateCoeff[9] + stateCoeffConstant[1];
        T z_fin = stateCoeff[10] + stateCoeff[11] + stateCoeff[12] + stateCoeff[13] + stateCoeff[14] + stateCoeffConstant[2];

        residual[0] = weight_ * (x_fin - T(local_goal_.x));
        residual[1] = weight_ * (y_fin - T(local_goal_.y));
        residual[2] = weight_ * (z_fin - T(local_goal_.z));
        
        return true;
    }

    double weight_;
    Planners::utils::Vec3i local_goal_;
    
    
private:


};


#endif