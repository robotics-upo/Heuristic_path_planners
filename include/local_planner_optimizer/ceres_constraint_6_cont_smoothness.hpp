#ifndef CERES_CONSTRAINTS_6_CONT_SMOOTHNESS
#define CERES_CONSTRAINTS_6_CONT_SMOOTHNESS

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

class Ceres6_SmoothnessContFunctor {

public:
    Ceres6_SmoothnessContFunctor(double weight, Planners::utils::Vec3i local_start, Planners::utils::Vec3i local_goal): weight_(weight), local_start_(local_start), local_goal_(local_goal) {}

    template <typename T>
    bool operator()(const T* const stateCoeff, T* residual) const {

        residual[0] = weight_ * (-4.0*(local_goal_.x + local_start_.x) + 10.0*stateCoeff[1] + 8.0*stateCoeff[3]);
        residual[1] = weight_ * (-10.0*(local_goal_.x - local_start_.x) + 24.0*stateCoeff[0] + 20.0*stateCoeff[2]);
        residual[2] = weight_ * (4.0*(local_goal_.x + local_start_.x) - 8.0*stateCoeff[1] - 8.0*stateCoeff[3]);
        residual[3] = weight_ * (8.0*(local_goal_.x - local_start_.x) - 16.0*stateCoeff[0] - 16.0*stateCoeff[2]);
        residual[4] = weight_ * (-4.0*(local_goal_.y + local_start_.y) + 10.0*stateCoeff[5] + 8.0*stateCoeff[7]);
        residual[5] = weight_ * (-10.0*(local_goal_.y - local_start_.y) + 24.0*stateCoeff[4] + 20.0*stateCoeff[6]);
        residual[6] = weight_ * (4.0*(local_goal_.y + local_start_.y) - 8.0*stateCoeff[5] - 8.0*stateCoeff[7]);
        residual[7] = weight_ * (8.0*(local_goal_.y - local_start_.y) - 16.0*stateCoeff[4] - 16.0*stateCoeff[6]);
        residual[8] = weight_ * (-4.0*(local_goal_.z + local_start_.z) + 10.0*stateCoeff[9] + 8.0*stateCoeff[11]);
        residual[9] = weight_ * (-10.0*(local_goal_.z - local_start_.z) + 24.0*stateCoeff[8] + 20.0*stateCoeff[10]);
        residual[10] = weight_ * (4.0*(local_goal_.z + local_start_.z) - 8.0*stateCoeff[9] - 8.0*stateCoeff[11]);
        residual[11] = weight_ * (8.0*(local_goal_.z - local_start_.z) - 16.0*stateCoeff[8] - 16.0*stateCoeff[10]);  


        return true;
    }

    double weight_;
    Planners::utils::Vec3i local_start_, local_goal_;    
private:


};

#endif