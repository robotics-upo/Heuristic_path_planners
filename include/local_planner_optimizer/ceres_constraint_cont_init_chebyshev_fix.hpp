#ifndef CERES_CONSTRAINTS_CONT_INIT_CHEBYSHEV_FIX
#define CERES_CONSTRAINTS_CONT_INIT_CHEBYSHEV_FIX

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

class FixStartGoalChebyshevFunctor {

public:
    FixStartGoalChebyshevFunctor(double weight, Planners::utils::Vec3i local_start, Planners::utils::Vec3i local_goal): weight_(weight), local_start_(local_start), local_goal_(local_goal) {}

    template <typename T>
    bool operator()(const T* const stateCoeff, T* residual) const {

        // When s = -1 and s = 1, xyz is very simple

        T x_ini = - stateCoeff[0] + stateCoeff[1] - stateCoeff[2] + stateCoeff[3] - stateCoeff[4] + stateCoeff[5];
        T y_ini = - stateCoeff[6] + stateCoeff[7] - stateCoeff[8] + stateCoeff[9] - stateCoeff[10] + stateCoeff[11];
        T z_ini = - stateCoeff[12] + stateCoeff[13] - stateCoeff[14] + stateCoeff[15] - stateCoeff[16] + stateCoeff[17];

        T x_fin = stateCoeff[0] + stateCoeff[1] + stateCoeff[2] + stateCoeff[3] + stateCoeff[4] + stateCoeff[5];
        T y_fin = stateCoeff[6] + stateCoeff[7] + stateCoeff[8] + stateCoeff[9] + stateCoeff[10] + stateCoeff[11];
        T z_fin = stateCoeff[12] + stateCoeff[13] + stateCoeff[14] + stateCoeff[15] + stateCoeff[16] + stateCoeff[17];

        residual[0] = weight_ * (x_ini - T(local_start_.x));
        residual[1] = weight_ * (y_ini - T(local_start_.y));
        residual[2] = weight_ * (z_ini - T(local_start_.z));
        residual[3] = weight_ * (x_fin - T(local_goal_.x));
        residual[4] = weight_ * (y_fin - T(local_goal_.y));
        residual[5] = weight_ * (z_fin - T(local_goal_.z));
        
        return true;
    }

    double weight_;
    Planners::utils::Vec3i local_start_, local_goal_;
    
    
private:


};

#endif