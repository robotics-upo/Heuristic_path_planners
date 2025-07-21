#ifndef CERES_CONSTRAINTS_CONT_INIT_SMOOTHNESS_G5
#define CERES_CONSTRAINTS_CONT_INIT_SMOOTHNESS_G5

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

class SmoothnessContInitG5Functor {

public:
    SmoothnessContInitG5Functor(double weight): weight_(weight) {}

    template <typename T>
    bool operator()(const T* const stateCoeff, T* residual) const {

        residual[0] = T(0);
        for (int i = 0; i < 3; i++){
            residual[0] += (stateCoeff[5*i+1] * stateCoeff[5*i+1] + stateCoeff[5*i+2] * stateCoeff[5*i+2] + stateCoeff[5*i+3] * stateCoeff[5*i+3] + stateCoeff[5*i+4] * stateCoeff[5*i+4]);
        }
        residual[0] = weight_ * residual[0];

        return true;
    }

    double weight_;
    
private:


};

#endif