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

        residual[0] = weight_ * stateCoeff[0];
        residual[1] = weight_ * stateCoeff[1];
        residual[2] = weight_ * stateCoeff[2];
        residual[3] = weight_ * stateCoeff[3];
        residual[4] = weight_ * stateCoeff[5];
        residual[5] = weight_ * stateCoeff[6];
        residual[6] = weight_ * stateCoeff[7];
        residual[7] = weight_ * stateCoeff[8]; 
        residual[8] = weight_ * stateCoeff[10];
        residual[9] = weight_ * stateCoeff[11];
        residual[10] = weight_ * stateCoeff[12];
        residual[11] = weight_ * stateCoeff[13];     

        return true;
    }

    double weight_;
    
private:


};

#endif