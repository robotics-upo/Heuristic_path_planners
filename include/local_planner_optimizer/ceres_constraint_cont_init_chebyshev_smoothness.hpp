#ifndef CERES_CONSTRAINTS_CONT_INIT_CHEBYSHEV_SMOOTHNESS
#define CERES_CONSTRAINTS_CONT_INIT_CHEBYSHEV_SMOOTHNESS

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

class SmoothnessContInitChebyshevFunctor {

public:
    SmoothnessContInitChebyshevFunctor(double weight): weight_(weight) {}

    template <typename T>
    bool operator()(const T* const stateCoeff, T* residual) const {

        residual[0] = weight_ * (2.0*stateCoeff[3] - 8.0*stateCoeff[1]);
        residual[1] = weight_ * (4.0*stateCoeff[2] - 20.0*stateCoeff[0]);
        residual[2] = weight_ * (8.0*stateCoeff[1]);
        residual[3] = weight_ * (16.0*stateCoeff[0]);
        residual[4] = weight_ * (2.0*stateCoeff[9] - 8.0*stateCoeff[7]);
        residual[5] = weight_ * (4.0*stateCoeff[8] - 20.0*stateCoeff[6]);
        residual[6] = weight_ * (8.0*stateCoeff[7]);
        residual[7] = weight_ * (16.0*stateCoeff[6]);
        residual[8] = weight_ * (2.0*stateCoeff[15] - 8.0*stateCoeff[13]);
        residual[9] = weight_ * (4.0*stateCoeff[14] - 20.0*stateCoeff[12]);
        residual[10] = weight_ * (8.0*stateCoeff[13]);
        residual[11] = weight_ * (16.0*stateCoeff[12]);
     

        return true;
    }

    double weight_;
    
private:


};

#endif