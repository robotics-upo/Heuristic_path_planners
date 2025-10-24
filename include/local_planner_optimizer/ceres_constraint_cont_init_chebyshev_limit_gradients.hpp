#ifndef CERES_CONSTRAINTS_CONT_INIT_CHEBYSHEV_LIMIT_GRADIENTS
#define CERES_CONSTRAINTS_CONT_INIT_CHEBYSHEV_LIMIT_GRADIENTS

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
#include <chrono>


#include <heuristic_planners/Vec3i.h>
#include <heuristic_planners/CoordinateList.h>

#include "Grid3D/local_grid3d.hpp"

#include <ceres/ceres.h>

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

class ReduceGradientsContInitChebyshevFunctor {

public:
    ReduceGradientsContInitChebyshevFunctor(double weight): weight_(weight) {}

    template <typename T>
    bool operator()(const T* const stateCoeff, T* residual) const {

        auto t0 = std::chrono::high_resolution_clock::now();

        // Residuals equal to first derivative in start/goal (s=+-1)
        residual[0] = weight_ * (25.0*stateCoeff[0] - 16.0*stateCoeff[1] + 9.0*stateCoeff[2] - 4.0*stateCoeff[3] + stateCoeff[4]);
        residual[1] = weight_ * (25.0*stateCoeff[6] - 16.0*stateCoeff[7] + 9.0*stateCoeff[8] - 4.0*stateCoeff[9] + stateCoeff[10]);
        residual[2] = weight_ * (25.0*stateCoeff[12] - 16.0*stateCoeff[13] + 9.0*stateCoeff[14] - 4.0*stateCoeff[15] + stateCoeff[16]);
        residual[3] = weight_ * (25.0*stateCoeff[0] + 16.0*stateCoeff[1] + 9.0*stateCoeff[2] + 4.0*stateCoeff[3] + stateCoeff[4]);
        residual[4] = weight_ * (25.0*stateCoeff[6] + 16.0*stateCoeff[7] + 9.0*stateCoeff[8] + 4.0*stateCoeff[9] + stateCoeff[10]);
        residual[5] = weight_ * (25.0*stateCoeff[12] + 16.0*stateCoeff[13] + 9.0*stateCoeff[14] + 4.0*stateCoeff[15] + stateCoeff[16]);


        auto t1 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> dt = t1 - t0;
        Planners::utils::safe_log("Cost Function - Reduce Gradients", dt.count());
        //std::cout << "[T] - Cost Function - Reduce Gradients: " << dt.count() << std::endl;
    
        return true;
    }

    double weight_;    
private:


};

#endif