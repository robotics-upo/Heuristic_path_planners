#ifndef CERES_CONSTRAINTS_5_CONT_SMOOTHNESS
#define CERES_CONSTRAINTS_5_CONT_SMOOTHNESS

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

class Ceres5_SmoothnessContFunctor {

public:
    Ceres5_SmoothnessContFunctor(double weight): weight_(weight) {}

    template <typename T>
    bool operator()(const T* const stateCoeff, T* residual) const {

        auto t0 = std::chrono::high_resolution_clock::now();

        double weight_high = weight_ * 50.0;

        residual[0] = weight_ * (2.0*stateCoeff[3] - 8.0*stateCoeff[1]);
        residual[1] = weight_ * (4.0*stateCoeff[2] - 20.0*stateCoeff[0]);
        residual[2] = weight_high * (8.0*stateCoeff[1]);
        residual[3] = weight_high * (16.0*stateCoeff[0]);
        
        residual[4] = weight_ * (2.0*stateCoeff[9] - 8.0*stateCoeff[7]);
        residual[5] = weight_ * (4.0*stateCoeff[8] - 20.0*stateCoeff[6]);
        residual[6] = weight_high * (8.0*stateCoeff[7]);
        residual[7] = weight_high * (16.0*stateCoeff[6]);
        
        residual[8]  = 100 * weight_ * (2.0*stateCoeff[15] - 8.0*stateCoeff[13]);
        residual[9]  = 100 * weight_ * (4.0*stateCoeff[14] - 20.0*stateCoeff[12]);
        residual[10] = 100 * weight_high * (8.0*stateCoeff[13]);
        residual[11] = 100 * weight_high * (16.0*stateCoeff[12]);

        auto t1 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> dt = t1 - t0;
        //Planners::utils::safe_log("Cost Function - Smoothness", dt.count());
        //std::cout << "[T] - Cost Function - Smoothness: " << dt.count() << std::endl;
    

        return true;
    }

    double weight_;    
private:


};

#endif