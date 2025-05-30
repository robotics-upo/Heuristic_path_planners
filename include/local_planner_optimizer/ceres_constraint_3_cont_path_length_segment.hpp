#ifndef CERES_CONSTRAINTS_3_CONT_PATH_LEGTH_SEGMENT
#define CERES_CONSTRAINTS_3_CONT_PATH_LEGTH_SEGMENT

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

class Ceres3_PathLengthContSegmentFunctor {

public:
    Ceres3_PathLengthContSegmentFunctor(double weight, double t0, double t1): weight_(weight), t0_(t0), t1_(t1) {}

    template <typename T>
    bool operator()(const T* const stateCoeff, const T* const stateCoeffConstant, T* residual) const {

        T x0 = stateCoeff[0] * ceres::pow(t0_, 3) + stateCoeff[1] * ceres::pow(t0_, 2) + stateCoeff[2] * t0_ + stateCoeffConstant[0];
        T y0 = stateCoeff[3] * ceres::pow(t0_, 3) + stateCoeff[4] * ceres::pow(t0_, 2) + stateCoeff[5] * t0_ + stateCoeffConstant[1];
        T z0 = stateCoeff[6] * ceres::pow(t0_, 3) + stateCoeff[7] * ceres::pow(t0_, 2) + stateCoeff[8] * t0_ + stateCoeffConstant[2];

        T x1 = stateCoeff[0] * ceres::pow(t1_, 3) + stateCoeff[1] * ceres::pow(t1_, 2) + stateCoeff[2] * t1_ + stateCoeffConstant[0];
        T y1 = stateCoeff[3] * ceres::pow(t1_, 3) + stateCoeff[4] * ceres::pow(t1_, 2) + stateCoeff[5] * t1_ + stateCoeffConstant[1];
        T z1 = stateCoeff[6] * ceres::pow(t1_, 3) + stateCoeff[7] * ceres::pow(t1_, 2) + stateCoeff[8] * t1_ + stateCoeffConstant[2];
        
        residual[0] = weight_ * (ceres::pow((x1-x0),2) + ceres::pow((y1-y0),2) + ceres::pow((z1-z0),2));

        return true;
    }

    double weight_, t0_, t1_;
    
private:


};

#endif