#ifndef CERES_CONSTRAINTS_CONT_PATH_LEGTH
#define CERES_CONSTRAINTS_CONT_PATH_LEGTH

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

#define SEGMENTS 100
#define t_final 10.0

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

class PathLengthContFunctor {

public:
    PathLengthContFunctor(double weight): weight_(weight) {}

    template <typename T>
    bool operator()(const T* const stateCoeff, const T* const stateCoeffConstant, T* residual) const {

        T t_step = T(t_final) / T(SEGMENTS);
        residual[0] = T(0);

        T x0 = stateCoeffConstant[0];
        T y0 = stateCoeffConstant[1];
        T z0 = stateCoeffConstant[2];

        for (int i = 0; i < SEGMENTS; i++){
            T t_act = t_step * T(i+1);

            T x1 = stateCoeff[0] * ceres::pow(t_act, 5) + stateCoeff[1] * ceres::pow(t_act, 4) + stateCoeff[2] * ceres::pow(t_act, 3) + stateCoeff[3] * ceres::pow(t_act, 2) + stateCoeff[4] * t_act + stateCoeffConstant[0];
            T y1 = stateCoeff[5] * ceres::pow(t_act, 5) + stateCoeff[6] * ceres::pow(t_act, 4) + stateCoeff[7] * ceres::pow(t_act, 3) + stateCoeff[8] * ceres::pow(t_act, 2) + stateCoeff[9] * t_act + stateCoeffConstant[1];
            T z1 = stateCoeff[10] * ceres::pow(t_act, 5) + stateCoeff[11] * ceres::pow(t_act, 4) + stateCoeff[12] * ceres::pow(t_act, 3) + stateCoeff[13] * ceres::pow(t_act, 2) + stateCoeff[14] * t_act + stateCoeffConstant[2];

            residual[0] += weight_ * ceres::sqrt(ceres::pow((x1-x0),2) + ceres::pow((y1-y0),2) + ceres::pow((z1-z0),2));
            x0 = x1;
            y0 = y1;
            z0 = z1;
        }

        return true;
    }

    double weight_;
    
private:


};

#endif