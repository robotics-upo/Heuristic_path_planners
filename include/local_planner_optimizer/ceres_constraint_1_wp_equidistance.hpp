#ifndef CERES_CONSTRAINTS_1_WP_EQUIDISTANCE
#define CERES_CONSTRAINTS_1_WP_EQUIDISTANCE

#include <iostream>
#include <fstream>
#include <string>
#include "utils/ros/ROSInterfaces.hpp"
#include "utils/SaveDataVariantToFile.hpp"
#include "utils/misc.hpp"
#include "utils/geometry_utils.hpp"
#include "utils/metrics.hpp"
#include <ros/ros.h>

#include <heuristic_planners/Vec3i.h>
#include <heuristic_planners/CoordinateList.h>

#include "Grid3D/local_grid3d.hpp"

#include <ceres/ceres.h>

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

class Ceres1_EquidistanceFunctor {

public:
    Ceres1_EquidistanceFunctor(double weight): weight_(weight) {}

    template <typename T>
    bool operator()(const T* const stateWP1, const T* const stateWP2, const T* const stateWP3, T* residual) const {

        T dx1 = stateWP2[0] - stateWP1[0];
        T dy1 = stateWP2[1] - stateWP1[1];
        T dz1 = stateWP2[2] - stateWP1[2];
        T dx2 = stateWP3[0] - stateWP2[0];
        T dy2 = stateWP3[1] - stateWP2[1];
        T dz2 = stateWP3[2] - stateWP2[2];

        residual[0] = weight_* ((dx2 * dx2 + dy2 * dy2 + dz2 * dz2) - (dx1 * dx1 + dy1 * dy1 + dz1 * dz1));

        return true;
    }

    double weight_;
    
private:


};

#endif