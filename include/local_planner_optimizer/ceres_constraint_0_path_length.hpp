#ifndef CERES_CONSTRAINTS_0_PATH_LEGTH
#define CERES_CONSTRAINTS_0_PATH_LEGTH

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

class Ceres0_PathLengthFunctor {

public:
    Ceres0_PathLengthFunctor(double weight): weight_(weight) {}

    template <typename T>
    bool operator()(const T* const stateWP1, const T* const stateWP2, T* residual) const {

        T dx = stateWP2[0] - stateWP1[0];
        T dy = stateWP2[1] - stateWP1[1];
        T dz = stateWP2[2] - stateWP1[2];

        residual[0] = weight_* (dx * dx + dy * dy + dz * dz);

        return true;
    }

    double weight_;
    
private:


};

#endif