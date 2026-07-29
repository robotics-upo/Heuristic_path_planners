#ifndef CERES_CONSTRAINTS_7_CONT_TRAJ_TIME
#define CERES_CONSTRAINTS_7_CONT_TRAJ_TIME

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

class Ceres7_TrajTimeFunctor {

public:
    explicit Ceres7_TrajTimeFunctor(double weight)
        : weight_(weight) {}

    template <typename T>
    bool operator()(const T* const stateCoeff, T* residual) const {

        residual[0] = T(weight_) * stateCoeff[18];

        return true;
    }

    double weight_;

private:

};

#endif
