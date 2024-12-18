#ifndef CERES_CONSTRAINTS_VELOCITY_CHANGE
#define CERES_CONSTRAINTS_VELOCITY_CHANGE

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

class VelocityChangeFunctor {

public:
    VelocityChangeFunctor(double weight, double desired_vel): weight_(weight), desired_vel_(desired_vel) {}

    template <typename T>
    bool operator()(const T* const stateWP1, T* residual) const {

        T vx = stateWP1[3];
        T vy = stateWP1[4];
        T vz = stateWP1[5];


        residual[0] = weight_* (T(desired_vel_) - ceres::sqrt(vx * vx + vy * vy + vz * vz)) * (T(desired_vel_)  - ceres::sqrt(vx * vx + vy * vy + vz * vz));

        return true;
    }

    double weight_;
    double desired_vel_;
    
private:


};

#endif