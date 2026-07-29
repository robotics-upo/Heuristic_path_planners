#ifndef CERES_CONSTRAINTS_7_CONT_GOAL_DYNAMIC_STATE
#define CERES_CONSTRAINTS_7_CONT_GOAL_DYNAMIC_STATE

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


class Ceres7_GoalDynamicStateFunctor {

public:
    Ceres7_GoalDynamicStateFunctor(
        double weight,
        double v_x_ms, double v_y_ms, double v_z_ms,
        double a_x_ms2, double a_y_ms2, double a_z_ms2,
        double resolution)
        : weight_(weight),
          v_x_ms_(v_x_ms), v_y_ms_(v_y_ms), v_z_ms_(v_z_ms),
          a_x_ms2_(a_x_ms2), a_y_ms2_(a_y_ms2), a_z_ms2_(a_z_ms2),
          resolution_(resolution) {}

    template <typename T>
    bool operator()(const T* const stateCoeff, T* residual) const {

        T traj_T = stateCoeff[18];

        T p1x = stateCoeff[4] - 3.0*stateCoeff[2] + 5.0*stateCoeff[0];
        T p2x = 2.0*stateCoeff[3] - 8.0*stateCoeff[1];
        T p3x = 4.0*stateCoeff[2] - 20.0*stateCoeff[0];
        T p4x = 8.0*stateCoeff[1];
        T p5x = 16.0*stateCoeff[0];

        T p1y = stateCoeff[10] - 3.0*stateCoeff[8] + 5.0*stateCoeff[6];
        T p2y = 2.0*stateCoeff[9] - 8.0*stateCoeff[7];
        T p3y = 4.0*stateCoeff[8] - 20.0*stateCoeff[6];
        T p4y = 8.0*stateCoeff[7];
        T p5y = 16.0*stateCoeff[6];

        T p1z = stateCoeff[16] - 3.0*stateCoeff[14] + 5.0*stateCoeff[12];
        T p2z = 2.0*stateCoeff[15] - 8.0*stateCoeff[13];
        T p3z = 4.0*stateCoeff[14] - 20.0*stateCoeff[12];
        T p4z = 8.0*stateCoeff[13];
        T p5z = 16.0*stateCoeff[12];

        // Dimensionless derivatives at s = +1 (trajectory goal)

        T vel_x_dimless = p1x + 2.0*p2x + 3.0*p3x + 4.0*p4x + 5.0*p5x;
        T vel_y_dimless = p1y + 2.0*p2y + 3.0*p3y + 4.0*p4y + 5.0*p5y;
        T vel_z_dimless = p1z + 2.0*p2z + 3.0*p3z + 4.0*p4z + 5.0*p5z;

        T acc_x_dimless = 2.0*p2x + 6.0*p3x + 12.0*p4x + 20.0*p5x;
        T acc_y_dimless = 2.0*p2y + 6.0*p3y + 12.0*p4y + 20.0*p5y;
        T acc_z_dimless = 2.0*p2z + 6.0*p3z + 12.0*p4z + 20.0*p5z;

        // Target dimensionless derivatives derived from real-world goal values and T
        //   v_dimless_target = v_goal_real * T / (2 * resolution)
        //   a_dimless_target = a_goal_real * T^2 / (4 * resolution)
        T inv_res = T(1.0 / resolution_);
        T vel_scale = traj_T * T(0.5) * inv_res;
        T acc_scale = traj_T * traj_T * T(0.25) * inv_res;

        T vel_x_target = T(v_x_ms_) * vel_scale;
        T vel_y_target = T(v_y_ms_) * vel_scale;
        T vel_z_target = T(v_z_ms_) * vel_scale;

        T acc_x_target = T(a_x_ms2_) * acc_scale;
        T acc_y_target = T(a_y_ms2_) * acc_scale;
        T acc_z_target = T(a_z_ms2_) * acc_scale;

        residual[0] = T(weight_) * (vel_x_dimless - vel_x_target);
        residual[1] = T(weight_) * (vel_y_dimless - vel_y_target);
        residual[2] = T(weight_) * (vel_z_dimless - vel_z_target);
        residual[3] = T(weight_) * (acc_x_dimless - acc_x_target);
        residual[4] = T(weight_) * (acc_y_dimless - acc_y_target);
        residual[5] = T(weight_) * (acc_z_dimless - acc_z_target);

        return true;
    }

    double weight_;
    double v_x_ms_, v_y_ms_, v_z_ms_;
    double a_x_ms2_, a_y_ms2_, a_z_ms2_;
    double resolution_;

private:

};

#endif
