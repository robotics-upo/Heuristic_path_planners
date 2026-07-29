#ifndef CERES_CONSTRAINTS_5_CONT_DYNAMIC_LIMITS
#define CERES_CONSTRAINTS_5_CONT_DYNAMIC_LIMITS

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

// Hard constraint: limits velocity and acceleration norms along the Chebyshev trajectory.
// One instance per sample point s_sample in (-1, 1). Instantiate in a loop in
// ceresOptimizerChebyshevContinuousPath, one per sample point.
//
// Residuals (2 total):
//   residual[0] = weight * max(0, ||vel(s)|| - max_vel)   -- velocity hinge loss
//   residual[1] = weight * max(0, ||acc(s)|| - max_acc)   -- acceleration hinge loss
//
// stateCoeff layout (18 elements, shared with all Ceres5 functors):
//   [0..5]   -> Chebyshev coefficients c5..c0 for x(s)
//   [6..11]  -> Chebyshev coefficients c5..c0 for y(s)
//   [12..17] -> Chebyshev coefficients c5..c0 for z(s)
//
// Monomial form: x(s) = p0 + p1*s + p2*s^2 + p3*s^3 + p4*s^4 + p5*s^5
//   velocity:     x'(s)  = p1 + s*(2*p2 + s*(3*p3 + s*(4*p4 + s*(5*p5))))
//   acceleration: x''(s) = 2*p2 + s*(6*p3 + s*(12*p4 + s*(20*p5)))
//
// Units: cells per s-unit (s in [-1,1]) — same as the vel_x/y/z_ini parameters
// defined alongside this constraint in ceresOptimizerChebyshevContinuousPath.

class Ceres5_DynamicLimitsContFunctor {

public:
    Ceres5_DynamicLimitsContFunctor(double weight, double s_sample, double max_vel, double max_acc)
        : weight_(weight), s_sample_(s_sample), max_vel_(max_vel), max_acc_(max_acc) {}

    template <typename T>
    bool operator()(const T* const stateCoeff, T* residual) const {

        // Chebyshev-to-monomial conversion (p0 not needed for derivatives)
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

        T s = T(s_sample_);

        // First derivative (velocity): x'(s), y'(s), z'(s)
        T vel_x = p1x + s*(2.0*p2x + s*(3.0*p3x + s*(4.0*p4x + s*(5.0*p5x))));
        T vel_y = p1y + s*(2.0*p2y + s*(3.0*p3y + s*(4.0*p4y + s*(5.0*p5y))));
        T vel_z = p1z + s*(2.0*p2z + s*(3.0*p3z + s*(4.0*p4z + s*(5.0*p5z))));

        // Second derivative (acceleration): x''(s), y''(s), z''(s)
        T acc_x = 2.0*p2x + s*(6.0*p3x + s*(12.0*p4x + s*(20.0*p5x)));
        T acc_y = 2.0*p2y + s*(6.0*p3y + s*(12.0*p4y + s*(20.0*p5y)));
        T acc_z = 2.0*p2z + s*(6.0*p3z + s*(12.0*p4z + s*(20.0*p5z)));

        // 3D norms (small epsilon avoids sqrt(0) gradient singularity)
        T vel_norm = ceres::sqrt(vel_x*vel_x + vel_y*vel_y + vel_z*vel_z + T(1e-10));
        T acc_norm = ceres::sqrt(acc_x*acc_x + acc_y*acc_y + acc_z*acc_z + T(1e-10));

        // Hinge-loss residuals: zero when within limit, grows linearly when violated
        T vel_violation = vel_norm - T(max_vel_);
        T acc_violation = acc_norm - T(max_acc_);

        residual[0] = T(weight_) * (vel_violation > T(0.0) ? vel_violation : T(0.0));
        residual[1] = T(weight_) * (acc_violation > T(0.0) ? acc_violation : T(0.0));

        return true;
    }

    double weight_, s_sample_, max_vel_, max_acc_;

private:

};

#endif
