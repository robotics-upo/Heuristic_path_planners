#ifndef CERES_CONSTRAINTS_5_CONT_FIX_INITIAL_VELOCITY
#define CERES_CONSTRAINTS_5_CONT_FIX_INITIAL_VELOCITY

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

class Ceres5_FixInitialVelocityContFunctor {

public:
    Ceres5_FixInitialVelocityContFunctor(double weight, double current_vel_x, double current_vel_y, double current_vel_z): weight_(weight), current_vel_x_(current_vel_x), current_vel_y_(current_vel_y), current_vel_z_(current_vel_z) {}

    template <typename T>
    bool operator()(const T* const stateCoeff, T* residual) const {

        auto t0 = std::chrono::high_resolution_clock::now();

        // Current velocity (T)
        T cvx = T(current_vel_x_);
        T cvy = T(current_vel_y_);
        T cvz = T(current_vel_z_);

        //  First derivative in start (s=-1), which is the current velocity for next iteration
        T vel_x_ini = 25.0*stateCoeff[0] - 16.0*stateCoeff[1] + 9.0*stateCoeff[2] - 4.0*stateCoeff[3] + stateCoeff[4];
        T vel_y_ini = 25.0*stateCoeff[6] - 16.0*stateCoeff[7] + 9.0*stateCoeff[8] - 4.0*stateCoeff[9] + stateCoeff[10];
        T vel_z_ini = 25.0*stateCoeff[12] - 16.0*stateCoeff[13] + 9.0*stateCoeff[14] - 4.0*stateCoeff[15] + stateCoeff[16];

        // We penalize difference in DIRECTION between both velocities
        T norm_current = sqrt(cvx*cvx + cvy*cvy + cvz*cvz);
        T norm_ini = sqrt(vel_x_ini*vel_x_ini + vel_y_ini*vel_y_ini + vel_z_ini*vel_z_ini);

        // Avoid numerical errors with small norms
        if (norm_current < T(1e-6) || norm_ini < T(1e-6)) {
        residual[0] = T(0.0);
        return true;
        }

        // Dot product
        T dot = (cvx * vel_x_ini +
                cvy * vel_y_ini +
                cvz * vel_z_ini) / (norm_current * norm_ini);

        // Residual: 0 when aligned, 2*weight when completly missaligned
        residual[0] = weight_ * (T(1.0) - dot);

        auto t1 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> dt = t1 - t0;
        //Planners::utils::safe_log("Cost Function - Fix Initial Velocity", dt.count());
        //std::cout << "[T] - Cost Function - Fix Initial Velocity: " << dt.count() << std::endl;
    
        return true;
    }

    double weight_, current_vel_x_, current_vel_y_, current_vel_z_;    
private:


};

#endif