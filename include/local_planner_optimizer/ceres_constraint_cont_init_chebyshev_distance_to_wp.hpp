#ifndef CERES_CONSTRAINTS_CONT_INIT_CHEBYSHEV_DISTANCE_TO_WP
#define CERES_CONSTRAINTS_CONT_INIT_CHEBYSHEV_DISTANCE_TO_WP

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
#include <vector>
#include <limits>


using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

class DistanceToWPChebyshevFunctor {
public:
    DistanceToWPChebyshevFunctor(double weight,
                                 const Planners::utils::CoordinateList& global_path_local_section,
                                 int f_segments)
        : weight_(weight),
          global_path_local_section_(global_path_local_section),
          f_segments_(f_segments) {}

    template <typename T>
    bool operator()(const T* const stateCoeff, T* residual) const {
        // --- 1. Calculate x, y, z coeff ---
        T p0x = stateCoeff[5] - stateCoeff[3] + stateCoeff[1];
        T p1x = stateCoeff[4] - T(3.0)*stateCoeff[2] + T(5.0)*stateCoeff[0];
        T p2x = T(2.0)*stateCoeff[3] - T(8.0)*stateCoeff[1];
        T p3x = T(4.0)*stateCoeff[2] - T(20.0)*stateCoeff[0];
        T p4x = T(8.0)*stateCoeff[1];
        T p5x = T(16.0)*stateCoeff[0];

        T p0y = stateCoeff[11] - stateCoeff[9] + stateCoeff[7];
        T p1y = stateCoeff[10] - T(3.0)*stateCoeff[8] + T(5.0)*stateCoeff[6];
        T p2y = T(2.0)*stateCoeff[9] - T(8.0)*stateCoeff[7];
        T p3y = T(4.0)*stateCoeff[8] - T(20.0)*stateCoeff[6];
        T p4y = T(8.0)*stateCoeff[7];
        T p5y = T(16.0)*stateCoeff[6];

        T p0z = stateCoeff[17] - stateCoeff[15] + stateCoeff[13];
        T p1z = stateCoeff[16] - T(3.0)*stateCoeff[14] + T(5.0)*stateCoeff[12];
        T p2z = T(2.0)*stateCoeff[15] - T(8.0)*stateCoeff[13];
        T p3z = T(4.0)*stateCoeff[14] - T(20.0)*stateCoeff[12];
        T p4z = T(8.0)*stateCoeff[13];
        T p5z = T(16.0)*stateCoeff[12];

        // --- 2. Evaluate all the points ---
        std::vector<T> traj_x(f_segments_), traj_y(f_segments_), traj_z(f_segments_);
        traj_x.reserve(f_segments_);
        traj_y.reserve(f_segments_);
        traj_z.reserve(f_segments_);

        for (int i = 0; i < f_segments_; ++i) {
            T s = T(-1.0) + T(2.0) * T(i) / T(f_segments_ - 1);
            traj_x[i] = p0x + s*(p1x + s*(p2x + s*(p3x + s*(p4x + s*p5x))));
            traj_y[i] = p0y + s*(p1y + s*(p2y + s*(p3y + s*(p4y + s*p5y))));
            traj_z[i] = p0z + s*(p1z + s*(p2z + s*(p3z + s*(p4z + s*p5z))));
        }

        // --- 3. Compare for each wp and add to the residual ---
        T total_cost = T(0.0);

        for (size_t wp_idx = 0; wp_idx < global_path_local_section_.size(); ++wp_idx) {
            const auto& wp = global_path_local_section_[wp_idx];
            T min_dist2 = T(1e12);

            for (int i = 0; i < f_segments_; ++i) {
                T dx = traj_x[i] - T(wp.x);
                T dy = traj_y[i] - T(wp.y);
                T dz = traj_z[i] - T(wp.z);
                T dist2 = dx*dx + dy*dy + dz*dz;
                if (dist2 < min_dist2)
                    min_dist2 = dist2;
            }
            total_cost += min_dist2;
        }

        residual[0] = weight_ * total_cost;

        return true;
    }

private:
    double weight_;
    int f_segments_;
    Planners::utils::CoordinateList global_path_local_section_;
};


#endif