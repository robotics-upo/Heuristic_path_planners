#ifndef CERES_CONSTRAINTS_7_CONT_PATH_LEGTH
#define CERES_CONSTRAINTS_7_CONT_PATH_LEGTH

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
#include <array>
#include <cmath>

#include <heuristic_planners/Vec3i.h>
#include <heuristic_planners/CoordinateList.h>

#include "Grid3D/local_grid3d.hpp"

#include <ceres/ceres.h>


using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

class Ceres7_PathLengthGaussLegendreFunctor {
public:
    Ceres7_PathLengthGaussLegendreFunctor(double weight)
        : weight_(weight) {}

    template <typename T>
    bool operator()(const T* const stateCoeff, T* residual) const {

        T p0x = stateCoeff[5] - stateCoeff[3] + stateCoeff[1];
        T p1x = stateCoeff[4] - 3.0*stateCoeff[2] + 5.0*stateCoeff[0];
        T p2x = 2.0*stateCoeff[3] - 8.0*stateCoeff[1];
        T p3x = 4.0*stateCoeff[2] - 20.0*stateCoeff[0];
        T p4x = 8.0*stateCoeff[1];
        T p5x = 16.0*stateCoeff[0];

        T p0y = stateCoeff[11] - stateCoeff[9] + stateCoeff[7];
        T p1y = stateCoeff[10] - 3.0*stateCoeff[8] + 5.0*stateCoeff[6];
        T p2y = 2.0*stateCoeff[9] - 8.0*stateCoeff[7];
        T p3y = 4.0*stateCoeff[8] - 20.0*stateCoeff[6];
        T p4y = 8.0*stateCoeff[7];
        T p5y = 16.0*stateCoeff[6];

        T p0z = stateCoeff[17] - stateCoeff[15] + stateCoeff[13];
        T p1z = stateCoeff[16] - 3.0*stateCoeff[14] + 5.0*stateCoeff[12];
        T p2z = 2.0*stateCoeff[15] - 8.0*stateCoeff[13];
        T p3z = 4.0*stateCoeff[14] - 20.0*stateCoeff[12];
        T p4z = 8.0*stateCoeff[13];
        T p5z = 16.0*stateCoeff[12];

        constexpr std::array<double,5> xi = {
            -0.9061798459386640,
            -0.5384693101056831,
             0.0,
             0.5384693101056831,
             0.9061798459386640
        };
        constexpr std::array<double,5> wi = {
            0.2369268850561891,
            0.4786286704993665,
            0.5688888888888889,
            0.4786286704993665,
            0.2369268850561891
        };

        T length = T(0);
        for (int i = 0; i < 5; ++i) {
            T s = T(xi[i]);

            // Horner derivatives
            T dx = p1x + s*(2.0*p2x + s*(3.0*p3x + s*(4.0*p4x + s*(5.0*p5x))));
            T dy = p1y + s*(2.0*p2y + s*(3.0*p3y + s*(4.0*p4y + s*(5.0*p5y))));
            T dz = p1z + s*(2.0*p2z + s*(3.0*p3z + s*(4.0*p4z + s*(5.0*p5z))));

            T integrand = ceres::sqrt(dx*dx + dy*dy + dz*dz);

            // Gauss–Legendre
            length += T(wi[i]) * integrand;
        }

        residual[0] = weight_ * length;

        return true;
    }

    double weight_;
};

#endif