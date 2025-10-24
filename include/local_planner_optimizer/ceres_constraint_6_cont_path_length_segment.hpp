#ifndef CERES_CONSTRAINTS_6_CONT_PATH_LEGTH_SEGMENT
#define CERES_CONSTRAINTS_6_CONT_PATH_LEGTH_SEGMENT

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

class Ceres6_PathLengthContSegmentFunctor {

public:
    Ceres6_PathLengthContSegmentFunctor(double weight, double s0, double s1, Planners::utils::Vec3i local_start, Planners::utils::Vec3i local_goal): weight_(weight), s0_(s0), s1_(s1), local_start_(local_start), local_goal_(local_goal) {}

    template <typename T>
    bool operator()(const T* const stateCoeff, T* residual) const {

        T p0x = 0.5*(local_goal_.x + local_start_.x) - stateCoeff[1];
        T p1x = 2.5*(local_goal_.x - local_start_.x) - 8.0*stateCoeff[0] - 4.0*stateCoeff[2];
        T p2x = -4.0*(local_goal_.x + local_start_.x) + 10.0*stateCoeff[1] + 8.0*stateCoeff[3];
        T p3x = -10.0*(local_goal_.x - local_start_.x) + 24.0*stateCoeff[0] + 20.0*stateCoeff[2];
        T p4x = 4.0*(local_goal_.x + local_start_.x) - 8.0*stateCoeff[1] - 8.0*stateCoeff[3];
        T p5x = 8.0*(local_goal_.x - local_start_.x) - 16.0*stateCoeff[0] - 16.0*stateCoeff[2];
        T p0y = 0.5*(local_goal_.y + local_start_.y) - stateCoeff[5];
        T p1y = 2.5*(local_goal_.y - local_start_.y) - 8.0*stateCoeff[4] - 4.0*stateCoeff[6];
        T p2y = -4.0*(local_goal_.y + local_start_.y) + 10.0*stateCoeff[5] + 8.0*stateCoeff[7];
        T p3y = -10.0*(local_goal_.y - local_start_.y) + 24.0*stateCoeff[4] + 20.0*stateCoeff[6];
        T p4y = 4.0*(local_goal_.y + local_start_.y) - 8.0*stateCoeff[5] - 8.0*stateCoeff[7];
        T p5y = 8.0*(local_goal_.y - local_start_.y) - 16.0*stateCoeff[4] - 16.0*stateCoeff[6];
        T p0z = 0.5*(local_goal_.z + local_start_.z) - stateCoeff[9];
        T p1z = 2.5*(local_goal_.z - local_start_.z) - 8.0*stateCoeff[8] - 4.0*stateCoeff[10];
        T p2z = -4.0*(local_goal_.z + local_start_.z) + 10.0*stateCoeff[9] + 8.0*stateCoeff[11];
        T p3z = -10.0*(local_goal_.z - local_start_.z) + 24.0*stateCoeff[8] + 20.0*stateCoeff[10];
        T p4z = 4.0*(local_goal_.z + local_start_.z) - 8.0*stateCoeff[9] - 8.0*stateCoeff[11];
        T p5z = 8.0*(local_goal_.z - local_start_.z) - 16.0*stateCoeff[8] - 16.0*stateCoeff[10];

        T x0 = p0x + s0_*(p1x + s0_*(p2x + s0_*(p3x + s0_*(p4x + s0_*p5x))));
        T y0 = p0y + s0_*(p1y + s0_*(p2y + s0_*(p3y + s0_*(p4y + s0_*p5y))));
        T z0 = p0z + s0_*(p1z + s0_*(p2z + s0_*(p3z + s0_*(p4z + s0_*p5z))));

        T x1 = p0x + s1_*(p1x + s1_*(p2x + s1_*(p3x + s1_*(p4x + s1_*p5x))));
        T y1 = p0y + s1_*(p1y + s1_*(p2y + s1_*(p3y + s1_*(p4y + s1_*p5y))));
        T z1 = p0z + s1_*(p1z + s1_*(p2z + s1_*(p3z + s1_*(p4z + s1_*p5z))));
   
        residual[0] = weight_ * (x1 - x0);
        residual[1] = weight_ * (y1 - y0);
        residual[2] = weight_ * (z1 - z0);

        return true;
    }

    double weight_, s0_, s1_;
    Planners::utils::Vec3i local_start_, local_goal_;
    
private:


};

#endif