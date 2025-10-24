#ifndef CERES_CONSTRAINTS_5_CONT_PATH_LEGTH_SEGMENT
#define CERES_CONSTRAINTS_5_CONT_PATH_LEGTH_SEGMENT

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

class Ceres5_PathLengthContSegmentFunctor {

public:
    Ceres5_PathLengthContSegmentFunctor(double weight, double s0, double s1): weight_(weight), s0_(s0), s1_(s1) {}

    template <typename T>
    bool operator()(const T* const stateCoeff, T* residual) const {

        auto t0 = std::chrono::high_resolution_clock::now();

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

        T x0 = p0x + s0_*(p1x + s0_*(p2x + s0_*(p3x + s0_*(p4x + s0_*p5x))));
        T y0 = p0y + s0_*(p1y + s0_*(p2y + s0_*(p3y + s0_*(p4y + s0_*p5y))));
        T z0 = p0z + s0_*(p1z + s0_*(p2z + s0_*(p3z + s0_*(p4z + s0_*p5z))));

        T x1 = p0x + s1_*(p1x + s1_*(p2x + s1_*(p3x + s1_*(p4x + s1_*p5x))));
        T y1 = p0y + s1_*(p1y + s1_*(p2y + s1_*(p3y + s1_*(p4y + s1_*p5y))));
        T z1 = p0z + s1_*(p1z + s1_*(p2z + s1_*(p3z + s1_*(p4z + s1_*p5z))));
   
        residual[0] = weight_ * (x1 - x0);
        residual[1] = weight_ * (y1 - y0);
        residual[2] = weight_ * (z1 - z0);

        auto t1 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> dt = t1 - t0;
        Planners::utils::safe_log("Cost Function - Path Length", dt.count());
        //std::cout << "[T] - Cost Function - Path Length: " << dt.count() << std::endl;

        return true;
    }

    double weight_, s0_, s1_;
    
private:


};

#endif