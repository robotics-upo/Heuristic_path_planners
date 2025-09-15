#ifndef CERES_CONSTRAINTS_4_CONT_SMOOTHNESS
#define CERES_CONSTRAINTS_4_CONT_SMOOTHNESS

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

class Ceres4_SmoothnessContFunctor {

public:
    Ceres4_SmoothnessContFunctor(double weight, int n_samples, int sample): weight_(weight), n_samples_(n_samples), sample_(sample) {}

    template <typename T>
    bool operator()(const T* const stateCoeff, T* residual) const {

        if (n_samples_ == 1)
        {
            T accel_act = T(0);

            for (int i = 0; i < 3; i++) { // x, y, z
                T a5 = stateCoeff[5*i + 0];
                T a4 = stateCoeff[5*i + 1];
                T a3 = stateCoeff[5*i + 2];
                T a2 = stateCoeff[5*i + 3];
                T p_ddot = 20.0*a5*a5 + 12.0*a4*a4 + 6.0*a3*a3 + 2.0*a2*a2;
                accel_act += p_ddot;
            }

            residual[0] = weight_ * accel_act;
        }
        else
        {
            T t_act = T(sample_ + 1)/T(n_samples_);
            T accel_act = T(0);

            for (int i = 0; i < 3; i++) { // x, y, z
                T a5 = stateCoeff[5*i + 0];
                T a4 = stateCoeff[5*i + 1];
                T a3 = stateCoeff[5*i + 2];
                T a2 = stateCoeff[5*i + 3];
                T p_ddot = 20.0*a5*t_act*t_act*t_act + 12.0*a4*t_act*t_act + 6.0*a3*t_act + 2.0*a2;
                accel_act += p_ddot * p_ddot;
            }

            residual[0] = weight_ / T(n_samples_) * accel_act;
        }
        

        

        return true;
    }

    double weight_;
    int n_samples_, sample_;
    
private:


};

#endif