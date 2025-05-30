#ifndef CERES_CONSTRAINTS_1_POS_VEL_COHERENCE
#define CERES_CONSTRAINTS_1_POS_VEL_COHERENCE

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

class Ceres1_PosVelCoherenceFunctor {

public:
    Ceres1_PosVelCoherenceFunctor(double weight): weight_(weight) {}

    template <typename T>
    bool operator()(const T* const stateWP1, const T* const stateWP2, T* residual) const {

        // Compute both vectors and the dot product
        T vel_total[3] = {stateWP1[3] + stateWP2[3], stateWP1[4] + stateWP2[4], stateWP1[5] + stateWP2[5]};
        T pos_dif[3] = {stateWP2[0] - stateWP1[0], stateWP2[1] - stateWP1[1], stateWP2[2] - stateWP1[2]};
        T dot_product = (vel_total[0] * pos_dif[0]) + (vel_total[1] * pos_dif[1]) + (vel_total[2] * pos_dif[2]);

        // Compute vector norms
		T arg1 = (vel_total[0] * vel_total[0]) + (vel_total[1] * vel_total[1]) + (vel_total[2] * vel_total[2]);
		T arg2 = (pos_dif[0] * pos_dif[0]) + (pos_dif[1] * pos_dif[1]) + (pos_dif[2] * pos_dif[2]);
		T norm_vector1, norm_vector2, cos_angle;
		
		if (arg1 < 0.0001 && arg1 > -0.0001)
			norm_vector1 = T{0.0};
		else
			norm_vector1 = ceres::sqrt(arg1);

		if (arg2 < 0.0001 && arg2 > -0.0001)
			norm_vector2 = T{0.0};
		else
			norm_vector2 = ceres::sqrt(arg2);

        // Compute cos(angle)
        if (norm_vector1 < 0.0001 || norm_vector2 < 0.0001)
            cos_angle = T{0.0};
        else
            cos_angle = dot_product/(norm_vector1 * norm_vector2);

        // Calculate residual
        T min_expected_cos = T{-1.0};
        T max_expected_cos = T{1.0};
        T min_cos_residual = T{20.0};
        T max_cos_residual = T{0.0}; // We want the angle to be 0 -> Residual is 0 when cos(angle) = 1

        residual[0] = weight_ * (min_cos_residual + ((cos_angle - min_expected_cos) * (max_cos_residual - min_cos_residual) / (max_expected_cos - min_expected_cos)));
        
        return true;
    }

    double weight_;
    
private:


};

#endif