#ifndef CERES_CONSTRAINTS_MIN_ACCELERATION
#define CERES_CONSTRAINTS_MIN_ACCELERATION

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

class MinAccelerationFunctor { // Minimiza la variación de la dirección de la velocidad (que mide la aceleración a módulo de la velocidad constante) ponderado con la distancia del desplazamiento entre WP

public:
    MinAccelerationFunctor(double weight): weight_(weight) {}

    template <typename T>
    bool operator()(const T* const stateWP1, const T* const stateWP2, T* residual) const {

        // Compute both vectors and the dot product
        T vel_start[3] = {stateWP1[3], stateWP1[4], stateWP1[5]};
        T vel_end[3] = {stateWP2[3], stateWP2[4], stateWP2[5]};
        T dot_product = (vel_end[0] * vel_start[0]) + (vel_end[1] * vel_start[1]) + (vel_end[2] * vel_start[2]);

        // Compute vector norms
		T arg1 = (vel_start[0] * vel_start[0]) + (vel_start[1] * vel_start[1]) + (vel_start[2] * vel_start[2]);
		T arg2 = (vel_end[0] * vel_end[0]) + (vel_end[1] * vel_end[1]) + (vel_end[2] * vel_end[2]);
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

        // Calculate position variation
        T displacement[3] = {stateWP2[0] - stateWP1[0], stateWP2[1] - stateWP1[1], stateWP2[2] - stateWP1[2]};
        T displacement_mod = ceres::sqrt(displacement[0] * displacement[0] + displacement[1] * displacement[1] + displacement[2] * displacement[2]);
        if (displacement_mod < 0.0001)
            displacement_mod = T{0.0001};

        residual[0] = weight_ * (min_cos_residual + ((cos_angle - min_expected_cos) * (max_cos_residual - min_cos_residual) / (max_expected_cos - min_expected_cos))) / displacement_mod;
        
        return true;
    }

    double weight_;
    
private:


};

#endif