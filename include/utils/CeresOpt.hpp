#ifndef CERESOPT_HPP
#define CERESOPT_HPP

#include <iostream>
#include "utils/ros/ROSInterfaces.hpp"
#include "utils/SaveDataVariantToFile.hpp"
#include "utils/misc.hpp"
#include "utils/geometry_utils.hpp"
#include "utils/metrics.hpp"
#include <ros/ros.h>
#include <chrono>

#include <heuristic_planners/Vec3i.h>
#include <heuristic_planners/CoordinateList.h>

#include "Grid3D/local_grid3d.hpp"

#include <ceres/ceres.h>

#include "local_planner_optimizer/ceres_constraint_dist_to_obstacle.hpp"
#include "local_planner_optimizer/ceres_constraint_wp_equidistance.hpp"
#include "local_planner_optimizer/ceres_constraint_path_length.hpp"
#include "local_planner_optimizer/ceres_constraint_smoothness.hpp"
#include "local_planner_optimizer/ceres_constraint_velocity_change.hpp"
#include "local_planner_optimizer/ceres_constraint_min_acceleration.hpp"
#include "local_planner_optimizer/ceres_constraint_pos_vel_coherence.hpp"


using ceres::AutoDiffCostFunction;
using ceres::NumericDiffCostFunction;
using ceres::CENTRAL;
using ceres::FORWARD;
using ceres::RIDDERS;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;
using ceres::LossFunction;
using ceres::CauchyLoss;

struct parameterBlockTrajectoryWP{
	double parameter[6];
};

struct parameterBlockPathWP{
	double parameter[3];
};

namespace Ceresopt{
    std::vector<double> InitVelCalculator(std::vector<parameterBlockTrajectoryWP> wp_state_vector, float total_travel_time, int num_wp, float res);
    Planners::utils::OptimizedPath ceresOptimizerPath(Planners::utils::CoordinateList initial_path, Local_Grid3d &_grid, float resolution_);
    Planners::utils::OptimizedTrajectory ceresOptimizerTrajectory(Planners::utils::CoordinateList initial_path, Local_Grid3d &_grid, float resolution_);
}


#endif // CERESOPT_HPP