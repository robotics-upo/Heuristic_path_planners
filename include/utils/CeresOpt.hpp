#ifndef CERESOPT_HPP
#define CERESOPT_HPP

#include <iostream>
#include <Eigen/Dense>
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

#include "local_planner_optimizer/ceres_constraint_0_dist_to_obstacle.hpp"
#include "local_planner_optimizer/ceres_constraint_0_wp_equidistance.hpp"
#include "local_planner_optimizer/ceres_constraint_0_path_length.hpp"
#include "local_planner_optimizer/ceres_constraint_0_smoothness.hpp"


#include "local_planner_optimizer/ceres_constraint_1_dist_to_obstacle.hpp"
#include "local_planner_optimizer/ceres_constraint_1_wp_equidistance.hpp"
#include "local_planner_optimizer/ceres_constraint_1_path_length.hpp"
#include "local_planner_optimizer/ceres_constraint_1_smoothness.hpp"
#include "local_planner_optimizer/ceres_constraint_1_velocity_change.hpp"
#include "local_planner_optimizer/ceres_constraint_1_min_acceleration.hpp"
#include "local_planner_optimizer/ceres_constraint_1_pos_vel_coherence.hpp"


#include "local_planner_optimizer/ceres_constraint_2_cont_dist_to_obstacle_segment.hpp"
#include "local_planner_optimizer/ceres_constraint_2_cont_fix_goal.hpp"
#include "local_planner_optimizer/ceres_constraint_2_cont_path_length_segment.hpp"
#include "local_planner_optimizer/ceres_constraint_2_cont_smoothness.hpp"


#include "local_planner_optimizer/ceres_constraint_3_cont_dist_to_obstacle_segment_pc.hpp"
#include "local_planner_optimizer/ceres_constraint_3_cont_fix_goal.hpp"
#include "local_planner_optimizer/ceres_constraint_3_cont_path_length_segment.hpp"


#include "local_planner_optimizer/ceres_constraint_cont_init_distance_to_wp.hpp"
#include "local_planner_optimizer/ceres_constraint_cont_init_smoothness.hpp"

// Deprecated
#include "local_planner_optimizer/ceres_constraint_cont_path_length.hpp"
#include "local_planner_optimizer/ceres_constraint_cont_dist_to_obstacle.hpp"





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

struct parameterBlockContinuousPath{
    double parameter[15];
};

struct parameterBlockContinuousPathConstant{
    double parameter[3];
};

namespace Ceresopt{
    std::vector<double> InitVelCalculator(std::vector<parameterBlockTrajectoryWP> wp_state_vector, float total_travel_time, int num_wp, float res);
    Planners::utils::OptimizedPath ceresOptimizerPath(Planners::utils::CoordinateList initial_path, Local_Grid3d &_grid, float resolution_);
    Planners::utils::OptimizedTrajectory ceresOptimizerTrajectory(Planners::utils::CoordinateList initial_path, Local_Grid3d &_grid, float resolution_);
    Planners::utils::OptimizedContinuousFunction ceresOptimizerContinuousPath(Eigen::VectorXd coeff_x, Eigen::VectorXd coeff_y, Eigen::VectorXd coeff_z, Planners::utils::Vec3i local_goal, Local_Grid3d &_grid, float resolution_);
    Planners::utils::OptimizedContinuousFunction ceresOptimizerContinuousPathInit(Eigen::VectorXd init_coeff_x, Eigen::VectorXd init_coeff_y, Eigen::VectorXd init_coeff_z, Planners::utils::CoordinateList global_path_local_section, double t_last);

    Planners::utils::OptimizedContinuousFunction ceresOptimizerTestContinuousPath(Eigen::VectorXd coeff_x, Eigen::VectorXd coeff_y, Eigen::VectorXd coeff_z, Planners::utils::Vec3i local_goal, float resolution_, const std::vector<Eigen::Vector3d>& obstacle_point_cloud);

}


#endif // CERESOPT_HPP