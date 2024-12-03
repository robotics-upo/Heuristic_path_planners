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

struct parameterBlockWP{
	double parameter[3];
};

namespace Ceresopt{
    Planners::utils::CoordinateList ceresOptimizer(Planners::utils::CoordinateList initial_path, Local_Grid3d &_grid);
}


#endif // CERESOPT_HPP