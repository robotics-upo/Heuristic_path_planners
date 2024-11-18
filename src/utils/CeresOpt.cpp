#include "utils/CeresOpt.hpp"

namespace Ceresopt
{

    Planners::utils::CoordinateList ceresOptimizer(Planners::utils::CoordinateList initial_path, Local_Grid3d &_grid)
    {
        // Convert trajectory to ceres state wp vector
        std::vector<parameterBlockWP> wp_state_vector;
        int num_wp = initial_path.size();
        wp_state_vector.reserve(num_wp);

        for (const auto& point: initial_path){
            parameterBlockWP newWP;
            newWP.parameter[0]=static_cast<double>(point.x);
            newWP.parameter[1]=static_cast<double>(point.y);
            newWP.parameter[2]=static_cast<double>(point.z);
            wp_state_vector.push_back(newWP);
        }

        // Print the entire wp_state_vector
        for (size_t i = 0; i < wp_state_vector.size(); ++i) {
            const auto& wp = wp_state_vector[i];
            std::cout << "Waypoint " << i + 1 << ": "
                    << "x = " << wp.parameter[0] << ", "
                    << "y = " << wp.parameter[1] << ", "
                    << "z = " << wp.parameter[2] << std::endl;
        }


        // Declare Ceres optimization problem
        ceres::Problem problem;

        // Equidistance function target distance squared

        double dist_target = 0;
        for (int i=0; i < (num_wp-1); i++){
            double tdx = wp_state_vector[i+1].parameter[0] - wp_state_vector[i].parameter[0];
            double tdy = wp_state_vector[i+1].parameter[1] - wp_state_vector[i].parameter[1];
            double tdz = wp_state_vector[i+1].parameter[2] - wp_state_vector[i].parameter[2];

            dist_target += tdx * tdx + tdy * tdy + tdz * tdz;
        }
        dist_target = dist_target/(num_wp-1);

        // Cost function weights

        double weight_equidistance = 1.0;
        double weight_path_length = 1.0;
        double weight_esdf = 1.0;

        // Define cost functions

        for (int i = 0; i < wp_state_vector.size() - 1; i++){

            // 1 - Equidistance cost function
            ceres::CostFunction* equidistance_function = new AutoDiffCostFunction<EquidistanceFunctor, 1, 3, 3>
                                                        (new EquidistanceFunctor(dist_target, weight_equidistance));
            problem.AddResidualBlock(equidistance_function, nullptr, wp_state_vector[i].parameter, wp_state_vector[i+1].parameter);

            // 2 - Path length cost function
            ceres::CostFunction* path_length_function = new AutoDiffCostFunction<PathLengthFunctor, 1, 3, 3>
                                                        (new PathLengthFunctor(weight_path_length));;
            problem.AddResidualBlock(path_length_function, nullptr, wp_state_vector[i].parameter, wp_state_vector[i+1].parameter);
        }


        // 3 - Distance to obstacles cost function
        for (int i = 0; i < wp_state_vector.size(); i++)
        {
            ceres::CostFunction* esdf_function = new ObstacleDistanceCostFunctor(&_grid, weight_esdf);
            problem.AddResidualBlock(esdf_function, nullptr, wp_state_vector[i].parameter);

        }

        // Freeze first and last points
        problem.SetParameterBlockConstant(wp_state_vector[0].parameter);   // Freeze first wp
        problem.SetParameterBlockConstant(wp_state_vector[wp_state_vector.size()-1].parameter);   // Freeze last wp

        // Solve problem
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = true;
        options.max_num_iterations = 100;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        // std::cout << summary.FullReport() << "\n";

        // Print the entire wp_state_vector
        for (size_t i = 0; i < wp_state_vector.size(); ++i) {
            const auto& wp = wp_state_vector[i];
            std::cout << "New Waypoint " << i + 1 << ": "
                    << "x = " << wp.parameter[0] << ", "
                    << "y = " << wp.parameter[1] << ", "
                    << "z = " << wp.parameter[2] << std::endl;
        }

        // Update the path with optimized values
        Planners::utils::CoordinateList optimized_path;
        for (int i=0; i<num_wp; i++){
            Planners::utils::Vec3i newpoint;
            newpoint.x = wp_state_vector[i].parameter[0];
            newpoint.y = wp_state_vector[i].parameter[1];
            newpoint.z = wp_state_vector[i].parameter[2];
            optimized_path.push_back(newpoint);
        }

        return optimized_path;
    }
}

