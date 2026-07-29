#include "utils/CeresOpt.hpp"

namespace Ceresopt
{
    std::vector<double> InitVelCalculator(std::vector<parameterBlockTrajectoryWP> wp_state_vector, double desired_vel, int num_wp, float res){
        // Compute initial velocity module
        // double vini_module = 0;
        // for (size_t i = 0; i < num_wp - 1; ++i){
        //     double total_dist_cell = sqrt((wp_state_vector[i+1].parameter[0]-wp_state_vector[i].parameter[0]) * (wp_state_vector[i+1].parameter[0]-wp_state_vector[i].parameter[0]) + 
        //     (wp_state_vector[i+1].parameter[1]-wp_state_vector[i].parameter[1]) * (wp_state_vector[i+1].parameter[1]-wp_state_vector[i].parameter[1]) + 
        //     (wp_state_vector[i+1].parameter[2]-wp_state_vector[i].parameter[2]) * (wp_state_vector[i+1].parameter[2]-wp_state_vector[i].parameter[2]));
        //     vini_module += total_dist_cell;
        // }
        // vini_module = vini_module / total_travel_time;

        double vini_module = desired_vel / res;

        // Convert to vectorized velocities
        std::vector<double> initial_vel_vector;
        initial_vel_vector.reserve(num_wp*3);

        double single_path_length = sqrt((wp_state_vector[1].parameter[0]-wp_state_vector[0].parameter[0]) * (wp_state_vector[1].parameter[0]-wp_state_vector[0].parameter[0]) + 
            (wp_state_vector[1].parameter[1]-wp_state_vector[0].parameter[1]) * (wp_state_vector[1].parameter[1]-wp_state_vector[0].parameter[1]) + 
            (wp_state_vector[1].parameter[2]-wp_state_vector[0].parameter[2]) * (wp_state_vector[1].parameter[2]-wp_state_vector[0].parameter[2]));
        initial_vel_vector[0] = vini_module * (wp_state_vector[1].parameter[0]-wp_state_vector[0].parameter[0]) / single_path_length;
        initial_vel_vector[1] = vini_module * (wp_state_vector[1].parameter[1]-wp_state_vector[0].parameter[1]) / single_path_length;
        initial_vel_vector[2] = vini_module * (wp_state_vector[1].parameter[2]-wp_state_vector[0].parameter[2]) / single_path_length;
        std::cout << "single_path_length = " << single_path_length << std::endl;

        for (size_t i = 1; i < num_wp -1; ++i){
            double single_path_length = sqrt((wp_state_vector[i+1].parameter[0]-wp_state_vector[i-1].parameter[0]) * (wp_state_vector[i+1].parameter[0]-wp_state_vector[i-1].parameter[0]) + 
                (wp_state_vector[i+1].parameter[1]-wp_state_vector[i-1].parameter[1]) * (wp_state_vector[i+1].parameter[1]-wp_state_vector[i-1].parameter[1]) + 
                (wp_state_vector[i+1].parameter[2]-wp_state_vector[i-1].parameter[2]) * (wp_state_vector[i+1].parameter[2]-wp_state_vector[i-1].parameter[2]));
            initial_vel_vector[3*i] = vini_module * (wp_state_vector[i+1].parameter[0]-wp_state_vector[i-1].parameter[0]) / single_path_length;
            initial_vel_vector[3*i+1] = vini_module * (wp_state_vector[i+1].parameter[1]-wp_state_vector[i-1].parameter[1]) / single_path_length;
            initial_vel_vector[3*i+2] = vini_module * (wp_state_vector[i+1].parameter[2]-wp_state_vector[i-1].parameter[2]) / single_path_length;
        }

        initial_vel_vector[3*num_wp - 3] = 0;
        initial_vel_vector[3*num_wp - 2] = 0; 
        initial_vel_vector[3*num_wp - 1] = 0;

        std::cout << "Calculated vel vector: " << initial_vel_vector << std::endl;
        
        return initial_vel_vector;

    }

    Planners::utils::OptimizedPath ceresOptimizerPath(Planners::utils::CoordinateList initial_path, Local_Grid3d &_grid, float res)
    {
        // Convert trajectory to ceres state wp vector
        std::vector<parameterBlockPathWP> wp_state_vector;
        int num_wp = initial_path.size();
        wp_state_vector.reserve(num_wp);

        for (const auto& point: initial_path){
            parameterBlockPathWP newWP;
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

        std::cout << "Created Ceres Problem" << std::endl;

       
        // Cost function weights

        double weight_equidistance = 1.0;
        double weight_path_length = 1.0;
        double weight_esdf = 10.0;
        double weight_smoothness = 1.0;

        // Define cost functions

        // 1/2 - Equidistance cost function + Smoothness cost function --> Tries to maintain equal distance between WP and avoid big changes in direction
        for (size_t i = 0; i < wp_state_vector.size() - 2; i++)
        {
            ceres::CostFunction* equidistance_function = new AutoDiffCostFunction<Ceres0_EquidistanceFunctor, 1, 3, 3, 3>
                                                        (new Ceres0_EquidistanceFunctor(weight_equidistance));
            ceres::CostFunction* smoothness_function = new AutoDiffCostFunction<Ceres0_SmoothnessFunctor, 1, 3, 3, 3>
                                                        (new Ceres0_SmoothnessFunctor(weight_smoothness));
            problem.AddResidualBlock(equidistance_function, nullptr, wp_state_vector[i].parameter, wp_state_vector[i+1].parameter, wp_state_vector[i+2].parameter);
            problem.AddResidualBlock(smoothness_function, nullptr, wp_state_vector[i].parameter, wp_state_vector[i+1].parameter, wp_state_vector[i+2].parameter);
        }

        // 3 - Path length cost function --> Tries to minimize path length
        for (size_t i = 0; i < wp_state_vector.size() - 1; i++)
        {
            ceres::CostFunction* path_length_function = new AutoDiffCostFunction<Ceres0_PathLengthFunctor, 1, 3, 3>
                                                        (new Ceres0_PathLengthFunctor(weight_path_length));
            problem.AddResidualBlock(path_length_function, nullptr, wp_state_vector[i].parameter, wp_state_vector[i+1].parameter);
        }


        // 4 - Distance to obstacles cost function --> Tries to maintain the biggest distance to obstacles possible
        for (size_t i = 0; i < wp_state_vector.size(); i++)
        {
            ceres::CostFunction* esdf_function = new Ceres0_ObstacleDistanceCostFunctor(&_grid, weight_esdf);
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
        options.num_threads = 12;

        std::cout << "Configured options" << std::endl;
        
        ceres::Solver::Summary summary;

        auto start_opt = std::chrono::high_resolution_clock::now();
        std::cout << "Starting solver" << std::endl;
        ceres::Solve(options, &problem, &summary);
        std::cout << "Exiting solver" << std::endl;
        auto end_opt = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> opt_duration = end_opt - start_opt;
        printf("TIEMPO DE OPTIMIZACIÓN: %.2f ms\n", opt_duration.count());


        // Print the entire wp_state_vector
        for (size_t i = 0; i < wp_state_vector.size(); ++i) {
            const auto& wp = wp_state_vector[i];
            std::cout << "New Waypoint " << i + 1 << ": "
                    << "x = " << wp.parameter[0] << ", "
                    << "y = " << wp.parameter[1] << ", "
                    << "z = " << wp.parameter[2] << std::endl;
        }

        // Update the path with optimized values
        std::cout << "Updating optimized path vector" << std::endl;
        Planners::utils::OptimizedPath optimized_path;

        for (int i=0; i<num_wp; i++)
        {
            Planners::utils::Vec3i newpoint_position;
            newpoint_position.x = wp_state_vector[i].parameter[0];
            newpoint_position.y = wp_state_vector[i].parameter[1];
            newpoint_position.z = wp_state_vector[i].parameter[2];
            optimized_path.positions.push_back(newpoint_position);
        }

        std::cout << "Returning to main function" << std::endl;
        return optimized_path;
    }

    Planners::utils::OptimizedTrajectory ceresOptimizerTrajectory(Planners::utils::CoordinateList initial_path, Local_Grid3d &_grid, float res)
    {
        // Convert trajectory to ceres state wp vector
        std::vector<parameterBlockTrajectoryWP> wp_state_vector;
        int num_wp = initial_path.size();
        wp_state_vector.reserve(num_wp);

        for (const auto& point: initial_path){
            parameterBlockTrajectoryWP newWP;
            newWP.parameter[0]=static_cast<double>(point.x);
            newWP.parameter[1]=static_cast<double>(point.y);
            newWP.parameter[2]=static_cast<double>(point.z);
            for(size_t i = 3; i < 6; ++i){
                newWP.parameter[i] = 0;
            }
            wp_state_vector.push_back(newWP);
        }
        
        // Initialize velocity (VELOCITIES IN STATE VECTOR ARE CELL VELOCITIES!!!)
            // Decide desired velocity for the entire path
        double desired_vel = 0.5; // (m/s)
            // Compute initial velocities
        std::vector<double> inivel_vector = InitVelCalculator(wp_state_vector, desired_vel, num_wp, res);
            // Assing the velocities to the correct state vector positions
        for (size_t i = 0; i < wp_state_vector.size(); i++) {
            wp_state_vector[i].parameter[3] = inivel_vector[i*3];
            wp_state_vector[i].parameter[4] = inivel_vector[i*3+1];
            wp_state_vector[i].parameter[5] = inivel_vector[i*3+2];
        }


        // Print the entire wp_state_vector
        for (size_t i = 0; i < wp_state_vector.size(); ++i) {
            const auto& wp = wp_state_vector[i];
            std::cout << "Waypoint " << i + 1 << ": "
                    << "x = " << wp.parameter[0] << ", "
                    << "y = " << wp.parameter[1] << ", "
                    << "z = " << wp.parameter[2] << ", "
                    << "vx = " << wp.parameter[3] << ", "
                    << "vy = " << wp.parameter[4] << ", "
                    << "vz = " << wp.parameter[5] << std::endl;
        }


        // Declare Ceres optimization problem
        ceres::Problem problem;

        std::cout << "Created Ceres Problem" << std::endl;

        // // Equidistance function target distance squared

        // double dist_target = 0;
        // for (int i=0; i < (num_wp-1); i++){
        //     double tdx = wp_state_vector[i+1].parameter[0] - wp_state_vector[i].parameter[0];
        //     double tdy = wp_state_vector[i+1].parameter[1] - wp_state_vector[i].parameter[1];
        //     double tdz = wp_state_vector[i+1].parameter[2] - wp_state_vector[i].parameter[2];

        //     dist_target += tdx * tdx + tdy * tdy + tdz * tdz;
        // }
        // dist_target = dist_target/(num_wp-1);

        // Cost function weights

        double weight_equidistance = 10.0;
        double weight_path_length = 10.0;
        double weight_esdf = 100.0;
        double weight_smoothness = 10.0;
        double weight_velocity_module = 6.0;
        double weight_min_acceleration = 1.0;
        double weight_pos_vel_coherence = 6.0;

        // Define cost functions

        // 1/2 - Equidistance cost function + Smoothness cost function --> Tries to maintain equal distance between WP and avoid big changes in direction
        for (size_t i = 0; i < wp_state_vector.size() - 2; i++)
        {
            ceres::CostFunction* equidistance_function = new AutoDiffCostFunction<Ceres1_EquidistanceFunctor, 1, 6, 6, 6>
                                                        (new Ceres1_EquidistanceFunctor(weight_equidistance));
            ceres::CostFunction* smoothness_function = new AutoDiffCostFunction<Ceres1_SmoothnessFunctor, 1, 6, 6, 6>
                                                        (new Ceres1_SmoothnessFunctor(weight_smoothness));
            problem.AddResidualBlock(equidistance_function, nullptr, wp_state_vector[i].parameter, wp_state_vector[i+1].parameter, wp_state_vector[i+2].parameter);
            problem.AddResidualBlock(smoothness_function, nullptr, wp_state_vector[i].parameter, wp_state_vector[i+1].parameter, wp_state_vector[i+2].parameter);
        }

        // 3 - Path length cost function --> Tries to minimize path length
        for (size_t i = 0; i < wp_state_vector.size() - 1; i++)
        {
            ceres::CostFunction* path_length_function = new AutoDiffCostFunction<Ceres1_PathLengthFunctor, 1, 6, 6>
                                                        (new Ceres1_PathLengthFunctor(weight_path_length));
            problem.AddResidualBlock(path_length_function, nullptr, wp_state_vector[i].parameter, wp_state_vector[i+1].parameter);
        }


        // 4 - Distance to obstacles cost function --> Tries to maintain the biggest distance to obstacles
        for (size_t i = 0; i < wp_state_vector.size(); i++)
        {
            ceres::CostFunction* esdf_function = new Ceres1_ObstacleDistanceCostFunctor(&_grid, weight_esdf);
            problem.AddResidualBlock(esdf_function, nullptr, wp_state_vector[i].parameter);

        }


        // 5 - Velocity module function --> Tries to maintain the desired velocity module
        for (size_t i = 0; i < wp_state_vector.size() - 1; i++)
        {
            ceres::CostFunction* velocity_change_function = new AutoDiffCostFunction<Ceres1_VelocityChangeFunctor, 1, 6>
                                                        (new Ceres1_VelocityChangeFunctor(weight_velocity_module, desired_vel));
            problem.AddResidualBlock(velocity_change_function, nullptr, wp_state_vector[i].parameter);
        }

        // 6 - Minimize acceleration function --> Tries to minimize acceleration along the trajectory

        for (size_t i = 0; i < wp_state_vector.size() - 2; i++)
        {
            ceres::CostFunction* min_acceleration_function = new AutoDiffCostFunction<Ceres1_MinAccelerationFunctor, 1, 6, 6>
                                                        (new Ceres1_MinAccelerationFunctor(weight_min_acceleration));
            problem.AddResidualBlock(min_acceleration_function, nullptr, wp_state_vector[i].parameter, wp_state_vector[i+1].parameter);
        }

        // 7 - Position-velocity coherence function --> Maintains the position-velocity direction coherence

        for (size_t i = 0; i < wp_state_vector.size() - 2; i++)
        {
            ceres::CostFunction* pos_vel_coherence_function = new AutoDiffCostFunction<Ceres1_PosVelCoherenceFunctor, 1, 6, 6>
                                                        (new Ceres1_PosVelCoherenceFunctor(weight_pos_vel_coherence));
            problem.AddResidualBlock(pos_vel_coherence_function, nullptr, wp_state_vector[i].parameter, wp_state_vector[i+1].parameter);
        }

        // Freeze first and last points
        problem.SetParameterBlockConstant(wp_state_vector[0].parameter);   // Freeze first wp
        problem.SetParameterBlockConstant(wp_state_vector[wp_state_vector.size()-1].parameter);   // Freeze last wp

        // Solve problem
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = true;
        options.max_num_iterations = 100;
        options.num_threads = 12;

        std::cout << "Configured options" << std::endl;
        
        ceres::Solver::Summary summary;

        auto start_opt = std::chrono::high_resolution_clock::now();
        std::cout << "Starting solver" << std::endl;
        ceres::Solve(options, &problem, &summary);
        std::cout << "Exiting solver" << std::endl;
        auto end_opt = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> opt_duration = end_opt - start_opt;
        printf("TIEMPO DE OPTIMIZACIÓN: %.2f ms\n", opt_duration.count());


        // std::cout << summary.FullReport() << "\n";

        // Print the entire wp_state_vector
        for (size_t i = 0; i < wp_state_vector.size(); ++i) {
            const auto& wp = wp_state_vector[i];
            std::cout << "New Waypoint " << i + 1 << ": "
                    << "x = " << wp.parameter[0] << ", "
                    << "y = " << wp.parameter[1] << ", "
                    << "z = " << wp.parameter[2] << ", "
                    << "vx = " << wp.parameter[3] << ", "
                    << "vy = " << wp.parameter[4] << ", "
                    << "vz = " << wp.parameter[5] << std::endl;
        }

        // Update the path with optimized values
        std::cout << "Updating optimized path vector" << std::endl;
        Planners::utils::OptimizedTrajectory optimized_path;

        for (int i=0; i<num_wp; i++)
        {
            Planners::utils::Vec3i newpoint_position;
            newpoint_position.x = wp_state_vector[i].parameter[0];
            newpoint_position.y = wp_state_vector[i].parameter[1];
            newpoint_position.z = wp_state_vector[i].parameter[2];
            optimized_path.positions.push_back(newpoint_position);

            Planners::utils::Vec3i newpoint_velocities;
            newpoint_velocities.x = wp_state_vector[i].parameter[3];
            newpoint_velocities.y = wp_state_vector[i].parameter[4];
            newpoint_velocities.z = wp_state_vector[i].parameter[5];
            optimized_path.velocities.push_back(newpoint_velocities);
        }

        std::cout << "Returning to main function" << std::endl;
        return optimized_path;
    }

    Planners::utils::OptimizedContinuousFunction ceresOptimizerContinuousPath(Eigen::VectorXd coeff_x, Eigen::VectorXd coeff_y, Eigen::VectorXd coeff_z, Planners::utils::Vec3i local_goal, Local_Grid3d &_grid, float resolution_)
    {
        // Convert function coeffs to state block (excluding the last one, that's fixed by the starting point)
        parameterBlockContinuousPath coeff_state_vector;
        parameterBlockContinuousPathConstant coeff_state_vector_constant;
        for (int i = 0; i < 3; i++) {
            coeff_state_vector.parameter[i] = coeff_x[i];
            coeff_state_vector.parameter[i + 3] = coeff_y[i];
            coeff_state_vector.parameter[i + 6] = coeff_z[i];
        }
        
        coeff_state_vector_constant.parameter[0] = coeff_x[3];
        coeff_state_vector_constant.parameter[1] = coeff_y[3];
        coeff_state_vector_constant.parameter[2] = coeff_z[3];

        // Declare Ceres optimization problem
        ceres::Problem problem;

        std::cout << "Created Ceres Problem" << std::endl;

        // Cost function weights

        double weight_path_length = 1.0;
        double weight_esdf = 1000.0;
        double weight_smoothness = 1.0;
        double weight_fix_goal = 2.0;

        // 1 - Path length cost function

        int path_length_seg = 10;
        double t_max_seg = 10.0;

        for(int i = 0; i < path_length_seg; i++)
        {
            double t0 = t_max_seg * i / path_length_seg;
            double t1 = t_max_seg * (i+1) / path_length_seg;

            ceres::CostFunction* path_length_cont_function_seg = new AutoDiffCostFunction<Ceres2_PathLengthContSegmentFunctor, 1, 9, 3>(new Ceres2_PathLengthContSegmentFunctor(weight_path_length, t0, t1));
        
            problem.AddResidualBlock(path_length_cont_function_seg, nullptr, coeff_state_vector.parameter, coeff_state_vector_constant.parameter);
        }

        // 2 - ESDF cost function

        int esdf_seg = 20;
        double t_max_esdf_seg = 10.0;

        for(int i = 1; i < esdf_seg; i++)
        {
            double t_esdf = t_max_esdf_seg * i / esdf_seg;

            ceres::CostFunction* esdf_cont_function_seg = new AutoDiffCostFunction<Ceres2_ObstacleDistanceCostContSegmentFunctor, 1, 9, 3>(new Ceres2_ObstacleDistanceCostContSegmentFunctor(_grid, t_esdf, esdf_seg, weight_esdf));

            problem.AddResidualBlock(esdf_cont_function_seg, nullptr, coeff_state_vector.parameter, coeff_state_vector_constant.parameter);
        }

        // // 3 - Smoothness cost function (by minimizing coeffs)

        // ceres::CostFunction* smoothness_cont_function = new AutoDiffCostFunction<Ceres2_SmoothnessContFunctor, 1, 9>(new Ceres2_SmoothnessContFunctor(weight_smoothness));
    
        // problem.AddResidualBlock(smoothness_cont_function, nullptr, coeff_state_vector.parameter);

        // 4 - Fixed local goal function (high weight needed)

        ceres::CostFunction* fixed_goal_cont_function = new AutoDiffCostFunction<Ceres2_FixGoalContFunctor, 1, 9, 3>(new Ceres2_FixGoalContFunctor(weight_fix_goal, local_goal));
        
        problem.AddResidualBlock(fixed_goal_cont_function, nullptr, coeff_state_vector.parameter, coeff_state_vector_constant.parameter);


        // Freeze independent coeffs

        problem.SetParameterBlockConstant(coeff_state_vector_constant.parameter);


        // Configure problem
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        //options.linear_solver_type = ceres::DENSE_SCHUR;
        options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        options.minimizer_progress_to_stdout = false;
        options.max_num_iterations = 40;
        options.num_threads = 14;
        options.use_nonmonotonic_steps = true;


        std::cout << "Configured options" << std::endl;
        
        ceres::Solver::Summary summary;

        // // Test 1
        // ceres::Problem::EvaluateOptions eval_options;
        // eval_options.apply_loss_function = false;  // Evaluar sin la función de pérdida

        // double total_cost = 0.0;
        // std::vector<double> test_residuals;

        // problem.Evaluate(eval_options, &total_cost, &test_residuals, nullptr, nullptr);

        // std::cout << "Costo antes de la optimización: " << total_cost << std::endl;

        // for (size_t i = 0; i < test_residuals.size(); ++i) {
        //     std::cout << "Residual " << i << ": " << test_residuals[i] << std::endl;
        // }


        // Solve

        auto start_opt = std::chrono::high_resolution_clock::now();
        std::cout << "Starting solver" << std::endl;
        ceres::Solve(options, &problem, &summary);
        std::cout << "Exiting solver" << std::endl;
        auto end_opt = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> opt_duration = end_opt - start_opt;
        printf("TIEMPO DE OPTIMIZACIÓN: %.2f ms\n", opt_duration.count());
        
        // Building the output
        std::cout << "Building output" << std::endl;
        Planners::utils::OptimizedContinuousFunction optimized_coeffs;

        // Resize
        optimized_coeffs.x_params.resize(4);
        optimized_coeffs.y_params.resize(4);
        optimized_coeffs.z_params.resize(4);

        // // Test 2
        // total_cost = 0.0;
        // problem.Evaluate(eval_options, &total_cost, &test_residuals, nullptr, nullptr);

        // std::cout << "Costo total después de la optimización: " << total_cost << std::endl;

        // for (size_t i = 0; i < test_residuals.size(); ++i) {
        //     std::cout << "Residual " << i << ": " << test_residuals[i] << std::endl;
        // }


        for (int i = 0; i < 3; i++) {
            optimized_coeffs.x_params[i] = coeff_state_vector.parameter[i];
            optimized_coeffs.y_params[i] = coeff_state_vector.parameter[i + 3];
            optimized_coeffs.z_params[i] = coeff_state_vector.parameter[i + 6];
        }
        optimized_coeffs.x_params[3] = coeff_state_vector_constant.parameter[0];
        optimized_coeffs.y_params[3] = coeff_state_vector_constant.parameter[1];
        optimized_coeffs.z_params[3] = coeff_state_vector_constant.parameter[2];

        std::cout << "Returning to main function" << std::endl;

        return optimized_coeffs;
    }

    Planners::utils::OptimizedContinuousFunction ceresOptimizerTestContinuousPath(Eigen::VectorXd coeff_x, Eigen::VectorXd coeff_y, Eigen::VectorXd coeff_z, Planners::utils::Vec3i local_goal, float resolution_, const std::vector<Eigen::Vector3d>& obstacle_point_cloud)
    {
        // Convert function coeffs to state block (excluding the last one, that's fixed by the starting point)
        parameterBlockContinuousPath coeff_state_vector;
        parameterBlockContinuousPathConstant coeff_state_vector_constant;
        for (int i = 0; i < 3; i++) {
            coeff_state_vector.parameter[i] = coeff_x[i];
            coeff_state_vector.parameter[i + 3] = coeff_y[i];
            coeff_state_vector.parameter[i + 6] = coeff_z[i];
        }
        
        coeff_state_vector_constant.parameter[0] = coeff_x[3];
        coeff_state_vector_constant.parameter[1] = coeff_y[3];
        coeff_state_vector_constant.parameter[2] = coeff_z[3];

        // Declare Ceres optimization problem
        ceres::Problem problem;

        std::cout << "Created Ceres Problem" << std::endl;

        // Cost function weights

        double weight_path_length = 2.0;
        double weight_esdf = 500.0;
        double weight_smoothness = 1.0;
        double weight_fix_goal = 3.0;

        // 1 - Path length cost function

        int path_length_seg = 10;
        double t_max_seg = 10.0;

        for(int i = 0; i < path_length_seg; i++)
        {
            double t0 = t_max_seg * i / path_length_seg;
            double t1 = t_max_seg * (i+1) / path_length_seg;

            ceres::CostFunction* path_length_cont_function_seg = new AutoDiffCostFunction<Ceres3_PathLengthContSegmentFunctor, 1, 9, 3>(new Ceres3_PathLengthContSegmentFunctor(weight_path_length, t0, t1));
        
            problem.AddResidualBlock(path_length_cont_function_seg, nullptr, coeff_state_vector.parameter, coeff_state_vector_constant.parameter);
        }

        // 2 - ESDF cost function

        int esdf_seg = 20;
        double t_max_esdf_seg = 10.0;

        //Max distancia en el grid local (automatizar en un futuro?) (EN UNIDADES DE CELDA)
        double max_local_dist = sqrt(30*30 + 30*30 + 16*16);

        for(int i = 1; i < esdf_seg; i++)
        {
            double t_esdf = t_max_esdf_seg * i / esdf_seg;

            ceres::CostFunction* esdf_cont_function_seg = new AutoDiffCostFunction<Ceres3_PCObstacleDistanceCostContSegmentFunctor, 1, 9, 3>(new Ceres3_PCObstacleDistanceCostContSegmentFunctor(t_esdf, esdf_seg, max_local_dist, obstacle_point_cloud, weight_esdf));

            problem.AddResidualBlock(esdf_cont_function_seg, nullptr, coeff_state_vector.parameter, coeff_state_vector_constant.parameter);
        }

        // 3 - Fixed local goal function (high weight needed)

        ceres::CostFunction* fixed_goal_cont_function = new AutoDiffCostFunction<Ceres3_FixGoalContFunctor, 1, 9, 3>(new Ceres3_FixGoalContFunctor(weight_fix_goal, local_goal));
        
        problem.AddResidualBlock(fixed_goal_cont_function, nullptr, coeff_state_vector.parameter, coeff_state_vector_constant.parameter);


        // Freeze independent coeffs

        problem.SetParameterBlockConstant(coeff_state_vector_constant.parameter);


        // Configure problem
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        //options.linear_solver_type = ceres::DENSE_SCHUR;
        options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        options.minimizer_progress_to_stdout = true;
        options.max_num_iterations = 5000;
        options.num_threads = 12;
        options.use_nonmonotonic_steps = true;


        std::cout << "Configured options" << std::endl;
        
        ceres::Solver::Summary summary;

        // Test 1
        ceres::Problem::EvaluateOptions eval_options;
        eval_options.apply_loss_function = false;  // Evaluar sin la función de pérdida

        double total_cost = 0.0;
        std::vector<double> test_residuals;

        problem.Evaluate(eval_options, &total_cost, &test_residuals, nullptr, nullptr);

        std::cout << "Costo antes de la optimización: " << total_cost << std::endl;

        for (size_t i = 0; i < test_residuals.size(); ++i) {
            std::cout << "Residual " << i << ": " << test_residuals[i] << std::endl;
        }


        // Solve

        auto start_opt = std::chrono::high_resolution_clock::now();
        std::cout << "Starting solver" << std::endl;
        ceres::Solve(options, &problem, &summary);
        std::cout << "Exiting solver" << std::endl;
        auto end_opt = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> opt_duration = end_opt - start_opt;
        printf("TIEMPO DE OPTIMIZACIÓN: %.2f ms\n", opt_duration.count());

        // Building the output
        std::cout << "Building output" << std::endl;
        Planners::utils::OptimizedContinuousFunction optimized_coeffs;

        // Resize
        optimized_coeffs.x_params.resize(4);
        optimized_coeffs.y_params.resize(4);
        optimized_coeffs.z_params.resize(4);

        // Test 2
        total_cost = 0.0;
        problem.Evaluate(eval_options, &total_cost, &test_residuals, nullptr, nullptr);

        std::cout << "Costo total después de la optimización: " << total_cost << std::endl;

        for (size_t i = 0; i < test_residuals.size(); ++i) {
            std::cout << "Residual " << i << ": " << test_residuals[i] << std::endl;
        }


        for (int i = 0; i < 3; i++) {
            optimized_coeffs.x_params[i] = coeff_state_vector.parameter[i];
            optimized_coeffs.y_params[i] = coeff_state_vector.parameter[i + 3];
            optimized_coeffs.z_params[i] = coeff_state_vector.parameter[i + 6];
        }
        optimized_coeffs.x_params[3] = coeff_state_vector_constant.parameter[0];
        optimized_coeffs.y_params[3] = coeff_state_vector_constant.parameter[1];
        optimized_coeffs.z_params[3] = coeff_state_vector_constant.parameter[2];

        std::cout << "Returning to main function" << std::endl;

        return optimized_coeffs;
    }

    Planners::utils::OptimizedContinuousFunction ceresOptimizerEvCallbackContinuousPath(Eigen::VectorXd coeff_x, Eigen::VectorXd coeff_y, Eigen::VectorXd coeff_z, double origin_x, double origin_y, double origin_z, Planners::utils::Vec3i local_goal, Local_Grid3d &_grid, torch::jit::script::Module& loaded_sdf, float resolution_, std::shared_ptr<voxblox::EsdfMap>& esdf_map_, bool use_voxfield)
    {

        // Convert function coeffs to state block (excluding the last one, that's fixed by the starting point)
        parameterBlockContinuousPath coeff_state_vector;
        parameterBlockContinuousPathConstant coeff_state_vector_constant;
        for (int i = 0; i < 5; i++) {
            coeff_state_vector.parameter[i] = coeff_x[i];
            coeff_state_vector.parameter[i + 5] = coeff_y[i];
            coeff_state_vector.parameter[i + 10] = coeff_z[i];
        }
        
        coeff_state_vector_constant.parameter[0] = coeff_x[5];
        coeff_state_vector_constant.parameter[1] = coeff_y[5];
        coeff_state_vector_constant.parameter[2] = coeff_z[5];

        // Declare Ceres optimization problem
        int path_length_seg = 5; // Segments for path length calculation
        int esdf_samp = 10; // Samples for ESDF calculation
        //int smoothness_samp = 1; // Samples for smoothness calculation (if samples = 1, take absolute value of coeffs)

        double t_max_seg = 1.0; // t_max for path length calculation (TODO -- MERGE WITH ALL T)
        double t_max_esdf_seg = 1.0; // t_max por esdf calculation (TODO -- MERGE WITH ALL T)

        
        CeresESDFUpdate evaluation_callback(coeff_state_vector, coeff_state_vector_constant, esdf_samp, t_max_esdf_seg, loaded_sdf, origin_x, origin_y, origin_z, resolution_, esdf_map_, use_voxfield);
        ceres::Problem problem;
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        options.minimizer_progress_to_stdout = false;
        options.function_tolerance  = 1e-4;
        options.gradient_tolerance  = 1e-6;
        options.parameter_tolerance = 1e-6;
        options.max_solver_time_in_seconds = 250e-3;

        options.max_num_iterations = 50;
        options.num_threads = 12;
        options.use_nonmonotonic_steps = true;
        options.evaluation_callback = &evaluation_callback;



        // std::cout << "Configured options" << std::endl;

        // std::cout << "Created Ceres Problem" << std::endl;

        // Cost function weights
        double weight_path_length = 10.0;
        double weight_esdf = 3.0; 
        double weight_smoothness = 0.1;
        double weight_fix_goal = 1e4;

        // 1 - Path length cost function

        for(int i = 0; i < path_length_seg; i++)
        {
            double t0 = t_max_seg * i / path_length_seg;
            double t1 = t_max_seg * (i+1) / path_length_seg;

            ceres::CostFunction* path_length_cont_function_seg = new AutoDiffCostFunction<Ceres4_PathLengthContSegmentFunctor, 3, 15, 3>(new Ceres4_PathLengthContSegmentFunctor(weight_path_length, t0, t1));
        
            problem.AddResidualBlock(path_length_cont_function_seg, nullptr, coeff_state_vector.parameter, coeff_state_vector_constant.parameter);
        }

        // 2 - ESDF cost function

        for(int i = 0; i < esdf_samp - 1; i++)
        {
            double t_esdf = t_max_esdf_seg * (i + 1) / (esdf_samp + 1);

            ceres::CostFunction* esdf_cont_function_seg = new AutoDiffCostFunction<Ceres4_ObstacleDistanceCostContSegmentFunctor, 1, 15, 3>(new Ceres4_ObstacleDistanceCostContSegmentFunctor(evaluation_callback, i, t_esdf, esdf_samp, weight_esdf));

            problem.AddResidualBlock(esdf_cont_function_seg, nullptr, coeff_state_vector.parameter, coeff_state_vector_constant.parameter);
        }

        // 3 - Smoothness cost function (by minimizing coeffs)

        // for(int i = 0; i < smoothness_samp; i++)
        // {
        //     ceres::CostFunction* smoothness_cont_function = new AutoDiffCostFunction<Ceres4_SmoothnessContFunctor, 1, 15>(new Ceres4_SmoothnessContFunctor(weight_smoothness, smoothness_samp, i));
    
        //     problem.AddResidualBlock(smoothness_cont_function, nullptr, coeff_state_vector.parameter);
        // }
        ceres::CostFunction* smoothness_cont_function = new AutoDiffCostFunction<Ceres4_SmoothnessContFunctor, 12, 15>(new Ceres4_SmoothnessContFunctor(weight_smoothness));

        problem.AddResidualBlock(smoothness_cont_function, nullptr, coeff_state_vector.parameter);


        // 4 - Fixed local goal function (high weight needed)

        ceres::CostFunction* fixed_goal_cont_function = new AutoDiffCostFunction<Ceres4_FixGoalContFunctor, 3, 15, 3>(new Ceres4_FixGoalContFunctor(weight_fix_goal, local_goal));
        
        problem.AddResidualBlock(fixed_goal_cont_function, nullptr, coeff_state_vector.parameter, coeff_state_vector_constant.parameter);


        // Freeze independent coeffs

        problem.SetParameterBlockConstant(coeff_state_vector_constant.parameter);

        
        ceres::Solver::Summary summary;

        // // Test 1
        // ceres::Problem::EvaluateOptions eval_options;
        // eval_options.apply_loss_function = false;  // Evaluar sin la función de pérdida

        // double total_cost = 0.0;
        // std::vector<double> test_residuals;

        // problem.Evaluate(eval_options, &total_cost, &test_residuals, nullptr, nullptr);

        // std::cout << "Costo antes de la optimización: " << total_cost << std::endl;

        // for (size_t i = 0; i < test_residuals.size(); ++i) {
        //     std::cout << "Residual " << i << ": " << test_residuals[i] << std::endl;
        // }
        //

        // Solve

        auto start_opt = std::chrono::high_resolution_clock::now();
        //std::cout << "Starting solver" << std::endl;
        ceres::Solve(options, &problem, &summary);
        //std::cout << "Exiting solver" << std::endl;
        auto end_opt = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> opt_duration = end_opt - start_opt;
        std::cout << summary.BriefReport() << "\n";
        printf("TIEMPO DE OPTIMIZACIÓN (MONOMIAL): %.2f ms\n", opt_duration.count());

        // Building the output
        // std::cout << "Building output" << std::endl;
        Planners::utils::OptimizedContinuousFunction optimized_coeffs;

        // Resize
        optimized_coeffs.x_params.resize(6);
        optimized_coeffs.y_params.resize(6);
        optimized_coeffs.z_params.resize(6);

        // //Test 2
        // total_cost = 0.0;
        // problem.Evaluate(eval_options, &total_cost, &test_residuals, nullptr, nullptr);

        // //std::cout << "Costo total después de la optimización: " << total_cost << std::endl;

        // for (size_t i = 0; i < test_residuals.size(); ++i) {
        //     std::cout << "Residual " << i << ": " << test_residuals[i] << std::endl;
        // }
        
        // int test_residuals_size = test_residuals.size();
        // double dist_to_goal_x = test_residuals[test_residuals_size-3] * resolution_ / weight_fix_goal;
        // double dist_to_goal_y = test_residuals[test_residuals_size-2] * resolution_ / weight_fix_goal;
        // double dist_to_goal_z = test_residuals[test_residuals_size-1] * resolution_ / weight_fix_goal;
        // std::cout << "Dist to goal: " << sqrt(dist_to_goal_x * dist_to_goal_x + dist_to_goal_y * dist_to_goal_y + dist_to_goal_z * dist_to_goal_z) << std::endl;
        

        for (int i = 0; i < 5; i++) {
            optimized_coeffs.x_params[i] = coeff_state_vector.parameter[i];
            optimized_coeffs.y_params[i] = coeff_state_vector.parameter[i + 5];
            optimized_coeffs.z_params[i] = coeff_state_vector.parameter[i + 10];
        }
        optimized_coeffs.x_params[5] = coeff_state_vector_constant.parameter[0];
        optimized_coeffs.y_params[5] = coeff_state_vector_constant.parameter[1];
        optimized_coeffs.z_params[5] = coeff_state_vector_constant.parameter[2];

        //std::cout << "Returning to main function" << std::endl;

        return optimized_coeffs;
    }

    Planners::utils::OptimizedContinuousFunction ceresOptimizerChebyshevContinuousPath(Eigen::VectorXd coeff_x, Eigen::VectorXd coeff_y, Eigen::VectorXd coeff_z, double origin_x, double origin_y, double origin_z, Planners::utils::Vec3i local_start, Planners::utils::Vec3i local_goal, Local_Grid3d &_grid, torch::jit::script::Module& loaded_sdf, float resolution_, std::shared_ptr<voxblox::EsdfMap>& esdf_map_, bool use_voxfield)
    {

        // Convert function coeffs to state block (excluding the last one, that's fixed by the starting point)
        parameterBlockChebyshev coeff_state_vector;
        for (int i = 0; i < 6; i++) {
            coeff_state_vector.parameter[i] = coeff_x[i];
            coeff_state_vector.parameter[i + 6] = coeff_y[i];
            coeff_state_vector.parameter[i + 12] = coeff_z[i];
        }

        // Declare Ceres optimization problem
        //int path_length_seg = 5; // Segments for path length calculation
        int esdf_samp = 20; // Samples for ESDF calculation
        
        CeresESDFUpdateChebyshev evaluation_callback_cheb(coeff_state_vector, esdf_samp, loaded_sdf, origin_x, origin_y, origin_z, resolution_, esdf_map_, use_voxfield);
        ceres::Problem problem;
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        // options.function_tolerance  = 1e-4;
        // options.gradient_tolerance  = 1e-6;
        // options.parameter_tolerance = 1e-6;
        options.max_solver_time_in_seconds = 500e-3;
        options.minimizer_progress_to_stdout = false;
        options.max_num_iterations = 200;
        options.num_threads = 12;
        options.use_nonmonotonic_steps = true;
        options.evaluation_callback = &evaluation_callback_cheb;

        ceres::Solver::Summary summary;

        // Cost function weights
        double weight_path_length = 100.0;
        double weight_esdf = 2500.0; //50
        double weight_smoothness = 2.0; //10
        double weight_fix_goal = 5e4;
        double weight_fix_initial_velocity = 5e4;
        //double weight_limit_goal_gradient = 5e4;
        double weight_dynamic_limits = 5e4;


        std::vector<std::pair<std::string, int>> cost_blocks;


        // 1 - Path length cost function

        ceres::CostFunction* path_length_cont_function = new AutoDiffCostFunction<Ceres5_PathLengthGaussLegendreFunctor, 1, 18>(new Ceres5_PathLengthGaussLegendreFunctor(weight_path_length));
        
        problem.AddResidualBlock(path_length_cont_function, nullptr, coeff_state_vector.parameter);

        cost_blocks.push_back({"Path length", 1});


        // 2 - ESDF cost function

        double weighted_weight_esdf = weight_esdf / esdf_samp;

        for(int i = 0; i < esdf_samp; i++)
        {
            double s_esdf = 2.0 * (i + 1.0) / (esdf_samp + 1.0) - 1.0;

            ceres::CostFunction* esdf_cont_function_seg = new AutoDiffCostFunction<Ceres5_ObstacleDistanceCostContSegmentFunctor, 1, 18>(new Ceres5_ObstacleDistanceCostContSegmentFunctor(evaluation_callback_cheb, i, s_esdf, esdf_samp, weight_esdf));

            problem.AddResidualBlock(esdf_cont_function_seg, nullptr, coeff_state_vector.parameter);

            cost_blocks.push_back({"ESDF", 1});
        }


        // 3 - Smoothness cost function (by minimizing coeffs)

        ceres::CostFunction* smoothness_cont_function = new AutoDiffCostFunction<Ceres5_SmoothnessContFunctor, 12, 18>(new Ceres5_SmoothnessContFunctor(weight_smoothness));

        problem.AddResidualBlock(smoothness_cont_function, nullptr, coeff_state_vector.parameter);

        cost_blocks.push_back({"Smoothness", 12});


        // 4 - Fixed local start and goal function (hard restriction)

        ceres::CostFunction* fixed_startgoal_cont_function = new AutoDiffCostFunction<Ceres5_FixStartGoalContFunctor, 6, 18>(new Ceres5_FixStartGoalContFunctor(weight_fix_goal, local_start, local_goal));
        
        problem.AddResidualBlock(fixed_startgoal_cont_function, nullptr, coeff_state_vector.parameter);

        cost_blocks.push_back({"Fixed start/goal", 6});


        // 5 - Velocity continuity (fix initial velocity to current velocity) (hard restriction)

        double vel_x_ini = 0.0;
        double vel_y_ini = 0.0;
        double vel_z_ini = 0.0;

        ceres::CostFunction* fix_initial_velocity_cont_function = new AutoDiffCostFunction<Ceres5_FixInitialVelocityContFunctor, 1, 18>(new Ceres5_FixInitialVelocityContFunctor(weight_fix_initial_velocity, vel_x_ini, vel_y_ini, vel_z_ini));
        
        problem.AddResidualBlock(fix_initial_velocity_cont_function, nullptr, coeff_state_vector.parameter);

        cost_blocks.push_back({"Velocity continuity", 1});

        // 6 - Reduce goal velocity (semi-hard restriction)

        // ceres::CostFunction* reduce_goal_gradient_cont_function = new AutoDiffCostFunction<Ceres5_ReduceGoalGradientContFunctor, 3, 18>(new Ceres5_ReduceGoalGradientContFunctor(weight_limit_goal_gradient));
        
        // problem.AddResidualBlock(reduce_goal_gradient_cont_function, nullptr, coeff_state_vector.parameter);

        // cost_blocks.push_back({"Reduce goal velocity", 3});


        // 7 - Dynamic limits (velocity and acceleration hard constraints)
        
        int n_dyn_samp = 20;
        double max_vel_param = 100.0;  // Maximum velocity norm (cells per s-unit)
        double max_acc_param = 100.0;  // Maximum acceleration norm (cells per s-unit^2)

        for (int i = 0; i < n_dyn_samp; i++)
        {
            double s_dyn = 2.0 * (i + 1.0) / (n_dyn_samp + 1.0) - 1.0;

            ceres::CostFunction* dyn_limits_function = new AutoDiffCostFunction<Ceres5_DynamicLimitsContFunctor, 2, 18>(
                new Ceres5_DynamicLimitsContFunctor(weight_dynamic_limits, s_dyn, max_vel_param, max_acc_param));

            problem.AddResidualBlock(dyn_limits_function, nullptr, coeff_state_vector.parameter);
            cost_blocks.push_back({"Dynamic limits", 2});
        }


        // Test 1
        ceres::Problem::EvaluateOptions eval_options;
        eval_options.apply_loss_function = false;  // Evaluar sin la función de pérdida

        double total_cost = 0.0;
        std::vector<double> test_residuals;

        problem.Evaluate(eval_options, &total_cost, &test_residuals, nullptr, nullptr);


        // Solve

        auto start_opt = std::chrono::high_resolution_clock::now();
        //std::cout << "Starting solver" << std::endl;
        ceres::Solve(options, &problem, &summary);
        //std::cout << "Exiting solver" << std::endl;
        auto end_opt = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> opt_duration = end_opt - start_opt;

        std::cout << summary.BriefReport() << "\n";
        //std::cout << "Número de iteraciones: "
        //        << summary.iterations.size() << std::endl;
        printf("TIEMPO DE OPTIMIZACIÓN (CHEBYSHEV): %.2f ms\n", opt_duration.count());

        // Building the output
        //std::cout << "Building output" << std::endl;
        Planners::utils::OptimizedContinuousFunction optimized_coeffs;

        // Resize
        optimized_coeffs.x_params.resize(6);
        optimized_coeffs.y_params.resize(6);
        optimized_coeffs.z_params.resize(6);

        //Test 2
        total_cost = 0.0;
        test_residuals.clear();
        problem.Evaluate(eval_options, &total_cost, &test_residuals, nullptr, nullptr);

        std::cout << "\n--- Residuals por bloque ---\n";

        size_t index = 0;
        std::map<std::string, double> grouped_squares;

        for (auto& block : cost_blocks) {
            const std::string& name = block.first;
            int num_residuals = block.second;

            double sum_sq = 0.0;
            for (int i = 0; i < num_residuals; ++i) {
                double r = test_residuals[index++];
                sum_sq += 0.5 * r * r;
            }
            grouped_squares[name] += sum_sq; // acumular por tipo
        }

        // Mostrar resultados agrupados
        for (const auto& kv : grouped_squares) {
            std::cout << kv.first << " residual = " << kv.second << std::endl;
        }

        std::cout << "Total cost (Ceres) = " << total_cost << std::endl;


        // //Imprimir cada residual
        // std::cout << "Residuals:" << std::endl;
        // for (size_t i = 0; i < test_residuals.size(); ++i) {
        //     std::cout << "Residual[" << i << "] = " << test_residuals[i] << std::endl;
        // }

        // int test_residuals_size = test_residuals.size();
        // double dist_to_goal_x = test_residuals[test_residuals_size-3] * resolution_ / weight_fix_goal;
        // double dist_to_goal_y = test_residuals[test_residuals_size-2] * resolution_ / weight_fix_goal;
        // double dist_to_goal_z = test_residuals[test_residuals_size-1] * resolution_ / weight_fix_goal;
        // std::cout << "Dist to goal: " << sqrt(dist_to_goal_x * dist_to_goal_x + dist_to_goal_y * dist_to_goal_y + dist_to_goal_z * dist_to_goal_z) << std::endl;
        

        for (int i = 0; i < 6; i++) {
            optimized_coeffs.x_params[i] = coeff_state_vector.parameter[i];
            optimized_coeffs.y_params[i] = coeff_state_vector.parameter[i + 6];
            optimized_coeffs.z_params[i] = coeff_state_vector.parameter[i + 12];
        }

        // Min distance to obstacles: minimum ESDF residual after last solver evaluation
        optimized_coeffs.min_dist_m = evaluation_callback_cheb.residuals().minCoeff();

        // Path length: Gauss-Legendre n=5 on the derivative of the final Chebyshev polynomial
        {
            const double* c = coeff_state_vector.parameter;
            const double p1x = c[4] - 3.0*c[2] + 5.0*c[0];
            const double p2x = 2.0*c[3] - 8.0*c[1];
            const double p3x = 4.0*c[2] - 20.0*c[0];
            const double p4x = 8.0*c[1];
            const double p5x = 16.0*c[0];
            const double p1y = c[10] - 3.0*c[8] + 5.0*c[6];
            const double p2y = 2.0*c[9] - 8.0*c[7];
            const double p3y = 4.0*c[8] - 20.0*c[6];
            const double p4y = 8.0*c[7];
            const double p5y = 16.0*c[6];
            const double p1z = c[16] - 3.0*c[14] + 5.0*c[12];
            const double p2z = 2.0*c[15] - 8.0*c[13];
            const double p3z = 4.0*c[14] - 20.0*c[12];
            const double p4z = 8.0*c[13];
            const double p5z = 16.0*c[12];
            constexpr std::array<double,5> xi = {-0.9061798459386640,-0.5384693101056831,0.0,0.5384693101056831,0.9061798459386640};
            constexpr std::array<double,5> wi = {0.2369268850561891,0.4786286704993665,0.5688888888888889,0.4786286704993665,0.2369268850561891};
            double length_cells = 0.0;
            for (int i = 0; i < 5; ++i) {
                const double s = xi[i];
                const double dx = p1x + s*(2.0*p2x + s*(3.0*p3x + s*(4.0*p4x + s*5.0*p5x)));
                const double dy = p1y + s*(2.0*p2y + s*(3.0*p3y + s*(4.0*p4y + s*5.0*p5y)));
                const double dz = p1z + s*(2.0*p2z + s*(3.0*p3z + s*(4.0*p4z + s*5.0*p5z)));
                length_cells += wi[i] * std::sqrt(dx*dx + dy*dy + dz*dz);
            }
            optimized_coeffs.path_length_m = length_cells * static_cast<double>(resolution_);
        }

        return optimized_coeffs;
    }

    Planners::utils::OptimizedContinuousFunction ceresOptimizerReducedChebyshevContinuousPath(Eigen::VectorXd coeff_x, Eigen::VectorXd coeff_y, Eigen::VectorXd coeff_z, double origin_x, double origin_y, double origin_z, Planners::utils::Vec3i local_start, Planners::utils::Vec3i local_goal, Local_Grid3d &_grid, torch::jit::script::Module& loaded_sdf, float resolution_, std::shared_ptr<voxblox::EsdfMap>& esdf_map_, bool use_voxfield)
    {

        // Convert function coeffs to state block (excluding the last one, that's fixed by the starting point)
        parameterBlockReducedChebyshev coeff_state_vector;
        for (int i = 0; i < 4; i++) {
            coeff_state_vector.parameter[i] = coeff_x[i];
            coeff_state_vector.parameter[i + 4] = coeff_y[i];
            coeff_state_vector.parameter[i + 8] = coeff_z[i];
        }

        // Declare Ceres optimization problem
        int path_length_seg = 5; // Segments for path length calculation
        int esdf_samp = 10; // Samples for ESDF calculation
        
        CeresESDFUpdateReducedChebyshev evaluation_callback_reduced_cheb(coeff_state_vector, esdf_samp, loaded_sdf, origin_x, origin_y, origin_z, local_start, local_goal, resolution_, esdf_map_, use_voxfield);
        ceres::Problem problem;
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        options.minimizer_progress_to_stdout = false;
        options.max_num_iterations = 50;
        options.num_threads = 12;
        options.use_nonmonotonic_steps = true;
        options.evaluation_callback = &evaluation_callback_reduced_cheb;



        std::cout << "Configured options" << std::endl;

        std::cout << "Created Ceres Problem" << std::endl;

        // Cost function weights
        double weight_path_length = 0.1;
        double weight_esdf = 10.0; 
        double weight_smoothness = 10.0;

        // 1 - Path length cost function

        for(int i = 0; i < path_length_seg; i++)
        {
            double s0 = 2.0 * i / path_length_seg - 1.0;
            double s1 = 2.0 * (i+1) / path_length_seg - 1.0;

            ceres::CostFunction* path_length_cont_function_seg = new AutoDiffCostFunction<Ceres6_PathLengthContSegmentFunctor, 3, 12>(new Ceres6_PathLengthContSegmentFunctor(weight_path_length, s0, s1, local_start, local_goal));
        
            problem.AddResidualBlock(path_length_cont_function_seg, nullptr, coeff_state_vector.parameter);
        }

        // 2 - ESDF cost function

        for(int i = 0; i < esdf_samp; i++)
        {
            double s_esdf = 2.0 * (i + 1.0) / (esdf_samp + 1.0) - 1.0;

            ceres::CostFunction* esdf_cont_function_seg = new AutoDiffCostFunction<Ceres6_ObstacleDistanceCostContSegmentFunctor, 1, 12>(new Ceres6_ObstacleDistanceCostContSegmentFunctor(evaluation_callback_reduced_cheb, i, s_esdf, esdf_samp, local_start, local_goal, weight_esdf));

            problem.AddResidualBlock(esdf_cont_function_seg, nullptr, coeff_state_vector.parameter);
        }

        // 3 - Smoothness cost function (by minimizing coeffs)


        ceres::CostFunction* smoothness_cont_function = new AutoDiffCostFunction<Ceres6_SmoothnessContFunctor, 12, 12>(new Ceres6_SmoothnessContFunctor(weight_smoothness, local_start, local_goal));

        problem.AddResidualBlock(smoothness_cont_function, nullptr, coeff_state_vector.parameter);



        ceres::Solver::Summary summary;

        // Test 1
        ceres::Problem::EvaluateOptions eval_options;
        eval_options.apply_loss_function = false;  // Evaluar sin la función de pérdida

        double total_cost = 0.0;
        std::vector<double> test_residuals;

        problem.Evaluate(eval_options, &total_cost, &test_residuals, nullptr, nullptr);


        // Solve

        auto start_opt = std::chrono::high_resolution_clock::now();
        std::cout << "Starting solver" << std::endl;
        ceres::Solve(options, &problem, &summary);
        std::cout << "Exiting solver" << std::endl;
        auto end_opt = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> opt_duration = end_opt - start_opt;
        printf("TIEMPO DE OPTIMIZACIÓN: %.2f ms\n", opt_duration.count());

        // Building the output
        std::cout << "Building output" << std::endl;
        Planners::utils::OptimizedContinuousFunction optimized_coeffs;

        // Resize
        optimized_coeffs.x_params.resize(4);
        optimized_coeffs.y_params.resize(4);
        optimized_coeffs.z_params.resize(4);

        //Test 2
        total_cost = 0.0;
        test_residuals.clear();
        problem.Evaluate(eval_options, &total_cost, &test_residuals, nullptr, nullptr);

        // Imprimir cada residual
        std::cout << "Residuals:" << std::endl;
        for (size_t i = 0; i < test_residuals.size(); ++i) {
            std::cout << "Residual[" << i << "] = " << test_residuals[i] << std::endl;
        }

        // int test_residuals_size = test_residuals.size();
        // double dist_to_goal_x = test_residuals[test_residuals_size-3] * resolution_ / weight_fix_goal;
        // double dist_to_goal_y = test_residuals[test_residuals_size-2] * resolution_ / weight_fix_goal;
        // double dist_to_goal_z = test_residuals[test_residuals_size-1] * resolution_ / weight_fix_goal;
        // std::cout << "Dist to goal: " << sqrt(dist_to_goal_x * dist_to_goal_x + dist_to_goal_y * dist_to_goal_y + dist_to_goal_z * dist_to_goal_z) << std::endl;
        

        for (int i = 0; i < 4; i++) {
            optimized_coeffs.x_params[i] = coeff_state_vector.parameter[i];
            optimized_coeffs.y_params[i] = coeff_state_vector.parameter[i + 4];
            optimized_coeffs.z_params[i] = coeff_state_vector.parameter[i + 8];
        }

        std::cout << "Returning to main function" << std::endl;

        return optimized_coeffs;
    }

    Planners::utils::OptimizedContinuousFunction ceresOptimizerContinuousPathInit(Eigen::VectorXd init_coeff_x, Eigen::VectorXd init_coeff_y, Eigen::VectorXd init_coeff_z, Planners::utils::CoordinateList global_path_local_section, double t_last)
    {
        // Estimation of t_n for each point in the local global path

        double wp_distance_total;
        //Planners::utils::Vec3i local_start;
        std::vector<double> dist_list;
        std::vector<double> t_list;

        for(int i=0; i<global_path_local_section.size(); i++)
        {
            if(i == 0)
            {
                double dist = sqrt((global_path_local_section[0].x - init_coeff_x[3]) * (global_path_local_section[0].x - init_coeff_x[3]) + (global_path_local_section[0].y - init_coeff_y[3]) * (global_path_local_section[0].y - init_coeff_y[3]) + (global_path_local_section[0].z - init_coeff_z[3]) * (global_path_local_section[0].z - init_coeff_z[3]));
                dist_list.push_back(dist);
                wp_distance_total += dist;
            }
            else
            {
                double dist = sqrt((global_path_local_section[i].x - global_path_local_section[i-1].x) * (global_path_local_section[i].x - global_path_local_section[i-1].x) + (global_path_local_section[i].y - global_path_local_section[i-1].y) * (global_path_local_section[i].y - global_path_local_section[i-1].y) + (global_path_local_section[i].z - global_path_local_section[i-1].z) * (global_path_local_section[i].z - global_path_local_section[i-1].z));
                dist_list.push_back(dist);
                wp_distance_total += dist;
            }
        }

        for(int i=0; i<global_path_local_section.size(); i++)
        {
            if(i == 0)
            {
                double t_i = dist_list[0] * t_last / wp_distance_total;
                t_list.push_back(t_i);
            }
            else if(i == global_path_local_section.size() - 1)
            {
                t_list.push_back(t_last);
            }
            else
            {
                double t_i = (dist_list[i] * t_last / wp_distance_total) + t_list[i-1];
                t_list.push_back(t_i);
            }
        }

        // Optimization

        parameterBlockContinuousPath coeff_state_vector;
        parameterBlockContinuousPathConstant coeff_state_vector_constant;
        for (int i = 0; i < 3; i++) {
            coeff_state_vector.parameter[i] = init_coeff_x[i];
            coeff_state_vector.parameter[i + 3] = init_coeff_y[i];
            coeff_state_vector.parameter[i + 6] = init_coeff_z[i];
        }
        
        coeff_state_vector_constant.parameter[0] = init_coeff_x[3];
        coeff_state_vector_constant.parameter[1] = init_coeff_y[3];
        coeff_state_vector_constant.parameter[2] = init_coeff_z[3];

        ceres::Problem problem;

        std::cout << "Created Ceres Problem" << std::endl;

        // Cost function weights

        double weight_distance_to_wp = 10.0;
        double weight_smoothness = 5.0;

        // 1. Cost Function - Distance to waypoints

        for (size_t i = 0; i < global_path_local_section.size(); i++)
        {
            ceres::CostFunction* distance_to_wp = new AutoDiffCostFunction<DistanceToWPFunctor, 1, 9, 3>
                                                        (new DistanceToWPFunctor(weight_distance_to_wp, global_path_local_section[i], t_list[i]));

            problem.AddResidualBlock(distance_to_wp, nullptr, coeff_state_vector.parameter, coeff_state_vector_constant.parameter);

        }

        // 2. Cost Function - Smoothness

        ceres::CostFunction* smoothness_function = new AutoDiffCostFunction<SmoothnessContInitFunctor, 1, 9>(new SmoothnessContInitFunctor(weight_smoothness));
    
        problem.AddResidualBlock(smoothness_function, nullptr, coeff_state_vector.parameter);


        // Freeze independent coeffs
        problem.SetParameterBlockConstant(coeff_state_vector_constant.parameter);


        // Solve problem
        ceres::Solver::Options options;
        //options.linear_solver_type = ceres::DENSE_QR;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.minimizer_progress_to_stdout = true;
        options.max_num_iterations = 200;
        options.num_threads = 12;
        options.use_nonmonotonic_steps = true;
        
        ceres::Solver::Summary summary;

        auto start_opt = std::chrono::high_resolution_clock::now();
        ceres::Solve(options, &problem, &summary);
        auto end_opt = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> opt_duration = end_opt - start_opt;

        // Building the output
        Planners::utils::OptimizedContinuousFunction optimized_coeffs;

        // Resize
        optimized_coeffs.x_params.resize(4);
        optimized_coeffs.y_params.resize(4);
        optimized_coeffs.z_params.resize(4);

        for (int i = 0; i < 3; i++) {
            optimized_coeffs.x_params[i] = coeff_state_vector.parameter[i];
            optimized_coeffs.y_params[i] = coeff_state_vector.parameter[i + 3];
            optimized_coeffs.z_params[i] = coeff_state_vector.parameter[i + 6];
        }
        optimized_coeffs.x_params[3] = coeff_state_vector_constant.parameter[0];
        optimized_coeffs.y_params[3] = coeff_state_vector_constant.parameter[1];
        optimized_coeffs.z_params[3] = coeff_state_vector_constant.parameter[2];

        std::cout << "Returning to main function" << std::endl;

        return optimized_coeffs;


    }

    Planners::utils::OptimizedContinuousFunction ceresOptimizerContinuousPathInitG5(Eigen::VectorXd init_coeff_x, Eigen::VectorXd init_coeff_y, Eigen::VectorXd init_coeff_z, Planners::utils::CoordinateList global_path_local_section, Planners::utils::Vec3i local_goal)
    {
        // Estimation of t_n for each point in the local global path

        double wp_distance_total;
        //Planners::utils::Vec3i local_start;
        std::vector<double> dist_list;
        std::vector<double> t_list;

        for(int i=0; i<global_path_local_section.size(); i++)
        {
            if(i == 0)
            {
                double dist = sqrt((global_path_local_section[0].x - init_coeff_x[3]) * (global_path_local_section[0].x - init_coeff_x[3]) + (global_path_local_section[0].y - init_coeff_y[3]) * (global_path_local_section[0].y - init_coeff_y[3]) + (global_path_local_section[0].z - init_coeff_z[3]) * (global_path_local_section[0].z - init_coeff_z[3]));
                dist_list.push_back(dist);
                wp_distance_total += dist;
            }
            else
            {
                double dist = sqrt((global_path_local_section[i].x - global_path_local_section[i-1].x) * (global_path_local_section[i].x - global_path_local_section[i-1].x) + (global_path_local_section[i].y - global_path_local_section[i-1].y) * (global_path_local_section[i].y - global_path_local_section[i-1].y) + (global_path_local_section[i].z - global_path_local_section[i-1].z) * (global_path_local_section[i].z - global_path_local_section[i-1].z));
                dist_list.push_back(dist);
                wp_distance_total += dist;
            }
        }

        for(int i=0; i<global_path_local_section.size(); i++)
        {
            if(i == 0)
            {
                double t_i = dist_list[0] / wp_distance_total;
                t_list.push_back(t_i);
            }
            else if(i == global_path_local_section.size() - 1)
            {
                t_list.push_back(1.0);
            }
            else
            {
                double t_i = (dist_list[i] / wp_distance_total) + t_list[i-1];
                t_list.push_back(t_i);
            }
        }

        // Optimization

        parameterBlockContinuousPath coeff_state_vector;
        parameterBlockContinuousPathConstant coeff_state_vector_constant;
        for (int i = 0; i < 5; i++) {
            coeff_state_vector.parameter[i] = init_coeff_x[i];
            coeff_state_vector.parameter[i + 5] = init_coeff_y[i];
            coeff_state_vector.parameter[i + 10] = init_coeff_z[i];
        }
        
        coeff_state_vector_constant.parameter[0] = init_coeff_x[5];
        coeff_state_vector_constant.parameter[1] = init_coeff_y[5];
        coeff_state_vector_constant.parameter[2] = init_coeff_z[5];

        ceres::Problem problem;

        //std::cout << "Created Ceres Problem" << std::endl;

        // Cost function weights

        double weight_distance_to_wp = 1e3;
        double weight_smoothness = 1.0;
        //double weight_fix_goal = 1e4;

        // 1. Cost Function - Distance to waypoints

        for (size_t i = 0; i < global_path_local_section.size(); i++)
        {
            ceres::CostFunction* distance_to_wp = new AutoDiffCostFunction<DistanceToWPG5Functor, 1, 15, 3>
                                                        (new DistanceToWPG5Functor(weight_distance_to_wp, global_path_local_section[i], t_list[i]));

            problem.AddResidualBlock(distance_to_wp, nullptr, coeff_state_vector.parameter, coeff_state_vector_constant.parameter);

        }


        //int f_segments = 50;

        //ceres::CostFunction* distance_to_wp = new AutoDiffCostFunction<DistanceToWPG5Functor, 1, 15, 3>(new DistanceToWPG5Functor(weight_distance_to_wp, global_path_local_section, f_segments));

        //problem.AddResidualBlock(distance_to_wp, nullptr, coeff_state_vector.parameter, coeff_state_vector_constant.parameter);

        // 2. Cost Function - Smoothness

        ceres::CostFunction* smoothness_function = new AutoDiffCostFunction<SmoothnessContInitG5Functor, 12, 15>(new SmoothnessContInitG5Functor(weight_smoothness));
    
        problem.AddResidualBlock(smoothness_function, nullptr, coeff_state_vector.parameter);

        // 3. Cost Function - Fix Goal

        //ceres::CostFunction* fixed_goal_function = new AutoDiffCostFunction<FixGoalInitG5Functor, 3, 15, 3>(new FixGoalInitG5Functor(weight_fix_goal, local_goal));
        
        //problem.AddResidualBlock(fixed_goal_function, nullptr, coeff_state_vector.parameter, coeff_state_vector_constant.parameter);



        // Freeze independent coeffs
        problem.SetParameterBlockConstant(coeff_state_vector_constant.parameter);
        


        // Solve problem
        ceres::Solver::Options options;
        //options.linear_solver_type = ceres::DENSE_QR;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.minimizer_progress_to_stdout = false;
        //options.max_solver_time_in_seconds = 50e-3;
        options.max_num_iterations = 50;
        options.num_threads = 12;
        options.use_nonmonotonic_steps = true;
        
        ceres::Solver::Summary summary;

        auto start_opt = std::chrono::high_resolution_clock::now();
        ceres::Solve(options, &problem, &summary);
        auto end_opt = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> opt_duration = end_opt - start_opt;

        //Test
        ceres::Problem::EvaluateOptions eval_options;
        eval_options.apply_loss_function = false; 
        std::vector<double> test_residuals;
        double total_cost = 0.0;
        problem.Evaluate(eval_options, &total_cost, &test_residuals, nullptr, nullptr);

        //std::cout << "Costo total después de la optimización: " << total_cost << std::endl;

        for (size_t i = 0; i < test_residuals.size(); ++i) {
            std::cout << "Residual " << i << ": " << test_residuals[i] << std::endl;
        }

        // Building the output
        Planners::utils::OptimizedContinuousFunction optimized_coeffs;

        // Resize
        optimized_coeffs.x_params.resize(6);
        optimized_coeffs.y_params.resize(6);
        optimized_coeffs.z_params.resize(6);

        for (int i = 0; i < 5; i++) {
            optimized_coeffs.x_params[i] = coeff_state_vector.parameter[i];
            optimized_coeffs.y_params[i] = coeff_state_vector.parameter[i + 5];
            optimized_coeffs.z_params[i] = coeff_state_vector.parameter[i + 10];
        }
        optimized_coeffs.x_params[5] = coeff_state_vector_constant.parameter[0];
        optimized_coeffs.y_params[5] = coeff_state_vector_constant.parameter[1];
        optimized_coeffs.z_params[5] = coeff_state_vector_constant.parameter[2];

        //std::cout << "Returning to main function" << std::endl;

        return optimized_coeffs;
    }

    Planners::utils::OptimizedContinuousFunction ceresOptimizerContinuousPathInitChebyshev(Eigen::VectorXd init_coeff_x, Eigen::VectorXd init_coeff_y, Eigen::VectorXd init_coeff_z, Planners::utils::CoordinateList global_path_local_section, Planners::utils::Vec3i local_start, Planners::utils::Vec3i local_goal)
    {
        // Optimization

        parameterBlockChebyshev coeff_state_vector;
        for (int i = 0; i < 6; i++) {
            coeff_state_vector.parameter[i] = init_coeff_x[i];
            coeff_state_vector.parameter[i + 6] = init_coeff_y[i];
            coeff_state_vector.parameter[i + 12] = init_coeff_z[i];
        }

        ceres::Problem problem;

        std::cout << "Created Ceres Problem" << std::endl;

        // Cost function weights

        double weight_distance_to_wp = 100.0;
        double weight_smoothness = 1.0;
        double weight_fix_start_goal = 1e4;
        double weight_limit_gradients = 1e4;

        // 1. Cost Function - Distance to waypoints

        int f_segments = 50; // Segmentation of the function

        ceres::CostFunction* distance_to_wp = new AutoDiffCostFunction<DistanceToWPChebyshevFunctor, 1, 18>(new DistanceToWPChebyshevFunctor(weight_distance_to_wp, global_path_local_section, f_segments));

        problem.AddResidualBlock(distance_to_wp, nullptr, coeff_state_vector.parameter);

        // 2. Cost Function - Smoothness

        ceres::CostFunction* smoothness_function = new AutoDiffCostFunction<SmoothnessContInitChebyshevFunctor, 12, 18>(new SmoothnessContInitChebyshevFunctor(weight_smoothness));
    
        problem.AddResidualBlock(smoothness_function, nullptr, coeff_state_vector.parameter);

        // 3. Cost Function - Fix start and goal

        ceres::CostFunction* fix_start_goal_function = new AutoDiffCostFunction<FixStartGoalChebyshevFunctor, 6, 18>(new FixStartGoalChebyshevFunctor(weight_fix_start_goal, local_start, local_goal));
    
        problem.AddResidualBlock(fix_start_goal_function, nullptr, coeff_state_vector.parameter);

        // 4 - Reduce start and goal gradients (semi-hard restriction)

        //ceres::CostFunction* reduce_gradients_cont_function = new AutoDiffCostFunction<ReduceGradientsContInitChebyshevFunctor, 6, 18>(new ReduceGradientsContInitChebyshevFunctor(weight_limit_gradients));
        
        //problem.AddResidualBlock(reduce_gradients_cont_function, nullptr, coeff_state_vector.parameter);


        // Solve problem
        ceres::Solver::Options options;
        //options.linear_solver_type = ceres::DENSE_QR;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.minimizer_progress_to_stdout = false;
        options.max_solver_time_in_seconds = 50e-3;
        options.max_num_iterations = 50;
        options.num_threads = 12;
        options.use_nonmonotonic_steps = true;
        
        ceres::Solver::Summary summary;

        auto start_opt = std::chrono::high_resolution_clock::now();
        ceres::Solve(options, &problem, &summary);
        auto end_opt = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> opt_duration = end_opt - start_opt;

        // Building the output
        Planners::utils::OptimizedContinuousFunction optimized_coeffs;

        // Resize
        optimized_coeffs.x_params.resize(6);
        optimized_coeffs.y_params.resize(6);
        optimized_coeffs.z_params.resize(6);

        for (int i = 0; i < 6; i++) {
            optimized_coeffs.x_params[i] = coeff_state_vector.parameter[i];
            optimized_coeffs.y_params[i] = coeff_state_vector.parameter[i + 6];
            optimized_coeffs.z_params[i] = coeff_state_vector.parameter[i + 12];
        }

        std::cout << "Returning to main function" << std::endl;

        return optimized_coeffs;
    }

    Planners::utils::OptimizedTimeContinuousFunction ceresOptimizerChebyshevTimeOpt(Eigen::VectorXd coeff_x, Eigen::VectorXd coeff_y, Eigen::VectorXd coeff_z, double T_ini, double origin_x, double origin_y, double origin_z, Planners::utils::Vec3i local_start, Planners::utils::Vec3i local_goal, Local_Grid3d &_grid, torch::jit::script::Module& loaded_sdf, float resolution_, std::shared_ptr<voxblox::EsdfMap>& esdf_map_, bool use_voxfield, double v_max_ms, double a_max_ms2, double j_max_ms3, double vel_x_ms, double vel_y_ms, double vel_z_ms, double acc_x_ms2, double acc_y_ms2, double acc_z_ms2, double goal_vel_x_ms, double goal_vel_y_ms, double goal_vel_z_ms, double goal_acc_x_ms2, double goal_acc_y_ms2, double goal_acc_z_ms2)
    { 

        // Convert function coeffs to state block (excluding the last one, that's fixed by the starting point)
        parameterBlockChebyshevTime coeff_state_vector;
        for (int i = 0; i < 6; i++) {
            coeff_state_vector.parameter[i] = coeff_x[i];
            coeff_state_vector.parameter[i + 6] = coeff_y[i];
            coeff_state_vector.parameter[i + 12] = coeff_z[i];
        }
        coeff_state_vector.parameter[18] = T_ini;

        // Load tunable parameters from the ROS parameter server (set via local_planner_configuration.yaml).
        // ros::NodeHandle("~") resolves to the local_planner_ros_node private namespace, where the YAML
        // is loaded by the launch file.  Defaults match the YAML so the node works without a restart.
        ros::NodeHandle nh_ceres("~");

        int    esdf_samp,  n_dyn_samp;
        double k_dyn_softplus;
        double weight_path_length, weight_esdf, weight_smoothness;
        double weight_fix_startgoal, weight_initial_dynamic, weight_goal_dynamic;
        double weight_dynamic_limits, weight_traj_time;

        nh_ceres.param("ceres7_esdf_samp",            esdf_samp,              20);
        nh_ceres.param("ceres7_dyn_samp",             n_dyn_samp,             50);
        nh_ceres.param("ceres7_dyn_k_softplus",       k_dyn_softplus,         50.0);
        nh_ceres.param("ceres7_weight_path_length",   weight_path_length,     100.0);
        nh_ceres.param("ceres7_weight_esdf",          weight_esdf,            2500.0);
        nh_ceres.param("ceres7_weight_smoothness",    weight_smoothness,      2.0);
        nh_ceres.param("ceres7_weight_fix_startgoal", weight_fix_startgoal,   5e4);
        nh_ceres.param("ceres7_weight_init_dynamic",  weight_initial_dynamic, 5e4);
        nh_ceres.param("ceres7_weight_goal_dynamic",  weight_goal_dynamic,    0.0);
        nh_ceres.param("ceres7_weight_dyn_limits",    weight_dynamic_limits,  5e5);
        nh_ceres.param("ceres7_weight_traj_time",     weight_traj_time,       1e4);

        double curvature_bound_factor;
        nh_ceres.param("ceres7_curvature_bound_factor", curvature_bound_factor, 0.5);

        // Declare Ceres optimization problem

        CeresESDFUpdateChebyshevTime evaluation_callback_cheb_time(coeff_state_vector, esdf_samp, loaded_sdf, origin_x, origin_y, origin_z, resolution_, esdf_map_, use_voxfield);
        ceres::Problem problem;
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        // options.function_tolerance  = 1e-4;
        // options.gradient_tolerance  = 1e-6;
        // options.parameter_tolerance = 1e-6;
        options.max_solver_time_in_seconds = 500e-3;
        options.minimizer_progress_to_stdout = false;
        options.max_num_iterations = 200;
        options.num_threads = 12;
        options.use_nonmonotonic_steps = true;
        options.evaluation_callback = &evaluation_callback_cheb_time;

        ceres::Solver::Summary summary;


        std::vector<std::pair<std::string, int>> cost_blocks;


        // 1 - Path Length Cost Function (soft cost - encourages short paths)

        ceres::CostFunction* path_length_cont_function = new AutoDiffCostFunction<Ceres7_PathLengthGaussLegendreFunctor, 1, 19>(new Ceres7_PathLengthGaussLegendreFunctor(weight_path_length));
        
        problem.AddResidualBlock(path_length_cont_function, nullptr, coeff_state_vector.parameter);

        cost_blocks.push_back({"Path length", 1});


        // 2 - ESDF Cost Function (soft cost - encourages obstacle avoidance)

        double weighted_weight_esdf = weight_esdf / esdf_samp;

        for(int i = 0; i < esdf_samp; i++)
        {
            double s_esdf = 2.0 * (i + 1.0) / (esdf_samp + 1.0) - 1.0;

            ceres::CostFunction* esdf_cont_function_seg = new AutoDiffCostFunction<Ceres7_ObstacleDistanceCostContSegmentFunctor, 1, 19>(new Ceres7_ObstacleDistanceCostContSegmentFunctor(evaluation_callback_cheb_time, i, s_esdf, esdf_samp, weight_esdf));

            problem.AddResidualBlock(esdf_cont_function_seg, nullptr, coeff_state_vector.parameter);

            cost_blocks.push_back({"ESDF", 1});
        }

        // 3 - Smoothness Cost Function (soft cost - encourages small high-order coefficients for smoother vel/acc/jerk profiles)

        ceres::CostFunction* smoothness_cont_function = new AutoDiffCostFunction<Ceres7_SmoothnessContFunctor, 12, 19>(new Ceres7_SmoothnessContFunctor(weight_smoothness));

        problem.AddResidualBlock(smoothness_cont_function, nullptr, coeff_state_vector.parameter);

        cost_blocks.push_back({"Smoothness", 12});


        // 4 - Fix Local Start/Goal Fuction (hard constraint - enforces local start/goal positions)

        ceres::CostFunction* fixed_startgoal_cont_function = new AutoDiffCostFunction<Ceres7_FixStartGoalContFunctor, 6, 19>(new Ceres7_FixStartGoalContFunctor(weight_fix_startgoal, local_start, local_goal));
        
        problem.AddResidualBlock(fixed_startgoal_cont_function, nullptr, coeff_state_vector.parameter);

        cost_blocks.push_back({"Fixed start/goal", 6});


        // 5 - Initial Dynamic State Fix (hard constraint - enforces dynamic continuity)

        ceres::CostFunction* fix_dyn_state_function = new AutoDiffCostFunction<Ceres7_FixInitialDynamicStateFunctor, 6, 19>(new Ceres7_FixInitialDynamicStateFunctor(weight_initial_dynamic,vel_x_ms, vel_y_ms, vel_z_ms, acc_x_ms2, acc_y_ms2, acc_z_ms2,static_cast<double>(resolution_)));

        problem.AddResidualBlock(fix_dyn_state_function, nullptr, coeff_state_vector.parameter);

        cost_blocks.push_back({"Initial Dynamic State Fix", 6});

        // 6 - Goal Dynamic State (soft cost - encourages arriving with desired vel/acc)

        if (weight_goal_dynamic > 0.0) {
            ceres::CostFunction* fix_goal_dyn_function = new AutoDiffCostFunction<Ceres7_GoalDynamicStateFunctor, 6, 19>(new Ceres7_GoalDynamicStateFunctor(weight_goal_dynamic, goal_vel_x_ms, goal_vel_y_ms, goal_vel_z_ms, goal_acc_x_ms2, goal_acc_y_ms2, goal_acc_z_ms2, static_cast<double>(resolution_)));
            
            problem.AddResidualBlock(fix_goal_dyn_function, nullptr, coeff_state_vector.parameter);
            
            cost_blocks.push_back({"Goal Dynamic State", 6});
        }

        // 7 - Dynamic Limits Enforcement (soft-hard constraint via softplus penalty)
        //     Single functor covering all n_dyn_samp points — Chebyshev→monomial conversion
        //     done once instead of n_dyn_samp times, saving (n_dyn_samp-1) redundant conversions
        //     per Jacobian call.  Uses DynamicAutoDiffCostFunction for runtime-configurable N.

        {
            auto* dyn_limits_all = new ceres::DynamicAutoDiffCostFunction<Ceres7_DynamicLimitsAllFunctor, 19>(new Ceres7_DynamicLimitsAllFunctor(n_dyn_samp, weight_dynamic_limits, v_max_ms, a_max_ms2, j_max_ms3, static_cast<double>(resolution_), k_dyn_softplus));
            dyn_limits_all->AddParameterBlock(19);
            dyn_limits_all->SetNumResiduals(n_dyn_samp * 3);
            problem.AddResidualBlock(dyn_limits_all, nullptr, coeff_state_vector.parameter);
            cost_blocks.push_back({"Dynamic limits", n_dyn_samp * 3});
        }

        // 8 - Trajectory Time Minimisation (soft cost — encourages smallest feasible T)

        ceres::CostFunction* traj_time_function = new AutoDiffCostFunction<Ceres7_TrajTimeFunctor, 1, 19>(new Ceres7_TrajTimeFunctor(weight_traj_time));

        problem.AddResidualBlock(traj_time_function, nullptr, coeff_state_vector.parameter);
        
        cost_blocks.push_back({"Trajectory time", 1});

        // Parameter bounds on high-order Chebyshev coefficients (c[0..3] per axis = T5/T4/T3/T2).
        // These are the only coefficients that produce oscillations / loops; c[4] and c[5] are the
        // linear term and the constant — they define the straight line between start and goal and
        // are left unbounded so the endpoints are always reachable.
        //
        // The bound scales with the start→goal distance so it adapts to path length:
        //   coeff_bound = curvature_bound_factor * ||goal - start||_cells
        // With factor=0.5 a path of 40 cells allows ±20 cells of oscillation per axis — enough
        // for any realistic obstacle avoidance detour without allowing loops.
        //
        // If the warm-start already violates these bounds Ceres projects the initial point onto
        // the feasible set automatically, effectively "de-looping" the init at zero extra cost.
        {
            double dist = std::sqrt(
                std::pow(local_goal.x - local_start.x, 2.0) +
                std::pow(local_goal.y - local_start.y, 2.0) +
                std::pow(local_goal.z - local_start.z, 2.0));
            double coeff_bound = curvature_bound_factor * dist;

            for (int j = 0; j < 4; j++) {
                for (int axis = 0; axis < 3; axis++) {
                    problem.SetParameterLowerBound(coeff_state_vector.parameter, j + axis * 6, -coeff_bound);
                    problem.SetParameterUpperBound(coeff_state_vector.parameter, j + axis * 6,  coeff_bound);
                }
            }
        }

        // Test 1
        ceres::Problem::EvaluateOptions eval_options;
        eval_options.apply_loss_function = false;  // Evaluar sin la función de pérdida

        double total_cost = 0.0;
        std::vector<double> test_residuals;

        problem.Evaluate(eval_options, &total_cost, &test_residuals, nullptr, nullptr);


        // Solve

        auto start_opt = std::chrono::high_resolution_clock::now();
        //std::cout << "Starting solver" << std::endl;
        ceres::Solve(options, &problem, &summary);
        //std::cout << "Exiting solver" << std::endl;
        auto end_opt = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> opt_duration = end_opt - start_opt;

        std::cout << summary.BriefReport() << "\n";
        //std::cout << "Número de iteraciones: "
        //        << summary.iterations.size() << std::endl;
        printf("TIEMPO DE OPTIMIZACIÓN (CHEBYSHEV): %.2f ms\n", opt_duration.count());

        // Building the output
        //std::cout << "Building output" << std::endl;
        Planners::utils::OptimizedTimeContinuousFunction optimized_coeffs;

        // Resize
        optimized_coeffs.x_params.resize(6);
        optimized_coeffs.y_params.resize(6);
        optimized_coeffs.z_params.resize(6);

        //Test 2
        total_cost = 0.0;
        test_residuals.clear();
        problem.Evaluate(eval_options, &total_cost, &test_residuals, nullptr, nullptr);

        // Accumulate 0.5*r² per block, preserving insertion order (blocks 1-8)
        size_t idx = 0;
        std::vector<std::string>      block_order;
        std::map<std::string, double> block_cost;

        for (auto& block : cost_blocks) {
            const std::string& name = block.first;
            double sum_sq = 0.0;
            for (int i = 0; i < block.second; ++i) {
                double r = test_residuals[idx++];
                sum_sq += 0.5 * r * r;
            }
            if (block_cost.find(name) == block_cost.end()) {
                block_order.push_back(name);
                block_cost[name] = 0.0;
            }
            block_cost[name] += sum_sq;
        }

        printf("\u250c\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2510\n");
        printf("\u2502  Ceres MODE 7 \u2014 cost per block (0.5 * r\u00b2)         \u2502\n");
        printf("\u251c\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u252c\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2524\n");
        for (const auto& name : block_order) {
            printf("\u2502  %-29s\u2502  %14.4f    \u2502\n", name.c_str(), block_cost[name]);
        }
        printf("\u251c\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2534\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2524\n");
        printf("\u2502  Total                             %14.4f    \u2502\n", total_cost);
        printf("\u2514\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2518\n");


        // //Imprimir cada residual
        // std::cout << "Residuals:" << std::endl;
        // for (size_t i = 0; i < test_residuals.size(); ++i) {
        //     std::cout << "Residual[" << i << "] = " << test_residuals[i] << std::endl;
        // }

        // int test_residuals_size = test_residuals.size();
        // double dist_to_goal_x = test_residuals[test_residuals_size-3] * resolution_ / weight_fix_goal;
        // double dist_to_goal_y = test_residuals[test_residuals_size-2] * resolution_ / weight_fix_goal;
        // double dist_to_goal_z = test_residuals[test_residuals_size-1] * resolution_ / weight_fix_goal;
        // std::cout << "Dist to goal: " << sqrt(dist_to_goal_x * dist_to_goal_x + dist_to_goal_y * dist_to_goal_y + dist_to_goal_z * dist_to_goal_z) << std::endl;
        

        for (int i = 0; i < 6; i++) {
            optimized_coeffs.x_params[i] = coeff_state_vector.parameter[i];
            optimized_coeffs.y_params[i] = coeff_state_vector.parameter[i + 6];
            optimized_coeffs.z_params[i] = coeff_state_vector.parameter[i + 12];
        }

        // Min distance to obstacles: minimum ESDF residual after last solver evaluation
        optimized_coeffs.min_dist_m = evaluation_callback_cheb_time.residuals().minCoeff();

        // Path length: Gauss-Legendre n=5 on the derivative of the final Chebyshev polynomial
        {
            const double* c = coeff_state_vector.parameter;
            const double p1x = c[4] - 3.0*c[2] + 5.0*c[0];
            const double p2x = 2.0*c[3] - 8.0*c[1];
            const double p3x = 4.0*c[2] - 20.0*c[0];
            const double p4x = 8.0*c[1];
            const double p5x = 16.0*c[0];
            const double p1y = c[10] - 3.0*c[8] + 5.0*c[6];
            const double p2y = 2.0*c[9] - 8.0*c[7];
            const double p3y = 4.0*c[8] - 20.0*c[6];
            const double p4y = 8.0*c[7];
            const double p5y = 16.0*c[6];
            const double p1z = c[16] - 3.0*c[14] + 5.0*c[12];
            const double p2z = 2.0*c[15] - 8.0*c[13];
            const double p3z = 4.0*c[14] - 20.0*c[12];
            const double p4z = 8.0*c[13];
            const double p5z = 16.0*c[12];
            constexpr std::array<double,5> xi = {-0.9061798459386640,-0.5384693101056831,0.0,0.5384693101056831,0.9061798459386640};
            constexpr std::array<double,5> wi = {0.2369268850561891,0.4786286704993665,0.5688888888888889,0.4786286704993665,0.2369268850561891};
            double length_cells = 0.0;
            for (int i = 0; i < 5; ++i) {
                const double s = xi[i];
                const double dx = p1x + s*(2.0*p2x + s*(3.0*p3x + s*(4.0*p4x + s*5.0*p5x)));
                const double dy = p1y + s*(2.0*p2y + s*(3.0*p3y + s*(4.0*p4y + s*5.0*p5y)));
                const double dz = p1z + s*(2.0*p2z + s*(3.0*p3z + s*(4.0*p4z + s*5.0*p5z)));
                length_cells += wi[i] * std::sqrt(dx*dx + dy*dy + dz*dz);
            }
            optimized_coeffs.path_length_m = length_cells * static_cast<double>(resolution_);
        }

        optimized_coeffs.T_param = coeff_state_vector.parameter[18];

        return optimized_coeffs;
    }
}

