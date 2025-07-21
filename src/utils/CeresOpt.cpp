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
        options.minimizer_progress_to_stdout = true;
        options.max_num_iterations = 5000;
        options.num_threads = 12;
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

    Planners::utils::OptimizedContinuousFunction ceresOptimizerEvCallbackContinuousPath(Eigen::VectorXd coeff_x, Eigen::VectorXd coeff_y, Eigen::VectorXd coeff_z, double origin_x, double origin_y, double origin_z, Planners::utils::Vec3i local_goal, Local_Grid3d &_grid, torch::jit::script::Module& loaded_sdf, float resolution_)
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
        int esdf_seg = 20;
        double t_max_esdf_seg = 10.0;
        
        CeresESDFUpdate evaluation_callback(coeff_state_vector, coeff_state_vector_constant, esdf_seg, t_max_esdf_seg, loaded_sdf, origin_x, origin_y, origin_z, resolution_);
        ceres::Problem problem;
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        options.minimizer_progress_to_stdout = true;
        options.max_num_iterations = 5000;
        options.num_threads = 12;
        options.use_nonmonotonic_steps = true;
        options.evaluation_callback = &evaluation_callback;



        std::cout << "Configured options" << std::endl;

        std::cout << "Created Ceres Problem" << std::endl;

        // Cost function weights

        double weight_path_length = 1.0;
        double weight_esdf = 10000.0;
        double weight_smoothness = 1.0;
        double weight_fix_goal = 20.0;

        // 1 - Path length cost function

        int path_length_seg = 10;
        double t_max_seg = 10.0;

        for(int i = 0; i < path_length_seg; i++)
        {
            double t0 = t_max_seg * i / path_length_seg;
            double t1 = t_max_seg * (i+1) / path_length_seg;

            ceres::CostFunction* path_length_cont_function_seg = new AutoDiffCostFunction<Ceres4_PathLengthContSegmentFunctor, 1, 15, 3>(new Ceres4_PathLengthContSegmentFunctor(weight_path_length, t0, t1));
        
            problem.AddResidualBlock(path_length_cont_function_seg, nullptr, coeff_state_vector.parameter, coeff_state_vector_constant.parameter);
        }

        // 2 - ESDF cost function

        for(int i = 1; i < esdf_seg - 1; i++)
        {
            double t_esdf = t_max_esdf_seg * i / esdf_seg;

            ceres::CostFunction* esdf_cont_function_seg = new AutoDiffCostFunction<Ceres4_ObstacleDistanceCostContSegmentFunctor, 1, 15, 3>(new Ceres4_ObstacleDistanceCostContSegmentFunctor(evaluation_callback, i-1, t_esdf, esdf_seg, weight_esdf));

            problem.AddResidualBlock(esdf_cont_function_seg, nullptr, coeff_state_vector.parameter, coeff_state_vector_constant.parameter);
        }

        // // 3 - Smoothness cost function (by minimizing coeffs)

        // ceres::CostFunction* smoothness_cont_function = new AutoDiffCostFunction<Ceres4_SmoothnessContFunctor, 1, 15>(new Ceres4_SmoothnessContFunctor(weight_smoothness));
    
        // problem.AddResidualBlock(smoothness_cont_function, nullptr, coeff_state_vector.parameter);

        // 4 - Fixed local goal function (high weight needed)

        ceres::CostFunction* fixed_goal_cont_function = new AutoDiffCostFunction<Ceres4_FixGoalContFunctor, 1, 15, 3>(new Ceres4_FixGoalContFunctor(weight_fix_goal, local_goal));
        
        problem.AddResidualBlock(fixed_goal_cont_function, nullptr, coeff_state_vector.parameter, coeff_state_vector_constant.parameter);


        // Freeze independent coeffs

        problem.SetParameterBlockConstant(coeff_state_vector_constant.parameter);

        
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
        optimized_coeffs.x_params.resize(6);
        optimized_coeffs.y_params.resize(6);
        optimized_coeffs.z_params.resize(6);

        // Test 2
        total_cost = 0.0;
        problem.Evaluate(eval_options, &total_cost, &test_residuals, nullptr, nullptr);

        std::cout << "Costo total después de la optimización: " << total_cost << std::endl;

        for (size_t i = 0; i < test_residuals.size(); ++i) {
            std::cout << "Residual " << i << ": " << test_residuals[i] << std::endl;
        }

        for (int i = 0; i < 5; i++) {
            optimized_coeffs.x_params[i] = coeff_state_vector.parameter[i];
            optimized_coeffs.y_params[i] = coeff_state_vector.parameter[i + 5];
            optimized_coeffs.z_params[i] = coeff_state_vector.parameter[i + 10];
        }
        optimized_coeffs.x_params[5] = coeff_state_vector_constant.parameter[0];
        optimized_coeffs.y_params[5] = coeff_state_vector_constant.parameter[1];
        optimized_coeffs.z_params[5] = coeff_state_vector_constant.parameter[2];

        std::cout << "Returning to main function" << std::endl;

        return optimized_coeffs;
    }

    Planners::utils::OptimizedContinuousFunction ceresOptimizerContinuousPathInit(Eigen::VectorXd init_coeff_x, Eigen::VectorXd init_coeff_y, Eigen::VectorXd init_coeff_z, Planners::utils::CoordinateList global_path_local_section, double t_last)
    {
        // Estimation of t_n for each point in the local global path

        double wp_distance_total;
        Planners::utils::Vec3i local_start;
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

    Planners::utils::OptimizedContinuousFunction ceresOptimizerContinuousPathInitG5(Eigen::VectorXd init_coeff_x, Eigen::VectorXd init_coeff_y, Eigen::VectorXd init_coeff_z, Planners::utils::CoordinateList global_path_local_section, double t_last)
    {
        // Estimation of t_n for each point in the local global path

        double wp_distance_total;
        Planners::utils::Vec3i local_start;
        std::vector<double> dist_list;
        std::vector<double> t_list;

        for(int i=0; i<global_path_local_section.size(); i++)
        {
            if(i == 0)
            {
                double dist = sqrt((global_path_local_section[0].x - init_coeff_x[5]) * (global_path_local_section[0].x - init_coeff_x[5]) + (global_path_local_section[0].y - init_coeff_y[5]) * (global_path_local_section[0].y - init_coeff_y[5]) + (global_path_local_section[0].z - init_coeff_z[5]) * (global_path_local_section[0].z - init_coeff_z[5]));
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
        for (int i = 0; i < 5; i++) {
            coeff_state_vector.parameter[i] = init_coeff_x[i];
            coeff_state_vector.parameter[i + 5] = init_coeff_y[i];
            coeff_state_vector.parameter[i + 10] = init_coeff_z[i];
        }
        
        coeff_state_vector_constant.parameter[0] = init_coeff_x[5];
        coeff_state_vector_constant.parameter[1] = init_coeff_y[5];
        coeff_state_vector_constant.parameter[2] = init_coeff_z[5];

        ceres::Problem problem;

        std::cout << "Created Ceres Problem" << std::endl;

        // Cost function weights

        double weight_distance_to_wp = 10.0;
        double weight_smoothness = 5.0;

        // 1. Cost Function - Distance to waypoints

        for (size_t i = 0; i < global_path_local_section.size(); i++)
        {
            ceres::CostFunction* distance_to_wp = new AutoDiffCostFunction<DistanceToWPG5Functor, 1, 15, 3>
                                                        (new DistanceToWPG5Functor(weight_distance_to_wp, global_path_local_section[i], t_list[i]));

            problem.AddResidualBlock(distance_to_wp, nullptr, coeff_state_vector.parameter, coeff_state_vector_constant.parameter);

        }

        // 2. Cost Function - Smoothness

        ceres::CostFunction* smoothness_function = new AutoDiffCostFunction<SmoothnessContInitG5Functor, 1, 15>(new SmoothnessContInitG5Functor(weight_smoothness));
    
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

        std::cout << "Returning to main function" << std::endl;

        return optimized_coeffs;
    }
}

