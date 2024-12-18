#include "utils/CeresOpt.hpp"

namespace Ceresopt
{
    std::vector<double> InitVelCalculator(std::vector<parameterBlockWP> wp_state_vector, double desired_vel, int num_wp, float res){
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

    Planners::utils::CoordinateList ceresOptimizer(Planners::utils::CoordinateList initial_path, Local_Grid3d &_grid, float res)
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
            for(size_t i = 3; i < 6; ++i){
                newWP.parameter[i] = 0;
            }
            wp_state_vector.push_back(newWP);
        }
        
        // Initialize velocity (VELOCITIES IN STATE VECTOR ARE CELL VELOCITIES!!!)
            // Decide total travel time (seconds)
        //float travel_time = 4;
            // Decide desired velocity for the entire path
        double desired_vel = 0.5; // (m/s)
            // Compute initial velocities
        //std::vector<double> inivel_vector = InitVelCalculator(wp_state_vector, travel_time, num_wp, res);
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

        double weight_equidistance = 1.0;
        double weight_path_length = 1.0;
        double weight_esdf = 10.0;
        double weight_smoothness = 1.0;
        double weight_velocity_module = 1.0;
        double weight_min_acceleration = 1.0;
        double weight_pos_vel_coherence = 1.0;

        // Define cost functions

        // 1 - Equidistance cost function + Smoothness cost function --> Tries to maintain equal distance between WP and avoid big changes in direction
        for (int i = 0; i < wp_state_vector.size() - 2; i++)
        {
            ceres::CostFunction* equidistance_function = new AutoDiffCostFunction<EquidistanceFunctor, 1, 6, 6, 6>
                                                        (new EquidistanceFunctor(weight_equidistance));
            ceres::CostFunction* smoothness_function = new AutoDiffCostFunction<SmoothnessFunctor, 1, 6, 6, 6>
                                                        (new SmoothnessFunctor(weight_smoothness));
            problem.AddResidualBlock(equidistance_function, nullptr, wp_state_vector[i].parameter, wp_state_vector[i+1].parameter, wp_state_vector[i+2].parameter);
            problem.AddResidualBlock(smoothness_function, nullptr, wp_state_vector[i].parameter, wp_state_vector[i+1].parameter, wp_state_vector[i+2].parameter);
        }

        // 2 - Path length cost function --> Tries to minimize path length
        for (int i = 0; i < wp_state_vector.size() - 1; i++)
        {
            ceres::CostFunction* path_length_function = new AutoDiffCostFunction<PathLengthFunctor, 1, 6, 6>
                                                        (new PathLengthFunctor(weight_path_length));
            problem.AddResidualBlock(path_length_function, nullptr, wp_state_vector[i].parameter, wp_state_vector[i+1].parameter);
        }


        // 3 - Distance to obstacles cost function --> Tries to maintain the biggest distance to obstacles possible
        for (int i = 0; i < wp_state_vector.size(); i++)
        {
            ceres::CostFunction* esdf_function = new ObstacleDistanceCostFunctor(&_grid, weight_esdf);
            problem.AddResidualBlock(esdf_function, nullptr, wp_state_vector[i].parameter);

        }


        // 4 - Velocity module function --> Tries to maintain the desired velocity module
        for (int i = 0; i < wp_state_vector.size() - 1; i++)
        {
            ceres::CostFunction* velocity_change_function = new AutoDiffCostFunction<VelocityChangeFunctor, 1, 6>
                                                        (new VelocityChangeFunctor(weight_velocity_module, desired_vel));
            problem.AddResidualBlock(velocity_change_function, nullptr, wp_state_vector[i].parameter);
        }

        // 5 - Minimize acceleration function --> Tries to minimize acceleration along the trajectory

        for (int i = 0; i < wp_state_vector.size() - 2; i++)
        {
            ceres::CostFunction* min_acceleration_function = new AutoDiffCostFunction<MinAccelerationFunctor, 1, 6, 6>
                                                        (new MinAccelerationFunctor(weight_min_acceleration));
            problem.AddResidualBlock(min_acceleration_function, nullptr, wp_state_vector[i].parameter, wp_state_vector[i+1].parameter);
        }

        // 6 - Position-velocity coherence function --> Maintains the position-velocity direction coherence

        for (int i = 0; i < wp_state_vector.size() - 2; i++)
        {
            ceres::CostFunction* pos_vel_coherence_function = new AutoDiffCostFunction<PosVelCoherenceFunctor, 1, 6, 6>
                                                        (new PosVelCoherenceFunctor(weight_pos_vel_coherence));
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
        Planners::utils::CoordinateList optimized_path;
        for (int i=0; i<num_wp; i++){
            Planners::utils::Vec3i newpoint;
            newpoint.x = wp_state_vector[i].parameter[0];
            newpoint.y = wp_state_vector[i].parameter[1];
            newpoint.z = wp_state_vector[i].parameter[2];
            optimized_path.push_back(newpoint);
        }

        std::cout << "Returning to main function" << std::endl;
        return optimized_path;
    }
}

