#ifndef CERES_CONSTRAINTS_DIST_TO_OBSTACLE
#define CERES_CONSTRAINTS_DIST_TO_OBSTACLE

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

using ceres::SizedCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

class ObstacleDistanceCostFunctor : public SizedCostFunction<1, 6>
    {
    public:
        ObstacleDistanceCostFunctor(Local_Grid3d* grid_, double weight): g_3D(grid_), weight_(weight)
        {}

        virtual ~ObstacleDistanceCostFunctor(void)
        {}

        virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
        {
            residuals[0] = 0;
            float x = parameters[0][0] * g_3D->m_resolution;
            float y = parameters[0][1] * g_3D->m_resolution;
            float z = parameters[0][2] * g_3D->m_resolution;
            float dist_;
            if(g_3D->isIntoMap(x, y, z))
            {
                TrilinearParams p = g_3D->computeDistInterpolation(x, y, z);
                dist_ = p.a0 + p.a1*x + p.a2*y + p.a3*z + p.a4*x*y + p.a5*x*z + p.a6*y*z + p.a7*x*y*z;
                if(dist_ > 0.01)
                    residuals[0] = weight_*1/dist_;
                else
                    residuals[0] = weight_*100.0;
                if (jacobians != nullptr && jacobians[0] != nullptr)
                {
                    int cte = -weight_/(dist_*dist_);
                    jacobians[0][0] = cte*(p.a1 + p.a4*y + p.a5*z + p.a7*y*z);
                    jacobians[0][1] = cte*(p.a2 + p.a4*x + p.a6*z + p.a7*x*z);
                    jacobians[0][2] = cte*(p.a3 + p.a5*x + p.a6*y + p.a7*x*y);
                    jacobians[0][3] = 0.0;
                    jacobians[0][4] = 0.0;
                    jacobians[0][5] = 0.0; 
                }
            }
            else // This might give an error
            {
                std::cout << "Dist - Ceres found point outside map" << std::endl;
                residuals[0] = 100;
                if (jacobians != nullptr && jacobians[0] != nullptr)
                {
                    jacobians[0][0] = 0.0;
                    jacobians[0][1] = 0.0;
                    jacobians[0][2] = 0.0;
                    jacobians[0][3] = 0.0;
                    jacobians[0][4] = 0.0;
                    jacobians[0][5] = 0.0;
                }
            }
            return true;
        }

        double weight_;
        Local_Grid3d* g_3D;

    private:
    };

#endif