#ifndef CERES_CONSTRAINTS_CONT_DIST_TO_OBSTACLE
#define CERES_CONSTRAINTS_CONT_DIST_TO_OBSTACLE

#include <iostream>
#include <fstream>
#include <string>
#include "utils/ros/ROSInterfaces.hpp"
#include "utils/SaveDataVariantToFile.hpp"
#include "utils/misc.hpp"
#include "utils/geometry_utils.hpp"
#include "utils/metrics.hpp"
#include <ros/ros.h>
#include <Eigen/Dense>


#include <heuristic_planners/Vec3i.h>
#include <heuristic_planners/CoordinateList.h>

#include "Grid3D/local_grid3d.hpp"

#include <ceres/ceres.h>

#define SEGMENTS 10
#define t_final 10.0

using ceres::SizedCostFunction;
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

class DistanceFunction : public SizedCostFunction<1, 3>
{
    public:
        DistanceFunction(Local_Grid3d &grid_): g_3D(grid_)
        {}

        virtual ~DistanceFunction(void)
        {}

        virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
        {
            residuals[0] = 0;
            double x = parameters[0][0] * g_3D.m_resolution;
            double y = parameters[0][1] * g_3D.m_resolution;
            double z = parameters[0][2] * g_3D.m_resolution;
            double dist_;
            if(g_3D.isIntoMap(x, y, z))
            {
                TrilinearParams p = g_3D.computeDistInterpolation(x, y, z);
                dist_ = p.a0 + p.a1*x + p.a2*y + p.a3*z + p.a4*x*y + p.a5*x*z + p.a6*y*z + p.a7*x*y*z;
                if(dist_ < 0.01)
                    dist_ = 0.01;
                residuals[0] = 1/dist_;
                if (jacobians != NULL && jacobians[0] != NULL)
                {
                    int cte = -1/(dist_*dist_);
                    jacobians[0][0] = cte*(p.a1 + p.a4*y + p.a5*z + p.a7*y*z);
                    jacobians[0][1] = cte*(p.a2 + p.a4*x + p.a6*z + p.a7*x*z);
                    jacobians[0][2] = cte*(p.a3 + p.a5*x + p.a6*y + p.a7*x*y);
                }
            }
            else // This might give an error
            {
                std::cout << "Cont - Ceres found point outside map" << std::endl;
                residuals[0] = 100;
                if (jacobians != nullptr && jacobians[0] != nullptr)
                {
                    jacobians[0][0] = 0.0;
                    jacobians[0][1] = 0.0;
                    jacobians[0][2] = 0.0;
                }
            }
            return true;
        }

        Local_Grid3d &g_3D;

    private:
};




class ObstacleDistanceCostContFunctor
{
 public:
    ObstacleDistanceCostContFunctor(Local_Grid3d &grid, double weight = 1.0)
      : grid_(grid), weight_(weight), distanceFunctor_(new DistanceFunction(grid))
    {
    }

    virtual ~ObstacleDistanceCostContFunctor(void) 
    {
    }

    template <typename T>
    bool operator()(const T* const stateCoeff, const T* const stateCoeffConstant, T* residual) const
    {   
        residual[0] = T(0);
        T t_step = T(t_final) / T(SEGMENTS);

        // For each sampled point along the function, calculate 1/dist and add it (weighted) to the residual 

        for (int i = 0; i < SEGMENTS; i++)
        {
            T p[3], invdist;

            T t_act = t_step * T(i+1);


            p[0] = stateCoeff[0] * ceres::pow(t_act, 3) + stateCoeff[1] * ceres::pow(t_act, 2) + stateCoeff[2] * t_act + stateCoeffConstant[0];
            p[1] = stateCoeff[3] * ceres::pow(t_act, 3) + stateCoeff[4] * ceres::pow(t_act, 2) + stateCoeff[5] * t_act + stateCoeffConstant[1];
            p[2] = stateCoeff[6] * ceres::pow(t_act, 3) + stateCoeff[7] * ceres::pow(t_act, 2) + stateCoeff[8] * t_act + stateCoeffConstant[2];

            // Compute distance
            distanceFunctor_(p, &invdist);

            // Compute weigthed residual
            residual[0] += weight_ * invdist;
        }

        // Normalization of the residual
        residual[0] = residual[0] / T(SEGMENTS); 

        return true;
    }

  private:

    // Constraint weighting
    double weight_;

    // Distance grid
    Local_Grid3d &grid_;

    // Distance funtion diferenciation
    ceres::CostFunctionToFunctor<1, 3> distanceFunctor_;
};

#endif