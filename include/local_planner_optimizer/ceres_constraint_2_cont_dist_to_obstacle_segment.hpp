#ifndef CERES_CONSTRAINTS_2_CONT_DIST_TO_OBSTACLE_SEGMENT
#define CERES_CONSTRAINTS_2_CONT_DIST_TO_OBSTACLE_SEGMENT

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
#include <mutex>


#include <heuristic_planners/Vec3i.h>
#include <heuristic_planners/CoordinateList.h>

#include "Grid3D/local_grid3d.hpp"

#include <ceres/ceres.h>


using ceres::SizedCostFunction;
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

class DistanceFunctionSegment : public SizedCostFunction<1, 3>
{
    public:
        DistanceFunctionSegment(Local_Grid3d &grid_): g_3D(grid_)
        {}

        virtual ~DistanceFunctionSegment(void)
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
                // residuals[0] = 6 - dist_;
                // std::cout << "Coordenadas del punto: " << parameters[0][0] << ", " << parameters[0][1] << ", " << parameters[0][2] << "  ||  Dist: " << dist_ << std::endl; 
                if (jacobians != NULL && jacobians[0] != NULL)
                {
                    int cte = -1/(dist_*dist_);
                    jacobians[0][0] = cte * (p.a1 + p.a4*y + p.a5*z + p.a7*y*z);
                    jacobians[0][1] = cte * (p.a2 + p.a4*x + p.a6*z + p.a7*x*z);
                    jacobians[0][2] = cte * (p.a3 + p.a5*x + p.a6*y + p.a7*x*y);
                }
            }
            else // This might give an error
            {
                static std::mutex cout_mutex;
                std::ostringstream oss;
                oss << "Seg - Ceres found point outside map: x=" << x << ", y=" << y << ", z=" << z << std::endl;
                {
                std::lock_guard<std::mutex> lock(cout_mutex);
                std::cout << oss.str();
                }
                residuals[0] = 6;
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




class Ceres2_ObstacleDistanceCostContSegmentFunctor
{
 public:
    Ceres2_ObstacleDistanceCostContSegmentFunctor(Local_Grid3d &grid, double t_act, int esdf_seg = 10, double weight = 1.0)
      : grid_(grid), t_act_(t_act), esdf_seg_(esdf_seg), weight_(weight), distanceFunctor_(new DistanceFunctionSegment(grid))
    {
    }

    virtual ~Ceres2_ObstacleDistanceCostContSegmentFunctor(void) 
    {
    }

    template <typename T>
    bool operator()(const T* const stateCoeff, const T* const stateCoeffConstant, T* residual) const
    {   
        T p[3], invdist;

        p[0] = stateCoeff[0] * ceres::pow(t_act_, 3) + stateCoeff[1] * ceres::pow(t_act_, 2) + stateCoeff[2] * t_act_ + stateCoeffConstant[0];
        p[1] = stateCoeff[3] * ceres::pow(t_act_, 3) + stateCoeff[4] * ceres::pow(t_act_, 2) + stateCoeff[5] * t_act_ + stateCoeffConstant[1];
        p[2] = stateCoeff[6] * ceres::pow(t_act_, 3) + stateCoeff[7] * ceres::pow(t_act_, 2) + stateCoeff[8] * t_act_ + stateCoeffConstant[2];

        // Compute distance
        distanceFunctor_(p, &invdist);

        // Compute weigthed residual
        residual[0] = weight_ / esdf_seg_ * invdist;

        return true;
    }

  private:

    // Constraint weighting and t_act
    double weight_, t_act_;

    int esdf_seg_;

    // Distance grid
    Local_Grid3d &grid_;

    // Distance funtion diferenciation
    ceres::CostFunctionToFunctor<1, 3> distanceFunctor_;
};

#endif