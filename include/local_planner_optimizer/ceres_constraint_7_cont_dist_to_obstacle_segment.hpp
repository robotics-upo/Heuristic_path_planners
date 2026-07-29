#ifndef CERES_CONSTRAINTS_7_CONT_DIST_TO_OBSTACLE_SEGMENT
#define CERES_CONSTRAINTS_7_CONT_DIST_TO_OBSTACLE_SEGMENT

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
#include <memory>
#include <mutex>


#include <heuristic_planners/Vec3i.h>
#include <heuristic_planners/CoordinateList.h>

#include "Grid3D/local_grid3d.hpp"

#include <ceres/ceres.h>

#include <voxblox_ros/conversions.h>
#include <voxblox/core/esdf_map.h>


using ceres::SizedCostFunction;
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;


class CeresESDFUpdateChebyshevTime : public ceres::EvaluationCallback {
public:
    CeresESDFUpdateChebyshevTime(parameterBlockChebyshevTime& coeff_state_vector,
                            int esdf_samp,
                            torch::jit::script::Module& loaded_sdf,
                            double origin_x,
                            double origin_y,
                            double origin_z,
                            float resolution,
                            std::shared_ptr<voxblox::EsdfMap>& esdf_map,
                            bool use_voxfield)
        : coeff_state_vector_(coeff_state_vector),
          esdf_samp_(esdf_samp),
          loaded_sdf_(loaded_sdf),
          origin_x_(origin_x),
          origin_y_(origin_y),
          origin_z_(origin_z),
          resolution_(resolution),
          esdf_map_(esdf_map),
          use_voxfield_(use_voxfield)
    {
        residuals_ = Eigen::VectorXd::Zero(esdf_samp_);
        jacobians_ = Eigen::MatrixXd::Zero(esdf_samp_, 3);
        PrepareForEvaluation(true, true);
    }
    void PrepareForEvaluation(bool evaluate_jacobians, bool new_evaluation_point) final {
        int num_points = esdf_samp_;
        Eigen::VectorXd local_residuals(esdf_samp_);
        Eigen::MatrixXd local_jacobians(esdf_samp_, 3);

        double p0x = coeff_state_vector_.parameter[5] - coeff_state_vector_.parameter[3] + coeff_state_vector_.parameter[1];
        double p1x = coeff_state_vector_.parameter[4] - 3.0*coeff_state_vector_.parameter[2] + 5.0*coeff_state_vector_.parameter[0];
        double p2x = 2.0*coeff_state_vector_.parameter[3] - 8.0*coeff_state_vector_.parameter[1];
        double p3x = 4.0*coeff_state_vector_.parameter[2] - 20.0*coeff_state_vector_.parameter[0];
        double p4x = 8.0*coeff_state_vector_.parameter[1];
        double p5x = 16.0*coeff_state_vector_.parameter[0];
        double p0y = coeff_state_vector_.parameter[11] - coeff_state_vector_.parameter[9] + coeff_state_vector_.parameter[7];
        double p1y = coeff_state_vector_.parameter[10] - 3.0*coeff_state_vector_.parameter[8] + 5.0*coeff_state_vector_.parameter[6];
        double p2y = 2.0*coeff_state_vector_.parameter[9] - 8.0*coeff_state_vector_.parameter[7];
        double p3y = 4.0*coeff_state_vector_.parameter[8] - 20.0*coeff_state_vector_.parameter[6];
        double p4y = 8.0*coeff_state_vector_.parameter[7];
        double p5y = 16.0*coeff_state_vector_.parameter[6];
        double p0z = coeff_state_vector_.parameter[17] - coeff_state_vector_.parameter[15] + coeff_state_vector_.parameter[13];
        double p1z = coeff_state_vector_.parameter[16] - 3.0*coeff_state_vector_.parameter[14] + 5.0*coeff_state_vector_.parameter[12];
        double p2z = 2.0*coeff_state_vector_.parameter[15] - 8.0*coeff_state_vector_.parameter[13];
        double p3z = 4.0*coeff_state_vector_.parameter[14] - 20.0*coeff_state_vector_.parameter[12];
        double p4z = 8.0*coeff_state_vector_.parameter[13];
        double p5z = 16.0*coeff_state_vector_.parameter[12];

        std::vector<Eigen::Vector3d> coords;
        coords.reserve(num_points);
        for (int i = 0; i < num_points; ++i) {
            double s_esdf_act = 2.0 * (i + 1.0) / (num_points + 1.0) - 1.0;
            Eigen::Vector3d coord;
            coord[0] = (p0x + s_esdf_act*(p1x + s_esdf_act*(p2x + s_esdf_act*(p3x + s_esdf_act*(p4x + s_esdf_act*p5x))))) * resolution_ + origin_x_;
            coord[1] = (p0y + s_esdf_act*(p1y + s_esdf_act*(p2y + s_esdf_act*(p3y + s_esdf_act*(p4y + s_esdf_act*p5y))))) * resolution_ + origin_y_;
            coord[2] = (p0z + s_esdf_act*(p1z + s_esdf_act*(p2z + s_esdf_act*(p3z + s_esdf_act*(p4z + s_esdf_act*p5z))))) * resolution_ + origin_z_;
            coords.push_back(coord);
        }

        if (use_voxfield_ && esdf_map_) {
            const double eps = 0.05;
            for (int i = 0; i < num_points; ++i) {
                const Eigen::Vector3d& coord = coords[i];
                double dist = 0.0;
                bool valid = esdf_map_->getDistanceAtPosition(coord, true, &dist);
                if (!valid) dist = 0.05;
                Eigen::Vector3d grad = Eigen::Vector3d::Zero();
                for (int d = 0; d < 3; ++d) {
                    Eigen::Vector3d plus = coord, minus = coord;
                    plus[d] += eps; minus[d] -= eps;
                    double dp = 0.0, dm = 0.0;
                    esdf_map_->getDistanceAtPosition(plus, true, &dp);
                    esdf_map_->getDistanceAtPosition(minus, true, &dm);
                    grad[d] = (dp - dm) / (2.0 * eps);
                }
                local_residuals(i) = dist;
                local_jacobians.row(i) = grad;
            }
        } else {
            torch::Tensor coords_tensor = torch::empty({num_points, 3}, torch::kFloat32);
            for (int i = 0; i < num_points; ++i)
                coords_tensor[i] = torch::tensor({(float)coords[i][0], (float)coords[i][1], (float)coords[i][2]});
            coords_tensor.set_requires_grad(true);

            torch::Tensor output_tensor = loaded_sdf_.forward({coords_tensor}).toTensor(); // [N, 1] o [N]
            if (output_tensor.dim() == 2 && output_tensor.size(1) == 1)
                output_tensor = output_tensor.squeeze(1);

            torch::Tensor grad_tensor = torch::autograd::grad({output_tensor.sum()}, {coords_tensor})[0];

            auto dist_cpu = output_tensor.to(torch::kCPU);
            auto grad_cpu = grad_tensor.to(torch::kCPU);
            auto dist_acc = dist_cpu.accessor<float,1>();
            auto grad_acc = grad_cpu.accessor<float,2>();

            for (int i = 0; i < num_points; ++i) {
                local_residuals(i) = dist_acc[i];
                local_jacobians(i,0) = grad_acc[i][0];
                local_jacobians(i,1) = grad_acc[i][1];
                local_jacobians(i,2) = grad_acc[i][2];
            }
        }

        residuals_ = local_residuals;
        jacobians_ = local_jacobians;
    }

    const Eigen::VectorXd& residuals() const { return residuals_; }
    const Eigen::MatrixXd& jacobians() const { return jacobians_; }

private:
    parameterBlockChebyshevTime& coeff_state_vector_;
    int esdf_samp_;
    double origin_x_, origin_y_, origin_z_;
    float resolution_;
    torch::jit::script::Module& loaded_sdf_;
    Eigen::VectorXd residuals_;
    Eigen::MatrixXd jacobians_;
    std::shared_ptr<voxblox::EsdfMap> esdf_map_;
    bool use_voxfield_;
};



class Ceres7DistanceFunctionSegment : public SizedCostFunction<1, 3>
{
    public:
        Ceres7DistanceFunctionSegment(const CeresESDFUpdateChebyshevTime& evaluation_callback_cheb_time, int index): evaluation_callback_cheb_time_(evaluation_callback_cheb_time), index_(index)
        {}

        virtual ~Ceres7DistanceFunctionSegment(void)
        {}

        virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
        {
            auto dist = evaluation_callback_cheb_time_.residuals()(index_);
            residuals[0] = dist;
            if (jacobians != nullptr && jacobians[0] != nullptr)
                {
                    jacobians[0][0] = evaluation_callback_cheb_time_.jacobians()(index_, 0);
                    jacobians[0][1] = evaluation_callback_cheb_time_.jacobians()(index_, 1);
                    jacobians[0][2] = evaluation_callback_cheb_time_.jacobians()(index_, 2);
                }

            return true;
        }

        const CeresESDFUpdateChebyshevTime& evaluation_callback_cheb_time_;
        int index_;
    private:
};


class Ceres7_ObstacleDistanceCostContSegmentFunctor
{
 public:
    Ceres7_ObstacleDistanceCostContSegmentFunctor(const CeresESDFUpdateChebyshevTime& evaluation_callback_cheb_time, int index, double s_act, int esdf_samp = 10, double weight = 1.0)
      : evaluation_callback_cheb_time_(evaluation_callback_cheb_time), index_(index), s_act_(s_act), esdf_samp_(esdf_samp), weight_(weight), distanceFunctor_(new Ceres7DistanceFunctionSegment(evaluation_callback_cheb_time_, index_))
    {
    }

    virtual ~Ceres7_ObstacleDistanceCostContSegmentFunctor(void) 
    {
    }

    template <typename T>
    bool operator()(const T* const stateCoeff, T* residual) const
    {   
        T p[3], dist;

        T p0x = stateCoeff[5] - stateCoeff[3] + stateCoeff[1];
        T p1x = stateCoeff[4] - 3.0*stateCoeff[2] + 5.0*stateCoeff[0];
        T p2x = 2.0*stateCoeff[3] - 8.0*stateCoeff[1];
        T p3x = 4.0*stateCoeff[2] - 20.0*stateCoeff[0];
        T p4x = 8.0*stateCoeff[1];
        T p5x = 16.0*stateCoeff[0];
        T p0y = stateCoeff[11] - stateCoeff[9] + stateCoeff[7];
        T p1y = stateCoeff[10] - 3.0*stateCoeff[8] + 5.0*stateCoeff[6];
        T p2y = 2.0*stateCoeff[9] - 8.0*stateCoeff[7];
        T p3y = 4.0*stateCoeff[8] - 20.0*stateCoeff[6];
        T p4y = 8.0*stateCoeff[7];
        T p5y = 16.0*stateCoeff[6];
        T p0z = stateCoeff[17] - stateCoeff[15] + stateCoeff[13];
        T p1z = stateCoeff[16] - 3.0*stateCoeff[14] + 5.0*stateCoeff[12];
        T p2z = 2.0*stateCoeff[15] - 8.0*stateCoeff[13];
        T p3z = 4.0*stateCoeff[14] - 20.0*stateCoeff[12];
        T p4z = 8.0*stateCoeff[13];
        T p5z = 16.0*stateCoeff[12];

        p[0] = p0x + s_act_*(p1x + s_act_*(p2x + s_act_*(p3x + s_act_*(p4x + s_act_*p5x))));
        p[1] = p0y + s_act_*(p1y + s_act_*(p2y + s_act_*(p3y + s_act_*(p4y + s_act_*p5y))));
        p[2] = p0z + s_act_*(p1z + s_act_*(p2z + s_act_*(p3z + s_act_*(p4z + s_act_*p5z))));

        distanceFunctor_(p, &dist);


        residual[0] = T(weight_) / T(esdf_samp_) * exp(T(-4.0) * (dist - T(1.5)));

        return true;
    }

  private:

    double weight_, s_act_;

    int esdf_samp_, index_;

    const CeresESDFUpdateChebyshevTime& evaluation_callback_cheb_time_;

    ceres::CostFunctionToFunctor<1, 3> distanceFunctor_;
};








#endif