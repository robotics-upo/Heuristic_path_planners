#ifndef CERES_CONSTRAINTS_6_CONT_DIST_TO_OBSTACLE_SEGMENT
#define CERES_CONSTRAINTS_6_CONT_DIST_TO_OBSTACLE_SEGMENT

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


class CeresESDFUpdateReducedChebyshev : public ceres::EvaluationCallback {
public:
    CeresESDFUpdateReducedChebyshev(parameterBlockReducedChebyshev& coeff_state_vector,
                            int esdf_samp,
                            torch::jit::script::Module& loaded_sdf,
                            double origin_x,
                            double origin_y,
                            double origin_z,
                            Planners::utils::Vec3i local_start,
                            Planners::utils::Vec3i local_goal,
                            float resolution,
                            std::shared_ptr<voxblox::EsdfMap>& esdf_map,
                            bool use_voxfield)
        : coeff_state_vector_(coeff_state_vector),
          esdf_samp_(esdf_samp),
          loaded_sdf_(loaded_sdf),
          origin_x_(origin_x),
          origin_y_(origin_y),
          origin_z_(origin_z),
          local_start_(local_start),
          local_goal_(local_goal),
          resolution_(resolution),
          esdf_map_(esdf_map),
          use_voxfield_(use_voxfield)
    {
        residuals_ = Eigen::VectorXd::Zero(esdf_samp_);
        jacobians_ = Eigen::MatrixXd::Zero(esdf_samp_, 3);
        PrepareForEvaluation(true, true);
        std::cout << "Evaluation Callback Created(using "
                  << (use_voxfield_ ? "Voxfield ESDF" : "Neural Network ESDF")
                  << ")" << std::endl;
    }

    void PrepareForEvaluation(bool evaluate_jacobians, bool new_evaluation_point) final {
        int num_points = esdf_samp_;
        Eigen::VectorXd local_residuals(esdf_samp_);
        Eigen::MatrixXd local_jacobians(esdf_samp_, 3);
        // std::vector<double> grad_norms;
        // grad_norms.reserve(esdf_samp_);

        double p0x = 0.5*(local_goal_.x + local_start_.x) - coeff_state_vector_.parameter[1];
        double p1x = 2.5*(local_goal_.x - local_start_.x) - 8.0*coeff_state_vector_.parameter[0] - 4.0*coeff_state_vector_.parameter[2];
        double p2x = -4.0*(local_goal_.x + local_start_.x) + 10.0*coeff_state_vector_.parameter[1] + 8.0*coeff_state_vector_.parameter[3];
        double p3x = -10.0*(local_goal_.x - local_start_.x) + 24.0*coeff_state_vector_.parameter[0] + 20.0*coeff_state_vector_.parameter[2];
        double p4x = 4.0*(local_goal_.x + local_start_.x) - 8.0*coeff_state_vector_.parameter[1] - 8.0*coeff_state_vector_.parameter[3];
        double p5x = 8.0*(local_goal_.x - local_start_.x) - 16.0*coeff_state_vector_.parameter[0] - 16.0*coeff_state_vector_.parameter[2];
        double p0y = 0.5*(local_goal_.y + local_start_.y) - coeff_state_vector_.parameter[5];
        double p1y = 2.5*(local_goal_.y - local_start_.y) - 8.0*coeff_state_vector_.parameter[4] - 4.0*coeff_state_vector_.parameter[6];
        double p2y = -4.0*(local_goal_.y + local_start_.y) + 10.0*coeff_state_vector_.parameter[5] + 8.0*coeff_state_vector_.parameter[7];
        double p3y = -10.0*(local_goal_.y - local_start_.y) + 24.0*coeff_state_vector_.parameter[4] + 20.0*coeff_state_vector_.parameter[6];
        double p4y = 4.0*(local_goal_.y + local_start_.y) - 8.0*coeff_state_vector_.parameter[5] - 8.0*coeff_state_vector_.parameter[7];
        double p5y = 8.0*(local_goal_.y - local_start_.y) - 16.0*coeff_state_vector_.parameter[4] - 16.0*coeff_state_vector_.parameter[6];
        double p0z = 0.5*(local_goal_.z + local_start_.z) - coeff_state_vector_.parameter[9];
        double p1z = 2.5*(local_goal_.z - local_start_.z) - 8.0*coeff_state_vector_.parameter[8] - 4.0*coeff_state_vector_.parameter[10];
        double p2z = -4.0*(local_goal_.z + local_start_.z) + 10.0*coeff_state_vector_.parameter[9] + 8.0*coeff_state_vector_.parameter[11];
        double p3z = -10.0*(local_goal_.z - local_start_.z) + 24.0*coeff_state_vector_.parameter[8] + 20.0*coeff_state_vector_.parameter[10];
        double p4z = 4.0*(local_goal_.z + local_start_.z) - 8.0*coeff_state_vector_.parameter[9] - 8.0*coeff_state_vector_.parameter[11];
        double p5z = 8.0*(local_goal_.z - local_start_.z) - 16.0*coeff_state_vector_.parameter[8] - 16.0*coeff_state_vector_.parameter[10];

        //std::vector<Eigen::Vector3d> coords;
        //coords.clear();

        for (int i = 0; i < esdf_samp_; ++i) {
            double s_esdf_act = 2.0 * (i + 1.0) / (esdf_samp_ + 1.0) - 1.0;

            Eigen::Vector3d coord;

            coord[0] = (p0x + s_esdf_act*(p1x + s_esdf_act*(p2x + s_esdf_act*(p3x + s_esdf_act*(p4x + s_esdf_act*p5x))))) * resolution_ + origin_x_;
            coord[1] = (p0y + s_esdf_act*(p1y + s_esdf_act*(p2y + s_esdf_act*(p3y + s_esdf_act*(p4y + s_esdf_act*p5y))))) * resolution_ + origin_y_;
            coord[2] = (p0z + s_esdf_act*(p1z + s_esdf_act*(p2z + s_esdf_act*(p3z + s_esdf_act*(p4z + s_esdf_act*p5z))))) * resolution_ + origin_z_;
            //coords.push_back(coord);
            double dist = 0.0;
            Eigen::Vector3d grad = Eigen::Vector3d::Zero();

            if (use_voxfield_ && esdf_map_) {
                // --- Query ESDF distance with interpolation ---
                bool valid = esdf_map_->getDistanceAtPosition(coord, true, &dist);
                if (!valid) dist = 0.05;  // fallback for unknown space

                // --- Approximate gradient via central finite differences ---
                const double eps = 0.05;
                for (int d = 0; d < 3; ++d) {
                    Eigen::Vector3d plus = coord;
                    Eigen::Vector3d minus = coord;
                    plus[d] += eps;
                    minus[d] -= eps;

                    double dist_plus = 0.0, dist_minus = 0.0;
                    esdf_map_->getDistanceAtPosition(plus, true, &dist_plus);
                    esdf_map_->getDistanceAtPosition(minus, true, &dist_minus);

                    grad[d] = (dist_plus - dist_minus) / (2.0 * eps);
                }
            } else {
                // --- Fallback: neural network evaluation ---
                torch::Tensor coord_tensor = torch::from_blob(coord.data(), {1, 3}, torch::kFloat64)
                                                .clone()
                                                .to(torch::kFloat32);
                coord_tensor.set_requires_grad(true);
                torch::Tensor output_tensor = loaded_sdf_.forward({coord_tensor}).toTensor();
                output_tensor.backward();
                auto grad_tensor = coord_tensor.grad();
                dist = output_tensor.item<float>();
                grad = Eigen::Map<Eigen::Vector3f>(grad_tensor.data_ptr<float>(), 3).cast<double>();
            }

            local_residuals(i) = dist;
            local_jacobians.row(i) = grad;

            // // Save grad module and print them all when they're ready
            // grad_norms.push_back(grad.norm());

            // // Cuando llegamos al último índice, imprimimos todo
            // if (i == num_points - 1) {
            //     std::cout << "Norma y vectores de gradiente por punto:" << std::endl;
            //     for (int j = 0; j < num_points; ++j) {
            //         double norm = local_jacobians.row(j).norm();
            //         Eigen::Vector3d grad_vec = local_jacobians.row(j);

            //         std::cout << norm << "  " << grad_vec.transpose() << std::endl;
            //     }
            // }
            // if (i == esdf_samp_ - 1) {
            //     std::cout << "Coordenadas evaluadas:" << std::endl;
            //     for (int j = 0; j < coords.size(); ++j) {
            //         std::cout << "Punto " << j << ": " << coords[j].transpose() << std::endl;
            //     }
            // }
        }
        std::cout << "Distances: " << local_residuals.transpose() << std::endl;

        residuals_ = local_residuals;
        jacobians_ = local_jacobians;
    }

    const Eigen::VectorXd& residuals() const { return residuals_; }
    const Eigen::MatrixXd& jacobians() const { return jacobians_; }

private:
    parameterBlockReducedChebyshev& coeff_state_vector_;
    int esdf_samp_;
    double origin_x_, origin_y_, origin_z_;
    Planners::utils::Vec3i local_start_, local_goal_;
    float resolution_;
    torch::jit::script::Module& loaded_sdf_;
    Eigen::VectorXd residuals_;
    Eigen::MatrixXd jacobians_;
    std::shared_ptr<voxblox::EsdfMap> esdf_map_;
    bool use_voxfield_;
};



class Ceres6DistanceFunctionSegment : public SizedCostFunction<1, 3>
{
    public:
        Ceres6DistanceFunctionSegment(const CeresESDFUpdateReducedChebyshev& evaluation_callback_reduced_cheb, int index): evaluation_callback_reduced_cheb_(evaluation_callback_reduced_cheb), index_(index)
        {}

        virtual ~Ceres6DistanceFunctionSegment(void)
        {}

        virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
        {
            auto dist = evaluation_callback_reduced_cheb_.residuals()(index_);
            residuals[0] = dist;
            if (jacobians != nullptr && jacobians[0] != nullptr)
                {
                    jacobians[0][0] = evaluation_callback_reduced_cheb_.jacobians()(index_, 0);
                    jacobians[0][1] = evaluation_callback_reduced_cheb_.jacobians()(index_, 1);
                    jacobians[0][2] = evaluation_callback_reduced_cheb_.jacobians()(index_, 2);
                }

            return true;
        }

        const CeresESDFUpdateReducedChebyshev& evaluation_callback_reduced_cheb_;
        int index_;
    private:
};


class Ceres6_ObstacleDistanceCostContSegmentFunctor
{
 public:
    Ceres6_ObstacleDistanceCostContSegmentFunctor(const CeresESDFUpdateReducedChebyshev& evaluation_callback_reduced_cheb, int index, double s_act, int esdf_samp, Planners::utils::Vec3i local_start, Planners::utils::Vec3i local_goal, double weight)
      : evaluation_callback_reduced_cheb_(evaluation_callback_reduced_cheb), index_(index), s_act_(s_act), esdf_samp_(esdf_samp), local_start_(local_start), local_goal_(local_goal), weight_(weight), distanceFunctor_(new Ceres6DistanceFunctionSegment(evaluation_callback_reduced_cheb_, index_))
    {
    }

    virtual ~Ceres6_ObstacleDistanceCostContSegmentFunctor(void) 
    {
    }

    template <typename T>
    bool operator()(const T* const stateCoeff, T* residual) const
    {   
        T p[3], dist;

        T p0x = 0.5*(local_goal_.x + local_start_.x) - stateCoeff[1];
        T p1x = 2.5*(local_goal_.x - local_start_.x) - 8.0*stateCoeff[0] - 4.0*stateCoeff[2];
        T p2x = -4.0*(local_goal_.x + local_start_.x) + 10.0*stateCoeff[1] + 8.0*stateCoeff[3];
        T p3x = -10.0*(local_goal_.x - local_start_.x) + 24.0*stateCoeff[0] + 20.0*stateCoeff[2];
        T p4x = 4.0*(local_goal_.x + local_start_.x) - 8.0*stateCoeff[1] - 8.0*stateCoeff[3];
        T p5x = 8.0*(local_goal_.x - local_start_.x) - 16.0*stateCoeff[0] - 16.0*stateCoeff[2];
        T p0y = 0.5*(local_goal_.y + local_start_.y) - stateCoeff[5];
        T p1y = 2.5*(local_goal_.y - local_start_.y) - 8.0*stateCoeff[4] - 4.0*stateCoeff[6];
        T p2y = -4.0*(local_goal_.y + local_start_.y) + 10.0*stateCoeff[5] + 8.0*stateCoeff[7];
        T p3y = -10.0*(local_goal_.y - local_start_.y) + 24.0*stateCoeff[4] + 20.0*stateCoeff[6];
        T p4y = 4.0*(local_goal_.y + local_start_.y) - 8.0*stateCoeff[5] - 8.0*stateCoeff[7];
        T p5y = 8.0*(local_goal_.y - local_start_.y) - 16.0*stateCoeff[4] - 16.0*stateCoeff[6];
        T p0z = 0.5*(local_goal_.z + local_start_.z) - stateCoeff[9];
        T p1z = 2.5*(local_goal_.z - local_start_.z) - 8.0*stateCoeff[8] - 4.0*stateCoeff[10];
        T p2z = -4.0*(local_goal_.z + local_start_.z) + 10.0*stateCoeff[9] + 8.0*stateCoeff[11];
        T p3z = -10.0*(local_goal_.z - local_start_.z) + 24.0*stateCoeff[8] + 20.0*stateCoeff[10];
        T p4z = 4.0*(local_goal_.z + local_start_.z) - 8.0*stateCoeff[9] - 8.0*stateCoeff[11];
        T p5z = 8.0*(local_goal_.z - local_start_.z) - 16.0*stateCoeff[8] - 16.0*stateCoeff[10];

        p[0] = p0x + s_act_*(p1x + s_act_*(p2x + s_act_*(p3x + s_act_*(p4x + s_act_*p5x))));
        p[1] = p0y + s_act_*(p1y + s_act_*(p2y + s_act_*(p3y + s_act_*(p4y + s_act_*p5y))));
        p[2] = p0z + s_act_*(p1z + s_act_*(p2z + s_act_*(p3z + s_act_*(p4z + s_act_*p5z))));

        // Compute distance
        distanceFunctor_(p, &dist);


        // Compute weight
        // residual[0] = T(weight_) / T(esdf_samp_) * exp(T(-4) * (dist - T(1.5)));
        residual[0] = T(weight_) / T(esdf_samp_) * exp(T(-5.0) * (dist - T(1.5)));



        return true;
    }

  private:

    // Constraint weighting and t_act
    double weight_, s_act_;

    int esdf_samp_, index_;

    Planners::utils::Vec3i local_start_, local_goal_;

    const CeresESDFUpdateReducedChebyshev& evaluation_callback_reduced_cheb_;

    // Distance funtion diferentiation
    ceres::CostFunctionToFunctor<1, 3> distanceFunctor_;
};








#endif