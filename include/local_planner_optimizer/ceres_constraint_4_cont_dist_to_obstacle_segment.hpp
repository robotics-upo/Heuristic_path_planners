#ifndef CERES_CONSTRAINTS_4_CONT_DIST_TO_OBSTACLE_SEGMENT
#define CERES_CONSTRAINTS_4_CONT_DIST_TO_OBSTACLE_SEGMENT

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


class CeresESDFUpdate : public ceres::EvaluationCallback {
public:
    CeresESDFUpdate(parameterBlockContinuousPath& coeff_state_vector,
                    parameterBlockContinuousPathConstant& coeff_state_vector_const,
                    int esdf_samp,
                    double t_max_esdf_seg,
                    torch::jit::script::Module& loaded_sdf,
                    double origin_x,
                    double origin_y,
                    double origin_z,
                    float resolution,
                    std::shared_ptr<voxblox::EsdfMap>& esdf_map,
                    bool use_voxfield)
        : coeff_state_vector_(coeff_state_vector),
          coeff_state_vector_const_(coeff_state_vector_const),
          esdf_samp_(esdf_samp),
          t_max_esdf_seg_(t_max_esdf_seg),
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

        for (int i = 0; i < esdf_samp_; ++i) {
            double t_esdf_act = t_max_esdf_seg_ * (i + 1) / (esdf_samp_ + 1);

            Eigen::Vector3d coord;
            coord[0] = (coeff_state_vector_.parameter[0] * std::pow(t_esdf_act, 5)
                        + coeff_state_vector_.parameter[1] * std::pow(t_esdf_act, 4)
                        + coeff_state_vector_.parameter[2] * std::pow(t_esdf_act, 3)
                        + coeff_state_vector_.parameter[3] * std::pow(t_esdf_act, 2)
                        + coeff_state_vector_.parameter[4] * t_esdf_act
                        + coeff_state_vector_const_.parameter[0]) * resolution_ + origin_x_;
            coord[1] = (coeff_state_vector_.parameter[5] * std::pow(t_esdf_act, 5)
                        + coeff_state_vector_.parameter[6] * std::pow(t_esdf_act, 4)
                        + coeff_state_vector_.parameter[7] * std::pow(t_esdf_act, 3)
                        + coeff_state_vector_.parameter[8] * std::pow(t_esdf_act, 2)
                        + coeff_state_vector_.parameter[9] * t_esdf_act
                        + coeff_state_vector_const_.parameter[1]) * resolution_ + origin_y_;
            coord[2] = (coeff_state_vector_.parameter[10] * std::pow(t_esdf_act, 5)
                        + coeff_state_vector_.parameter[11] * std::pow(t_esdf_act, 4)
                        + coeff_state_vector_.parameter[12] * std::pow(t_esdf_act, 3)
                        + coeff_state_vector_.parameter[13] * std::pow(t_esdf_act, 2)
                        + coeff_state_vector_.parameter[14] * t_esdf_act
                        + coeff_state_vector_const_.parameter[2]) * resolution_ + origin_z_;

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
        }
        std::cout << "Distances: " << local_residuals.transpose() << std::endl;

        residuals_ = local_residuals;
        jacobians_ = local_jacobians;
    }

    const Eigen::VectorXd& residuals() const { return residuals_; }
    const Eigen::MatrixXd& jacobians() const { return jacobians_; }

private:
    parameterBlockContinuousPath& coeff_state_vector_;
    parameterBlockContinuousPathConstant& coeff_state_vector_const_;
    int esdf_samp_;
    double t_max_esdf_seg_, origin_x_, origin_y_, origin_z_;
    float resolution_;
    torch::jit::script::Module& loaded_sdf_;
    Eigen::VectorXd residuals_;
    Eigen::MatrixXd jacobians_;
    std::shared_ptr<voxblox::EsdfMap> esdf_map_;
    bool use_voxfield_;
};



class Ceres4DistanceFunctionSegment : public SizedCostFunction<1, 3>
{
    public:
        Ceres4DistanceFunctionSegment(const CeresESDFUpdate& evaluation_callback, int index): evaluation_callback_(evaluation_callback), index_(index)
        {}

        virtual ~Ceres4DistanceFunctionSegment(void)
        {}

        virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
        {
            auto dist = evaluation_callback_.residuals()(index_);
            residuals[0] = dist;
            if (jacobians != nullptr && jacobians[0] != nullptr)
                {
                    jacobians[0][0] = evaluation_callback_.jacobians()(index_, 0);
                    jacobians[0][1] = evaluation_callback_.jacobians()(index_, 1);
                    jacobians[0][2] = evaluation_callback_.jacobians()(index_, 2);
                }

            return true;
        }

        const CeresESDFUpdate& evaluation_callback_;
        int index_;
    private:
};


class Ceres4_ObstacleDistanceCostContSegmentFunctor
{
 public:
    Ceres4_ObstacleDistanceCostContSegmentFunctor(const CeresESDFUpdate& evaluation_callback, int index, double t_act, int esdf_samp = 10, double weight = 1.0)
      : evaluation_callback_(evaluation_callback), index_(index), t_act_(t_act), esdf_samp_(esdf_samp), weight_(weight), distanceFunctor_(new Ceres4DistanceFunctionSegment(evaluation_callback_, index_))
    {
    }

    virtual ~Ceres4_ObstacleDistanceCostContSegmentFunctor(void) 
    {
    }

    template <typename T>
    bool operator()(const T* const stateCoeff, const T* const stateCoeffConstant, T* residual) const
    {   
        T p[3], dist;

        p[0] = stateCoeff[0] * ceres::pow(t_act_, 5) + stateCoeff[1] * ceres::pow(t_act_, 4) + stateCoeff[2] * ceres::pow(t_act_, 3) + stateCoeff[3] * ceres::pow(t_act_, 2) + stateCoeff[4] * t_act_ + stateCoeffConstant[0];
        p[1] = stateCoeff[5] * ceres::pow(t_act_, 5) + stateCoeff[6] * ceres::pow(t_act_, 4) + stateCoeff[7] * ceres::pow(t_act_, 3) + stateCoeff[8] * ceres::pow(t_act_, 2) + stateCoeff[9] * t_act_ + stateCoeffConstant[1];
        p[2] = stateCoeff[10] * ceres::pow(t_act_, 5) + stateCoeff[11] * ceres::pow(t_act_, 4) + stateCoeff[12] * ceres::pow(t_act_, 3) + stateCoeff[13] * ceres::pow(t_act_, 2) + stateCoeff[14] * t_act_ + stateCoeffConstant[2];

        // Compute distance
        distanceFunctor_(p, &dist);


        // Compute weight
        // residual[0] = T(weight_) / T(esdf_samp_) * exp(T(-4) * (dist - T(1.5)));
        residual[0] = T(weight_) / T(esdf_samp_) * exp(T(-4) * (dist - T(1.5)));



        return true;
    }

  private:

    // Constraint weighting and t_act
    double weight_, t_act_;

    int esdf_samp_, index_;

    const CeresESDFUpdate& evaluation_callback_;

    // Distance funtion diferenciation
    ceres::CostFunctionToFunctor<1, 3> distanceFunctor_;
};








#endif