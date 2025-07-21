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


class CeresESDFUpdate : public ceres::EvaluationCallback {
    public:
        CeresESDFUpdate(parameterBlockContinuousPath& coeff_state_vector,
                        parameterBlockContinuousPathConstant& coeff_state_vector_const,
                        int esdf_seg,
                        double t_max_esdf_seg,
                        torch::jit::script::Module& loaded_sdf,
                        double origin_x,
                        double origin_y,
                        double origin_z,
                        float resolution)
            : coeff_state_vector_(coeff_state_vector),
            coeff_state_vector_const_(coeff_state_vector_const),
            esdf_seg_(esdf_seg),
            t_max_esdf_seg_(t_max_esdf_seg),
            loaded_sdf_(loaded_sdf),
            origin_x_(origin_x),
            origin_y_(origin_y),
            origin_z_(origin_z),
            resolution_(resolution)
        {
            residuals_ = Eigen::VectorXd::Zero(esdf_seg_ - 2);
            jacobians_ = Eigen::MatrixXd::Zero(esdf_seg_ - 2, 3);
            PrepareForEvaluation(true, true);
            std::cout << "Evaluation Callback Created" << std::endl;
        }

        void PrepareForEvaluation(bool evaluate_jacobians, bool new_evaluation_point) final {
            int num_points = esdf_seg_ - 2;
            torch::Tensor coordinates_tensor = torch::empty({num_points, 3}, torch::kFloat32);
            auto accessor = coordinates_tensor.accessor<float, 2>();

            for (int i = 0; i < num_points; ++i) {
                double t_esdf_act = t_max_esdf_seg_ * (i + 1) / esdf_seg_;

                accessor[i][0] = (coeff_state_vector_.parameter[0] * std::pow(t_esdf_act, 5)
                            + coeff_state_vector_.parameter[1] * std::pow(t_esdf_act, 4)
                            + coeff_state_vector_.parameter[2] * std::pow(t_esdf_act, 3)
                            + coeff_state_vector_.parameter[3] * std::pow(t_esdf_act, 2)
                            + coeff_state_vector_.parameter[4] * t_esdf_act
                            + coeff_state_vector_const_.parameter[0]) * resolution_ + origin_x_;
                accessor[i][1] = (coeff_state_vector_.parameter[5] * std::pow(t_esdf_act, 5)
                            + coeff_state_vector_.parameter[6] * std::pow(t_esdf_act, 4)
                            + coeff_state_vector_.parameter[7] * std::pow(t_esdf_act, 3)
                            + coeff_state_vector_.parameter[8] * std::pow(t_esdf_act, 2)
                            + coeff_state_vector_.parameter[9] * t_esdf_act 
                            + coeff_state_vector_const_.parameter[1]) * resolution_ + origin_y_;
                accessor[i][2] = (coeff_state_vector_.parameter[10] * std::pow(t_esdf_act, 5)
                            + coeff_state_vector_.parameter[11] * std::pow(t_esdf_act, 4)
                            + coeff_state_vector_.parameter[12] * std::pow(t_esdf_act, 3)
                            + coeff_state_vector_.parameter[13] * std::pow(t_esdf_act, 2)
                            + coeff_state_vector_.parameter[14] * t_esdf_act 
                            + coeff_state_vector_const_.parameter[2]) * resolution_ + origin_z_;
            }

            coordinates_tensor.set_requires_grad(true);

            torch::Tensor output_tensor, grad_outputs, input_gradients;
            auto start = std::chrono::high_resolution_clock::now();
            output_tensor = loaded_sdf_.forward({coordinates_tensor}).toTensor();
            grad_outputs = torch::ones_like(output_tensor);
            output_tensor.backward(grad_outputs);
            input_gradients = coordinates_tensor.grad();
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> duration = end - start;

            output_tensor = output_tensor.contiguous();
            input_gradients = input_gradients.contiguous();

            TORCH_CHECK(output_tensor.sizes()[0] == num_points, "Output tensor size mismatch");
            TORCH_CHECK(input_gradients.sizes()[0] == num_points && input_gradients.sizes()[1] == 3,
                        "Input gradients size mismatch");

            float* output_ptr = output_tensor.data_ptr<float>();
            float* grad_ptr = input_gradients.data_ptr<float>();

            Eigen::VectorXd local_residuals(num_points);
            Eigen::MatrixXd local_jacobians(num_points, 3);


            for (int i = 0; i < num_points; ++i) {
                local_residuals(i) = output_ptr[i];
                local_jacobians(i, 0) = grad_ptr[3 * i + 0];
                local_jacobians(i, 1) = grad_ptr[3 * i + 1];
                local_jacobians(i, 2) = grad_ptr[3 * i + 2];
            }

            residuals_ = local_residuals;
            jacobians_ = local_jacobians;
        }

        

        const Eigen::VectorXd& residuals() const { return residuals_; }
        const Eigen::MatrixXd& jacobians() const { return jacobians_; }

    private:
        parameterBlockContinuousPath& coeff_state_vector_;
        parameterBlockContinuousPathConstant& coeff_state_vector_const_;
        int esdf_seg_;
        double t_max_esdf_seg_, origin_x_, origin_y_, origin_z_;
        float resolution_;
        torch::jit::script::Module& loaded_sdf_;
        Eigen::VectorXd residuals_;
        Eigen::MatrixXd jacobians_;
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
            //residuals[0] = evaluation_callback_.residuals()(index_);
            // residuals[0] = 1 / dist;
            residuals[0] = dist;
            //auto auxcont = -1/(dist * dist);
            auto auxcont = 1;
            //if (residuals[0] < 0) residuals[0] = -residuals[0];
            if (jacobians != nullptr && jacobians[0] != nullptr)
                {
                    jacobians[0][0] = auxcont * evaluation_callback_.jacobians()(index_, 0);
                    jacobians[0][1] = auxcont * evaluation_callback_.jacobians()(index_, 1);
                    jacobians[0][2] = auxcont * evaluation_callback_.jacobians()(index_, 2);
                }

            // -------------------------------TEST--------------------------------------------
            // double x = parameters[0][0] * g_3D.m_resolution;
            // double y = parameters[0][1] * g_3D.m_resolution;
            // double z = parameters[0][2] * g_3D.m_resolution;
            // double dist_;
            // static std::mutex cout_mutex;
            // if(g_3D.isIntoMap(x, y, z))
            // {
            //     TrilinearParams p = g_3D.computeDistInterpolation(x, y, z);
            //     dist_ = p.a0 + p.a1*x + p.a2*y + p.a3*z + p.a4*x*y + p.a5*x*z + p.a6*y*z + p.a7*x*y*z;
            //     if(dist_ < 0.01)
            //         dist_ = 0.01;
            //     //int cte = -1/(dist_*dist_);
            //     int cte = 1;
            //     double j1 = cte * (p.a1 + p.a4*y + p.a5*z + p.a7*y*z);
            //     double j2 = cte * (p.a2 + p.a4*x + p.a6*z + p.a7*x*z);
            //     double j3 = cte * (p.a3 + p.a5*x + p.a6*y + p.a7*x*y);

            //     if (jacobians != nullptr && jacobians[0] != nullptr)
            //     {
            //         std::ostringstream oss;
            //         oss << "grid / network generated jacobians:\n";
            //         oss << j1 << ", " << j2 << ", " << j3 << "\n";
            //         oss << jacobians[0][0] << ", " << jacobians[0][1] << ", " << jacobians[0][2] << std::endl;
            //         {
            //             std::lock_guard<std::mutex> lock(cout_mutex);
            //             std::cout << oss.str();
            //         }
            //     }
            
            // }
            // else
            // {
            //     double j1 = 0.0;
            //     double j2 = 0.0;
            //     double j3 = 0.0;
            //     if (jacobians != nullptr && jacobians[0] != nullptr)
            //     {
            //         std::ostringstream oss;
            //         oss << "OUT grid / network generated jacobians:\n";
            //         oss << j1 << ", " << j2 << ", " << j3 << "\n";
            //         oss << jacobians[0][0] << ", " << jacobians[0][1] << ", " << jacobians[0][2] << std::endl;
            //         {
            //             std::lock_guard<std::mutex> lock(cout_mutex);
            //             std::cout << oss.str();
            //         }
            //     }
            // }
            
            // -------------------------------TEST--------------------------------------------

            return true;
        }

        const CeresESDFUpdate& evaluation_callback_;
        int index_;
    private:
};


class Ceres4_ObstacleDistanceCostContSegmentFunctor
{
 public:
    Ceres4_ObstacleDistanceCostContSegmentFunctor(const CeresESDFUpdate& evaluation_callback, int index, double t_act, int esdf_seg = 10, double weight = 1.0)
      : evaluation_callback_(evaluation_callback), index_(index), t_act_(t_act), esdf_seg_(esdf_seg), weight_(weight), distanceFunctor_(new Ceres4DistanceFunctionSegment(evaluation_callback_, index))
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

        // Compute weigthed residual
        //residual[0] = T(weight_) / (T(esdf_seg_) * dist);
        residual[0] = T(weight_) / (T(esdf_seg_)) * exp(T(-4) * (dist - T(1)));

        return true;
    }

  private:

    // Constraint weighting and t_act
    double weight_, t_act_;

    int esdf_seg_, index_;

    const CeresESDFUpdate& evaluation_callback_;

    // Distance funtion diferenciation
    ceres::CostFunctionToFunctor<1, 3> distanceFunctor_;
};








#endif