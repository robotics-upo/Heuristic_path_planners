#ifndef CERES_CONSTRAINTS_7_CONT_DYNAMIC_LIMITS_ALL
#define CERES_CONSTRAINTS_7_CONT_DYNAMIC_LIMITS_ALL

#include <vector>
#include <ceres/ceres.h>

class Ceres7_DynamicLimitsAllFunctor {
public:
    Ceres7_DynamicLimitsAllFunctor(
        int    n_samp,
        double weight,
        double v_max_ms, double a_max_ms2, double j_max_ms3,
        double resolution,
        double k_softplus = 50.0)
        : n_samp_(n_samp), weight_(weight),
          v_max_(v_max_ms), a_max_(a_max_ms2), j_max_(j_max_ms3),
          res_(resolution), k_(k_softplus)
    {
        s_samples_.resize(n_samp_);
        for (int i = 0; i < n_samp_; ++i)
            s_samples_[i] = -1.0 + 2.0 * i / (n_samp_ - 1.0);
    }

    template <typename T>
    inline T softplus(T x) const {
        T kx = T(k_) * x;
        if (kx > T(0.0))
            return x + ceres::log(T(1.0) + ceres::exp(-kx)) / T(k_);
        else
            return ceres::log(T(1.0) + ceres::exp(kx)) / T(k_);
    }

    // DynamicAutoDiffCostFunction interface: parameters[0] = stateCoeff[19]
    template <typename T>
    bool operator()(T const* const* parameters, T* residuals) const {
        const T* sc = parameters[0];

        T traj_T = sc[18];

        T p1x = sc[4] - T(3.0)*sc[2] + T(5.0)*sc[0];
        T p2x = T(2.0)*sc[3] - T(8.0)*sc[1];
        T p3x = T(4.0)*sc[2] - T(20.0)*sc[0];
        T p4x = T(8.0)*sc[1];
        T p5x = T(16.0)*sc[0];

        T p1y = sc[10] - T(3.0)*sc[8] + T(5.0)*sc[6];
        T p2y = T(2.0)*sc[9]  - T(8.0)*sc[7];
        T p3y = T(4.0)*sc[8]  - T(20.0)*sc[6];
        T p4y = T(8.0)*sc[7];
        T p5y = T(16.0)*sc[6];

        T p1z = sc[16] - T(3.0)*sc[14] + T(5.0)*sc[12];
        T p2z = T(2.0)*sc[15] - T(8.0)*sc[13];
        T p3z = T(4.0)*sc[14] - T(20.0)*sc[12];
        T p4z = T(8.0)*sc[13];
        T p5z = T(16.0)*sc[12];

        T inv_T  = T(1.0) / traj_T;
        T inv_T2 = inv_T  * inv_T;
        T inv_T3 = inv_T2 * inv_T;
        T eps    = T(1e-10);

        for (int i = 0; i < n_samp_; ++i) {
            T s  = T(s_samples_[i]);
            T s2 = s  * s;
            T s3 = s2 * s;
            T s4 = s3 * s;

            // p'(s)
            T dpx = p1x + T(2.0)*p2x*s + T(3.0)*p3x*s2 + T(4.0)*p4x*s3 + T(5.0)*p5x*s4;
            T dpy = p1y + T(2.0)*p2y*s + T(3.0)*p3y*s2 + T(4.0)*p4y*s3 + T(5.0)*p5y*s4;
            T dpz = p1z + T(2.0)*p2z*s + T(3.0)*p3z*s2 + T(4.0)*p4z*s3 + T(5.0)*p5z*s4;

            // p''(s)
            T d2px = T(2.0)*p2x + T(6.0)*p3x*s  + T(12.0)*p4x*s2 + T(20.0)*p5x*s3;
            T d2py = T(2.0)*p2y + T(6.0)*p3y*s  + T(12.0)*p4y*s2 + T(20.0)*p5y*s3;
            T d2pz = T(2.0)*p2z + T(6.0)*p3z*s  + T(12.0)*p4z*s2 + T(20.0)*p5z*s3;

            // p'''(s)
            T d3px = T(6.0)*p3x + T(24.0)*p4x*s + T(60.0)*p5x*s2;
            T d3py = T(6.0)*p3y + T(24.0)*p4y*s + T(60.0)*p5y*s2;
            T d3pz = T(6.0)*p3z + T(24.0)*p4z*s + T(60.0)*p5z*s2;

            T norm_dp  = ceres::sqrt(dpx*dpx   + dpy*dpy   + dpz*dpz   + eps);
            T norm_d2p = ceres::sqrt(d2px*d2px + d2py*d2py + d2pz*d2pz + eps);
            T norm_d3p = ceres::sqrt(d3px*d3px + d3py*d3py + d3pz*d3pz + eps);

            T v_real = norm_dp  * T(2.0 * res_) * inv_T;
            T a_real = norm_d2p * T(4.0 * res_) * inv_T2;
            T j_real = norm_d3p * T(8.0 * res_) * inv_T3;

            residuals[3*i + 0] = T(weight_ / n_samp_) * softplus(v_real / T(v_max_) - T(1.0));
            residuals[3*i + 1] = T(weight_ / n_samp_) * softplus(a_real / T(a_max_) - T(1.0));
            residuals[3*i + 2] = T(weight_ / n_samp_) * softplus(j_real / T(j_max_) - T(1.0));
        }
        return true;
    }

private:
    int    n_samp_;
    double weight_, v_max_, a_max_, j_max_, res_, k_;
    std::vector<double> s_samples_;
};

#endif
