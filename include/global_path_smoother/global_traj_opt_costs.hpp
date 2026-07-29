/**
 * @file global_traj_opt_costs.hpp
 * @brief Ceres cost functors for the global polynomial path smoother.
 *
 * Polynomial model (degree 5, monomial basis, t ∈ [0, 1]):
 *
 *   x(t) = sc[ 0]*t^5 + sc[ 1]*t^4 + sc[ 2]*t^3 + sc[ 3]*t^2 + sc[ 4]*t + sk[0]
 *   y(t) = sc[ 5]*t^5 + sc[ 6]*t^4 + sc[ 7]*t^3 + sc[ 8]*t^2 + sc[ 9]*t + sk[1]
 *   z(t) = sc[10]*t^5 + sc[11]*t^4 + sc[12]*t^3 + sc[13]*t^2 + sc[14]*t + sk[2]
 *
 * Parameter blocks:
 *   sc[15]  —  free coefficients  [cx0..cx4, cy0..cy4, cz0..cz4]
 *   sk[3]   —  constant terms     [cx5, cy5, cz5]  (fixed to start point)
 *
 * This header has NO dependencies on the rest of the project: only <ceres/ceres.h>.
 */

#ifndef GLOBAL_TRAJ_OPT_COSTS_HPP
#define GLOBAL_TRAJ_OPT_COSTS_HPP

#include <ceres/ceres.h>


// ============================================================================
// Cost 1 — Waypoint attachment
// ============================================================================
/**
 * @brief Penalises the squared 3-D distance between the trajectory at t_i
 *        and the i-th target waypoint.
 *
 *   residual[0] = weight * ( (x(t_i) - wx)² + (y(t_i) - wy)² + (z(t_i) - wz)² )
 *
 * Residuals : 1
 * Param blocks : sc[15], sk[3]
 */
struct GlobalWPAttachCost
{
    GlobalWPAttachCost(double weight,
                       double wx, double wy, double wz,
                       double t)
        : weight_(weight), wx_(wx), wy_(wy), wz_(wz), t_(t) {}

    template <typename T>
    bool operator()(const T* const sc, const T* const sk, T* r) const
    {
        const T t  = T(t_);
        const T t2 = t * t;
        const T t3 = t2 * t;
        const T t4 = t3 * t;
        const T t5 = t4 * t;

        const T x = sc[ 0]*t5 + sc[ 1]*t4 + sc[ 2]*t3 + sc[ 3]*t2 + sc[ 4]*t + sk[0];
        const T y = sc[ 5]*t5 + sc[ 6]*t4 + sc[ 7]*t3 + sc[ 8]*t2 + sc[ 9]*t + sk[1];
        const T z = sc[10]*t5 + sc[11]*t4 + sc[12]*t3 + sc[13]*t2 + sc[14]*t + sk[2];

        const T dx = x - T(wx_);
        const T dy = y - T(wy_);
        const T dz = z - T(wz_);

        r[0] = T(weight_) * (dx*dx + dy*dy + dz*dz);
        return true;
    }

    double weight_;
    double wx_, wy_, wz_;
    double t_;
};


// ============================================================================
// Cost 2 — Fix goal at t = 1
// ============================================================================
/**
 * @brief Hard constraint: P(1) must coincide with the goal waypoint.
 *
 * At t=1 all monomials equal 1, so P(1) = sum of all coefficients:
 *   x(1) = sc[0]+sc[1]+sc[2]+sc[3]+sc[4] + sk[0]
 *
 * Residuals : 3  (x, y, z)
 * Param blocks : sc[15], sk[3]
 */
struct GlobalFixGoalCost
{
    GlobalFixGoalCost(double weight,
                      double gx, double gy, double gz)
        : weight_(weight), gx_(gx), gy_(gy), gz_(gz) {}

    template <typename T>
    bool operator()(const T* const sc, const T* const sk, T* r) const
    {
        const T x_end = sc[ 0] + sc[ 1] + sc[ 2] + sc[ 3] + sc[ 4] + sk[0];
        const T y_end = sc[ 5] + sc[ 6] + sc[ 7] + sc[ 8] + sc[ 9] + sk[1];
        const T z_end = sc[10] + sc[11] + sc[12] + sc[13] + sc[14] + sk[2];

        r[0] = T(weight_) * (x_end - T(gx_));
        r[1] = T(weight_) * (y_end - T(gy_));
        r[2] = T(weight_) * (z_end - T(gz_));
        return true;
    }

    double weight_;
    double gx_, gy_, gz_;
};


// ============================================================================
// Cost 3 — Smoothness
// ============================================================================
/**
 * @brief Penalises the high-degree coefficients (t^5..t^2) to encourage
 *        smooth, low-complexity trajectories.
 *
 * Each residual is weighted by its monomial degree so that higher-degree
 * terms are penalised more heavily:
 *   r_k = weight * degree_k * sc_k
 *
 * Residuals : 12  (4 high-degree coefficients × 3 axes)
 * Param block : sc[15]   (the fixed constant term sk is not penalised)
 */
struct GlobalSmoothnessCost
{
    explicit GlobalSmoothnessCost(double weight) : weight_(weight) {}

    template <typename T>
    bool operator()(const T* const sc, T* r) const
    {
        // x axis  — sc[0..3] = cx0(t^5), cx1(t^4), cx2(t^3), cx3(t^2)
        r[ 0] = T(weight_ * 5.0) * sc[ 0];
        r[ 1] = T(weight_ * 4.0) * sc[ 1];
        r[ 2] = T(weight_ * 3.0) * sc[ 2];
        r[ 3] = T(weight_ * 2.0) * sc[ 3];
        // y axis  — sc[5..8]
        r[ 4] = T(weight_ * 5.0) * sc[ 5];
        r[ 5] = T(weight_ * 4.0) * sc[ 6];
        r[ 6] = T(weight_ * 3.0) * sc[ 7];
        r[ 7] = T(weight_ * 2.0) * sc[ 8];
        // z axis  — sc[10..13]
        r[ 8] = T(weight_ * 5.0) * sc[10];
        r[ 9] = T(weight_ * 4.0) * sc[11];
        r[10] = T(weight_ * 3.0) * sc[12];
        r[11] = T(weight_ * 2.0) * sc[13];
        return true;
    }

    double weight_;
};

#endif // GLOBAL_TRAJ_OPT_COSTS_HPP
