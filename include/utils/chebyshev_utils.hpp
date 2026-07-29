#ifndef CHEBYSHEV_UTILS_HPP
#define CHEBYSHEV_UTILS_HPP

#include <vector>
#include <cmath>
#include <algorithm>
#include <cstdio>

// Computes the minimum feasible trajectory duration T (seconds) for a degree-5
// Chebyshev trajectory so that real-world velocity, acceleration and jerk limits
// are satisfied everywhere along τ∈[-1,1].
//
// Scaling law (chain rule, t_real = (T/2)*(τ+1), dτ/dt_real = 2/T):
//   v_real  = resolution * (2/T)   * ||p'(τ)||_cells   →  T_v = 2·res·max||p'||         / v_max
//   a_real  = resolution * (2/T)²  * ||p''(τ)||_cells  →  T_a = 2·√(res·max||p''||      / a_max)
//   j_real  = resolution * (2/T)³  * ||p'''(τ)||_cells →  T_j = 2·∛(res·max||p'''||     / j_max)
//   T_min   = max(T_v, T_a, T_j)
//
// cx, cy, cz: 6-element Chebyshev coefficient vectors.
//   Layout matches all Ceres5 functors: index 0 = highest degree (c5), index 5 = c0.
// resolution: metres per grid cell.
// v_max_ms:   maximum 3D velocity norm (m/s).
// a_max_ms2:  maximum 3D acceleration norm (m/s²).
// j_max_ms3:  maximum 3D jerk norm (m/s³).
// n_samples:  uniform samples in τ∈[-1,1] used to find the maxima (200 is sufficient for deg-5).
inline double computeT_min_chebyshev(
    const std::vector<double>& cx,
    const std::vector<double>& cy,
    const std::vector<double>& cz,
    double resolution,
    double v_max_ms,
    double a_max_ms2,
    double j_max_ms3,
    int n_samples = 200)
{
    // Chebyshev → monomial coefficients (same conversion used inside all Ceres5 cost functors).
    // c[0]=c5 (highest degree), c[5]=c0.
    // Only p[1]..p[5] are needed (p[0] does not contribute to derivatives).
    auto mono = [](const std::vector<double>& c, double p[6]) {
        p[1] = c[4] - 3.0*c[2] + 5.0*c[0];
        p[2] = 2.0*c[3] - 8.0*c[1];
        p[3] = 4.0*c[2] - 20.0*c[0];
        p[4] = 8.0*c[1];
        p[5] = 16.0*c[0];
    };

    double px[6], py[6], pz[6];
    mono(cx, px);
    mono(cy, py);
    mono(cz, pz);

    double max_v_sq = 0.0, max_a_sq = 0.0, max_j_sq = 0.0;

    for (int i = 0; i <= n_samples; ++i) {
        const double s = -1.0 + 2.0 * i / n_samples;

        // p'(s)   = p1 + s*(2p2 + s*(3p3 + s*(4p4 + s*5p5)))
        const double vx = px[1] + s*(2.0*px[2] + s*(3.0*px[3] + s*(4.0*px[4] + s*5.0*px[5])));
        const double vy = py[1] + s*(2.0*py[2] + s*(3.0*py[3] + s*(4.0*py[4] + s*5.0*py[5])));
        const double vz = pz[1] + s*(2.0*pz[2] + s*(3.0*pz[3] + s*(4.0*pz[4] + s*5.0*pz[5])));
        max_v_sq = std::max(max_v_sq, vx*vx + vy*vy + vz*vz);

        // p''(s)  = 2p2 + s*(6p3 + s*(12p4 + s*20p5))
        const double ax = 2.0*px[2] + s*(6.0*px[3] + s*(12.0*px[4] + s*20.0*px[5]));
        const double ay = 2.0*py[2] + s*(6.0*py[3] + s*(12.0*py[4] + s*20.0*py[5]));
        const double az = 2.0*pz[2] + s*(6.0*pz[3] + s*(12.0*pz[4] + s*20.0*pz[5]));
        max_a_sq = std::max(max_a_sq, ax*ax + ay*ay + az*az);

        // p'''(s) = 6p3 + s*(24p4 + s*60p5)
        const double jx = 6.0*px[3] + s*(24.0*px[4] + s*60.0*px[5]);
        const double jy = 6.0*py[3] + s*(24.0*py[4] + s*60.0*py[5]);
        const double jz = 6.0*pz[3] + s*(24.0*pz[4] + s*60.0*pz[5]);
        max_j_sq = std::max(max_j_sq, jx*jx + jy*jy + jz*jz);
    }

    const double max_v = std::sqrt(max_v_sq);
    const double max_a = std::sqrt(max_a_sq);
    const double max_j = std::sqrt(max_j_sq);

    const double T_v = 2.0 * resolution * max_v / v_max_ms;
    const double T_a = 2.0 * std::sqrt(resolution * max_a / a_max_ms2);
    const double T_j = 2.0 * std::cbrt(resolution * max_j / j_max_ms3);

    return std::max({T_v, T_a, T_j});
}

#endif
