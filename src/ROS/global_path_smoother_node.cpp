/**
 * @file global_path_smoother_node.cpp
 * @brief Suaviza una lista de waypoints globales ajustando un polinomio de
 *        grado 5 mediante Ceres Solver. Sin suscripciones ni publicaciones
 *        ROS; los waypoints se leen del servidor de parametros (YAML) y los
 *        resultados se imprimen por pantalla.
 *
 * Pipeline:
 *  1. Leer waypoints y opciones del servidor de parametros ROS (YAML).
 *  2. Filtrar waypoints con Ramer-Douglas-Peucker (epsilon en casillas).
 *  3. Asignar t_i en [0,1] a cada waypoint clave (longitud de arco acumulada).
 *  4. Solucion inicial: linea recta primer->ultimo waypoint.
 *       cx[4] = goal.x - start.x   (termino lineal,    libre)
 *       cx[5] = start.x            (termino constante, FIJO)
 *       cx[0..3] = 0
 *  5. Ajuste Ceres:
 *       a. WP attachment  -- ||P(t_i) - wp_i||^2
 *       b. Fix goal       -- P(1) = ultimo waypoint (restriccion dura)
 *       c. Smoothness     -- penaliza coefs. de alto grado
 *  6. Imprimir coeficientes del polinomio suavizado y waypoints muestreados.
 *
 * Configurar waypoints y pesos:
 *   config/global_path_smoother_waypoints.yaml
 *
 * Ejecutar:
 *   roslaunch heuristic_planners global_path_smoother.launch
 */

#include <iostream>
#include <iomanip>
#include <vector>
#include <array>
#include <chrono>
#include <cmath>
#include <string>

#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <ceres/ceres.h>
#include "global_path_smoother/global_traj_opt_costs.hpp"

using ceres::AutoDiffCostFunction;
using ceres::Problem;
using ceres::Solver;

// Waypoint en casillas
using WP3d = std::array<double, 3>;


// ============================================================================
// Lectura de waypoints desde el servidor de parametros
// ============================================================================
std::vector<WP3d> loadWaypoints(const ros::NodeHandle& nh)
{
    XmlRpc::XmlRpcValue wp_list;
    if (!nh.getParam("waypoints", wp_list)) {
        ROS_FATAL("[GlobalPathSmoother] Param 'waypoints' no encontrado. "
                  "Carga el YAML en el launch con <rosparam>.");
        ros::shutdown();
        exit(EXIT_FAILURE);
    }
    if (wp_list.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        wp_list.size() < 2) {
        ROS_FATAL("[GlobalPathSmoother] 'waypoints' debe ser un array "
                  "con al menos 2 elementos.");
        ros::shutdown();
        exit(EXIT_FAILURE);
    }

    std::vector<WP3d> wps;
    wps.reserve(wp_list.size());
    for (int i = 0; i < wp_list.size(); ++i) {
        WP3d p;
        p[0] = static_cast<double>(wp_list[i]["x"]);
        p[1] = static_cast<double>(wp_list[i]["y"]);
        p[2] = static_cast<double>(wp_list[i]["z"]);
        wps.push_back(p);
    }
    return wps;
}


// ============================================================================
// Ramer-Douglas-Peucker (RDP): simplificacion de polilinea 3D
// ============================================================================
// Distancia perpendicular del punto p al segmento [a, b]
static double rdpDist(const WP3d& p, const WP3d& a, const WP3d& b)
{
    const double dx = b[0]-a[0], dy = b[1]-a[1], dz = b[2]-a[2];
    const double len2 = dx*dx + dy*dy + dz*dz;
    if (len2 < 1e-12) {
        const double ex = p[0]-a[0], ey = p[1]-a[1], ez = p[2]-a[2];
        return std::sqrt(ex*ex + ey*ey + ez*ez);
    }
    double t = ((p[0]-a[0])*dx + (p[1]-a[1])*dy + (p[2]-a[2])*dz) / len2;
    t = std::max(0.0, std::min(1.0, t));
    const double rx = a[0]+t*dx - p[0];
    const double ry = a[1]+t*dy - p[1];
    const double rz = a[2]+t*dz - p[2];
    return std::sqrt(rx*rx + ry*ry + rz*rz);
}

// Recursion RDP: marca en keep[] los indices a conservar
static void rdpRecurse(const std::vector<WP3d>& pts, int lo, int hi,
                       double eps, std::vector<bool>& keep)
{
    if (hi <= lo + 1) return;
    double maxD = 0.0;
    int    maxI = lo;
    for (int i = lo+1; i < hi; ++i) {
        const double d = rdpDist(pts[i], pts[lo], pts[hi]);
        if (d > maxD) { maxD = d; maxI = i; }
    }
    if (maxD > eps) {
        keep[maxI] = true;
        rdpRecurse(pts, lo,   maxI, eps, keep);
        rdpRecurse(pts, maxI, hi,   eps, keep);
    }
}

// Devuelve los waypoints clave y sus indices originales.
// Ademas de RDP, inserta el punto medio de cualquier segmento cuya longitud
// de arco supere max_segment_len (en casillas), para evitar que el polinomio
// oscile en tramos largos y rectos (p.ej. un pasillo horizontal).
// Pasar max_segment_len <= 0 para deshabilitar esta subdivision.
static std::vector<WP3d> rdpFilter(const std::vector<WP3d>& wps, double eps,
                                    std::vector<int>& kept_indices,
                                    double max_segment_len = 20.0)
{
    const int N = static_cast<int>(wps.size());
    if (N <= 2) {
        kept_indices = {0, N-1};
        return wps;
    }
    std::vector<bool> keep(N, false);
    keep[0] = keep[N-1] = true;
    rdpRecurse(wps, 0, N-1, eps, keep);

    // Subdivision de segmentos largos: entre dos puntos marcados consecutivos,
    // si la distancia supera max_segment_len, insertar el punto de la lista
    // original mas cercano al centro del intervalo.
    if (max_segment_len > 0.0) {
        bool changed = true;
        while (changed) {
            changed = false;
            int prev = -1;
            for (int i = 0; i < N; ++i) {
                if (!keep[i]) continue;
                if (prev >= 0) {
                    double dx = wps[i][0] - wps[prev][0];
                    double dy = wps[i][1] - wps[prev][1];
                    double dz = wps[i][2] - wps[prev][2];
                    double seg_len = std::sqrt(dx*dx + dy*dy + dz*dz);
                    if (seg_len > max_segment_len) {
                        int mid = (prev + i) / 2;
                        keep[mid] = true;
                        changed = true;
                    }
                }
                prev = i;
            }
        }
    }

    kept_indices.clear();
    std::vector<WP3d> out;
    for (int i = 0; i < N; ++i) {
        if (keep[i]) {
            kept_indices.push_back(i);
            out.push_back(wps[i]);
        }
    }
    return out;
}


// ============================================================================
// Evaluacion del polinomio en t dado
// ============================================================================
static WP3d evalPoly(const double* sc, const double* sk, double t)
{
    const double t2=t*t, t3=t2*t, t4=t3*t, t5=t4*t;
    WP3d p;
    p[0] = sc[ 0]*t5+sc[ 1]*t4+sc[ 2]*t3+sc[ 3]*t2+sc[ 4]*t+sk[0];
    p[1] = sc[ 5]*t5+sc[ 6]*t4+sc[ 7]*t3+sc[ 8]*t2+sc[ 9]*t+sk[1];
    p[2] = sc[10]*t5+sc[11]*t4+sc[12]*t3+sc[13]*t2+sc[14]*t+sk[2];
    return p;
}


// ============================================================================
// main
// ============================================================================
int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_path_smoother_node");
    ros::NodeHandle nh("~");   // namespace privado -> lee params del YAML

    // ── 0. Leer parametros ───────────────────────────────────────────────────
    std::vector<WP3d> wps_all = loadWaypoints(nh);

    int    n_samples;
    double weight_wp, weight_fix_goal, weight_smooth, rdp_epsilon, max_segment_len;
    nh.param("n_samples",        n_samples,        50);
    nh.param("weight_wp",        weight_wp,        1e3);
    nh.param("weight_fix_goal",  weight_fix_goal,  1e5);
    nh.param("weight_smooth",    weight_smooth,    1.0);
    nh.param("rdp_epsilon",      rdp_epsilon,      1.0);   // casillas
    nh.param("max_segment_len",  max_segment_len,  20.0);  // casillas; <=0 = off

    const int N_all = static_cast<int>(wps_all.size());

    // ── Imprimir waypoints de entrada ────────────────────────────────────────
    std::cout << "\n";
    std::cout << "===========================================================\n";
    std::cout << "          Global Polynomial Path Smoother                   \n";
    std::cout << "===========================================================\n\n";
    std::cout << "Waypoints de entrada (" << N_all << "):\n";
    for (int i = 0; i < N_all; ++i) {
        std::cout << "[" << static_cast<int>(wps_all[i][0]) << ", "
                  << static_cast<int>(wps_all[i][1]) << ", "
                  << static_cast<int>(wps_all[i][2]) << "]";
        if (i < N_all - 1) std::cout << ", ";
    }
    std::cout << "\n\n";

    // ====================================================================
    // PASO 1 -- RDP: filtrado de waypoints clave
    // ====================================================================
    std::vector<int>  rdp_idx;
    std::vector<WP3d> wps = rdpFilter(wps_all, rdp_epsilon, rdp_idx, max_segment_len);
    const int N = static_cast<int>(wps.size());

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "RDP (epsilon = " << rdp_epsilon << " casillas)  →  "
              << N << " waypoints clave de " << N_all << ":\n";
    for (int i = 0; i < N; ++i) {
        std::cout << "  [idx " << std::setw(3) << rdp_idx[i] << "]  ["
                  << static_cast<int>(wps[i][0]) << ", "
                  << static_cast<int>(wps[i][1]) << ", "
                  << static_cast<int>(wps[i][2]) << "]\n";
    }
    std::cout << "\n";

    const WP3d& start = wps.front();
    const WP3d& goal  = wps.back();

    // ====================================================================
    // PASO 2 -- Solucion inicial: linea recta start -> goal
    // ====================================================================
    double sc[15] = {};   // todos a cero
    double sk[3]  = { start[0], start[1], start[2] };

    sc[4]  = goal[0] - start[0];   // cx4
    sc[9]  = goal[1] - start[1];   // cy4
    sc[14] = goal[2] - start[2];   // cz4

    // ====================================================================
    // PASO 3 -- Parametrizacion por longitud de arco acumulada
    // ====================================================================
    std::vector<double> t_list(N, 0.0);
    {
        std::vector<double> chord(N, 0.0);
        double total = 0.0;
        for (int i = 1; i < N; ++i) {
            const double dx = wps[i][0] - wps[i-1][0];
            const double dy = wps[i][1] - wps[i-1][1];
            const double dz = wps[i][2] - wps[i-1][2];
            chord[i] = std::sqrt(dx*dx + dy*dy + dz*dz);
            total   += chord[i];
        }
        if (total < 1e-12) {
            ROS_WARN("[GlobalPathSmoother] Longitud de arco ~0. "
                     "Usando parametrizacion uniforme.");
            for (int i = 0; i < N; ++i)
                t_list[i] = static_cast<double>(i) / (N - 1);
        } else {
            for (int i = 1; i < N; ++i)
                t_list[i] = t_list[i-1] + chord[i] / total;
        }
    }
    t_list.front() = 0.0;
    t_list.back()  = 1.0;

    // ====================================================================
    // PASO 4 -- Construccion y resolucion del problema Ceres
    // ====================================================================
    Problem problem;

    // 4a. Adhesion a waypoints clave
    for (int i = 0; i < N; ++i) {
        auto* cf = new AutoDiffCostFunction<GlobalWPAttachCost, 1, 15, 3>(
            new GlobalWPAttachCost(weight_wp,
                                   wps[i][0], wps[i][1], wps[i][2],
                                   t_list[i]));
        problem.AddResidualBlock(cf, nullptr, sc, sk);
    }

    // 4b. Restriccion de llegada al goal en t=1
    {
        auto* cf = new AutoDiffCostFunction<GlobalFixGoalCost, 3, 15, 3>(
            new GlobalFixGoalCost(weight_fix_goal,
                                  goal[0], goal[1], goal[2]));
        problem.AddResidualBlock(cf, nullptr, sc, sk);
    }

    // 4c. Suavidad (penaliza coefs. de alto grado)
    {
        auto* cf = new AutoDiffCostFunction<GlobalSmoothnessCost, 12, 15>(
            new GlobalSmoothnessCost(weight_smooth));
        problem.AddResidualBlock(cf, nullptr, sc);
    }

    // Fijar el termino constante sk = start (nunca se optimiza)
    problem.SetParameterBlockConstant(sk);

    // ── Opciones del solver (leidas desde el servidor de parametros) ─────────
    int  solver_max_iter, solver_num_threads;
    bool solver_nonmonotonic, solver_verbose;
    std::string solver_linear, solver_trust;

    nh.param("solver_max_iterations",     solver_max_iter,      500);
    nh.param("solver_num_threads",        solver_num_threads,   4);
    nh.param("solver_nonmonotonic_steps", solver_nonmonotonic,  true);
    nh.param("solver_verbose",            solver_verbose,       false);
    nh.param<std::string>("solver_linear_type",  solver_linear, std::string("DENSE_SCHUR"));
    nh.param<std::string>("solver_trust_region", solver_trust,  std::string("LEVENBERG_MARQUARDT"));

    Solver::Options options;
    options.max_num_iterations           = solver_max_iter;
    options.num_threads                  = solver_num_threads;
    options.use_nonmonotonic_steps       = solver_nonmonotonic;
    options.minimizer_progress_to_stdout = solver_verbose;

    // linear_solver_type: DENSE_QR | DENSE_SCHUR | SPARSE_NORMAL_CHOLESKY
    if      (solver_linear == "DENSE_QR")
        options.linear_solver_type = ceres::DENSE_QR;
    else if (solver_linear == "DENSE_SCHUR")
        options.linear_solver_type = ceres::DENSE_SCHUR;
    else if (solver_linear == "SPARSE_NORMAL_CHOLESKY")
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    else {
        ROS_WARN("[GlobalPathSmoother] solver_linear_type '%s' desconocido, "
                 "usando DENSE_SCHUR.", solver_linear.c_str());
        options.linear_solver_type = ceres::DENSE_SCHUR;
    }

    // trust_region_strategy_type: LEVENBERG_MARQUARDT | DOGLEG
    if      (solver_trust == "LEVENBERG_MARQUARDT")
        options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    else if (solver_trust == "DOGLEG")
        options.trust_region_strategy_type = ceres::DOGLEG;
    else {
        ROS_WARN("[GlobalPathSmoother] solver_trust_region '%s' desconocido, "
                 "usando LEVENBERG_MARQUARDT.", solver_trust.c_str());
        options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    }
    // ────────────────────────────────────────────────────────────────────────

    Solver::Summary summary;
    auto t0 = std::chrono::high_resolution_clock::now();
    ceres::Solve(options, &problem, &summary);
    auto t1 = std::chrono::high_resolution_clock::now();
    double elapsed_ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::cout << "-----------------------------------------------------------\n";
    std::cout << summary.BriefReport() << "\n";
    std::cout << "Tiempo de suavizado: " << std::fixed
              << std::setprecision(2) << elapsed_ms << " ms\n";
    std::cout << "-----------------------------------------------------------\n\n";

    // ====================================================================
    // DESGLOSE DE COSTES FINALES
    // ====================================================================
    // Ceres minimiza  0.5 * ||r||^2  →  coste_bloque = 0.5 * sum(r_i^2)
    //
    // Bloque 1 (WP attachment, N residuos de dim 1):
    //   r[0] = weight_wp * (||P(t_i) - wp_i||^2)
    double cost_wp = 0.0;
    for (int i = 0; i < N; ++i) {
        const WP3d p = evalPoly(sc, sk, t_list[i]);
        const double dx = p[0]-wps[i][0], dy = p[1]-wps[i][1], dz = p[2]-wps[i][2];
        const double r  = weight_wp * (dx*dx + dy*dy + dz*dz);
        cost_wp += 0.5 * r * r;
    }

    // Bloque 2 (Fix goal, 3 residuos):
    //   r[k] = weight_fix_goal * (P(1)[k] - goal[k])
    const WP3d p1 = evalPoly(sc, sk, 1.0);
    const double rgx = weight_fix_goal * (p1[0] - goal[0]);
    const double rgy = weight_fix_goal * (p1[1] - goal[1]);
    const double rgz = weight_fix_goal * (p1[2] - goal[2]);
    const double cost_goal = 0.5 * (rgx*rgx + rgy*rgy + rgz*rgz);

    // Bloque 3 (Smoothness, 12 residuos):
    //   r[k] = weight_smooth * degree_k * sc[k]   (grados 5,4,3,2 por eje)
    double cost_smooth = 0.0;
    {
        const double degs[4] = {5.0, 4.0, 3.0, 2.0};
        for (int ax = 0; ax < 3; ++ax) {
            for (int d = 0; d < 4; ++d) {
                const double r = weight_smooth * degs[d] * sc[ax * 5 + d];
                cost_smooth += 0.5 * r * r;
            }
        }
    }

    const double cost_total = cost_wp + cost_goal + cost_smooth;
    const double pct = (cost_total > 1e-30) ? 100.0 / cost_total : 0.0;

    std::cout << std::fixed;
    std::cout << "Desglose de costes (iteracion final):\n";
    std::cout << "  WP attachment  : " << std::setw(14) << std::setprecision(4) << cost_wp
              << "  (" << std::setprecision(1) << std::setw(5) << cost_wp    * pct << " %)"
              << "   weight_wp       = " << weight_wp       << "\n";
    std::cout << "  Fix goal       : " << std::setw(14) << std::setprecision(4) << cost_goal
              << "  (" << std::setprecision(1) << std::setw(5) << cost_goal   * pct << " %)"
              << "   weight_fix_goal = " << weight_fix_goal << "\n";
    std::cout << "  Smoothness     : " << std::setw(14) << std::setprecision(4) << cost_smooth
              << "  (" << std::setprecision(1) << std::setw(5) << cost_smooth * pct << " %)"
              << "   weight_smooth   = " << weight_smooth   << "\n";
    std::cout << "  TOTAL          : " << std::setw(14) << std::setprecision(4) << cost_total << "\n\n";

    // ====================================================================
    // RESULTADO 1 -- Coeficientes del polinomio suavizado
    // ====================================================================
    //   P(t) = c[0]*t^5 + c[1]*t^4 + c[2]*t^3 + c[3]*t^2 + c[4]*t + c[5]
    std::cout << "RESULTADO 1 -- Polinomio suavizado\n";
    std::cout << "  P(t) = c[0]*t^5 + c[1]*t^4 + c[2]*t^3 + c[3]*t^2 + c[4]*t + c[5]\n\n";
    std::cout << std::setprecision(8);

    std::cout << "  cx = [";
    for (int i = 0;  i <  5; ++i) std::cout << sc[i]  << (i < 4 ? ", " : "");
    std::cout << " | " << sk[0] << "]\n";

    std::cout << "  cy = [";
    for (int i = 5;  i < 10; ++i) std::cout << sc[i]  << (i < 9 ? ", " : "");
    std::cout << " | " << sk[1] << "]\n";

    std::cout << "  cz = [";
    for (int i = 10; i < 15; ++i) std::cout << sc[i]  << (i < 14 ? ", " : "");
    std::cout << " | " << sk[2] << "]\n\n";

    // Verificacion de condiciones de contorno
    const double x0v = sk[0], y0v = sk[1], z0v = sk[2];
    const double x1v = sc[0]+sc[1]+sc[2]+sc[3]+sc[4]+sk[0];
    const double y1v = sc[5]+sc[6]+sc[7]+sc[8]+sc[9]+sk[1];
    const double z1v = sc[10]+sc[11]+sc[12]+sc[13]+sc[14]+sk[2];

    std::cout << std::setprecision(4);
    std::cout << "  Verificacion contorno:\n";
    std::cout << "    P(0) = (" << x0v << ", " << y0v << ", " << z0v << ")"
              << "   start = (" << start[0] << ", " << start[1] << ", "
              << start[2] << ")\n";
    std::cout << "    P(1) = (" << x1v << ", " << y1v << ", " << z1v << ")"
              << "   goal  = (" << goal[0]  << ", " << goal[1]  << ", "
              << goal[2]  << ")\n\n";

    // ====================================================================
    // RESULTADO 2 -- Waypoints muestreados del path suavizado
    // ====================================================================
    std::cout << "RESULTADO 2 -- Waypoints muestreados ("
              << n_samples + 1 << " puntos, t = 0 -> 1)\n";

    for (int i = 0; i <= n_samples; ++i) {
        const double t  = static_cast<double>(i) / static_cast<double>(n_samples);
        const double t2 = t*t, t3 = t2*t, t4 = t3*t, t5 = t4*t;

        const double x = sc[ 0]*t5 + sc[ 1]*t4 + sc[ 2]*t3
                       + sc[ 3]*t2 + sc[ 4]*t  + sk[0];
        const double y = sc[ 5]*t5 + sc[ 6]*t4 + sc[ 7]*t3
                       + sc[ 8]*t2 + sc[ 9]*t  + sk[1];
        const double z = sc[10]*t5 + sc[11]*t4 + sc[12]*t3
                       + sc[13]*t2 + sc[14]*t  + sk[2];

        std::cout << std::fixed << std::setprecision(4)
                  << "[" << x << ", " << y << ", " << z << "]";
        if (i < n_samples) std::cout << ", ";
    }
    std::cout << "\n";

    std::cout << "\n[GlobalPathSmoother] Listo.\n\n";
    return 0;
}
