#ifndef METRICS_HPP
#define METRICS_HPP

#include <iostream>
#include <tuple>
#include <numeric>
#include "utils/utils.hpp"
#include "utils/geometry_utils.hpp"

namespace Planners
{

    namespace utils
    {
        namespace metrics
        {
            using namespace Planners::utils::geometry;
            using CurvatureData = std::tuple<double, double, double, double>;
            using DistancesData = std::tuple<double, double, double, double>;
            using AnglesData = std::tuple<double, double, double, double, double, std::vector<double>>;

            CurvatureData calculatePathCurvature(const CoordinateList &_data);

            AnglesData calculatePathAnglesMetrics(const CoordinateList &_data, const int _threshold_angle);

            DistancesData calculateDistancesMetrics(const std::vector<std::pair<utils::Vec3i, double>> &_distances_pair);

            template <typename T, template <typename, typename...> class V, typename... Args>
            T getAverage(const V<T, Args...> &_data);

            template <typename T, template <typename, typename...> class V, typename... Args>
            T getStdDev(const V<T, Args...> &_data, const T &_average);
        }
    }

}
#endif