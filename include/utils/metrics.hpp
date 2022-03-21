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

            /**
             * @brief 
             * 
             * @param _data 
             * @return CurvatureData 
             */
            CurvatureData calculatePathCurvature(const CoordinateList &_data);

            /**
             * @brief 
             * 
             * @param _data 
             * @param _threshold_angle 
             * @return AnglesData 
             */
            AnglesData calculatePathAnglesMetrics(const CoordinateList &_data, const int _threshold_angle);

            /**
             * @brief 
             * 
             * @param _distances_pair 
             * @return DistancesData 
             */
            DistancesData calculateDistancesMetrics(const std::vector<std::pair<utils::Vec3i, double>> &_distances_pair);

            /**
             * @brief Get the Average object
             * 
             * @tparam T 
             * @tparam V 
             * @tparam Args 
             * @param _data 
             * @return T 
             */
            template <typename T, template <typename, typename...> class V, typename... Args>
            T getAverage(const V<T, Args...> &_data);

            /**
             * @brief Get the Std Dev object
             * 
             * @tparam T 
             * @tparam V 
             * @tparam Args 
             * @param _data 
             * @param _average 
             * @return T 
             */
            template <typename T, template <typename, typename...> class V, typename... Args>
            T getStdDev(const V<T, Args...> &_data, const T &_average);
        }
    }

}
#endif