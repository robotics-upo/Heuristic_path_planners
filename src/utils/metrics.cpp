#include "utils/metrics.hpp"

namespace Planners
{

    namespace utils
    {

        namespace metrics
        {

            CurvatureData calculatePathCurvature(const CoordinateList &_data)
            {
                std::vector<double> curvatures;

                for (size_t i = 1; i < _data.size() - 2; ++i)
                {
                    double R = utils::geometry::getCircunferenceRadius(_data[i - 1], _data[i], _data[i + 1]);
                    if (R != std::numeric_limits<double>::infinity())
                    {
                        curvatures.push_back(1 / R);
                    }
                    else
                    {
                        curvatures.push_back(0);
                    }
                }

                if (curvatures.size() > 0)
                {
                    const double av_curvature       = getAverage(curvatures);
                    const double curv_sigma         = getStdDev(curvatures, av_curvature);
                    const auto [curv_min, curv_max] = std::minmax_element(begin(curvatures), end(curvatures));

                    return std::make_tuple(av_curvature, curv_sigma, *curv_min, *curv_max);
                }
                return std::make_tuple(0.0, 0.0, 0.0, 0.0);
            }
            AnglesData calculatePathAnglesMetrics(const CoordinateList &_data, const int _threshold_angle)
            {
                std::vector<double> angles;
                int changes{0};
                for (size_t i = 1; i < _data.size() - 2; ++i)
                {
                    double alpha = (M_PI - utils::geometry::angleBetweenThreePoints(_data[i - 1], _data[i], _data[i + 1]));

                    if (alpha > _threshold_angle / 180.0 * M_PI ||
                        alpha < _threshold_angle / 180 * M_PI)
                    {
                        angles.push_back(alpha);
                        changes++;
                    }
                }

                if (angles.size() > 0)
                {
                    const double av_angles              = getAverage(angles);
                    const double angles_simga           = getStdDev(angles, av_angles);
                    const auto [angles_min, angles_max] = std::minmax_element(begin(angles), end(angles));

                    return std::make_tuple(av_angles, angles_simga, *angles_min, *angles_max, changes, angles);
                }
                return std::make_tuple(0, 0, 0, 0, changes, angles);
            }
            DistancesData calculateDistancesMetrics(const std::vector<std::pair<utils::Vec3i, double>> &_distances_pair)
            {
                if( _distances_pair.size() > 0 ){

                    std::vector<double> distances;
                    for (auto &it : _distances_pair)
                        distances.push_back(it.second);

                    const auto [min, max] = std::minmax_element(begin(distances), end(distances));
                    const double mean           = getAverage(distances);
                    const double std_dev        = getStdDev(distances, mean);
                    return std::make_tuple(mean, std_dev, *min, *max);
                }
                return std::make_tuple(0, 0, 0, 0);
            }
            template <typename T, template <typename, typename...> class V, typename... Args>
            inline T getAverage(const V<T, Args...> &_data)
            {
                return std::accumulate(_data.begin(), _data.end(), 0.0) / _data.size();
            }

            template <typename T, template <typename, typename...> class V, typename... Args>
            inline T getStdDev(const V<T, Args...> &_data, const T &_average)
            {
                T std_dev{0};
                if (_data.size() > 0)
                {
                    for (const auto &it : _data)
                        std_dev += pow(it - _average, 2);

                    std_dev = sqrt(std_dev / _data.size());
                }
                return std_dev;
            }

        } //ns metrics
    } //ns utils
} //ns Planners