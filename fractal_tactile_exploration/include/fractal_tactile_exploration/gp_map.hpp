#ifndef FRACTAL_TACTILE_EXPLORATION__GP_MAP_HPP_
#define FRACTAL_TACTILE_EXPLORATION__GP_MAP_HPP_

#include <cstddef>
#include <vector>

namespace fractal_tactile_exploration
{

struct MapParameters
{
  double geometry_length_scale{0.015};
  double geometry_signal_stddev{0.002};
  double geometry_prior{0.0};
  double compliance_length_scale{0.020};
  double compliance_signal_stddev{0.25};
  double compliance_prior{0.5};
  double measurement_stddev{0.0005};
};

struct MapSample
{
  double x;
  double y;
  double height;
  double compliance;
};

struct MapEstimate
{
  double height_mean;
  double height_variance;
  double compliance_mean;
  double compliance_variance;
};

class GaussianProcessMap
{
public:
  explicit GaussianProcessMap(MapParameters parameters);
  void add_sample(MapSample sample);
  MapEstimate estimate(double x, double y) const;
  std::size_t sample_count() const;

private:
  double kernel(double x1, double y1, double x2, double y2, double length_scale, double signal_stddev) const;
  std::vector<double> solve(std::vector<double> matrix, std::vector<double> vector) const;
  MapParameters parameters_;
  std::vector<MapSample> samples_;
};

}  // namespace fractal_tactile_exploration

#endif  // FRACTAL_TACTILE_EXPLORATION__GP_MAP_HPP_