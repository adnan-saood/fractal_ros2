#include "fractal_tactile_exploration/gp_map.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace fractal_tactile_exploration
{

GaussianProcessMap::GaussianProcessMap(MapParameters parameters) : parameters_(std::move(parameters)) {}

void GaussianProcessMap::add_sample(MapSample sample) { samples_.push_back(sample); }

std::size_t GaussianProcessMap::sample_count() const { return samples_.size(); }

double GaussianProcessMap::kernel(double x1, double y1, double x2, double y2,
  double length_scale, double signal_stddev) const
{
  const double dx = x1 - x2;
  const double dy = y1 - y2;
  return signal_stddev * signal_stddev * std::exp(-0.5 * (dx * dx + dy * dy) / (length_scale * length_scale));
}

std::vector<double> GaussianProcessMap::solve(std::vector<double> matrix, std::vector<double> vector) const
{
  const std::size_t size = vector.size();
  for (std::size_t column = 0; column < size; ++column) {
    std::size_t pivot = column;
    for (std::size_t row = column + 1U; row < size; ++row) {
      if (std::abs(matrix[row * size + column]) > std::abs(matrix[pivot * size + column])) pivot = row;
    }
    if (std::abs(matrix[pivot * size + column]) < 1e-14) throw std::runtime_error("Singular GP covariance matrix");
    for (std::size_t swap_column = column; swap_column < size; ++swap_column) std::swap(matrix[column * size + swap_column], matrix[pivot * size + swap_column]);
    std::swap(vector[column], vector[pivot]);
    const double diagonal = matrix[column * size + column];
    for (std::size_t current_column = column; current_column < size; ++current_column) matrix[column * size + current_column] /= diagonal;
    vector[column] /= diagonal;
    for (std::size_t row = 0; row < size; ++row) {
      if (row == column) continue;
      const double scale = matrix[row * size + column];
      for (std::size_t current_column = column; current_column < size; ++current_column) matrix[row * size + current_column] -= scale * matrix[column * size + current_column];
      vector[row] -= scale * vector[column];
    }
  }
  return vector;
}

MapEstimate GaussianProcessMap::estimate(double x, double y) const
{
  if (samples_.empty()) {
    return {parameters_.geometry_prior, parameters_.geometry_signal_stddev * parameters_.geometry_signal_stddev,
      parameters_.compliance_prior, parameters_.compliance_signal_stddev * parameters_.compliance_signal_stddev};
  }
  const std::size_t count = samples_.size();
  auto predict = [&](double length_scale, double signal_stddev, double prior, bool geometry) {
    std::vector<double> covariance(count * count);
    std::vector<double> residual(count);
    std::vector<double> cross_covariance(count);
    for (std::size_t row = 0; row < count; ++row) {
      residual[row] = (geometry ? samples_[row].height : samples_[row].compliance) - prior;
      cross_covariance[row] = kernel(samples_[row].x, samples_[row].y, x, y, length_scale, signal_stddev);
      for (std::size_t column = 0; column < count; ++column) {
        covariance[row * count + column] = kernel(samples_[row].x, samples_[row].y, samples_[column].x, samples_[column].y, length_scale, signal_stddev);
      }
      covariance[row * count + row] += parameters_.measurement_stddev * parameters_.measurement_stddev;
    }
    const auto alpha = solve(covariance, residual);
    const auto covariance_inverse_cross = solve(covariance, cross_covariance);
    double mean = prior;
    double variance = signal_stddev * signal_stddev;
    for (std::size_t index = 0; index < count; ++index) {
      mean += cross_covariance[index] * alpha[index];
      variance -= cross_covariance[index] * covariance_inverse_cross[index];
    }
    return std::pair<double, double>{mean, std::max(0.0, variance)};
  };
  const auto geometry = predict(parameters_.geometry_length_scale, parameters_.geometry_signal_stddev, parameters_.geometry_prior, true);
  const auto compliance = predict(parameters_.compliance_length_scale, parameters_.compliance_signal_stddev, parameters_.compliance_prior, false);
  return {geometry.first, geometry.second, compliance.first, compliance.second};
}

}  // namespace fractal_tactile_exploration