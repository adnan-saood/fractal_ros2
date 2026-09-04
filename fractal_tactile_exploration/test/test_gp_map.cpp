#include <gtest/gtest.h>

#include "fractal_tactile_exploration/gp_map.hpp"

namespace fractal_tactile_exploration
{

TEST(GaussianProcessMapTest, ReturnsPriorsBeforeObservation)
{
  MapParameters parameters;
  parameters.geometry_prior = 0.012;
  parameters.compliance_prior = 0.3;
  GaussianProcessMap map(parameters);

  const auto estimate = map.estimate(0.0, 0.0);

  EXPECT_DOUBLE_EQ(estimate.height_mean, parameters.geometry_prior);
  EXPECT_DOUBLE_EQ(estimate.compliance_mean, parameters.compliance_prior);
  EXPECT_GT(estimate.height_variance, 0.0);
  EXPECT_GT(estimate.compliance_variance, 0.0);
}

TEST(GaussianProcessMapTest, ObservationUpdatesBothLatentFields)
{
  MapParameters parameters;
  parameters.measurement_stddev = 1e-6;
  GaussianProcessMap map(parameters);
  map.add_sample({0.01, -0.02, 0.007, 0.8});

  const auto estimate = map.estimate(0.01, -0.02);

  EXPECT_NEAR(estimate.height_mean, 0.007, 1e-8);
  EXPECT_NEAR(estimate.compliance_mean, 0.8, 1e-8);
  EXPECT_LT(estimate.height_variance, parameters.geometry_signal_stddev * parameters.geometry_signal_stddev);
  EXPECT_LT(estimate.compliance_variance, parameters.compliance_signal_stddev * parameters.compliance_signal_stddev);
}

}  // namespace fractal_tactile_exploration