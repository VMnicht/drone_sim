#include <cmath>

#include <gtest/gtest.h>

#include "drone_core/disturbance_model.hpp"
#include "drone_core/sensor_noise_model.hpp"
#include "drone_core/trajectory_generator.hpp"

namespace drone_core
{
namespace
{

TEST(AnalyticTrajectoryGenerator, CircleIsClosedAndDerivativesAreAnalytic)
{
  AnalyticTrajectoryParameters parameters;
  parameters.type = AnalyticTrajectoryType::kCircle;
  parameters.center = Eigen::Vector3d{1.0, -2.0, 1.5};
  parameters.radius_x = 2.0;
  parameters.radius_y = 2.0;
  parameters.period = 10.0;
  parameters.face_velocity = true;
  const AnalyticTrajectoryGenerator generator(parameters);

  const Reference beginning = generator.sample(0.0);
  const Reference end = generator.sample(parameters.period);
  EXPECT_TRUE(beginning.position_world.isApprox(end.position_world, 1.0e-12));
  EXPECT_TRUE(beginning.velocity_world.isApprox(end.velocity_world, 1.0e-12));
  EXPECT_NEAR(beginning.velocity_world.y(), 4.0 * kPi / 10.0, 1.0e-12);
  EXPECT_NEAR(beginning.yaw, 0.5 * kPi, 1.0e-12);
  EXPECT_NEAR(beginning.yaw_rate, 2.0 * kPi / 10.0, 1.0e-12);
}

TEST(AnalyticTrajectoryGenerator, FigureEightCrossesCenterTwicePerPeriod)
{
  AnalyticTrajectoryParameters parameters;
  parameters.type = AnalyticTrajectoryType::kFigureEight;
  parameters.center = Eigen::Vector3d{0.5, 0.25, 2.0};
  parameters.radius_x = 2.0;
  parameters.radius_y = 1.0;
  parameters.period = 8.0;
  const AnalyticTrajectoryGenerator generator(parameters);

  EXPECT_TRUE(generator.sample(0.0).position_world.isApprox(parameters.center, 1.0e-12));
  EXPECT_TRUE(
    generator.sample(0.5 * parameters.period).position_world.isApprox(parameters.center, 1.0e-12));
  EXPECT_TRUE(generator.sample(parameters.period).position_world.isApprox(parameters.center, 1.0e-12));
}

TEST(DisturbanceModel, GustRespectsConfiguredWindow)
{
  DisturbanceModelParameters parameters;
  parameters.mode = DisturbanceMode::kGust;
  parameters.start_time = 2.0;
  parameters.duration = 1.0;
  parameters.amplitude.force_world = Eigen::Vector3d{3.0, 0.0, 0.0};
  DisturbanceModel model(parameters);

  EXPECT_TRUE(model.sample(1.9, 0.01).force_world.isZero());
  EXPECT_TRUE(model.sample(2.5, 0.01).force_world.isApprox(Eigen::Vector3d{3.0, 0.0, 0.0}));
  EXPECT_TRUE(model.sample(3.1, 0.01).force_world.isZero());
}

TEST(DisturbanceModel, SeededRandomWindIsExactlyReplayable)
{
  DisturbanceModelParameters parameters;
  parameters.mode = DisturbanceMode::kRandom;
  parameters.random_seed = 42U;
  parameters.amplitude.force_world = Eigen::Vector3d{1.0, 2.0, 3.0};
  DisturbanceModel first(parameters);
  DisturbanceModel second(parameters);

  for (int index = 0; index < 100; ++index) {
    const double time = 0.01 * static_cast<double>(index);
    EXPECT_TRUE(first.sample(time, 0.01).force_world.isApprox(
      second.sample(time, 0.01).force_world, 0.0));
  }
}

TEST(SensorNoiseModel, ZeroNoiseReturnsTruthAndSpecificForce)
{
  SensorNoiseModel model;
  State truth;
  truth.position_world = Eigen::Vector3d{1.0, 2.0, 3.0};
  truth.velocity_world = Eigen::Vector3d{0.1, 0.2, 0.3};
  truth.angular_velocity_body = Eigen::Vector3d{0.4, 0.5, 0.6};
  const SensorSample sample = model.sample(truth, Eigen::Vector3d::Zero(), 9.81, 0.01);

  EXPECT_TRUE(sample.position_world.isApprox(truth.position_world));
  EXPECT_TRUE(sample.velocity_world.isApprox(truth.velocity_world));
  EXPECT_TRUE(sample.angular_velocity_body.isApprox(truth.angular_velocity_body));
  EXPECT_TRUE(sample.linear_acceleration_body.isApprox(Eigen::Vector3d{0.0, 0.0, 9.81}));
}

TEST(SensorNoiseModel, ResetReplaysSeededNoiseAndBiasRandomWalk)
{
  SensorNoiseParameters parameters;
  parameters.position_standard_deviation = Eigen::Vector3d::Constant(0.1);
  parameters.accelerometer_bias_random_walk = Eigen::Vector3d::Constant(0.01);
  parameters.gyroscope_bias_random_walk = Eigen::Vector3d::Constant(0.02);
  parameters.random_seed = 1234U;
  SensorNoiseModel model(parameters);
  State truth;

  const SensorSample first = model.sample(truth, Eigen::Vector3d::Zero(), 9.81, 0.01);
  model.reset();
  const SensorSample replay = model.sample(truth, Eigen::Vector3d::Zero(), 9.81, 0.01);
  EXPECT_TRUE(first.position_world.isApprox(replay.position_world, 0.0));
  EXPECT_TRUE(first.accelerometer_bias.isApprox(replay.accelerometer_bias, 0.0));
  EXPECT_TRUE(first.gyroscope_bias.isApprox(replay.gyroscope_bias, 0.0));
}

}  // namespace
}  // namespace drone_core
