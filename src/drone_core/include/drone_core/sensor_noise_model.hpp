#pragma once

#include <cstdint>
#include <random>

#include "drone_core/types.hpp"

namespace drone_core
{

struct SensorNoiseParameters
{
  Eigen::Vector3d position_standard_deviation{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_standard_deviation{Eigen::Vector3d::Zero()};
  Eigen::Vector3d orientation_standard_deviation{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accelerometer_standard_deviation{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyroscope_standard_deviation{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accelerometer_initial_bias{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyroscope_initial_bias{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accelerometer_bias_random_walk{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyroscope_bias_random_walk{Eigen::Vector3d::Zero()};
  std::uint32_t random_seed{1U};
};

struct SensorSample
{
  Eigen::Vector3d position_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_world{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation_body_to_world{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d linear_acceleration_body{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_velocity_body{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accelerometer_bias{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyroscope_bias{Eigen::Vector3d::Zero()};

  [[nodiscard]] bool isFinite() const;
};

/// Deterministic simplified IMU/odometry noise model without ROS dependencies.
class SensorNoiseModel
{
public:
  explicit SensorNoiseModel(const SensorNoiseParameters & parameters = {});

  void reset();
  [[nodiscard]] SensorSample sample(
    const State & truth,
    const Eigen::Vector3d & linear_acceleration_world,
    double gravity,
    double dt);
  [[nodiscard]] const SensorNoiseParameters & parameters() const noexcept;

private:
  void validateParameters() const;
  [[nodiscard]] Eigen::Vector3d whiteNoise(const Eigen::Vector3d & standard_deviation);

  SensorNoiseParameters parameters_;
  std::mt19937 random_engine_;
  std::normal_distribution<double> normal_{0.0, 1.0};
  Eigen::Vector3d accelerometer_bias_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyroscope_bias_{Eigen::Vector3d::Zero()};
};

}  // namespace drone_core
