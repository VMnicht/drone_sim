#include "drone_core/sensor_noise_model.hpp"

#include <cmath>
#include <stdexcept>

namespace drone_core
{
namespace
{

bool nonnegativeFinite(const Eigen::Vector3d & value)
{
  return value.array().isFinite().all() && (value.array() >= 0.0).all();
}

bool finiteQuaternion(const Eigen::Quaterniond & value)
{
  return std::isfinite(value.w()) && std::isfinite(value.x()) &&
         std::isfinite(value.y()) && std::isfinite(value.z());
}

}  // namespace

bool SensorSample::isFinite() const
{
  return position_world.array().isFinite().all() &&
         velocity_world.array().isFinite().all() &&
         finiteQuaternion(orientation_body_to_world) &&
         linear_acceleration_body.array().isFinite().all() &&
         angular_velocity_body.array().isFinite().all() &&
         accelerometer_bias.array().isFinite().all() &&
         gyroscope_bias.array().isFinite().all();
}

SensorNoiseModel::SensorNoiseModel(const SensorNoiseParameters & parameters)
: parameters_(parameters), random_engine_(parameters.random_seed)
{
  validateParameters();
  reset();
}

void SensorNoiseModel::reset()
{
  random_engine_.seed(parameters_.random_seed);
  normal_.reset();
  accelerometer_bias_ = parameters_.accelerometer_initial_bias;
  gyroscope_bias_ = parameters_.gyroscope_initial_bias;
}

const SensorNoiseParameters & SensorNoiseModel::parameters() const noexcept
{
  return parameters_;
}

Eigen::Vector3d SensorNoiseModel::whiteNoise(const Eigen::Vector3d & standard_deviation)
{
  return standard_deviation.cwiseProduct(
    Eigen::Vector3d{normal_(random_engine_), normal_(random_engine_), normal_(random_engine_)});
}

SensorSample SensorNoiseModel::sample(
  const State & truth,
  const Eigen::Vector3d & linear_acceleration_world,
  double gravity,
  double dt)
{
  if (!truth.isFinite() || !linear_acceleration_world.array().isFinite().all() ||
    !std::isfinite(gravity) || gravity <= 0.0 || !std::isfinite(dt) || dt <= 0.0)
  {
    throw std::invalid_argument("Invalid truth state or sensor sample timing");
  }

  const double random_walk_scale = std::sqrt(dt);
  accelerometer_bias_ += random_walk_scale *
    whiteNoise(parameters_.accelerometer_bias_random_walk);
  gyroscope_bias_ += random_walk_scale * whiteNoise(parameters_.gyroscope_bias_random_walk);

  SensorSample result;
  result.position_world = truth.position_world + whiteNoise(parameters_.position_standard_deviation);
  result.velocity_world = truth.velocity_world + whiteNoise(parameters_.velocity_standard_deviation);

  const Eigen::Vector3d orientation_error = whiteNoise(
    parameters_.orientation_standard_deviation);
  const double angle = orientation_error.norm();
  Eigen::Quaterniond error_rotation = Eigen::Quaterniond::Identity();
  if (angle > 1.0e-12) {
    error_rotation = Eigen::Quaterniond(Eigen::AngleAxisd(angle, orientation_error / angle));
  }
  result.orientation_body_to_world =
    (truth.orientation_body_to_world * error_rotation).normalized();

  const Eigen::Vector3d specific_force_world =
    linear_acceleration_world - Eigen::Vector3d{0.0, 0.0, -gravity};
  result.linear_acceleration_body = truth.orientation_body_to_world.conjugate() *
    specific_force_world + accelerometer_bias_ +
    whiteNoise(parameters_.accelerometer_standard_deviation);
  result.angular_velocity_body = truth.angular_velocity_body + gyroscope_bias_ +
    whiteNoise(parameters_.gyroscope_standard_deviation);
  result.accelerometer_bias = accelerometer_bias_;
  result.gyroscope_bias = gyroscope_bias_;
  if (!result.isFinite()) {
    throw std::runtime_error("Sensor noise model produced a non-finite sample");
  }
  return result;
}

void SensorNoiseModel::validateParameters() const
{
  if (!nonnegativeFinite(parameters_.position_standard_deviation) ||
    !nonnegativeFinite(parameters_.velocity_standard_deviation) ||
    !nonnegativeFinite(parameters_.orientation_standard_deviation) ||
    !nonnegativeFinite(parameters_.accelerometer_standard_deviation) ||
    !nonnegativeFinite(parameters_.gyroscope_standard_deviation) ||
    !parameters_.accelerometer_initial_bias.array().isFinite().all() ||
    !parameters_.gyroscope_initial_bias.array().isFinite().all() ||
    !nonnegativeFinite(parameters_.accelerometer_bias_random_walk) ||
    !nonnegativeFinite(parameters_.gyroscope_bias_random_walk))
  {
    throw std::invalid_argument("Invalid sensor noise parameters");
  }
}

}  // namespace drone_core
