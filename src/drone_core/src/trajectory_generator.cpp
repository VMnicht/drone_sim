#include "drone_core/trajectory_generator.hpp"

#include <cmath>
#include <stdexcept>

namespace drone_core
{

AnalyticTrajectoryGenerator::AnalyticTrajectoryGenerator(
  const AnalyticTrajectoryParameters & parameters)
: parameters_(parameters)
{
  validateParameters();
}

const AnalyticTrajectoryParameters & AnalyticTrajectoryGenerator::parameters() const noexcept
{
  return parameters_;
}

Reference AnalyticTrajectoryGenerator::sample(double time_seconds) const
{
  if (!std::isfinite(time_seconds)) {
    throw std::invalid_argument("Trajectory time must be finite");
  }

  Reference reference;
  reference.position_world = parameters_.center;
  reference.yaw = parameters_.fixed_yaw;
  if (parameters_.type == AnalyticTrajectoryType::kHold) {
    return reference;
  }

  const double angular_rate = 2.0 * kPi / parameters_.period;
  const double theta = angular_rate * time_seconds + parameters_.phase;
  const double sine = std::sin(theta);
  const double cosine = std::cos(theta);

  if (parameters_.type == AnalyticTrajectoryType::kCircle) {
    reference.position_world.x() += parameters_.radius_x * cosine;
    reference.position_world.y() += parameters_.radius_y * sine;
    reference.velocity_world.x() = -parameters_.radius_x * angular_rate * sine;
    reference.velocity_world.y() = parameters_.radius_y * angular_rate * cosine;
    reference.acceleration_world.x() =
      -parameters_.radius_x * angular_rate * angular_rate * cosine;
    reference.acceleration_world.y() =
      -parameters_.radius_y * angular_rate * angular_rate * sine;
    return applyYaw(reference);
  }

  // Gerono lemniscate: x = a*sin(theta), y = b*sin(theta)*cos(theta).
  reference.position_world.x() += parameters_.radius_x * sine;
  reference.position_world.y() += parameters_.radius_y * sine * cosine;
  reference.velocity_world.x() = parameters_.radius_x * angular_rate * cosine;
  reference.velocity_world.y() =
    parameters_.radius_y * angular_rate * std::cos(2.0 * theta);
  reference.acceleration_world.x() =
    -parameters_.radius_x * angular_rate * angular_rate * sine;
  reference.acceleration_world.y() =
    -2.0 * parameters_.radius_y * angular_rate * angular_rate * std::sin(2.0 * theta);
  return applyYaw(reference);
}

Reference AnalyticTrajectoryGenerator::applyYaw(Reference reference) const
{
  if (!parameters_.face_velocity) {
    reference.yaw = parameters_.fixed_yaw;
    reference.yaw_rate = 0.0;
    return reference;
  }

  const Eigen::Vector2d velocity = reference.velocity_world.head<2>();
  const Eigen::Vector2d acceleration = reference.acceleration_world.head<2>();
  const double speed_squared = velocity.squaredNorm();
  if (speed_squared <= parameters_.minimum_yaw_speed * parameters_.minimum_yaw_speed) {
    reference.yaw = parameters_.fixed_yaw;
    reference.yaw_rate = 0.0;
    return reference;
  }
  reference.yaw = std::atan2(velocity.y(), velocity.x());
  reference.yaw_rate =
    (velocity.x() * acceleration.y() - velocity.y() * acceleration.x()) / speed_squared;
  return reference;
}

void AnalyticTrajectoryGenerator::validateParameters() const
{
  if (!parameters_.center.array().isFinite().all() ||
    !std::isfinite(parameters_.radius_x) || parameters_.radius_x <= 0.0 ||
    !std::isfinite(parameters_.radius_y) || parameters_.radius_y <= 0.0 ||
    !std::isfinite(parameters_.period) || parameters_.period <= 0.0 ||
    !std::isfinite(parameters_.phase) || !std::isfinite(parameters_.fixed_yaw) ||
    !std::isfinite(parameters_.minimum_yaw_speed) || parameters_.minimum_yaw_speed <= 0.0)
  {
    throw std::invalid_argument("Invalid analytic trajectory parameters");
  }
}

}  // namespace drone_core
