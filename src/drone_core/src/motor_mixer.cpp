#include "drone_core/motor_mixer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/LU>

namespace drone_core
{

Eigen::Matrix4d allocationMatrix(const QuadrotorParameters & parameters)
{
  const double xy = parameters.arm_length / std::sqrt(2.0);
  const double kf = parameters.thrust_coefficient;
  const double km = parameters.drag_moment_coefficient;

  Eigen::Matrix4d matrix;
  // Motor order: front-left CCW, rear-left CW, rear-right CCW, front-right CW.
  matrix <<
    kf, kf, kf, kf,
    xy * kf, xy * kf, -xy * kf, -xy * kf,
    -xy * kf, xy * kf, xy * kf, -xy * kf,
    km, -km, km, -km;
  return matrix;
}

MotorMixer::MotorMixer(const QuadrotorParameters & parameters)
: parameters_(parameters),
  allocation_matrix_(allocationMatrix(parameters)),
  inverse_allocation_matrix_(Eigen::Matrix4d::Zero())
{
  const Eigen::FullPivLU<Eigen::Matrix4d> decomposition(allocation_matrix_);
  if (!allocation_matrix_.array().isFinite().all() || !decomposition.isInvertible()) {
    throw std::invalid_argument("Motor allocation matrix is singular or non-finite");
  }
  inverse_allocation_matrix_ = decomposition.inverse();
}

MotorCommand MotorMixer::mix(const BodyWrench & desired_wrench) const
{
  if (!std::isfinite(desired_wrench.thrust) || !desired_wrench.torque.array().isFinite().all()) {
    throw std::invalid_argument("Desired wrench must be finite");
  }

  const Eigen::Vector4d wrench_vector{
    std::max(0.0, desired_wrench.thrust), desired_wrench.torque.x(),
    desired_wrench.torque.y(), desired_wrench.torque.z()};
  Eigen::Vector4d squared_speeds = inverse_allocation_matrix_ * wrench_vector;
  const double minimum_squared =
    parameters_.minimum_motor_speed * parameters_.minimum_motor_speed;
  const double maximum_squared =
    parameters_.maximum_motor_speed * parameters_.maximum_motor_speed;

  MotorCommand result{};
  for (std::size_t i = 0; i < kMotorCount; ++i) {
    squared_speeds[static_cast<Eigen::Index>(i)] = std::clamp(
      squared_speeds[static_cast<Eigen::Index>(i)], minimum_squared, maximum_squared);
    result[i] = std::sqrt(squared_speeds[static_cast<Eigen::Index>(i)]);
  }
  return result;
}

BodyWrench MotorMixer::wrench(const MotorCommand & motor_speeds) const
{
  Eigen::Vector4d squared_speeds;
  for (std::size_t i = 0; i < kMotorCount; ++i) {
    if (!std::isfinite(motor_speeds[i])) {
      throw std::invalid_argument("Motor speed must be finite");
    }
    const double clamped = std::clamp(
      motor_speeds[i], parameters_.minimum_motor_speed, parameters_.maximum_motor_speed);
    squared_speeds[static_cast<Eigen::Index>(i)] = clamped * clamped;
  }

  const Eigen::Vector4d result = allocation_matrix_ * squared_speeds;
  return BodyWrench{result[0], result.tail<3>()};
}

}  // namespace drone_core
