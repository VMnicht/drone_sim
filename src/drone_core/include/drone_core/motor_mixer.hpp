#pragma once

#include <Eigen/Core>

#include "drone_core/types.hpp"

namespace drone_core
{

/// Allocation matrix mapping squared motor speeds to [T, tau_x, tau_y, tau_z].
[[nodiscard]] Eigen::Matrix4d allocationMatrix(const QuadrotorParameters & parameters);

class MotorMixer
{
public:
  explicit MotorMixer(const QuadrotorParameters & parameters);

  [[nodiscard]] MotorCommand mix(const BodyWrench & desired_wrench) const;
  [[nodiscard]] BodyWrench wrench(const MotorCommand & motor_speeds) const;

private:
  QuadrotorParameters parameters_;
  Eigen::Matrix4d allocation_matrix_;
  Eigen::Matrix4d inverse_allocation_matrix_;
};

}  // namespace drone_core

