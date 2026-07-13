#pragma once

#include "drone_core/motor_mixer.hpp"
#include "drone_core/types.hpp"

namespace drone_core
{

struct Reference
{
  Eigen::Vector3d position_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration_world{Eigen::Vector3d::Zero()};
  double yaw{0.0};
  double yaw_rate{0.0};

  [[nodiscard]] bool isFinite() const;
};

struct ModelBasedControllerParameters
{
  // Position gains generate acceleration feedback in world coordinates.
  Eigen::Vector3d position_gain{3.0, 3.0, 6.0};
  Eigen::Vector3d velocity_gain{3.5, 3.5, 4.5};
  // Geometric attitude gains generate body torque.
  Eigen::Vector3d attitude_gain{0.40, 0.40, 0.25};
  Eigen::Vector3d angular_rate_gain{0.16, 0.16, 0.10};
  Eigen::Vector3d maximum_torque{1.0, 1.0, 0.5};
  double maximum_horizontal_acceleration{3.0};
  double maximum_vertical_acceleration{6.0};
  double maximum_tilt{25.0 * kPi / 180.0};
  double maximum_thrust_to_weight{2.5};
};

struct ControlOutput
{
  BodyWrench desired_wrench{};
  MotorCommand motor_angular_velocity{0.0, 0.0, 0.0, 0.0};
  Eigen::Vector3d desired_force_world{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d desired_orientation_body_to_world{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d position_error{Eigen::Vector3d::Zero()};
  Eigen::Vector3d attitude_error{Eigen::Vector3d::Zero()};
};

/// Cascaded nonlinear geometric controller using the quadrotor mass, inertia,
/// gravity and actuator allocation model directly.
class ModelBasedController
{
public:
  ModelBasedController(
    const QuadrotorParameters & model_parameters,
    const ModelBasedControllerParameters & controller_parameters = {});

  [[nodiscard]] ControlOutput compute(
    const State & state, const Reference & reference) const;

private:
  void validateParameters() const;

  QuadrotorParameters model_parameters_;
  ModelBasedControllerParameters controller_parameters_;
  MotorMixer mixer_;
};

}  // namespace drone_core
