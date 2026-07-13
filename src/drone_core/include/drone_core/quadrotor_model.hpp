#pragma once

#include "drone_core/types.hpp"

namespace drone_core
{

/// ROS-independent X-configuration quadrotor rigid-body model.
///
/// Frames follow ROS REP-103 conventions without depending on ROS types:
/// world is ENU, body is FLU, and the quaternion rotates body vectors into world.
/// Motor order is front-left, rear-left, rear-right, front-right.
class QuadrotorModel
{
public:
  explicit QuadrotorModel(const QuadrotorParameters & parameters = {});

  [[nodiscard]] const QuadrotorParameters & parameters() const noexcept;
  [[nodiscard]] const State & state() const noexcept;
  [[nodiscard]] double hoverMotorSpeed() const;

  void reset(const State & state = {});
  DynamicsResult step(
    const MotorCommand & command, double dt, const Disturbance & disturbance = {});
  [[nodiscard]] BodyWrench computeBodyWrench(const MotorCommand & motor_speeds) const;

private:
  void validateParameters() const;
  [[nodiscard]] MotorCommand updateMotors(const MotorCommand & command, double dt) const;
  void applyGroundConstraint(State & state) const;

  QuadrotorParameters parameters_;
  State state_;
};

}  // namespace drone_core
