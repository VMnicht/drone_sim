#include "drone_core/quadrotor_model.hpp"

#include "drone_core/motor_mixer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace drone_core
{
namespace
{

bool finiteVector(const Eigen::Vector3d & value)
{
  return value.array().isFinite().all();
}

bool finiteQuaternion(const Eigen::Quaterniond & value)
{
  return std::isfinite(value.w()) && std::isfinite(value.x()) &&
         std::isfinite(value.y()) && std::isfinite(value.z());
}

}  // namespace

bool State::isFinite() const
{
  if (!finiteVector(position_world) || !finiteVector(velocity_world) ||
    !finiteVector(angular_velocity_body) || !finiteQuaternion(orientation_body_to_world))
  {
    return false;
  }
  return std::all_of(
    motor_angular_velocity.begin(), motor_angular_velocity.end(),
    [](double value) {return std::isfinite(value);});
}

QuadrotorModel::QuadrotorModel(const QuadrotorParameters & parameters)
: parameters_(parameters)
{
  validateParameters();
  reset();
}

const QuadrotorParameters & QuadrotorModel::parameters() const noexcept
{
  return parameters_;
}

const State & QuadrotorModel::state() const noexcept
{
  return state_;
}

double QuadrotorModel::hoverMotorSpeed() const
{
  return std::sqrt(
    parameters_.mass * parameters_.gravity /
    (static_cast<double>(kMotorCount) * parameters_.thrust_coefficient));
}

void QuadrotorModel::reset(const State & state)
{
  if (!state.isFinite()) {
    throw std::invalid_argument("Cannot reset quadrotor with a non-finite state");
  }
  const double quaternion_norm = state.orientation_body_to_world.norm();
  if (quaternion_norm < 1e-12) {
    throw std::invalid_argument("Cannot reset quadrotor with a zero quaternion");
  }
  state_ = state;
  state_.orientation_body_to_world.normalize();
  for (double & motor_speed : state_.motor_angular_velocity) {
    motor_speed = std::clamp(
      motor_speed, parameters_.minimum_motor_speed, parameters_.maximum_motor_speed);
  }
  applyGroundConstraint(state_);
}

DynamicsResult QuadrotorModel::step(
  const MotorCommand & command, double dt, const Disturbance & disturbance)
{
  if (!std::isfinite(dt) || dt <= 0.0) {
    throw std::invalid_argument("Dynamics time step must be finite and positive");
  }
  if (!disturbance.isFinite()) {
    throw std::invalid_argument("Disturbance must be finite");
  }

  State next = state_;
  next.motor_angular_velocity = updateMotors(command, dt);
  const BodyWrench wrench = computeBodyWrench(next.motor_angular_velocity);

  const Eigen::Vector3d thrust_body{0.0, 0.0, wrench.thrust};
  const Eigen::Vector3d gravity_world{0.0, 0.0, -parameters_.gravity};
  const Eigen::Vector3d linear_acceleration_world =
    next.orientation_body_to_world * thrust_body / parameters_.mass + gravity_world -
    parameters_.linear_drag_coefficient * next.velocity_world / parameters_.mass +
    disturbance.force_world / parameters_.mass;

  const Eigen::Matrix3d inertia = parameters_.inertia_diagonal.asDiagonal();
  const Eigen::Vector3d damped_torque =
    wrench.torque + disturbance.torque_body -
    parameters_.angular_damping.cwiseProduct(next.angular_velocity_body);
  const Eigen::Vector3d angular_acceleration_body = parameters_.inertia_diagonal.cwiseInverse().cwiseProduct(
    damped_torque - next.angular_velocity_body.cross(inertia * next.angular_velocity_body));

  // Semi-implicit Euler for translation and angular velocity.
  next.velocity_world += linear_acceleration_world * dt;
  next.position_world += next.velocity_world * dt;
  next.angular_velocity_body += angular_acceleration_body * dt;

  // Integrate body angular velocity by a right-multiplied incremental rotation.
  const double angular_speed = next.angular_velocity_body.norm();
  if (angular_speed > 1e-12) {
    const Eigen::AngleAxisd delta_rotation(angular_speed * dt, next.angular_velocity_body / angular_speed);
    next.orientation_body_to_world =
      (next.orientation_body_to_world * Eigen::Quaterniond(delta_rotation)).normalized();
  } else {
    next.orientation_body_to_world.normalize();
  }

  applyGroundConstraint(next);
  if (!next.isFinite()) {
    throw std::runtime_error("Quadrotor dynamics produced a non-finite state");
  }

  state_ = next;
  return DynamicsResult{
    state_, linear_acceleration_world, angular_acceleration_body, wrench, disturbance};
}

BodyWrench QuadrotorModel::computeBodyWrench(const MotorCommand & motor_speeds) const
{
  return MotorMixer(parameters_).wrench(motor_speeds);
}

void QuadrotorModel::validateParameters() const
{
  if (!std::isfinite(parameters_.mass) || parameters_.mass <= 0.0 ||
    !std::isfinite(parameters_.gravity) || parameters_.gravity <= 0.0 ||
    !finiteVector(parameters_.inertia_diagonal) ||
    (parameters_.inertia_diagonal.array() <= 0.0).any() ||
    !std::isfinite(parameters_.arm_length) || parameters_.arm_length <= 0.0 ||
    !std::isfinite(parameters_.thrust_coefficient) || parameters_.thrust_coefficient <= 0.0 ||
    !std::isfinite(parameters_.drag_moment_coefficient) ||
    parameters_.drag_moment_coefficient < 0.0 ||
    !std::isfinite(parameters_.motor_time_constant) || parameters_.motor_time_constant <= 0.0 ||
    !std::isfinite(parameters_.minimum_motor_speed) ||
    !std::isfinite(parameters_.maximum_motor_speed) ||
    parameters_.minimum_motor_speed < 0.0 ||
    parameters_.maximum_motor_speed <= parameters_.minimum_motor_speed ||
    !std::isfinite(parameters_.linear_drag_coefficient) ||
    parameters_.linear_drag_coefficient < 0.0 ||
    !finiteVector(parameters_.angular_damping) ||
    (parameters_.angular_damping.array() < 0.0).any() ||
    !std::isfinite(parameters_.ground_height) ||
    !std::isfinite(parameters_.ground_restitution) ||
    parameters_.ground_restitution < 0.0 || parameters_.ground_restitution > 1.0)
  {
    throw std::invalid_argument("Invalid quadrotor parameters");
  }
}

MotorCommand QuadrotorModel::updateMotors(const MotorCommand & command, double dt) const
{
  MotorCommand updated{};
  const double response = 1.0 - std::exp(-dt / parameters_.motor_time_constant);
  for (std::size_t i = 0; i < kMotorCount; ++i) {
    if (!std::isfinite(command[i])) {
      throw std::invalid_argument("Motor command must be finite");
    }
    const double clamped_command = std::clamp(
      command[i], parameters_.minimum_motor_speed, parameters_.maximum_motor_speed);
    updated[i] = state_.motor_angular_velocity[i] +
      response * (clamped_command - state_.motor_angular_velocity[i]);
    updated[i] = std::clamp(
      updated[i], parameters_.minimum_motor_speed, parameters_.maximum_motor_speed);
  }
  return updated;
}

void QuadrotorModel::applyGroundConstraint(State & state) const
{
  if (state.position_world.z() < parameters_.ground_height) {
    state.position_world.z() = parameters_.ground_height;
    if (state.velocity_world.z() < 0.0) {
      state.velocity_world.z() = -parameters_.ground_restitution * state.velocity_world.z();
    }
  }
}

}  // namespace drone_core
