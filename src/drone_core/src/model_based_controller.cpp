#include "drone_core/model_based_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace drone_core
{
namespace
{

Eigen::Vector3d vee(const Eigen::Matrix3d & skew_symmetric)
{
  return Eigen::Vector3d{
    skew_symmetric(2, 1), skew_symmetric(0, 2), skew_symmetric(1, 0)};
}

Eigen::Vector3d clampNorm(const Eigen::Vector3d & value, double maximum_norm)
{
  const double norm = value.norm();
  if (norm > maximum_norm && norm > 1e-12) {
    return value * (maximum_norm / norm);
  }
  return value;
}

}  // namespace

bool Reference::isFinite() const
{
  return position_world.array().isFinite().all() &&
         velocity_world.array().isFinite().all() &&
         acceleration_world.array().isFinite().all() &&
         std::isfinite(yaw) && std::isfinite(yaw_rate);
}

ModelBasedController::ModelBasedController(
  const QuadrotorParameters & model_parameters,
  const ModelBasedControllerParameters & controller_parameters)
: model_parameters_(model_parameters),
  controller_parameters_(controller_parameters),
  mixer_(model_parameters)
{
  validateParameters();
}

ControlOutput ModelBasedController::compute(
  const State & state, const Reference & reference) const
{
  if (!state.isFinite() || !reference.isFinite()) {
    throw std::invalid_argument("Controller state and reference must be finite");
  }

  const Eigen::Quaterniond orientation = state.orientation_body_to_world.normalized();
  const Eigen::Matrix3d rotation = orientation.toRotationMatrix();
  const Eigen::Vector3d position_error = state.position_world - reference.position_world;
  const Eigen::Vector3d velocity_error = state.velocity_world - reference.velocity_world;

  Eigen::Vector3d feedback_acceleration =
    -controller_parameters_.position_gain.cwiseProduct(position_error) -
    controller_parameters_.velocity_gain.cwiseProduct(velocity_error);
  Eigen::Vector3d horizontal_feedback{
    feedback_acceleration.x(), feedback_acceleration.y(), 0.0};
  horizontal_feedback = clampNorm(
    horizontal_feedback, controller_parameters_.maximum_horizontal_acceleration);
  feedback_acceleration.x() = horizontal_feedback.x();
  feedback_acceleration.y() = horizontal_feedback.y();
  feedback_acceleration.z() = std::clamp(
    feedback_acceleration.z(), -controller_parameters_.maximum_vertical_acceleration,
    controller_parameters_.maximum_vertical_acceleration);

  Eigen::Vector3d desired_force = model_parameters_.mass *
    (reference.acceleration_world + feedback_acceleration +
    Eigen::Vector3d{0.0, 0.0, model_parameters_.gravity});

  // Enforce the tilt limit by bounding horizontal force relative to vertical force.
  Eigen::Vector3d horizontal_force{desired_force.x(), desired_force.y(), 0.0};
  const double maximum_horizontal_force =
    std::max(0.0, desired_force.z()) * std::tan(controller_parameters_.maximum_tilt);
  horizontal_force = clampNorm(horizontal_force, maximum_horizontal_force);
  desired_force.x() = horizontal_force.x();
  desired_force.y() = horizontal_force.y();

  Eigen::Vector3d desired_body_z = Eigen::Vector3d::UnitZ();
  if (desired_force.norm() > 1e-9) {
    desired_body_z = desired_force.normalized();
  }
  const Eigen::Vector3d heading{
    std::cos(reference.yaw), std::sin(reference.yaw), 0.0};
  Eigen::Vector3d desired_body_y = desired_body_z.cross(heading);
  if (desired_body_y.norm() < 1e-9) {
    desired_body_y = desired_body_z.cross(Eigen::Vector3d::UnitY());
  }
  desired_body_y.normalize();
  const Eigen::Vector3d desired_body_x = desired_body_y.cross(desired_body_z).normalized();

  Eigen::Matrix3d desired_rotation;
  desired_rotation.col(0) = desired_body_x;
  desired_rotation.col(1) = desired_body_y;
  desired_rotation.col(2) = desired_body_z;

  const Eigen::Vector3d attitude_error = 0.5 * vee(
    desired_rotation.transpose() * rotation - rotation.transpose() * desired_rotation);
  const Eigen::Vector3d desired_angular_velocity_body =
    rotation.transpose() * Eigen::Vector3d{0.0, 0.0, reference.yaw_rate};
  const Eigen::Vector3d angular_rate_error =
    state.angular_velocity_body - desired_angular_velocity_body;
  const Eigen::Matrix3d inertia = model_parameters_.inertia_diagonal.asDiagonal();

  Eigen::Vector3d desired_torque =
    -controller_parameters_.attitude_gain.cwiseProduct(attitude_error) -
    controller_parameters_.angular_rate_gain.cwiseProduct(angular_rate_error) +
    state.angular_velocity_body.cross(inertia * state.angular_velocity_body);
  for (Eigen::Index i = 0; i < 3; ++i) {
    desired_torque[i] = std::clamp(
      desired_torque[i], -controller_parameters_.maximum_torque[i],
      controller_parameters_.maximum_torque[i]);
  }

  const double maximum_thrust = controller_parameters_.maximum_thrust_to_weight *
    model_parameters_.mass * model_parameters_.gravity;
  const double desired_thrust = std::clamp(
    desired_force.dot(rotation.col(2)), 0.0, maximum_thrust);
  const BodyWrench desired_wrench{desired_thrust, desired_torque};

  ControlOutput output;
  output.desired_wrench = desired_wrench;
  output.motor_angular_velocity = mixer_.mix(desired_wrench);
  output.desired_force_world = desired_force;
  output.desired_orientation_body_to_world = desired_rotation;
  output.position_error = position_error;
  output.attitude_error = attitude_error;
  return output;
}

void ModelBasedController::validateParameters() const
{
  const bool vector_parameters_valid =
    controller_parameters_.position_gain.array().isFinite().all() &&
    controller_parameters_.velocity_gain.array().isFinite().all() &&
    controller_parameters_.attitude_gain.array().isFinite().all() &&
    controller_parameters_.angular_rate_gain.array().isFinite().all() &&
    controller_parameters_.maximum_torque.array().isFinite().all() &&
    (controller_parameters_.position_gain.array() >= 0.0).all() &&
    (controller_parameters_.velocity_gain.array() >= 0.0).all() &&
    (controller_parameters_.attitude_gain.array() >= 0.0).all() &&
    (controller_parameters_.angular_rate_gain.array() >= 0.0).all() &&
    (controller_parameters_.maximum_torque.array() > 0.0).all();
  if (!vector_parameters_valid ||
    !std::isfinite(controller_parameters_.maximum_horizontal_acceleration) ||
    controller_parameters_.maximum_horizontal_acceleration <= 0.0 ||
    !std::isfinite(controller_parameters_.maximum_vertical_acceleration) ||
    controller_parameters_.maximum_vertical_acceleration <= 0.0 ||
    !std::isfinite(controller_parameters_.maximum_tilt) ||
    controller_parameters_.maximum_tilt <= 0.0 ||
    controller_parameters_.maximum_tilt >= 0.5 * kPi ||
    !std::isfinite(controller_parameters_.maximum_thrust_to_weight) ||
    controller_parameters_.maximum_thrust_to_weight <= 1.0)
  {
    throw std::invalid_argument("Invalid model-based controller parameters");
  }
}

}  // namespace drone_core

