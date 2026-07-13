#pragma once

#include <array>
#include <cmath>
#include <cstddef>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace drone_core
{

constexpr std::size_t kMotorCount = 4U;
constexpr double kPi = 3.14159265358979323846;
constexpr double kRpmToRadPerSecond = 2.0 * kPi / 60.0;
constexpr double kRadPerSecondToRpm = 60.0 / (2.0 * kPi);

using MotorCommand = std::array<double, kMotorCount>;

struct QuadrotorParameters
{
  double mass{1.0};
  double gravity{9.81};
  Eigen::Vector3d inertia_diagonal{0.02, 0.02, 0.04};
  double arm_length{0.17};
  double thrust_coefficient{1.91e-6};
  double drag_moment_coefficient{2.60e-7};
  double motor_time_constant{0.03};
  double minimum_motor_speed{0.0};
  double maximum_motor_speed{2300.0};
  double linear_drag_coefficient{0.05};
  Eigen::Vector3d angular_damping{0.002, 0.002, 0.002};
  double ground_height{0.0};
  double ground_restitution{0.0};
};

struct State
{
  Eigen::Vector3d position_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_world{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation_body_to_world{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d angular_velocity_body{Eigen::Vector3d::Zero()};
  MotorCommand motor_angular_velocity{0.0, 0.0, 0.0, 0.0};

  [[nodiscard]] bool isFinite() const;
};

struct BodyWrench
{
  double thrust{0.0};
  Eigen::Vector3d torque{Eigen::Vector3d::Zero()};
};

struct Disturbance
{
  Eigen::Vector3d force_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d torque_body{Eigen::Vector3d::Zero()};

  [[nodiscard]] bool isFinite() const
  {
    return force_world.array().isFinite().all() && torque_body.array().isFinite().all();
  }
};

struct DynamicsResult
{
  State state{};
  Eigen::Vector3d linear_acceleration_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_acceleration_body{Eigen::Vector3d::Zero()};
  BodyWrench body_wrench{};
  Disturbance applied_disturbance{};
};

}  // namespace drone_core
