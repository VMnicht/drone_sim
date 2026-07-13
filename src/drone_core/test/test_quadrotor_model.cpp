#include <array>
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "drone_core/quadrotor_model.hpp"

namespace drone_core
{
namespace
{

TEST(QuadrotorModel, HoverSpeedBalancesWeightAndTorque)
{
  QuadrotorParameters parameters;
  parameters.linear_drag_coefficient = 0.0;
  parameters.angular_damping.setZero();
  QuadrotorModel model(parameters);

  const double hover_speed = model.hoverMotorSpeed();
  State hover_state;
  hover_state.position_world.z() = 1.5;
  hover_state.motor_angular_velocity.fill(hover_speed);
  model.reset(hover_state);

  const MotorCommand hover_command{hover_speed, hover_speed, hover_speed, hover_speed};
  const auto result = model.step(hover_command, 0.005);

  EXPECT_NEAR(result.body_wrench.thrust, parameters.mass * parameters.gravity, 1e-10);
  EXPECT_TRUE(result.body_wrench.torque.isZero(1e-12));
  EXPECT_TRUE(result.linear_acceleration_world.isZero(1e-10));
  EXPECT_TRUE(result.angular_acceleration_body.isZero(1e-10));
  EXPECT_NEAR(result.state.position_world.z(), 1.5, 1e-10);
}

TEST(QuadrotorModel, MotorResponseUsesFirstOrderExactUpdate)
{
  QuadrotorParameters parameters;
  parameters.motor_time_constant = 0.1;
  QuadrotorModel model(parameters);
  const MotorCommand command{1000.0, 1000.0, 1000.0, 1000.0};

  const auto result = model.step(command, 0.1);
  const double expected = 1000.0 * (1.0 - std::exp(-1.0));

  for (const double speed : result.state.motor_angular_velocity) {
    EXPECT_NEAR(speed, expected, 1e-10);
  }
}

TEST(QuadrotorModel, SymmetricMotorSpeedsCancelAllTorques)
{
  QuadrotorModel model;
  const MotorCommand speeds{800.0, 800.0, 800.0, 800.0};
  const BodyWrench wrench = model.computeBodyWrench(speeds);

  EXPECT_GT(wrench.thrust, 0.0);
  EXPECT_TRUE(wrench.torque.isZero(1e-12));
}

TEST(QuadrotorModel, MotorLayoutProducesExpectedTorqueSigns)
{
  QuadrotorModel model;

  const BodyWrench front_left = model.computeBodyWrench({900.0, 0.0, 0.0, 0.0});
  EXPECT_GT(front_left.torque.x(), 0.0);
  EXPECT_LT(front_left.torque.y(), 0.0);
  EXPECT_GT(front_left.torque.z(), 0.0);

  const BodyWrench rear_left = model.computeBodyWrench({0.0, 900.0, 0.0, 0.0});
  EXPECT_GT(rear_left.torque.x(), 0.0);
  EXPECT_GT(rear_left.torque.y(), 0.0);
  EXPECT_LT(rear_left.torque.z(), 0.0);
}

TEST(QuadrotorModel, MotorCommandsAreClamped)
{
  QuadrotorParameters parameters;
  parameters.maximum_motor_speed = 1200.0;
  parameters.motor_time_constant = 1e-4;
  QuadrotorModel model(parameters);

  const auto result = model.step({-100.0, 5000.0, 5000.0, -1.0}, 1.0);
  EXPECT_DOUBLE_EQ(result.state.motor_angular_velocity[0], 0.0);
  EXPECT_DOUBLE_EQ(result.state.motor_angular_velocity[1], 1200.0);
  EXPECT_DOUBLE_EQ(result.state.motor_angular_velocity[2], 1200.0);
  EXPECT_DOUBLE_EQ(result.state.motor_angular_velocity[3], 0.0);
}

TEST(QuadrotorModel, QuaternionStaysNormalized)
{
  QuadrotorParameters parameters;
  parameters.ground_height = -1000.0;
  QuadrotorModel model(parameters);
  State state;
  state.position_world.z() = 10.0;
  state.angular_velocity_body = Eigen::Vector3d{0.3, -0.2, 0.7};
  model.reset(state);

  for (int i = 0; i < 2000; ++i) {
    model.step({0.0, 0.0, 0.0, 0.0}, 0.002);
  }

  EXPECT_NEAR(model.state().orientation_body_to_world.norm(), 1.0, 1e-12);
  EXPECT_TRUE(model.state().isFinite());
}

TEST(QuadrotorModel, GroundConstraintPreventsPenetration)
{
  QuadrotorParameters parameters;
  parameters.ground_height = 0.0;
  QuadrotorModel model(parameters);
  State falling;
  falling.position_world.z() = 0.001;
  falling.velocity_world.z() = -1.0;
  model.reset(falling);

  const auto result = model.step({0.0, 0.0, 0.0, 0.0}, 0.01);
  EXPECT_DOUBLE_EQ(result.state.position_world.z(), 0.0);
  EXPECT_DOUBLE_EQ(result.state.velocity_world.z(), 0.0);
}

TEST(QuadrotorModel, RejectsNonFiniteInput)
{
  QuadrotorModel model;
  MotorCommand command{0.0, 0.0, 0.0, 0.0};
  command[2] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(model.step(command, 0.01), std::invalid_argument);
  EXPECT_THROW(model.step({0.0, 0.0, 0.0, 0.0}, 0.0), std::invalid_argument);
}

TEST(QuadrotorModel, AppliesExplicitExternalDisturbance)
{
  QuadrotorParameters parameters;
  parameters.linear_drag_coefficient = 0.0;
  parameters.angular_damping.setZero();
  QuadrotorModel model(parameters);
  State hover_state;
  hover_state.position_world.z() = 1.5;
  hover_state.motor_angular_velocity.fill(model.hoverMotorSpeed());
  model.reset(hover_state);
  Disturbance disturbance;
  disturbance.force_world = Eigen::Vector3d{1.0, -2.0, 0.0};
  disturbance.torque_body = Eigen::Vector3d{0.0, 0.0, 0.04};

  const auto result = model.step(
    {model.hoverMotorSpeed(), model.hoverMotorSpeed(), model.hoverMotorSpeed(),
      model.hoverMotorSpeed()},
    0.001, disturbance);

  EXPECT_NEAR(result.linear_acceleration_world.x(), 1.0, 1e-10);
  EXPECT_NEAR(result.linear_acceleration_world.y(), -2.0, 1e-10);
  EXPECT_NEAR(result.angular_acceleration_body.z(), 1.0, 1e-10);
  EXPECT_TRUE(result.applied_disturbance.force_world.isApprox(disturbance.force_world));
}

}  // namespace
}  // namespace drone_core
