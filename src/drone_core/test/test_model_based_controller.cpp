#include <cmath>

#include <gtest/gtest.h>

#include "drone_core/model_based_controller.hpp"
#include "drone_core/quadrotor_model.hpp"

namespace drone_core
{
namespace
{

TEST(MotorMixer, RoundTripPreservesUnsaturatedWrench)
{
  const QuadrotorParameters model_parameters;
  const MotorMixer mixer(model_parameters);
  const BodyWrench desired{9.81, Eigen::Vector3d{0.03, -0.02, 0.01}};

  const MotorCommand motors = mixer.mix(desired);
  const BodyWrench reconstructed = mixer.wrench(motors);

  EXPECT_NEAR(reconstructed.thrust, desired.thrust, 1e-10);
  EXPECT_TRUE(reconstructed.torque.isApprox(desired.torque, 1e-10));
}

TEST(ModelBasedController, ExactHoverProducesHoverMotorSpeed)
{
  const QuadrotorParameters model_parameters;
  const ModelBasedController controller(model_parameters);
  State state;
  state.position_world = Eigen::Vector3d{0.0, 0.0, 1.5};
  Reference reference;
  reference.position_world = state.position_world;

  const ControlOutput output = controller.compute(state, reference);
  const double expected_hover_speed = std::sqrt(
    model_parameters.mass * model_parameters.gravity /
    (4.0 * model_parameters.thrust_coefficient));

  EXPECT_NEAR(output.desired_wrench.thrust, model_parameters.mass * model_parameters.gravity, 1e-12);
  EXPECT_TRUE(output.desired_wrench.torque.isZero(1e-12));
  for (const double motor_speed : output.motor_angular_velocity) {
    EXPECT_NEAR(motor_speed, expected_hover_speed, 1e-10);
  }
}

TEST(ModelBasedController, HorizontalErrorCommandsTiltTowardGoal)
{
  const QuadrotorParameters model_parameters;
  const ModelBasedController controller(model_parameters);
  State state;
  state.position_world.z() = 1.5;
  Reference reference;
  reference.position_world = Eigen::Vector3d{1.0, 0.0, 1.5};

  const ControlOutput output = controller.compute(state, reference);

  EXPECT_GT(output.desired_force_world.x(), 0.0);
  EXPECT_GT(output.desired_orientation_body_to_world(0, 2), 0.0);
  EXPECT_LE(
    std::acos(output.desired_orientation_body_to_world(2, 2)),
    35.0 * kPi / 180.0 + 1e-12);
}

TEST(ModelBasedController, ClosedLoopTakeoffConvergesToHover)
{
  const QuadrotorParameters model_parameters;
  QuadrotorModel dynamics(model_parameters);
  const ModelBasedController controller(model_parameters);
  Reference reference;
  reference.position_world = Eigen::Vector3d{0.0, 0.0, 1.5};

  ControlOutput control;
  constexpr double dynamics_dt = 0.005;
  for (int step = 0; step < 2400; ++step) {
    if (step % 2 == 0) {
      control = controller.compute(dynamics.state(), reference);
    }
    dynamics.step(control.motor_angular_velocity, dynamics_dt);
  }

  EXPECT_NEAR(dynamics.state().position_world.x(), 0.0, 1e-6);
  EXPECT_NEAR(dynamics.state().position_world.y(), 0.0, 1e-6);
  EXPECT_NEAR(dynamics.state().position_world.z(), 1.5, 0.03);
  EXPECT_NEAR(dynamics.state().velocity_world.norm(), 0.0, 0.03);
  EXPECT_NEAR(dynamics.state().orientation_body_to_world.angularDistance(
    Eigen::Quaterniond::Identity()), 0.0, 1e-6);
}

TEST(ModelBasedController, ClosedLoopPointTargetConvergesWithMotorLag)
{
  const QuadrotorParameters model_parameters;
  QuadrotorModel dynamics(model_parameters);
  const ModelBasedController controller(model_parameters);
  Reference reference;
  reference.position_world = Eigen::Vector3d{2.0, 1.0, 1.5};

  ControlOutput control;
  constexpr double dynamics_dt = 0.005;
  for (int step = 0; step < 4000; ++step) {
    if (step % 2 == 0) {
      control = controller.compute(dynamics.state(), reference);
    }
    dynamics.step(control.motor_angular_velocity, dynamics_dt);
  }

  EXPECT_LT((dynamics.state().position_world - reference.position_world).norm(), 0.05);
  EXPECT_LT(dynamics.state().velocity_world.norm(), 0.05);
  EXPECT_LT(
    dynamics.state().orientation_body_to_world.angularDistance(Eigen::Quaterniond::Identity()),
    0.02);
}

}  // namespace
}  // namespace drone_core
