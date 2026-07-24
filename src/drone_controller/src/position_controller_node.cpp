#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "drone_core/model_based_controller.hpp"
#include "drone_msgs/msg/motor_rpm.hpp"
#include "drone_msgs/msg/trajectory_point.hpp"

namespace drone_controller
{
namespace
{

Eigen::Vector3d vectorParameter(
  rclcpp::Node & node, const std::string & name, const std::vector<double> & default_value)
{
  const auto values = node.declare_parameter<std::vector<double>>(name, default_value);
  if (values.size() != 3U ||
    !std::all_of(values.begin(), values.end(), [](double value) {return std::isfinite(value);}))
  {
    throw std::invalid_argument("Parameter '" + name + "' must contain three finite values");
  }
  return Eigen::Vector3d{values[0], values[1], values[2]};
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  return std::atan2(
    2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
    1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z));
}

geometry_msgs::msg::Quaternion yawQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion result;
  result.w = std::cos(0.5 * yaw);
  result.z = std::sin(0.5 * yaw);
  return result;
}

}  // namespace

class PositionControllerNode : public rclcpp::Node
{
public:
  PositionControllerNode()
  : Node("position_controller_node")
  {
    const auto model_parameters = loadModelParameters();
    const auto controller_parameters = loadControllerParameters();
    hover_motor_rpm_ = std::sqrt(
      model_parameters.mass * model_parameters.gravity /
      (4.0 * model_parameters.thrust_coefficient)) * drone_core::kRadPerSecondToRpm;
    controller_ = std::make_unique<drone_core::ModelBasedController>(
      model_parameters, controller_parameters);

    controller_frequency_ = declare_parameter<double>("controller_frequency", 100.0);
    odometry_timeout_ = declare_parameter<double>("odometry_timeout", 0.2);
    odometry_timeout_hover_enabled_ =
      declare_parameter<bool>("odometry_timeout_hover_enabled", true);
    auto_takeoff_ = declare_parameter<bool>("auto_takeoff", true);
    world_frame_ = declare_parameter<std::string>("world_frame", "map");
    body_frame_ = declare_parameter<std::string>("body_frame", "base_link");
    reference_.position_world = vectorParameter(*this, "takeoff_position", {0.0, 0.0, 1.5});
    reference_.yaw = declare_parameter<double>("takeoff_yaw", 0.0);
    motor_command_topic_ =
      declare_parameter<std::string>("motor_command_topic", "/drone/motor_rpm_cmd");
    reference_topic_ =
      declare_parameter<std::string>("reference_topic", "/drone/reference");
    odometry_topic_ = declare_parameter<std::string>("odometry_topic", "/drone/odom");
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/drone/goal");
    trajectory_reference_topic_ = declare_parameter<std::string>(
      "trajectory_reference_topic", "/drone/trajectory_reference");
    command_qos_depth_ = declare_parameter<int>("command_qos_depth", 20);
    reference_qos_depth_ = declare_parameter<int>("reference_qos_depth", 1);
    odometry_qos_depth_ = declare_parameter<int>("odometry_qos_depth", 20);
    goal_qos_depth_ = declare_parameter<int>("goal_qos_depth", 10);
    trajectory_qos_depth_ = declare_parameter<int>("trajectory_qos_depth", 10);
    minimum_quaternion_norm_ = declare_parameter<double>("minimum_quaternion_norm", 1e-9);
    goal_change_position_tolerance_ =
      declare_parameter<double>("goal_change_position_tolerance", 1e-6);
    goal_change_yaw_tolerance_ =
      declare_parameter<double>("goal_change_yaw_tolerance", 1e-6);
    diagnostics_log_throttle_ms_ =
      declare_parameter<int>("diagnostics_log_throttle_ms", 1000);

    if (!std::isfinite(controller_frequency_) || controller_frequency_ <= 0.0 ||
      !std::isfinite(odometry_timeout_) || odometry_timeout_ <= 0.0 ||
      !reference_.isFinite() || !std::isfinite(minimum_quaternion_norm_) ||
      minimum_quaternion_norm_ <= 0.0 ||
      !std::isfinite(goal_change_position_tolerance_) ||
      goal_change_position_tolerance_ < 0.0 ||
      !std::isfinite(goal_change_yaw_tolerance_) || goal_change_yaw_tolerance_ < 0.0 ||
      command_qos_depth_ <= 0 || reference_qos_depth_ <= 0 ||
      odometry_qos_depth_ <= 0 || goal_qos_depth_ <= 0 || trajectory_qos_depth_ <= 0 ||
      diagnostics_log_throttle_ms_ <= 0 || world_frame_.empty() || body_frame_.empty() ||
      motor_command_topic_.empty() || reference_topic_.empty() ||
      odometry_topic_.empty() || goal_topic_.empty() || trajectory_reference_topic_.empty())
    {
      throw std::invalid_argument("Invalid controller timing or takeoff reference");
    }
    has_reference_ = auto_takeoff_;

    command_publisher_ = create_publisher<drone_msgs::msg::MotorRPM>(
      motor_command_topic_, command_qos_depth_);
    reference_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      reference_topic_, rclcpp::QoS(reference_qos_depth_).transient_local().reliable());
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_, odometry_qos_depth_,
      std::bind(&PositionControllerNode::odometryCallback, this, std::placeholders::_1));
    goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, goal_qos_depth_,
      std::bind(&PositionControllerNode::goalCallback, this, std::placeholders::_1));
    trajectory_subscription_ = create_subscription<drone_msgs::msg::TrajectoryPoint>(
      trajectory_reference_topic_, trajectory_qos_depth_,
      std::bind(&PositionControllerNode::trajectoryCallback, this, std::placeholders::_1));

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / controller_frequency_));
    timer_ = create_wall_timer(period, std::bind(&PositionControllerNode::update, this));

    RCLCPP_INFO(
      get_logger(), "Model-based controller ready at %.1f Hz; auto takeoff %s to (%.2f, %.2f, %.2f)",
      controller_frequency_, auto_takeoff_ ? "enabled" : "disabled",
      reference_.position_world.x(), reference_.position_world.y(), reference_.position_world.z());
  }

private:
  drone_core::QuadrotorParameters loadModelParameters()
  {
    drone_core::QuadrotorParameters parameters;
    parameters.mass = declare_parameter<double>("mass", parameters.mass);
    parameters.gravity = declare_parameter<double>("gravity", parameters.gravity);
    parameters.inertia_diagonal = vectorParameter(
      *this, "inertia_diagonal",
      {parameters.inertia_diagonal.x(), parameters.inertia_diagonal.y(),
        parameters.inertia_diagonal.z()});
    parameters.arm_length = declare_parameter<double>("arm_length", parameters.arm_length);
    parameters.thrust_coefficient =
      declare_parameter<double>("thrust_coefficient", parameters.thrust_coefficient);
    parameters.drag_moment_coefficient =
      declare_parameter<double>("drag_moment_coefficient", parameters.drag_moment_coefficient);
    parameters.minimum_motor_speed =
      declare_parameter<double>("minimum_motor_speed", parameters.minimum_motor_speed);
    parameters.maximum_motor_speed =
      declare_parameter<double>("maximum_motor_speed", parameters.maximum_motor_speed);
    return parameters;
  }

  drone_core::ModelBasedControllerParameters loadControllerParameters()
  {
    drone_core::ModelBasedControllerParameters parameters;
    parameters.position_gain = vectorParameter(
      *this, "position_gain",
      {parameters.position_gain.x(), parameters.position_gain.y(), parameters.position_gain.z()});
    parameters.velocity_gain = vectorParameter(
      *this, "velocity_gain",
      {parameters.velocity_gain.x(), parameters.velocity_gain.y(), parameters.velocity_gain.z()});
    parameters.attitude_gain = vectorParameter(
      *this, "attitude_gain",
      {parameters.attitude_gain.x(), parameters.attitude_gain.y(), parameters.attitude_gain.z()});
    parameters.angular_rate_gain = vectorParameter(
      *this, "angular_rate_gain",
      {parameters.angular_rate_gain.x(), parameters.angular_rate_gain.y(),
        parameters.angular_rate_gain.z()});
    parameters.maximum_torque = vectorParameter(
      *this, "maximum_torque",
      {parameters.maximum_torque.x(), parameters.maximum_torque.y(), parameters.maximum_torque.z()});
    parameters.maximum_horizontal_acceleration = declare_parameter<double>(
      "maximum_horizontal_acceleration", parameters.maximum_horizontal_acceleration);
    parameters.maximum_vertical_acceleration = declare_parameter<double>(
      "maximum_vertical_acceleration", parameters.maximum_vertical_acceleration);
    const double maximum_tilt_degrees = declare_parameter<double>("maximum_tilt_degrees", 35.0);
    parameters.maximum_tilt = maximum_tilt_degrees * drone_core::kPi / 180.0;
    parameters.maximum_thrust_to_weight = declare_parameter<double>(
      "maximum_thrust_to_weight", parameters.maximum_thrust_to_weight);
    return parameters;
  }

  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    Eigen::Quaterniond orientation{
      message->pose.pose.orientation.w, message->pose.pose.orientation.x,
      message->pose.pose.orientation.y, message->pose.pose.orientation.z};
    if (!orientation.coeffs().array().isFinite().all() ||
      orientation.norm() < minimum_quaternion_norm_)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), diagnostics_log_throttle_ms_,
        "Rejected invalid odometry quaternion");
      return;
    }
    orientation.normalize();
    state_.position_world = Eigen::Vector3d{
      message->pose.pose.position.x, message->pose.pose.position.y,
      message->pose.pose.position.z};
    state_.orientation_body_to_world = orientation;
    const Eigen::Vector3d velocity_body{
      message->twist.twist.linear.x, message->twist.twist.linear.y,
      message->twist.twist.linear.z};
    state_.velocity_world = orientation * velocity_body;
    state_.angular_velocity_body = Eigen::Vector3d{
      message->twist.twist.angular.x, message->twist.twist.angular.y,
      message->twist.twist.angular.z};
    if (!state_.isFinite()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), diagnostics_log_throttle_ms_,
        "Rejected non-finite odometry");
      return;
    }
    has_odometry_ = true;
    last_odometry_wall_time_ = std::chrono::steady_clock::now();
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    drone_core::Reference candidate;
    candidate.position_world = Eigen::Vector3d{
      message->pose.position.x, message->pose.position.y, message->pose.position.z};
    candidate.yaw = yawFromQuaternion(message->pose.orientation);
    if (!candidate.isFinite()) {
      RCLCPP_ERROR(get_logger(), "Rejected non-finite goal");
      return;
    }
    const bool changed = !has_reference_ ||
      (candidate.position_world - reference_.position_world).norm() >
      goal_change_position_tolerance_ ||
      std::abs(candidate.yaw - reference_.yaw) > goal_change_yaw_tolerance_;
    reference_ = candidate;
    has_reference_ = true;
    if (changed) {
      RCLCPP_INFO(
        get_logger(), "New goal: (%.2f, %.2f, %.2f), yaw %.2f",
        reference_.position_world.x(), reference_.position_world.y(),
        reference_.position_world.z(), reference_.yaw);
    }
  }

  void trajectoryCallback(const drone_msgs::msg::TrajectoryPoint::SharedPtr message)
  {
    drone_core::Reference candidate;
    candidate.position_world = Eigen::Vector3d{
      message->position.x, message->position.y, message->position.z};
    candidate.velocity_world = Eigen::Vector3d{
      message->velocity.x, message->velocity.y, message->velocity.z};
    candidate.acceleration_world = Eigen::Vector3d{
      message->acceleration.x, message->acceleration.y, message->acceleration.z};
    candidate.yaw = message->yaw;
    candidate.yaw_rate = message->yaw_rate;
    if (!candidate.isFinite()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), diagnostics_log_throttle_ms_,
        "Rejected non-finite trajectory reference");
      return;
    }
    reference_ = candidate;
    has_reference_ = true;
  }

  void update()
  {
    const rclcpp::Time stamp = now();
    const auto wall_now = std::chrono::steady_clock::now();
    if (!has_odometry_ || !has_reference_) {
      publishZeroCommand(stamp);
      return;
    }
    const bool odometry_timed_out =
      std::chrono::duration<double>(wall_now - last_odometry_wall_time_).count() >
      odometry_timeout_;
    if (odometry_timed_out) {
      if (!odometry_failsafe_active_) {
        RCLCPP_WARN(get_logger(), "Odometry timed out; applying motor failsafe");
        odometry_failsafe_active_ = true;
      }
      if (odometry_timeout_hover_enabled_) {
        publishHoverCommand(stamp);
      } else {
        publishZeroCommand(stamp);
      }
      return;
    }
    if (odometry_failsafe_active_) {
      RCLCPP_INFO(get_logger(), "Odometry recovered; resuming closed-loop control");
      odometry_failsafe_active_ = false;
    }

    try {
      const auto output = controller_->compute(state_, reference_);
      drone_msgs::msg::MotorRPM command;
      command.header.stamp = stamp;
      command.header.frame_id = body_frame_;
      for (std::size_t i = 0; i < drone_core::kMotorCount; ++i) {
        command.rpm[i] =
          output.motor_angular_velocity[i] * drone_core::kRadPerSecondToRpm;
      }
      command_publisher_->publish(command);
      publishReference(stamp);
    } catch (const std::exception & exception) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), diagnostics_log_throttle_ms_,
        "Controller update failed: %s", exception.what());
      publishZeroCommand(stamp);
    }
  }

  void publishReference(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::PoseStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = world_frame_;
    message.pose.position.x = reference_.position_world.x();
    message.pose.position.y = reference_.position_world.y();
    message.pose.position.z = reference_.position_world.z();
    message.pose.orientation = yawQuaternion(reference_.yaw);
    reference_publisher_->publish(message);
  }

  void publishZeroCommand(const rclcpp::Time & stamp)
  {
    drone_msgs::msg::MotorRPM command;
    command.header.stamp = stamp;
    command.header.frame_id = body_frame_;
    command.rpm.fill(0.0);
    command_publisher_->publish(command);
  }

  void publishHoverCommand(const rclcpp::Time & stamp)
  {
    drone_msgs::msg::MotorRPM command;
    command.header.stamp = stamp;
    command.header.frame_id = body_frame_;
    command.rpm.fill(hover_motor_rpm_);
    command_publisher_->publish(command);
  }

  std::unique_ptr<drone_core::ModelBasedController> controller_;
  drone_core::State state_;
  drone_core::Reference reference_;
  bool has_odometry_{false};
  bool has_reference_{false};
  bool auto_takeoff_{true};
  bool odometry_timeout_hover_enabled_{true};
  bool odometry_failsafe_active_{false};
  double controller_frequency_{100.0};
  double odometry_timeout_{0.2};
  double hover_motor_rpm_{0.0};
  std::string world_frame_{"map"};
  std::string body_frame_{"base_link"};
  std::string motor_command_topic_{"/drone/motor_rpm_cmd"};
  std::string reference_topic_{"/drone/reference"};
  std::string odometry_topic_{"/drone/odom"};
  std::string goal_topic_{"/drone/goal"};
  std::string trajectory_reference_topic_{"/drone/trajectory_reference"};
  int command_qos_depth_{20};
  int reference_qos_depth_{1};
  int odometry_qos_depth_{20};
  int goal_qos_depth_{10};
  int trajectory_qos_depth_{10};
  double minimum_quaternion_norm_{1e-9};
  double goal_change_position_tolerance_{1e-6};
  double goal_change_yaw_tolerance_{1e-6};
  int diagnostics_log_throttle_ms_{1000};
  std::chrono::steady_clock::time_point last_odometry_wall_time_;

  rclcpp::Publisher<drone_msgs::msg::MotorRPM>::SharedPtr command_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr reference_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  rclcpp::Subscription<drone_msgs::msg::TrajectoryPoint>::SharedPtr trajectory_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace drone_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<drone_controller::PositionControllerNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("position_controller_node"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
