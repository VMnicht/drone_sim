#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "drone_core/quadrotor_model.hpp"
#include "drone_msgs/msg/motor_rpm.hpp"

namespace drone_dynamics
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

geometry_msgs::msg::Quaternion toQuaternionMessage(const Eigen::Quaterniond & quaternion)
{
  geometry_msgs::msg::Quaternion message;
  message.w = quaternion.w();
  message.x = quaternion.x();
  message.y = quaternion.y();
  message.z = quaternion.z();
  return message;
}

}  // namespace

class QuadrotorDynamicsNode : public rclcpp::Node
{
public:
  QuadrotorDynamicsNode()
  : Node("quadrotor_dynamics_node")
  {
    const auto parameters = loadModelParameters();
    disturbance_.force_world =
      vectorParameter(*this, "disturbance_force_world", {0.0, 0.0, 0.0});
    disturbance_.torque_body =
      vectorParameter(*this, "disturbance_torque_body", {0.0, 0.0, 0.0});
    simulation_frequency_ = declare_parameter<double>("simulation_frequency", 200.0);
    command_timeout_ = declare_parameter<double>("command_timeout", 0.5);
    path_publish_frequency_ = declare_parameter<double>("path_publish_frequency", 20.0);
    maximum_path_points_ = declare_parameter<int>("maximum_path_points", 4000);
    world_frame_ = declare_parameter<std::string>("world_frame", "map");
    body_frame_ = declare_parameter<std::string>("body_frame", "base_link");

    if (!std::isfinite(simulation_frequency_) || simulation_frequency_ <= 0.0 ||
      !std::isfinite(command_timeout_) || command_timeout_ < 0.0 ||
      !std::isfinite(path_publish_frequency_) || path_publish_frequency_ <= 0.0 ||
      maximum_path_points_ <= 0)
    {
      throw std::invalid_argument("Invalid simulation, timeout, or path parameter");
    }

    initial_state_.position_world = vectorParameter(*this, "initial_position", {0.0, 0.0, 0.0});
    initial_state_.velocity_world = vectorParameter(*this, "initial_velocity", {0.0, 0.0, 0.0});
    initial_state_.angular_velocity_body =
      vectorParameter(*this, "initial_angular_velocity", {0.0, 0.0, 0.0});
    const Eigen::Vector3d initial_rpy = vectorParameter(*this, "initial_rpy", {0.0, 0.0, 0.0});
    initial_state_.orientation_body_to_world =
      Eigen::AngleAxisd(initial_rpy.z(), Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(initial_rpy.y(), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(initial_rpy.x(), Eigen::Vector3d::UnitX());

    model_ = std::make_unique<drone_core::QuadrotorModel>(parameters);
    model_->reset(initial_state_);
    simulation_dt_ = 1.0 / simulation_frequency_;
    path_stride_ = std::max(
      1, static_cast<int>(std::lround(simulation_frequency_ / path_publish_frequency_)));

    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/drone/odom", 20);
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>("/drone/imu", 20);
    path_publisher_ = create_publisher<nav_msgs::msg::Path>("/drone/path", 10);
    motor_publisher_ = create_publisher<drone_msgs::msg::MotorRPM>("/drone/motor_rpm", 20);
    motor_subscription_ = create_subscription<drone_msgs::msg::MotorRPM>(
      "/drone/motor_rpm_cmd", 20,
      std::bind(&QuadrotorDynamicsNode::motorCommandCallback, this, std::placeholders::_1));
    reset_service_ = create_service<std_srvs::srv::Empty>(
      "/drone/reset",
      std::bind(
        &QuadrotorDynamicsNode::resetCallback, this,
        std::placeholders::_1, std::placeholders::_2));
    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    path_.header.frame_id = world_frame_;
    last_command_time_ = now();
    const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(simulation_dt_));
    timer_ = create_wall_timer(timer_period, std::bind(&QuadrotorDynamicsNode::update, this));

    RCLCPP_INFO(
      get_logger(),
      "Dynamics ready at %.1f Hz; hover speed %.2f rad/s (%.1f RPM)",
      simulation_frequency_, model_->hoverMotorSpeed(),
      model_->hoverMotorSpeed() * drone_core::kRadPerSecondToRpm);
    if (!disturbance_.force_world.isZero() || !disturbance_.torque_body.isZero()) {
      RCLCPP_WARN(get_logger(), "A non-zero external disturbance is enabled");
    }
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
    parameters.motor_time_constant =
      declare_parameter<double>("motor_time_constant", parameters.motor_time_constant);
    parameters.minimum_motor_speed =
      declare_parameter<double>("minimum_motor_speed", parameters.minimum_motor_speed);
    parameters.maximum_motor_speed =
      declare_parameter<double>("maximum_motor_speed", parameters.maximum_motor_speed);
    parameters.linear_drag_coefficient =
      declare_parameter<double>("linear_drag_coefficient", parameters.linear_drag_coefficient);
    parameters.angular_damping = vectorParameter(
      *this, "angular_damping",
      {parameters.angular_damping.x(), parameters.angular_damping.y(), parameters.angular_damping.z()});
    parameters.ground_height =
      declare_parameter<double>("ground_height", parameters.ground_height);
    parameters.ground_restitution =
      declare_parameter<double>("ground_restitution", parameters.ground_restitution);
    return parameters;
  }

  void motorCommandCallback(const drone_msgs::msg::MotorRPM::SharedPtr message)
  {
    drone_core::MotorCommand converted{};
    for (std::size_t i = 0; i < drone_core::kMotorCount; ++i) {
      if (!std::isfinite(message->rpm[i])) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000, "Rejected non-finite motor RPM command");
        return;
      }
      converted[i] = message->rpm[i] * drone_core::kRpmToRadPerSecond;
    }
    motor_command_ = converted;
    last_command_time_ = now();
  }

  void resetCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    model_->reset(initial_state_);
    motor_command_.fill(0.0);
    path_.poses.clear();
    update_count_ = 0U;
    last_command_time_ = now();
    RCLCPP_INFO(get_logger(), "Dynamics state reset");
  }

  void update()
  {
    drone_core::MotorCommand effective_command = motor_command_;
    if (command_timeout_ > 0.0 && (now() - last_command_time_).seconds() > command_timeout_) {
      effective_command.fill(0.0);
    }

    try {
      const auto result = model_->step(effective_command, simulation_dt_, disturbance_);
      const rclcpp::Time stamp = now();
      publishState(result, stamp);
    } catch (const std::exception & exception) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "Dynamics update failed: %s", exception.what());
      motor_command_.fill(0.0);
    }
  }

  void publishState(const drone_core::DynamicsResult & result, const rclcpp::Time & stamp)
  {
    const auto & state = result.state;
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = world_frame_;
    odometry.child_frame_id = body_frame_;
    odometry.pose.pose.position.x = state.position_world.x();
    odometry.pose.pose.position.y = state.position_world.y();
    odometry.pose.pose.position.z = state.position_world.z();
    odometry.pose.pose.orientation = toQuaternionMessage(state.orientation_body_to_world);
    const Eigen::Vector3d velocity_body =
      state.orientation_body_to_world.conjugate() * state.velocity_world;
    odometry.twist.twist.linear.x = velocity_body.x();
    odometry.twist.twist.linear.y = velocity_body.y();
    odometry.twist.twist.linear.z = velocity_body.z();
    odometry.twist.twist.angular.x = state.angular_velocity_body.x();
    odometry.twist.twist.angular.y = state.angular_velocity_body.y();
    odometry.twist.twist.angular.z = state.angular_velocity_body.z();
    odometry_publisher_->publish(odometry);

    sensor_msgs::msg::Imu imu;
    imu.header = odometry.header;
    imu.header.frame_id = body_frame_;
    imu.orientation = odometry.pose.pose.orientation;
    imu.angular_velocity = odometry.twist.twist.angular;
    const Eigen::Vector3d gravity_world{0.0, 0.0, -model_->parameters().gravity};
    const Eigen::Vector3d specific_force_body = state.orientation_body_to_world.conjugate() *
      (result.linear_acceleration_world - gravity_world);
    imu.linear_acceleration.x = specific_force_body.x();
    imu.linear_acceleration.y = specific_force_body.y();
    imu.linear_acceleration.z = specific_force_body.z();
    imu_publisher_->publish(imu);

    drone_msgs::msg::MotorRPM motor_rpm;
    motor_rpm.header.stamp = stamp;
    motor_rpm.header.frame_id = body_frame_;
    for (std::size_t i = 0; i < drone_core::kMotorCount; ++i) {
      motor_rpm.rpm[i] =
        state.motor_angular_velocity[i] * drone_core::kRadPerSecondToRpm;
    }
    motor_publisher_->publish(motor_rpm);

    geometry_msgs::msg::TransformStamped transform;
    transform.header = odometry.header;
    transform.child_frame_id = body_frame_;
    transform.transform.translation.x = state.position_world.x();
    transform.transform.translation.y = state.position_world.y();
    transform.transform.translation.z = state.position_world.z();
    transform.transform.rotation = odometry.pose.pose.orientation;
    transform_broadcaster_->sendTransform(transform);

    ++update_count_;
    if (update_count_ % static_cast<std::size_t>(path_stride_) == 0U) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = odometry.header;
      pose.pose = odometry.pose.pose;
      path_.header.stamp = stamp;
      path_.poses.push_back(pose);
      if (path_.poses.size() > static_cast<std::size_t>(maximum_path_points_)) {
        path_.poses.erase(path_.poses.begin());
      }
      path_publisher_->publish(path_);
    }
  }

  std::unique_ptr<drone_core::QuadrotorModel> model_;
  drone_core::State initial_state_;
  drone_core::MotorCommand motor_command_{0.0, 0.0, 0.0, 0.0};
  drone_core::Disturbance disturbance_{};
  double simulation_frequency_{200.0};
  double simulation_dt_{0.005};
  double command_timeout_{0.5};
  double path_publish_frequency_{20.0};
  int path_stride_{10};
  int maximum_path_points_{4000};
  std::size_t update_count_{0U};
  std::string world_frame_{"map"};
  std::string body_frame_{"base_link"};
  rclcpp::Time last_command_time_;
  nav_msgs::msg::Path path_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<drone_msgs::msg::MotorRPM>::SharedPtr motor_publisher_;
  rclcpp::Subscription<drone_msgs::msg::MotorRPM>::SharedPtr motor_subscription_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace drone_dynamics

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<drone_dynamics::QuadrotorDynamicsNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("quadrotor_dynamics_node"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
