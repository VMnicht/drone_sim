#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "drone_core/disturbance_model.hpp"
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

drone_core::DisturbanceMode disturbanceMode(const std::string & value)
{
  if (value == "constant") {
    return drone_core::DisturbanceMode::kConstant;
  }
  if (value == "sinusoidal") {
    return drone_core::DisturbanceMode::kSinusoidal;
  }
  if (value == "gust") {
    return drone_core::DisturbanceMode::kGust;
  }
  if (value == "random") {
    return drone_core::DisturbanceMode::kRandom;
  }
  throw std::invalid_argument(
          "disturbance_mode must be constant, sinusoidal, gust, or random");
}

bool consumePeriod(double & accumulator, const double period)
{
  if (accumulator + 1.0e-12 < period) {
    return false;
  }
  accumulator -= period;
  // Subtraction is numerically stable around an exact period. fmod at that
  // boundary can return a value infinitesimally below `period`, which makes
  // every later fixed step look due and silently doubles publication load.
  if (accumulator < 1.0e-12) {
    accumulator = 0.0;
  } else if (accumulator >= period) {
    // A capped catch-up burst still emits at most one sample per ROS callback.
    accumulator = std::fmod(accumulator, period);
  }
  return true;
}

}  // namespace

class QuadrotorDynamicsNode : public rclcpp::Node
{
public:
  QuadrotorDynamicsNode()
  : Node("quadrotor_dynamics_node")
  {
    const auto parameters = loadModelParameters();
    drone_core::DisturbanceModelParameters disturbance_parameters;
    disturbance_parameters.mode = disturbanceMode(
      declare_parameter<std::string>("disturbance_mode", "constant"));
    disturbance_parameters.constant.force_world =
      vectorParameter(*this, "disturbance_force_world", {0.0, 0.0, 0.0});
    disturbance_parameters.constant.torque_body =
      vectorParameter(*this, "disturbance_torque_body", {0.0, 0.0, 0.0});
    disturbance_parameters.amplitude.force_world =
      vectorParameter(*this, "disturbance_amplitude_force_world", {0.0, 0.0, 0.0});
    disturbance_parameters.amplitude.torque_body =
      vectorParameter(*this, "disturbance_amplitude_torque_body", {0.0, 0.0, 0.0});
    disturbance_parameters.start_time =
      declare_parameter<double>("disturbance_start_time", 0.0);
    disturbance_parameters.duration =
      declare_parameter<double>("disturbance_duration", 0.0);
    disturbance_parameters.frequency =
      declare_parameter<double>("disturbance_frequency", 0.5);
    disturbance_parameters.random_correlation_time =
      declare_parameter<double>("disturbance_random_correlation_time", 0.5);
    const int disturbance_random_seed =
      declare_parameter<int>("disturbance_random_seed", 1);
    if (disturbance_random_seed < 0) {
      throw std::invalid_argument("disturbance_random_seed must be non-negative");
    }
    disturbance_parameters.random_seed = static_cast<std::uint32_t>(disturbance_random_seed);
    disturbance_enabled_ = declare_parameter<bool>("disturbance_enabled", false);
    simulation_frequency_ = declare_parameter<double>("simulation_frequency", 200.0);
    state_publish_frequency_ = declare_parameter<double>("state_publish_frequency", 100.0);
    path_sample_frequency_ = declare_parameter<double>("path_sample_frequency", 10.0);
    command_timeout_ = declare_parameter<double>("command_timeout", 0.5);
    command_timeout_hover_enabled_ =
      declare_parameter<bool>("command_timeout_hover_enabled", true);
    path_publish_frequency_ = declare_parameter<double>("path_publish_frequency", 5.0);
    maximum_path_points_ = declare_parameter<int>("maximum_path_points", 1200);
    timing_diagnostics_frequency_ =
      declare_parameter<double>("timing_diagnostics_frequency", 0.2);
    world_frame_ = declare_parameter<std::string>("world_frame", "map");
    body_frame_ = declare_parameter<std::string>("body_frame", "base_link");
    odometry_topic_ =
      declare_parameter<std::string>("truth_odometry_topic", "/drone/truth/odom");
    imu_topic_ = declare_parameter<std::string>("truth_imu_topic", "/drone/truth/imu");
    path_topic_ = declare_parameter<std::string>("path_topic", "/drone/path");
    motor_state_topic_ =
      declare_parameter<std::string>("motor_state_topic", "/drone/motor_rpm");
    motor_command_topic_ =
      declare_parameter<std::string>("actuator_command_topic", "/drone/motor_rpm_faulted");
    reset_service_name_ = declare_parameter<std::string>("reset_service", "/drone/reset");
    disturbance_enable_service_name_ = declare_parameter<std::string>(
      "disturbance_enable_service", "/drone/disturbance/enable");
    disturbance_topic_ =
      declare_parameter<std::string>("disturbance_topic", "/drone/disturbance");
    state_qos_depth_ = declare_parameter<int>("state_qos_depth", 20);
    path_qos_depth_ = declare_parameter<int>("path_qos_depth", 10);
    command_qos_depth_ = declare_parameter<int>("command_qos_depth", 20);
    diagnostics_log_throttle_ms_ =
      declare_parameter<int>("diagnostics_log_throttle_ms", 1000);

    if (!std::isfinite(simulation_frequency_) || simulation_frequency_ <= 0.0 ||
      !std::isfinite(state_publish_frequency_) || state_publish_frequency_ <= 0.0 ||
      state_publish_frequency_ > simulation_frequency_ ||
      !std::isfinite(path_sample_frequency_) || path_sample_frequency_ <= 0.0 ||
      path_sample_frequency_ > state_publish_frequency_ ||
      !std::isfinite(command_timeout_) || command_timeout_ < 0.0 ||
      !std::isfinite(path_publish_frequency_) || path_publish_frequency_ <= 0.0 ||
      path_publish_frequency_ > path_sample_frequency_ ||
      !std::isfinite(timing_diagnostics_frequency_) || timing_diagnostics_frequency_ <= 0.0 ||
      maximum_path_points_ <= 0 || state_qos_depth_ <= 0 || path_qos_depth_ <= 0 ||
      command_qos_depth_ <= 0 || diagnostics_log_throttle_ms_ <= 0 ||
      world_frame_.empty() || body_frame_.empty() || odometry_topic_.empty() ||
      imu_topic_.empty() || path_topic_.empty() || motor_state_topic_.empty() ||
      motor_command_topic_.empty() || reset_service_name_.empty() || disturbance_topic_.empty() ||
      disturbance_enable_service_name_.empty())
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
    disturbance_model_ =
      std::make_unique<drone_core::DisturbanceModel>(disturbance_parameters);
    model_->reset(initial_state_);
    simulation_dt_ = 1.0 / simulation_frequency_;
    state_publish_period_ = 1.0 / state_publish_frequency_;
    path_sample_period_ = 1.0 / path_sample_frequency_;
    path_publish_period_ = 1.0 / path_publish_frequency_;
    timing_diagnostics_period_ = 1.0 / timing_diagnostics_frequency_;

    odometry_publisher_ =
      create_publisher<nav_msgs::msg::Odometry>(odometry_topic_, state_qos_depth_);
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, state_qos_depth_);
    path_publisher_ = create_publisher<nav_msgs::msg::Path>(path_topic_, path_qos_depth_);
    motor_publisher_ =
      create_publisher<drone_msgs::msg::MotorRPM>(motor_state_topic_, state_qos_depth_);
    disturbance_publisher_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
      disturbance_topic_, state_qos_depth_);
    motor_subscription_ = create_subscription<drone_msgs::msg::MotorRPM>(
      motor_command_topic_, command_qos_depth_,
      std::bind(&QuadrotorDynamicsNode::motorCommandCallback, this, std::placeholders::_1));
    reset_service_ = create_service<std_srvs::srv::Empty>(
      reset_service_name_,
      std::bind(
        &QuadrotorDynamicsNode::resetCallback, this,
        std::placeholders::_1, std::placeholders::_2));
    disturbance_enable_service_ = create_service<std_srvs::srv::SetBool>(
      disturbance_enable_service_name_,
      std::bind(
        &QuadrotorDynamicsNode::disturbanceEnableCallback, this,
        std::placeholders::_1, std::placeholders::_2));
    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    path_.header.frame_id = world_frame_;
    last_command_wall_time_ = std::chrono::steady_clock::now();
    diagnostics_window_start_ = std::chrono::steady_clock::now();
    const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(simulation_dt_));
    timer_ = create_wall_timer(timer_period, std::bind(&QuadrotorDynamicsNode::update, this));

    RCLCPP_INFO(
      get_logger(),
      "Dynamics ready: fixed-step %.1f Hz, state %.1f Hz, path sample/publish %.1f/%.1f Hz; "
      "hover speed %.2f rad/s (%.1f RPM)",
      simulation_frequency_, state_publish_frequency_, path_sample_frequency_,
      path_publish_frequency_, model_->hoverMotorSpeed(),
      model_->hoverMotorSpeed() * drone_core::kRadPerSecondToRpm);
    if (!disturbance_parameters.constant.force_world.isZero() ||
      !disturbance_parameters.constant.torque_body.isZero() ||
      !disturbance_parameters.amplitude.force_world.isZero() ||
      !disturbance_parameters.amplitude.torque_body.isZero())
    {
      RCLCPP_INFO(
        get_logger(), "A non-zero external disturbance is configured (%s)",
        disturbance_enabled_ ? "enabled" : "disabled");
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
          get_logger(), *get_clock(), diagnostics_log_throttle_ms_,
          "Rejected non-finite motor RPM command");
        return;
      }
      converted[i] = message->rpm[i] * drone_core::kRpmToRadPerSecond;
    }
    motor_command_ = converted;
    has_received_command_ = true;
    last_command_wall_time_ = std::chrono::steady_clock::now();
  }

  void resetCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    model_->reset(initial_state_);
    disturbance_model_->reset();
    motor_command_.fill(0.0);
    has_received_command_ = false;
    command_failsafe_active_ = false;
    path_.poses.clear();
    path_history_.clear();
    elapsed_simulation_time_ = 0.0;
    state_publish_accumulator_ = 0.0;
    path_sample_accumulator_ = 0.0;
    path_publish_accumulator_ = 0.0;
    diagnostics_window_start_ = std::chrono::steady_clock::now();
    integration_steps_in_window_ = 0U;
    state_publications_in_window_ = 0U;
    path_publications_in_window_ = 0U;
    last_command_wall_time_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "Dynamics state reset");
  }

  void disturbanceEnableCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    disturbance_enabled_ = request->data;
    disturbance_model_->reset();
    response->success = true;
    response->message = disturbance_enabled_ ? "disturbance enabled" : "disturbance disabled";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  void update()
  {
    const auto wall_now = std::chrono::steady_clock::now();
    drone_core::MotorCommand effective_command = motor_command_;
    if (has_received_command_ && command_timeout_ > 0.0 &&
      std::chrono::duration<double>(wall_now - last_command_wall_time_).count() > command_timeout_)
    {
      if (!command_failsafe_active_) {
        RCLCPP_WARN(get_logger(), "Motor command timed out; applying actuator failsafe");
        command_failsafe_active_ = true;
      }
      effective_command.fill(
        command_timeout_hover_enabled_ ? model_->hoverMotorSpeed() : 0.0);
    } else if (command_failsafe_active_) {
      RCLCPP_INFO(get_logger(), "Motor command recovered; leaving actuator failsafe");
      command_failsafe_active_ = false;
    }

    try {
      const drone_core::Disturbance disturbance = disturbance_enabled_ ?
        disturbance_model_->sample(elapsed_simulation_time_, simulation_dt_) :
        drone_core::Disturbance{};
      const auto result = model_->step(effective_command, simulation_dt_, disturbance);
      elapsed_simulation_time_ += simulation_dt_;
      ++integration_steps_in_window_;
      state_publish_accumulator_ += simulation_dt_;
      path_sample_accumulator_ += simulation_dt_;
      path_publish_accumulator_ += simulation_dt_;
      if (consumePeriod(state_publish_accumulator_, state_publish_period_)) {
        publishState(result, now());
        ++state_publications_in_window_;
      }
      publishTimingDiagnostics(wall_now);
    } catch (const std::exception & exception) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), diagnostics_log_throttle_ms_,
        "Dynamics update failed: %s", exception.what());
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

    geometry_msgs::msg::WrenchStamped disturbance;
    disturbance.header.stamp = stamp;
    disturbance.header.frame_id = world_frame_;
    disturbance.wrench.force.x = result.applied_disturbance.force_world.x();
    disturbance.wrench.force.y = result.applied_disturbance.force_world.y();
    disturbance.wrench.force.z = result.applied_disturbance.force_world.z();
    disturbance.wrench.torque.x = result.applied_disturbance.torque_body.x();
    disturbance.wrench.torque.y = result.applied_disturbance.torque_body.y();
    disturbance.wrench.torque.z = result.applied_disturbance.torque_body.z();
    disturbance_publisher_->publish(disturbance);

    geometry_msgs::msg::TransformStamped transform;
    transform.header = odometry.header;
    transform.child_frame_id = body_frame_;
    transform.transform.translation.x = state.position_world.x();
    transform.transform.translation.y = state.position_world.y();
    transform.transform.translation.z = state.position_world.z();
    transform.transform.rotation = odometry.pose.pose.orientation;
    transform_broadcaster_->sendTransform(transform);

    if (consumePeriod(path_sample_accumulator_, path_sample_period_)) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = odometry.header;
      pose.pose = odometry.pose.pose;
      path_history_.push_back(pose);
      while (path_history_.size() > static_cast<std::size_t>(maximum_path_points_)) {
        path_history_.pop_front();
      }
    }
    if (consumePeriod(path_publish_accumulator_, path_publish_period_)) {
      path_.header.stamp = stamp;
      path_.poses.assign(path_history_.begin(), path_history_.end());
      path_publisher_->publish(path_);
      ++path_publications_in_window_;
    }
  }

  void publishTimingDiagnostics(const std::chrono::steady_clock::time_point & wall_now)
  {
    const double window =
      std::chrono::duration<double>(wall_now - diagnostics_window_start_).count();
    if (window + 1.0e-12 < timing_diagnostics_period_) {
      return;
    }
    RCLCPP_INFO(
      get_logger(),
      "Timing %.2f s: integration %.1f Hz, state/TF %.1f Hz, path %.1f Hz",
      window, static_cast<double>(integration_steps_in_window_) / window,
      static_cast<double>(state_publications_in_window_) / window,
      static_cast<double>(path_publications_in_window_) / window);
    diagnostics_window_start_ = wall_now;
    integration_steps_in_window_ = 0U;
    state_publications_in_window_ = 0U;
    path_publications_in_window_ = 0U;
  }

  std::unique_ptr<drone_core::QuadrotorModel> model_;
  std::unique_ptr<drone_core::DisturbanceModel> disturbance_model_;
  drone_core::State initial_state_;
  drone_core::MotorCommand motor_command_{0.0, 0.0, 0.0, 0.0};
  double simulation_frequency_{200.0};
  double simulation_dt_{0.005};
  double state_publish_frequency_{100.0};
  double state_publish_period_{0.01};
  double path_sample_frequency_{10.0};
  double path_sample_period_{0.1};
  double path_publish_period_{0.2};
  double elapsed_simulation_time_{0.0};
  double command_timeout_{0.5};
  bool command_timeout_hover_enabled_{true};
  bool command_failsafe_active_{false};
  bool has_received_command_{false};
  double path_publish_frequency_{5.0};
  int maximum_path_points_{1200};
  double timing_diagnostics_frequency_{0.2};
  double timing_diagnostics_period_{5.0};
  double state_publish_accumulator_{0.0};
  double path_sample_accumulator_{0.0};
  double path_publish_accumulator_{0.0};
  std::chrono::steady_clock::time_point diagnostics_window_start_;
  std::size_t integration_steps_in_window_{0U};
  std::size_t state_publications_in_window_{0U};
  std::size_t path_publications_in_window_{0U};
  std::string world_frame_{"map"};
  std::string body_frame_{"base_link"};
  std::string odometry_topic_{"/drone/odom"};
  std::string imu_topic_{"/drone/imu"};
  std::string path_topic_{"/drone/path"};
  std::string motor_state_topic_{"/drone/motor_rpm"};
  std::string motor_command_topic_{"/drone/motor_rpm_cmd"};
  std::string reset_service_name_{"/drone/reset"};
  std::string disturbance_enable_service_name_{"/drone/disturbance/enable"};
  std::string disturbance_topic_{"/drone/disturbance"};
  bool disturbance_enabled_{false};
  int state_qos_depth_{20};
  int path_qos_depth_{10};
  int command_qos_depth_{20};
  int diagnostics_log_throttle_ms_{1000};
  std::chrono::steady_clock::time_point last_command_wall_time_;
  nav_msgs::msg::Path path_;
  std::deque<geometry_msgs::msg::PoseStamped> path_history_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<drone_msgs::msg::MotorRPM>::SharedPtr motor_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr disturbance_publisher_;
  rclcpp::Subscription<drone_msgs::msg::MotorRPM>::SharedPtr motor_subscription_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr disturbance_enable_service_;
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
