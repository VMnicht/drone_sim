#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include "drone_core/trajectory_generator.hpp"
#include "drone_msgs/msg/trajectory_point.hpp"

namespace drone_trajectory
{
namespace
{

Eigen::Vector3d vectorParameter(
  rclcpp::Node & node, const std::string & name, const std::vector<double> & defaults)
{
  const auto values = node.declare_parameter<std::vector<double>>(name, defaults);
  if (values.size() != 3U ||
    !std::all_of(values.begin(), values.end(), [](double value) {return std::isfinite(value);}))
  {
    throw std::invalid_argument("Parameter '" + name + "' must contain three finite values");
  }
  return Eigen::Vector3d{values[0], values[1], values[2]};
}

drone_core::AnalyticTrajectoryType trajectoryType(const std::string & value)
{
  if (value == "hold") {
    return drone_core::AnalyticTrajectoryType::kHold;
  }
  if (value == "circle") {
    return drone_core::AnalyticTrajectoryType::kCircle;
  }
  if (value == "figure_eight") {
    return drone_core::AnalyticTrajectoryType::kFigureEight;
  }
  throw std::invalid_argument("trajectory_type must be hold, circle, or figure_eight");
}

geometry_msgs::msg::Quaternion yawQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion result;
  result.w = std::cos(0.5 * yaw);
  result.z = std::sin(0.5 * yaw);
  return result;
}

}  // namespace

class AnalyticTrajectoryNode : public rclcpp::Node
{
public:
  AnalyticTrajectoryNode()
  : Node("analytic_trajectory_node")
  {
    drone_core::AnalyticTrajectoryParameters parameters;
    trajectory_type_name_ = declare_parameter<std::string>("trajectory_type", "circle");
    parameters.type = trajectoryType(trajectory_type_name_);
    parameters.center = vectorParameter(*this, "trajectory_center", {0.0, 0.0, 1.5});
    parameters.radius_x = declare_parameter<double>("trajectory_radius_x", 1.0);
    parameters.radius_y = declare_parameter<double>("trajectory_radius_y", 1.0);
    parameters.period = declare_parameter<double>("trajectory_period", 8.0);
    parameters.phase = declare_parameter<double>("trajectory_phase", 0.0);
    parameters.fixed_yaw = declare_parameter<double>("trajectory_fixed_yaw", 0.0);
    parameters.face_velocity = declare_parameter<bool>("trajectory_face_velocity", false);
    parameters.minimum_yaw_speed =
      declare_parameter<double>("trajectory_minimum_yaw_speed", 1.0e-6);

    update_frequency_ = declare_parameter<double>("trajectory_update_frequency", 50.0);
    start_delay_ = declare_parameter<double>("trajectory_start_delay", 1.0);
    preview_points_ = declare_parameter<int>("trajectory_preview_points", 200);
    preview_horizon_ = declare_parameter<double>("trajectory_preview_horizon", 8.0);
    world_frame_ = declare_parameter<std::string>("world_frame", "map");
    reference_topic_ = declare_parameter<std::string>(
      "trajectory_reference_topic", "/drone/trajectory_reference");
    path_topic_ =
      declare_parameter<std::string>("trajectory_path_topic", "/drone/trajectory_path");
    reference_qos_depth_ = declare_parameter<int>("trajectory_qos_depth", 10);
    path_qos_depth_ = declare_parameter<int>("trajectory_path_qos_depth", 1);

    if (!std::isfinite(update_frequency_) || update_frequency_ <= 0.0 ||
      !std::isfinite(start_delay_) || start_delay_ < 0.0 ||
      preview_points_ < 2 || !std::isfinite(preview_horizon_) || preview_horizon_ <= 0.0 ||
      world_frame_.empty() || reference_topic_.empty() || path_topic_.empty() ||
      reference_qos_depth_ <= 0 || path_qos_depth_ <= 0)
    {
      throw std::invalid_argument("Invalid trajectory adapter parameters");
    }

    generator_ = std::make_unique<drone_core::AnalyticTrajectoryGenerator>(parameters);
    reference_publisher_ = create_publisher<drone_msgs::msg::TrajectoryPoint>(
      reference_topic_, reference_qos_depth_);
    path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      path_topic_, rclcpp::QoS(path_qos_depth_).transient_local().reliable());
    start_wall_time_ = std::chrono::steady_clock::now();
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / update_frequency_));
    timer_ = create_wall_timer(period, std::bind(&AnalyticTrajectoryNode::update, this));
    publishPreview();
    RCLCPP_INFO(
      get_logger(), "Analytic %s trajectory ready at %.1f Hz",
      trajectory_type_name_.c_str(), update_frequency_);
  }

private:
  void update()
  {
    const rclcpp::Time stamp = now();
    const double elapsed = std::max(
      0.0,
      std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_wall_time_).count() - start_delay_);
    const drone_core::Reference reference = generator_->sample(elapsed);
    drone_msgs::msg::TrajectoryPoint message;
    message.header.stamp = stamp;
    message.header.frame_id = world_frame_;
    message.position.x = reference.position_world.x();
    message.position.y = reference.position_world.y();
    message.position.z = reference.position_world.z();
    message.velocity.x = reference.velocity_world.x();
    message.velocity.y = reference.velocity_world.y();
    message.velocity.z = reference.velocity_world.z();
    message.acceleration.x = reference.acceleration_world.x();
    message.acceleration.y = reference.acceleration_world.y();
    message.acceleration.z = reference.acceleration_world.z();
    message.yaw = reference.yaw;
    message.yaw_rate = reference.yaw_rate;
    reference_publisher_->publish(message);
  }

  void publishPreview()
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = world_frame_;
    path.poses.reserve(static_cast<std::size_t>(preview_points_));
    for (int index = 0; index < preview_points_; ++index) {
      const double time = preview_horizon_ * static_cast<double>(index) /
        static_cast<double>(preview_points_ - 1);
      const drone_core::Reference reference = generator_->sample(time);
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = reference.position_world.x();
      pose.pose.position.y = reference.position_world.y();
      pose.pose.position.z = reference.position_world.z();
      pose.pose.orientation = yawQuaternion(reference.yaw);
      path.poses.push_back(pose);
    }
    path_publisher_->publish(path);
  }

  std::unique_ptr<drone_core::AnalyticTrajectoryGenerator> generator_;
  std::string trajectory_type_name_{"circle"};
  double update_frequency_{50.0};
  double start_delay_{1.0};
  int preview_points_{200};
  double preview_horizon_{8.0};
  std::string world_frame_{"map"};
  std::string reference_topic_{"/drone/trajectory_reference"};
  std::string path_topic_{"/drone/trajectory_path"};
  int reference_qos_depth_{10};
  int path_qos_depth_{1};
  std::chrono::steady_clock::time_point start_wall_time_;
  rclcpp::Publisher<drone_msgs::msg::TrajectoryPoint>::SharedPtr reference_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace drone_trajectory

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<drone_trajectory::AnalyticTrajectoryNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("analytic_trajectory_node"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
