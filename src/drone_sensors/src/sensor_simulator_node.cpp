#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_srvs/srv/empty.hpp>

#include "drone_core/sensor_noise_model.hpp"

namespace drone_sensors
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

geometry_msgs::msg::Quaternion quaternionMessage(const Eigen::Quaterniond & value)
{
  geometry_msgs::msg::Quaternion result;
  result.w = value.w();
  result.x = value.x();
  result.y = value.y();
  result.z = value.z();
  return result;
}

void setCovarianceDiagonal(
  std::array<double, 36> & covariance,
  const Eigen::Vector3d & first_standard_deviation,
  const Eigen::Vector3d & second_standard_deviation)
{
  covariance.fill(0.0);
  covariance[0] = first_standard_deviation.x() * first_standard_deviation.x();
  covariance[7] = first_standard_deviation.y() * first_standard_deviation.y();
  covariance[14] = first_standard_deviation.z() * first_standard_deviation.z();
  covariance[21] = second_standard_deviation.x() * second_standard_deviation.x();
  covariance[28] = second_standard_deviation.y() * second_standard_deviation.y();
  covariance[35] = second_standard_deviation.z() * second_standard_deviation.z();
}

void setCovarianceDiagonal(
  std::array<double, 9> & covariance, const Eigen::Vector3d & standard_deviation)
{
  covariance.fill(0.0);
  covariance[0] = standard_deviation.x() * standard_deviation.x();
  covariance[4] = standard_deviation.y() * standard_deviation.y();
  covariance[8] = standard_deviation.z() * standard_deviation.z();
}

}  // namespace

class SensorSimulatorNode : public rclcpp::Node
{
public:
  SensorSimulatorNode()
  : Node("sensor_simulator_node")
  {
    noise_enabled_ = declare_parameter<bool>("sensor_noise_enabled", true);
    drone_core::SensorNoiseParameters parameters;
    parameters.position_standard_deviation =
      vectorParameter(*this, "position_noise_stddev", {0.01, 0.01, 0.015});
    parameters.velocity_standard_deviation =
      vectorParameter(*this, "velocity_noise_stddev", {0.02, 0.02, 0.03});
    parameters.orientation_standard_deviation =
      vectorParameter(*this, "orientation_noise_stddev", {0.002, 0.002, 0.004});
    parameters.accelerometer_standard_deviation =
      vectorParameter(*this, "accelerometer_noise_stddev", {0.03, 0.03, 0.04});
    parameters.gyroscope_standard_deviation =
      vectorParameter(*this, "gyroscope_noise_stddev", {0.002, 0.002, 0.003});
    parameters.accelerometer_initial_bias =
      vectorParameter(*this, "accelerometer_initial_bias", {0.0, 0.0, 0.0});
    parameters.gyroscope_initial_bias =
      vectorParameter(*this, "gyroscope_initial_bias", {0.0, 0.0, 0.0});
    parameters.accelerometer_bias_random_walk =
      vectorParameter(*this, "accelerometer_bias_random_walk", {0.001, 0.001, 0.001});
    parameters.gyroscope_bias_random_walk =
      vectorParameter(*this, "gyroscope_bias_random_walk", {0.0001, 0.0001, 0.0001});
    const int random_seed = declare_parameter<int>("sensor_random_seed", 11);
    if (random_seed < 0) {
      throw std::invalid_argument("sensor_random_seed must be non-negative");
    }
    parameters.random_seed = static_cast<std::uint32_t>(random_seed);
    configured_parameters_ = parameters;
    if (!noise_enabled_) {
      parameters = drone_core::SensorNoiseParameters{};
      parameters.random_seed = configured_parameters_.random_seed;
    }

    drone_core::SensorNoiseParameters gps_parameters;
    const double gps_horizontal_stddev =
      declare_parameter<double>("gps_horizontal_noise_stddev", 0.5);
    const double gps_vertical_stddev =
      declare_parameter<double>("gps_vertical_noise_stddev", 0.8);
    const int gps_random_seed = declare_parameter<int>("gps_random_seed", 29);
    const int gps_dropout_random_seed =
      declare_parameter<int>("gps_dropout_random_seed", 30);
    gps_parameters.position_standard_deviation = Eigen::Vector3d{
      gps_horizontal_stddev, gps_horizontal_stddev, gps_vertical_stddev};
    gps_parameters.random_seed = static_cast<std::uint32_t>(gps_random_seed);

    gravity_ = declare_parameter<double>("gravity", 9.81);
    publish_frequency_ = declare_parameter<double>("sensor_publish_frequency", 100.0);
    gps_enabled_ = declare_parameter<bool>("gps_enabled", true);
    gps_publish_frequency_ = declare_parameter<double>("gps_publish_frequency", 5.0);
    gps_dropout_probability_ = declare_parameter<double>("gps_dropout_probability", 0.02);
    gps_origin_latitude_ = declare_parameter<double>("gps_origin_latitude", 22.302711);
    gps_origin_longitude_ = declare_parameter<double>("gps_origin_longitude", 114.177216);
    gps_origin_altitude_ = declare_parameter<double>("gps_origin_altitude", 10.0);
    gps_earth_radius_ = declare_parameter<double>("gps_earth_radius", 6378137.0);
    gps_minimum_longitude_cosine_ =
      declare_parameter<double>("gps_minimum_longitude_cosine", 1.0e-6);
    minimum_sensor_dt_ = declare_parameter<double>("sensor_minimum_dt", 1.0e-6);
    sensor_output_delay_ = declare_parameter<double>("sensor_output_delay", 0.0);
    gps_output_delay_ = declare_parameter<double>("gps_output_delay", 0.0);
    minimum_quaternion_norm_ =
      declare_parameter<double>("sensor_minimum_quaternion_norm", 1.0e-9);
    world_frame_ = declare_parameter<std::string>("world_frame", "map");
    body_frame_ = declare_parameter<std::string>("body_frame", "base_link");
    truth_odometry_topic_ =
      declare_parameter<std::string>("truth_odometry_topic", "/drone/truth/odom");
    truth_imu_topic_ = declare_parameter<std::string>("truth_imu_topic", "/drone/truth/imu");
    odometry_topic_ = declare_parameter<std::string>("odometry_topic", "/drone/odom");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/drone/imu");
    gps_topic_ = declare_parameter<std::string>("gps_topic", "/drone/gps");
    reset_service_name_ =
      declare_parameter<std::string>("sensor_reset_service", "/drone/sensors/reset");
    truth_qos_depth_ = declare_parameter<int>("sensor_truth_qos_depth", 20);
    output_qos_depth_ = declare_parameter<int>("sensor_output_qos_depth", 20);

    if (!std::isfinite(gravity_) || gravity_ <= 0.0 ||
      !std::isfinite(publish_frequency_) || publish_frequency_ <= 0.0 ||
      !std::isfinite(gps_publish_frequency_) || gps_publish_frequency_ <= 0.0 ||
      !std::isfinite(gps_dropout_probability_) || gps_dropout_probability_ < 0.0 ||
      gps_dropout_probability_ >= 1.0 || !std::isfinite(gps_origin_latitude_) ||
      std::abs(gps_origin_latitude_) > 90.0 || !std::isfinite(gps_origin_longitude_) ||
      std::abs(gps_origin_longitude_) > 180.0 || !std::isfinite(gps_origin_altitude_) ||
      !std::isfinite(gps_horizontal_stddev) || gps_horizontal_stddev < 0.0 ||
      !std::isfinite(gps_vertical_stddev) || gps_vertical_stddev < 0.0 ||
      gps_random_seed < 0 || gps_dropout_random_seed < 0 ||
      !std::isfinite(gps_earth_radius_) || gps_earth_radius_ <= 0.0 ||
      !std::isfinite(gps_minimum_longitude_cosine_) ||
      gps_minimum_longitude_cosine_ <= 0.0 || gps_minimum_longitude_cosine_ > 1.0 ||
      !std::isfinite(minimum_sensor_dt_) || minimum_sensor_dt_ <= 0.0 ||
      !std::isfinite(sensor_output_delay_) || sensor_output_delay_ < 0.0 ||
      !std::isfinite(gps_output_delay_) || gps_output_delay_ < 0.0 ||
      !std::isfinite(minimum_quaternion_norm_) || minimum_quaternion_norm_ <= 0.0 ||
      world_frame_.empty() || body_frame_.empty() || truth_odometry_topic_.empty() ||
      truth_imu_topic_.empty() || odometry_topic_.empty() || imu_topic_.empty() || gps_topic_.empty() ||
      reset_service_name_.empty() || truth_qos_depth_ <= 0 || output_qos_depth_ <= 0)
    {
      throw std::invalid_argument("Invalid sensor adapter parameters");
    }

    noise_model_ = std::make_unique<drone_core::SensorNoiseModel>(parameters);
    gps_noise_model_ = std::make_unique<drone_core::SensorNoiseModel>(gps_parameters);
    gps_random_seed_ = static_cast<std::uint32_t>(gps_dropout_random_seed);
    gps_random_engine_.seed(gps_random_seed_);
    odometry_publisher_ =
      create_publisher<nav_msgs::msg::Odometry>(odometry_topic_, output_qos_depth_);
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, output_qos_depth_);
    gps_publisher_ = create_publisher<sensor_msgs::msg::NavSatFix>(gps_topic_, output_qos_depth_);
    truth_imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      truth_imu_topic_, truth_qos_depth_,
      std::bind(&SensorSimulatorNode::imuCallback, this, std::placeholders::_1));
    truth_odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      truth_odometry_topic_, truth_qos_depth_,
      std::bind(&SensorSimulatorNode::odometryCallback, this, std::placeholders::_1));
    reset_service_ = create_service<std_srvs::srv::Empty>(
      reset_service_name_,
      std::bind(
        &SensorSimulatorNode::resetCallback, this,
        std::placeholders::_1, std::placeholders::_2));
    minimum_publish_period_ = 1.0 / publish_frequency_;
    minimum_gps_publish_period_ = 1.0 / gps_publish_frequency_;
    RCLCPP_INFO(
      get_logger(), "Sensor simulator ready at %.1f Hz; noise %s",
      publish_frequency_, noise_enabled_ ? "enabled" : "disabled");
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr message)
  {
    latest_specific_force_body_ = Eigen::Vector3d{
      message->linear_acceleration.x,
      message->linear_acceleration.y,
      message->linear_acceleration.z};
    has_imu_ = latest_specific_force_body_.array().isFinite().all();
  }

  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    const auto wall_now = std::chrono::steady_clock::now();
    if (has_last_publish_time_ &&
      std::chrono::duration<double>(wall_now - last_publish_wall_time_).count() <
      minimum_publish_period_)
    {
      return;
    }
    Eigen::Quaterniond orientation{
      message->pose.pose.orientation.w, message->pose.pose.orientation.x,
      message->pose.pose.orientation.y, message->pose.pose.orientation.z};
    if (!orientation.coeffs().array().isFinite().all() ||
      orientation.norm() < minimum_quaternion_norm_)
    {
      RCLCPP_WARN(get_logger(), "Rejected truth odometry with invalid quaternion");
      return;
    }
    orientation.normalize();
    drone_core::State truth;
    truth.position_world = Eigen::Vector3d{
      message->pose.pose.position.x, message->pose.pose.position.y,
      message->pose.pose.position.z};
    truth.orientation_body_to_world = orientation;
    truth.velocity_world = orientation * Eigen::Vector3d{
      message->twist.twist.linear.x, message->twist.twist.linear.y,
      message->twist.twist.linear.z};
    truth.angular_velocity_body = Eigen::Vector3d{
      message->twist.twist.angular.x, message->twist.twist.angular.y,
      message->twist.twist.angular.z};
    if (!truth.isFinite()) {
      RCLCPP_WARN(get_logger(), "Rejected non-finite truth odometry");
      return;
    }

    double dt = minimum_publish_period_;
    if (has_last_publish_time_) {
      dt = std::max(
        minimum_sensor_dt_,
        std::chrono::duration<double>(wall_now - last_publish_wall_time_).count());
    }
    const Eigen::Vector3d specific_force = has_imu_ ?
      latest_specific_force_body_ : Eigen::Vector3d{0.0, 0.0, gravity_};
    const Eigen::Vector3d acceleration_world = orientation * specific_force +
      Eigen::Vector3d{0.0, 0.0, -gravity_};
    const drone_core::SensorSample sample = noise_model_->sample(
      truth, acceleration_world, gravity_, dt);
    publish(sample, message->header.stamp, wall_now);
    publishGpsIfDue(truth, acceleration_world, dt, message->header.stamp, wall_now);
    flushDelayedOutputs(wall_now);
    last_publish_wall_time_ = wall_now;
    has_last_publish_time_ = true;
  }

  void publishGpsIfDue(
    const drone_core::State & truth,
    const Eigen::Vector3d & acceleration_world,
    double dt,
    const builtin_interfaces::msg::Time & message_stamp,
    const std::chrono::steady_clock::time_point & wall_now)
  {
    if (!gps_enabled_ ||
      (has_last_gps_publish_time_ &&
      std::chrono::duration<double>(wall_now - last_gps_publish_wall_time_).count() <
      minimum_gps_publish_period_))
    {
      return;
    }
    last_gps_publish_wall_time_ = wall_now;
    has_last_gps_publish_time_ = true;
    if (uniform_(gps_random_engine_) < gps_dropout_probability_) {
      return;
    }
    const auto gps_sample = gps_noise_model_->sample(
      truth, acceleration_world, gravity_, std::max(dt, minimum_gps_publish_period_));
    constexpr double radians_to_degrees = 180.0 / drone_core::kPi;
    const double origin_latitude_radians = gps_origin_latitude_ / radians_to_degrees;
    sensor_msgs::msg::NavSatFix gps;
    gps.header.stamp = message_stamp;
    gps.header.frame_id = world_frame_;
    gps.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    gps.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    gps.latitude = gps_origin_latitude_ +
      gps_sample.position_world.y() / gps_earth_radius_ * radians_to_degrees;
    gps.longitude = gps_origin_longitude_ +
      gps_sample.position_world.x() /
      (gps_earth_radius_ * std::max(
        gps_minimum_longitude_cosine_, std::cos(origin_latitude_radians))) *
      radians_to_degrees;
    gps.altitude = gps_origin_altitude_ + gps_sample.position_world.z();
    gps.position_covariance.fill(0.0);
    const auto & deviations = gps_noise_model_->parameters().position_standard_deviation;
    gps.position_covariance[0] = deviations.x() * deviations.x();
    gps.position_covariance[4] = deviations.y() * deviations.y();
    gps.position_covariance[8] = deviations.z() * deviations.z();
    gps.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    if (gps_output_delay_ <= 0.0) {
      gps_publisher_->publish(gps);
    } else {
      delayed_gps_.push_back(
        {wall_now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(gps_output_delay_)), std::move(gps)});
    }
  }

  void publish(
    const drone_core::SensorSample & sample,
    const builtin_interfaces::msg::Time & stamp,
    const std::chrono::steady_clock::time_point & wall_now)
  {
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = world_frame_;
    odometry.child_frame_id = body_frame_;
    odometry.pose.pose.position.x = sample.position_world.x();
    odometry.pose.pose.position.y = sample.position_world.y();
    odometry.pose.pose.position.z = sample.position_world.z();
    odometry.pose.pose.orientation = quaternionMessage(sample.orientation_body_to_world);
    const Eigen::Vector3d velocity_body =
      sample.orientation_body_to_world.conjugate() * sample.velocity_world;
    odometry.twist.twist.linear.x = velocity_body.x();
    odometry.twist.twist.linear.y = velocity_body.y();
    odometry.twist.twist.linear.z = velocity_body.z();
    odometry.twist.twist.angular.x = sample.angular_velocity_body.x();
    odometry.twist.twist.angular.y = sample.angular_velocity_body.y();
    odometry.twist.twist.angular.z = sample.angular_velocity_body.z();
    setCovarianceDiagonal(
      odometry.pose.covariance,
      configured_parameters_.position_standard_deviation,
      configured_parameters_.orientation_standard_deviation);
    setCovarianceDiagonal(
      odometry.twist.covariance,
      configured_parameters_.velocity_standard_deviation,
      configured_parameters_.gyroscope_standard_deviation);
    sensor_msgs::msg::Imu imu;
    imu.header = odometry.header;
    imu.header.frame_id = body_frame_;
    imu.orientation = odometry.pose.pose.orientation;
    imu.angular_velocity.x = sample.angular_velocity_body.x();
    imu.angular_velocity.y = sample.angular_velocity_body.y();
    imu.angular_velocity.z = sample.angular_velocity_body.z();
    imu.linear_acceleration.x = sample.linear_acceleration_body.x();
    imu.linear_acceleration.y = sample.linear_acceleration_body.y();
    imu.linear_acceleration.z = sample.linear_acceleration_body.z();
    setCovarianceDiagonal(
      imu.orientation_covariance, configured_parameters_.orientation_standard_deviation);
    setCovarianceDiagonal(
      imu.angular_velocity_covariance, configured_parameters_.gyroscope_standard_deviation);
    setCovarianceDiagonal(
      imu.linear_acceleration_covariance,
      configured_parameters_.accelerometer_standard_deviation);
    if (sensor_output_delay_ <= 0.0) {
      odometry_publisher_->publish(odometry);
      imu_publisher_->publish(imu);
    } else {
      delayed_sensor_outputs_.push_back(
        {wall_now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(sensor_output_delay_)),
          std::move(odometry), std::move(imu)});
    }
  }

  void flushDelayedOutputs(const std::chrono::steady_clock::time_point & wall_now)
  {
    while (!delayed_sensor_outputs_.empty() &&
      delayed_sensor_outputs_.front().release_time <= wall_now)
    {
      odometry_publisher_->publish(delayed_sensor_outputs_.front().odometry);
      imu_publisher_->publish(delayed_sensor_outputs_.front().imu);
      delayed_sensor_outputs_.pop_front();
    }
    while (!delayed_gps_.empty() && delayed_gps_.front().release_time <= wall_now) {
      gps_publisher_->publish(delayed_gps_.front().message);
      delayed_gps_.pop_front();
    }
  }

  void resetCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    noise_model_->reset();
    gps_noise_model_->reset();
    gps_random_engine_.seed(gps_random_seed_);
    has_last_publish_time_ = false;
    has_imu_ = false;
    has_last_gps_publish_time_ = false;
    delayed_sensor_outputs_.clear();
    delayed_gps_.clear();
    RCLCPP_INFO(get_logger(), "Sensor noise state reset");
  }

  std::unique_ptr<drone_core::SensorNoiseModel> noise_model_;
  std::unique_ptr<drone_core::SensorNoiseModel> gps_noise_model_;
  struct DelayedSensorOutput
  {
    std::chrono::steady_clock::time_point release_time;
    nav_msgs::msg::Odometry odometry;
    sensor_msgs::msg::Imu imu;
  };
  struct DelayedGpsOutput
  {
    std::chrono::steady_clock::time_point release_time;
    sensor_msgs::msg::NavSatFix message;
  };
  drone_core::SensorNoiseParameters configured_parameters_;
  bool noise_enabled_{true};
  double gravity_{9.81};
  double publish_frequency_{100.0};
  bool gps_enabled_{true};
  double gps_publish_frequency_{5.0};
  double minimum_gps_publish_period_{0.2};
  double gps_dropout_probability_{0.02};
  double gps_origin_latitude_{22.302711};
  double gps_origin_longitude_{114.177216};
  double gps_origin_altitude_{10.0};
  double gps_earth_radius_{6378137.0};
  double gps_minimum_longitude_cosine_{1.0e-6};
  double minimum_sensor_dt_{1.0e-6};
  double sensor_output_delay_{0.0};
  double gps_output_delay_{0.0};
  double minimum_publish_period_{0.01};
  double minimum_quaternion_norm_{1.0e-9};
  std::string world_frame_{"map"};
  std::string body_frame_{"base_link"};
  std::string truth_odometry_topic_{"/drone/truth/odom"};
  std::string truth_imu_topic_{"/drone/truth/imu"};
  std::string odometry_topic_{"/drone/odom"};
  std::string imu_topic_{"/drone/imu"};
  std::string gps_topic_{"/drone/gps"};
  std::string reset_service_name_{"/drone/sensors/reset"};
  int truth_qos_depth_{20};
  int output_qos_depth_{20};
  Eigen::Vector3d latest_specific_force_body_{Eigen::Vector3d::Zero()};
  bool has_imu_{false};
  bool has_last_publish_time_{false};
  bool has_last_gps_publish_time_{false};
  std::chrono::steady_clock::time_point last_publish_wall_time_;
  std::chrono::steady_clock::time_point last_gps_publish_wall_time_;
  std::uint32_t gps_random_seed_{30U};
  std::mt19937 gps_random_engine_;
  std::uniform_real_distribution<double> uniform_{0.0, 1.0};
  std::deque<DelayedSensorOutput> delayed_sensor_outputs_;
  std::deque<DelayedGpsOutput> delayed_gps_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr truth_odometry_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr truth_imu_subscription_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
};

}  // namespace drone_sensors

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<drone_sensors::SensorSimulatorNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("sensor_simulator_node"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
