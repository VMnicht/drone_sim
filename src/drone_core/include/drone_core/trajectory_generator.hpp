#pragma once

#include "drone_core/model_based_controller.hpp"

namespace drone_core
{

enum class AnalyticTrajectoryType
{
  kHold,
  kCircle,
  kFigureEight,
};

struct AnalyticTrajectoryParameters
{
  AnalyticTrajectoryType type{AnalyticTrajectoryType::kHold};
  Eigen::Vector3d center{0.0, 0.0, 1.5};
  double radius_x{1.0};
  double radius_y{1.0};
  double period{8.0};
  double phase{0.0};
  double fixed_yaw{0.0};
  bool face_velocity{false};
  double minimum_yaw_speed{1.0e-6};
};

/// ROS-independent analytic reference generator.
///
/// Circle and figure-eight references include analytic velocity and
/// acceleration feed-forward terms. Time is expressed in seconds from the
/// beginning of the mission.
class AnalyticTrajectoryGenerator
{
public:
  explicit AnalyticTrajectoryGenerator(const AnalyticTrajectoryParameters & parameters = {});

  [[nodiscard]] const AnalyticTrajectoryParameters & parameters() const noexcept;
  [[nodiscard]] Reference sample(double time_seconds) const;

private:
  void validateParameters() const;
  [[nodiscard]] Reference applyYaw(Reference reference) const;

  AnalyticTrajectoryParameters parameters_;
};

}  // namespace drone_core
