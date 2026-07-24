#pragma once

#include <cstdint>
#include <random>

#include "drone_core/types.hpp"

namespace drone_core
{

enum class DisturbanceMode
{
  kConstant,
  kSinusoidal,
  kGust,
  kRandom,
};

struct DisturbanceModelParameters
{
  DisturbanceMode mode{DisturbanceMode::kConstant};
  Disturbance constant{};
  Disturbance amplitude{};
  double start_time{0.0};
  double duration{0.0};
  double frequency{0.5};
  double random_correlation_time{0.5};
  std::uint32_t random_seed{1U};
};

/// Stateful, deterministic and ROS-independent disturbance source.
class DisturbanceModel
{
public:
  explicit DisturbanceModel(const DisturbanceModelParameters & parameters = {});

  void reset();
  [[nodiscard]] Disturbance sample(double time_seconds, double dt);
  [[nodiscard]] const DisturbanceModelParameters & parameters() const noexcept;

private:
  void validateParameters() const;
  [[nodiscard]] bool active(double time_seconds) const;
  [[nodiscard]] Eigen::Vector3d randomVector();

  DisturbanceModelParameters parameters_;
  std::mt19937 random_engine_;
  std::normal_distribution<double> normal_{0.0, 1.0};
  Disturbance random_state_{};
};

}  // namespace drone_core
