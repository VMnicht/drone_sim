#include "drone_core/disturbance_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace drone_core
{

DisturbanceModel::DisturbanceModel(const DisturbanceModelParameters & parameters)
: parameters_(parameters), random_engine_(parameters.random_seed)
{
  validateParameters();
}

void DisturbanceModel::reset()
{
  random_engine_.seed(parameters_.random_seed);
  normal_.reset();
  random_state_ = {};
}

const DisturbanceModelParameters & DisturbanceModel::parameters() const noexcept
{
  return parameters_;
}

bool DisturbanceModel::active(double time_seconds) const
{
  if (time_seconds < parameters_.start_time) {
    return false;
  }
  return parameters_.duration <= 0.0 ||
         time_seconds <= parameters_.start_time + parameters_.duration;
}

Eigen::Vector3d DisturbanceModel::randomVector()
{
  return Eigen::Vector3d{normal_(random_engine_), normal_(random_engine_), normal_(random_engine_)};
}

Disturbance DisturbanceModel::sample(double time_seconds, double dt)
{
  if (!std::isfinite(time_seconds) || !std::isfinite(dt) || dt <= 0.0) {
    throw std::invalid_argument("Disturbance time and dt must be finite; dt must be positive");
  }
  if (!active(time_seconds)) {
    return {};
  }

  Disturbance result = parameters_.constant;
  if (parameters_.mode == DisturbanceMode::kConstant) {
    return result;
  }

  if (parameters_.mode == DisturbanceMode::kSinusoidal) {
    const double phase = 2.0 * kPi * parameters_.frequency *
      (time_seconds - parameters_.start_time);
    result.force_world += std::sin(phase) * parameters_.amplitude.force_world;
    result.torque_body += std::sin(phase) * parameters_.amplitude.torque_body;
    return result;
  }

  if (parameters_.mode == DisturbanceMode::kGust) {
    result.force_world += parameters_.amplitude.force_world;
    result.torque_body += parameters_.amplitude.torque_body;
    return result;
  }

  const double decay = std::exp(-dt / parameters_.random_correlation_time);
  const double innovation_scale = std::sqrt(std::max(0.0, 1.0 - decay * decay));
  random_state_.force_world = decay * random_state_.force_world +
    innovation_scale * parameters_.amplitude.force_world.cwiseProduct(randomVector());
  random_state_.torque_body = decay * random_state_.torque_body +
    innovation_scale * parameters_.amplitude.torque_body.cwiseProduct(randomVector());
  result.force_world += random_state_.force_world;
  result.torque_body += random_state_.torque_body;
  return result;
}

void DisturbanceModel::validateParameters() const
{
  if (!parameters_.constant.isFinite() || !parameters_.amplitude.isFinite() ||
    !std::isfinite(parameters_.start_time) || parameters_.start_time < 0.0 ||
    !std::isfinite(parameters_.duration) || parameters_.duration < 0.0 ||
    !std::isfinite(parameters_.frequency) || parameters_.frequency < 0.0 ||
    !std::isfinite(parameters_.random_correlation_time) ||
    parameters_.random_correlation_time <= 0.0)
  {
    throw std::invalid_argument("Invalid disturbance model parameters");
  }
}

}  // namespace drone_core
