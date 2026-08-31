#pragma once

#include <algorithm>
#include <cmath>

#include "go2w_motion_control/angle_utils.hpp"

namespace go2w_motion_control {

struct YawControlParameters {
  double kp{0.8};
  double tolerance_deg{2.0};
  double slow_zone_deg{20.0};
  double minimum_rate{0.05};
};

inline double ComputeLogicalYawRate(double error_rad, double goal_max_rate,
                                    const YawControlParameters &parameters) {
  const double error_deg = std::abs(RadiansToDegrees(error_rad));
  if (error_deg <= parameters.tolerance_deg) {
    return 0.0;
  }
  double allowed_rate = goal_max_rate;
  if (parameters.slow_zone_deg > parameters.tolerance_deg &&
      error_deg < parameters.slow_zone_deg) {
    const double ratio =
        (error_deg - parameters.tolerance_deg) /
        (parameters.slow_zone_deg - parameters.tolerance_deg);
    allowed_rate = parameters.minimum_rate +
                   std::max(0.0, ratio) *
                       (goal_max_rate - parameters.minimum_rate);
  }
  double command = std::clamp(parameters.kp * error_rad, -allowed_rate,
                              allowed_rate);
  if (std::abs(command) < parameters.minimum_rate) {
    command = std::copysign(parameters.minimum_rate, error_rad);
  }
  return command;
}

inline bool ErrorCrossedTarget(double previous_error, double current_error) {
  return previous_error != 0.0 && current_error != 0.0 &&
         std::signbit(previous_error) != std::signbit(current_error);
}

inline double ComputeTurnLongitudinalCompensation(
    double error_rad, double tolerance_deg, double taper_deg,
    double configured_vx, double maximum_abs_vx) {
  if (!std::isfinite(error_rad) || !std::isfinite(configured_vx) ||
      !std::isfinite(maximum_abs_vx) || maximum_abs_vx <= 0.0) {
    return 0.0;
  }
  const double error_deg = std::abs(RadiansToDegrees(error_rad));
  if (error_deg <= tolerance_deg) return 0.0;
  double scale = 1.0;
  if (taper_deg > tolerance_deg && error_deg < taper_deg) {
    scale = (error_deg - tolerance_deg) / (taper_deg - tolerance_deg);
  }
  return std::clamp(configured_vx, -maximum_abs_vx, maximum_abs_vx) *
         std::clamp(scale, 0.0, 1.0);
}

}  // namespace go2w_motion_control
