#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "go2w_motion_interfaces/action/motion_command.hpp"

namespace go2w_motion_control {

struct GoalLimits {
  double max_abs_vx{0.20};
  double max_abs_vy{0.0};
  double max_abs_yaw_rate_timed{0.20};
  double max_duration_sec{10.0};
  double max_abs_relative_yaw_deg{180.0};
  double max_abs_yaw_rate_turn{0.25};
};

struct GoalContext {
  bool armed{false};
  bool active_goal{false};
  bool lease_available{false};
  bool state_fresh{false};
  uint32_t robot_error_code{0};
  int yaw_command_sign{0};
};

struct ValidationResult {
  bool valid{false};
  uint16_t error_code{0};
  std::string message;
};

inline bool IsAllowedInitialMode(uint8_t robot_mode,
                                 const std::vector<int64_t> &allowed_modes) {
  for (const auto allowed_mode : allowed_modes) {
    if (allowed_mode >= 0 && allowed_mode <= 255 &&
        robot_mode == static_cast<uint8_t>(allowed_mode)) {
      return true;
    }
  }
  return false;
}

inline ValidationResult ValidateGoal(
    const go2w_motion_interfaces::action::MotionCommand::Goal &goal,
    const GoalLimits &limits, const GoalContext &context) {
  using Action = go2w_motion_interfaces::action::MotionCommand;
  auto fail = [](uint16_t code, const std::string &message) {
    return ValidationResult{false, code, message};
  };
  if (!context.armed) {
    return fail(Action::Result::ERROR_NOT_ARMED, "motion is not armed");
  }
  if (context.active_goal) {
    return fail(Action::Result::ERROR_CONCURRENT_GOAL,
                "another goal is active");
  }
  if (!context.lease_available) {
    return fail(Action::Result::ERROR_LEASE_UNAVAILABLE,
                "Sport lease is unavailable");
  }
  if (!context.state_fresh) {
    return fail(Action::Result::ERROR_STATE_STALE,
                "robot state is stale");
  }
  if (context.robot_error_code != 0) {
    return fail(Action::Result::ERROR_ROBOT_ERROR,
                "robot error_code is nonzero");
  }
  const double values[] = {goal.vx, goal.vy, goal.yaw_rate,
                           goal.duration_sec, goal.relative_yaw_deg,
                           goal.max_yaw_rate, goal.timeout_sec};
  for (double value : values) {
    if (!std::isfinite(value)) {
      return fail(Action::Result::ERROR_INVALID_GOAL,
                  "all goal fields must be finite");
    }
  }
  if (goal.mode == Action::Goal::MODE_TIMED_VELOCITY) {
    if (goal.duration_sec <= 0.0 ||
        goal.duration_sec > limits.max_duration_sec ||
        std::abs(goal.vx) > limits.max_abs_vx ||
        std::abs(goal.vy) > limits.max_abs_vy ||
        std::abs(goal.yaw_rate) > limits.max_abs_yaw_rate_timed ||
        (goal.vx == 0.0 && goal.vy == 0.0 && goal.yaw_rate == 0.0)) {
      return fail(Action::Result::ERROR_INVALID_GOAL,
                  "timed velocity goal violates configured limits");
    }
  } else if (goal.mode == Action::Goal::MODE_RELATIVE_YAW) {
    if (std::abs(goal.relative_yaw_deg) <= 2.0 ||
        std::abs(goal.relative_yaw_deg) >
            limits.max_abs_relative_yaw_deg ||
        goal.max_yaw_rate <= 0.0 ||
        goal.max_yaw_rate > limits.max_abs_yaw_rate_turn || goal.vx != 0.0 ||
        goal.vy != 0.0) {
      return fail(Action::Result::ERROR_INVALID_GOAL,
                  "relative yaw goal violates configured limits");
    }
    if (context.yaw_command_sign != 1 && context.yaw_command_sign != -1) {
      return fail(Action::Result::ERROR_DIRECTION_NOT_CALIBRATED,
                  "yaw command direction is not calibrated");
    }
  } else {
    return fail(Action::Result::ERROR_INVALID_GOAL, "unknown motion mode");
  }
  return ValidationResult{true, Action::Result::ERROR_NONE, "accepted"};
}

}  // namespace go2w_motion_control
