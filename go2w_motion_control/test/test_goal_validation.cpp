#include <gtest/gtest.h>

#include <limits>

#include "go2w_motion_control/safety_guard.hpp"

using Action = go2w_motion_interfaces::action::MotionCommand;
using namespace go2w_motion_control;

namespace {
GoalContext Ready() { return {true, false, true, true, 0, 1}; }

Action::Goal Timed() {
  Action::Goal goal;
  goal.mode = Action::Goal::MODE_TIMED_VELOCITY;
  goal.vx = 0.05;
  goal.duration_sec = 0.5;
  return goal;
}
}  // namespace

TEST(GoalValidation, AcceptsBoundedTimedVelocity) {
  EXPECT_TRUE(ValidateGoal(Timed(), GoalLimits{}, Ready()).valid);
}

TEST(GoalValidation, RejectsNanInfAndLimits) {
  auto goal = Timed();
  goal.vx = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(ValidateGoal(goal, GoalLimits{}, Ready()).valid);
  goal = Timed();
  goal.duration_sec = std::numeric_limits<float>::infinity();
  EXPECT_FALSE(ValidateGoal(goal, GoalLimits{}, Ready()).valid);
  goal = Timed();
  goal.vy = 0.01;
  EXPECT_FALSE(ValidateGoal(goal, GoalLimits{}, Ready()).valid);
  goal = Timed();
  goal.duration_sec = 11.0;
  EXPECT_FALSE(ValidateGoal(goal, GoalLimits{}, Ready()).valid);
}

TEST(GoalValidation, RejectsSafetyContext) {
  auto context = Ready();
  context.armed = false;
  EXPECT_EQ(ValidateGoal(Timed(), GoalLimits{}, context).error_code,
            Action::Result::ERROR_NOT_ARMED);
  context = Ready();
  context.lease_available = false;
  EXPECT_EQ(ValidateGoal(Timed(), GoalLimits{}, context).error_code,
            Action::Result::ERROR_LEASE_UNAVAILABLE);
  context = Ready();
  context.state_fresh = false;
  EXPECT_EQ(ValidateGoal(Timed(), GoalLimits{}, context).error_code,
            Action::Result::ERROR_STATE_STALE);
  context = Ready();
  context.active_goal = true;
  EXPECT_EQ(ValidateGoal(Timed(), GoalLimits{}, context).error_code,
            Action::Result::ERROR_CONCURRENT_GOAL);
}

TEST(GoalValidation, RelativeYawRequiresCalibrationAndLimits) {
  Action::Goal goal;
  goal.mode = Action::Goal::MODE_RELATIVE_YAW;
  goal.relative_yaw_deg = 30.0;
  goal.max_yaw_rate = 0.12;
  auto context = Ready();
  context.yaw_command_sign = 0;
  EXPECT_EQ(ValidateGoal(goal, GoalLimits{}, context).error_code,
            Action::Result::ERROR_DIRECTION_NOT_CALIBRATED);
  context.yaw_command_sign = 1;
  EXPECT_TRUE(ValidateGoal(goal, GoalLimits{}, context).valid);
  goal.relative_yaw_deg = 181.0;
  EXPECT_FALSE(ValidateGoal(goal, GoalLimits{}, context).valid);
}

TEST(GoalValidation, InitialModeAllowlistRejectsUnknownModes) {
  const std::vector<int64_t> allowed{1};
  EXPECT_TRUE(IsAllowedInitialMode(1, allowed));
  EXPECT_FALSE(IsAllowedInitialMode(3, allowed));
  EXPECT_FALSE(IsAllowedInitialMode(14, allowed));
  EXPECT_FALSE(IsAllowedInitialMode(1, std::vector<int64_t>{-1, 256}));
}
