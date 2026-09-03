#pragma once

#include <atomic>
#include <chrono>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "go2w_motion_control/goal_logger.hpp"
#include "go2w_motion_control/leased_sport_client.hpp"
#include "go2w_motion_control/motion_state_monitor.hpp"
#include "go2w_motion_control/safety_guard.hpp"
#include "go2w_motion_control/yaw_controller.hpp"
#include "go2w_motion_interfaces/action/motion_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace go2w_motion_control {

class MotionActionServer : public rclcpp::Node {
 public:
  using MotionCommand = go2w_motion_interfaces::action::MotionCommand;
  using GoalHandle = rclcpp_action::ServerGoalHandle<MotionCommand>;

  explicit MotionActionServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~MotionActionServer() override;

 private:
  struct Parameters {
    std::string expected_motion_name;
    std::string action_name;
    std::string arm_service;
    std::string emergency_stop_service;
    bool require_arm{true};
    bool require_low_state{true};
    bool dry_run{false};
    double arm_timeout_sec{60.0};
    double state_timeout_sec{0.5};
    double response_timeout_sec{2.0};
    int pre_stop_count{3};
    int final_stop_count{3};
    double stop_interval_sec{0.1};
    double final_stationary_timeout_sec{3.0};
    int stationary_stable_samples{5};
    double stationary_max_abs_vx{0.02};
    double stationary_max_abs_vy{0.02};
    double stationary_max_abs_yaw_rate{0.03};
    double timed_refresh_hz{2.0};
    double turn_control_hz{20.0};
    double turn_publish_max_hz{5.0};
    double command_change_threshold{0.01};
    double turn_kp{0.8};
    double turn_tolerance_deg{2.0};
    double turn_slow_zone_deg{20.0};
    int turn_stable_samples{5};
    double maximum_overshoot_deg{8.0};
    double final_yaw_acceptance_deg{5.0};
    double turn_longitudinal_compensation_vx{0.05};
    double turn_compensation_max_abs_vx{0.06};
    double turn_compensation_taper_deg{5.0};
    double post_turn_zero_velocity_hold_sec{1.0};
    double post_turn_rollback_control_sec{3.0};
    double post_turn_rollback_deadband_radps{0.03};
    double post_turn_rollback_gain{0.8};
    double post_turn_rollback_max_vx{0.10};
    int post_turn_rollback_stable_samples{8};
    double wheel_radius_m{0.089};
    bool allow_reverse_correction{false};
    double min_abs_yaw_rate_turn{0.05};
    int yaw_command_sign{0};
    std::vector<int64_t> allowed_initial_modes;
    std::filesystem::path log_root;
    GoalLimits goal_limits;
  };

  struct ExecutionState {
    std::chrono::steady_clock::time_point started;
    float elapsed_sec{0.0F};
    float actual_relative_yaw_deg{0.0F};
    int32_t last_move_status{-9999};
    int32_t last_stop_status{-9999};
    double initial_unwrapped_yaw{0.0};
    bool yaw_reference_initialized{false};
    double last_turn_compensation_vx{0.0};
    double peak_abs_turn_compensation_vx{0.0};
    bool post_turn_zero_velocity_hold_completed{false};
    bool post_turn_rollback_completed{false};
    double post_turn_rollback_peak_vx{0.0};
    double post_turn_rollback_peak_abs_dq{0.0};
  };

  struct MotionFailure {
    uint16_t code;
    std::string message;
    bool canceled{false};
  };

  Parameters LoadParameters();
  rclcpp_action::GoalResponse HandleGoal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const MotionCommand::Goal> goal);
  rclcpp_action::CancelResponse HandleCancel(
      const std::shared_ptr<GoalHandle> goal_handle);
  void HandleAccepted(const std::shared_ptr<GoalHandle> goal_handle);
  void Execute(const std::shared_ptr<GoalHandle> goal_handle,
               const std::string &goal_id);
  void RunTimedVelocity(const std::shared_ptr<GoalHandle> &goal_handle,
                        ExecutionState &execution,
                        const std::shared_ptr<GoalLogger> &logger);
  void RunRelativeYaw(const std::shared_ptr<GoalHandle> &goal_handle,
                      ExecutionState &execution,
                      const std::shared_ptr<GoalLogger> &logger);
  void CheckRuntimeSafety(const std::shared_ptr<GoalHandle> &goal_handle,
                          const std::shared_ptr<GoalLogger> &logger);
  void PublishFeedback(const std::shared_ptr<GoalHandle> &goal_handle,
                       const ExecutionState &execution, double target_yaw_deg,
                       double current_yaw_deg, double yaw_error_deg,
                       const std::shared_ptr<GoalLogger> &logger);
  bool StopAndVerify(const std::string &reason, int32_t *last_status,
                     const std::shared_ptr<GoalLogger> &logger,
                     int stop_count = -1);
  void HandleArm(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                 std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void HandleEmergencyStop(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void CheckArmTimeout();
  bool MotionModeFreshAndExpected() const;
  bool InitialRobotModeAllowed(uint8_t robot_mode) const;
  static std::string GoalId(const rclcpp_action::GoalUUID &uuid);

  Parameters parameters_;
  std::unique_ptr<MotionStateMonitor> state_monitor_;
  std::unique_ptr<LeasedSportClient> sport_client_;
  rclcpp_action::Server<MotionCommand>::SharedPtr action_server_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arm_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_stop_service_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr motion_name_sub_;
  rclcpp::TimerBase::SharedPtr arm_timer_;
  rclcpp::CallbackGroup::SharedPtr action_group_;
  rclcpp::CallbackGroup::SharedPtr arm_group_;
  rclcpp::CallbackGroup::SharedPtr emergency_group_;
  std::filesystem::path session_log_directory_;
  std::atomic_bool armed_{false};
  std::atomic_bool active_goal_{false};
  std::atomic_bool goal_reserved_{false};
  std::atomic_bool stop_requested_{false};
  std::atomic_bool shutting_down_{false};
  std::chrono::steady_clock::time_point armed_activity_time_{};
  mutable std::mutex mode_mutex_;
  std::string motion_name_;
  std::chrono::steady_clock::time_point motion_name_time_{};
  mutable std::mutex logger_mutex_;
  std::shared_ptr<GoalLogger> active_logger_;
  std::mutex worker_mutex_;
  std::thread worker_;
};

}  // namespace go2w_motion_control
