#include "go2w_motion_control/motion_action_server.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <unistd.h>

#include "nlohmann/json.hpp"

using namespace std::chrono_literals;

namespace go2w_motion_control {

MotionActionServer::MotionActionServer(const rclcpp::NodeOptions &options)
    : rclcpp::Node("go2w_motion_action_server", options),
      parameters_(LoadParameters()) {
  const auto sport_request_topic =
      get_parameter("sport_request_topic").as_string();
  const auto sport_response_topic =
      get_parameter("sport_response_topic").as_string();
  const auto sport_state_topic = get_parameter("sport_state_topic").as_string();
  const auto low_state_topic = get_parameter("low_state_topic").as_string();
  const auto lease_id_topic = get_parameter("lease_id_topic").as_string();
  const auto lease_alive_topic = get_parameter("lease_alive_topic").as_string();
  const auto motion_name_topic = get_parameter("motion_name_topic").as_string();
  const auto sdk_command_socket =
      get_parameter("sdk_command_socket").as_string();
  const auto lease_status_timeout =
      get_parameter("lease_status_timeout_sec").as_double();

  session_log_directory_ = parameters_.log_root / WallTimestampForPath();
  std::filesystem::create_directories(session_log_directory_);

  state_monitor_ = std::make_unique<MotionStateMonitor>(
      this, sport_state_topic, low_state_topic, parameters_.require_low_state,
      parameters_.wheel_radius_m);
  sport_client_ = std::make_unique<LeasedSportClient>(
      this, sport_request_topic, sport_response_topic, lease_id_topic,
      lease_alive_topic, sdk_command_socket, lease_status_timeout,
      parameters_.dry_run);
  sport_client_->SetEventCallback(
      [this](const std::string &kind, const RequestResult &result,
             const std::string &parameter) {
        std::shared_ptr<GoalLogger> logger;
        {
          std::lock_guard<std::mutex> lock(logger_mutex_);
          logger = active_logger_;
        }
        if (logger) logger->LogRequestEvent(kind, result, parameter);
      });

  const auto transient_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  motion_name_sub_ = create_subscription<std_msgs::msg::String>(
      motion_name_topic, transient_qos,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mode_mutex_);
        motion_name_ = msg->data;
        motion_name_time_ = std::chrono::steady_clock::now();
      });

  action_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  arm_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  emergency_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  action_server_ = rclcpp_action::create_server<MotionCommand>(
      this, parameters_.action_name,
      std::bind(&MotionActionServer::HandleGoal, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&MotionActionServer::HandleCancel, this, std::placeholders::_1),
      std::bind(&MotionActionServer::HandleAccepted, this,
                std::placeholders::_1),
      rcl_action_server_get_default_options(), action_group_);

  arm_service_ = create_service<std_srvs::srv::SetBool>(
      parameters_.arm_service,
      std::bind(&MotionActionServer::HandleArm, this, std::placeholders::_1,
                std::placeholders::_2),
      rmw_qos_profile_services_default, arm_group_);
  emergency_stop_service_ = create_service<std_srvs::srv::Trigger>(
      parameters_.emergency_stop_service,
      std::bind(&MotionActionServer::HandleEmergencyStop, this,
                std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, emergency_group_);
  arm_timer_ = create_wall_timer(
      1s, std::bind(&MotionActionServer::CheckArmTimeout, this), arm_group_);

  RCLCPP_INFO(get_logger(),
              "Go2-W motion Action ready: action=%s dry_run=%s log=%s",
              parameters_.action_name.c_str(),
              parameters_.dry_run ? "true" : "false",
              session_log_directory_.c_str());
}

MotionActionServer::~MotionActionServer() {
  shutting_down_.store(true);
  stop_requested_.store(true);
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (worker_.joinable()) worker_.join();
  }
  if (sport_client_ && sport_client_->LeaseAvailable()) {
    int32_t ignored = -9999;
    sport_client_->StopRepeatedly(
        parameters_.final_stop_count,
        std::chrono::milliseconds(
            static_cast<int>(parameters_.stop_interval_sec * 1000.0)),
        std::chrono::milliseconds(
            static_cast<int>(parameters_.response_timeout_sec * 1000.0)),
        &ignored);
  }
}

MotionActionServer::Parameters MotionActionServer::LoadParameters() {
  Parameters p;
  declare_parameter<std::string>("robot_ip", "192.168.123.18");
  p.expected_motion_name =
      declare_parameter<std::string>("expected_motion_name", "ai-w");
  declare_parameter<std::string>("sport_request_topic", "/api/sport/request");
  declare_parameter<std::string>("sport_response_topic", "/api/sport/response");
  declare_parameter<std::string>("sport_state_topic", "/lf/sportmodestate");
  declare_parameter<std::string>("low_state_topic", "/lf/lowstate");
  declare_parameter<std::string>("lease_id_topic", "/go2w/sport_lease/id");
  declare_parameter<std::string>("lease_alive_topic",
                                 "/go2w/sport_lease/alive");
  declare_parameter<std::string>("motion_name_topic",
                                 "/go2w/motion_mode/name");
  declare_parameter<std::string>("sdk_command_socket",
                                 "/tmp/go2w_sdk_motion.sock");
  p.action_name = declare_parameter<std::string>("action_name", "/go2w/motion");
  p.arm_service = declare_parameter<std::string>("arm_service", "/go2w/arm");
  p.emergency_stop_service = declare_parameter<std::string>(
      "emergency_stop_service", "/go2w/emergency_stop");
  p.require_arm = declare_parameter<bool>("require_arm", true);
  p.require_low_state = declare_parameter<bool>("require_low_state", true);
  p.dry_run = declare_parameter<bool>("dry_run", false);
  p.arm_timeout_sec = declare_parameter<double>("arm_timeout_sec", 60.0);
  p.state_timeout_sec = declare_parameter<double>("state_timeout_sec", 0.5);
  declare_parameter<double>("lease_status_timeout_sec", 1.0);
  p.response_timeout_sec =
      declare_parameter<double>("response_timeout_sec", 2.0);
  p.pre_stop_count = declare_parameter<int>("pre_stop_count", 3);
  p.final_stop_count = declare_parameter<int>("final_stop_count", 3);
  p.stop_interval_sec = declare_parameter<double>("stop_interval_sec", 0.1);
  p.final_stationary_timeout_sec =
      declare_parameter<double>("final_stationary_timeout_sec", 3.0);
  declare_parameter<bool>("reject_concurrent_goals", true);
  p.goal_limits.max_abs_vx = declare_parameter<double>("max_abs_vx", 0.20);
  p.goal_limits.max_abs_vy = declare_parameter<double>("max_abs_vy", 0.0);
  p.goal_limits.max_abs_yaw_rate_timed =
      declare_parameter<double>("max_abs_yaw_rate_timed", 0.20);
  p.goal_limits.max_duration_sec =
      declare_parameter<double>("max_duration_sec", 10.0);
  p.goal_limits.max_abs_relative_yaw_deg =
      declare_parameter<double>("max_abs_relative_yaw_deg", 180.0);
  p.goal_limits.max_abs_yaw_rate_turn =
      declare_parameter<double>("max_abs_yaw_rate_turn", 0.25);
  p.min_abs_yaw_rate_turn =
      declare_parameter<double>("min_abs_yaw_rate_turn", 0.05);
  p.stationary_max_abs_vx =
      declare_parameter<double>("stationary_max_abs_vx", 0.02);
  p.stationary_max_abs_vy =
      declare_parameter<double>("stationary_max_abs_vy", 0.02);
  p.stationary_max_abs_yaw_rate =
      declare_parameter<double>("stationary_max_abs_yaw_rate", 0.03);
  p.stationary_stable_samples =
      declare_parameter<int>("stationary_stable_samples", 5);
  p.timed_refresh_hz = declare_parameter<double>("timed_refresh_hz", 2.0);
  p.turn_control_hz = declare_parameter<double>("turn_control_hz", 20.0);
  p.turn_publish_max_hz =
      declare_parameter<double>("turn_publish_max_hz", 5.0);
  p.command_change_threshold =
      declare_parameter<double>("command_change_threshold", 0.01);
  p.turn_kp = declare_parameter<double>("turn_kp", 0.8);
  p.turn_tolerance_deg =
      declare_parameter<double>("turn_tolerance_deg", 2.0);
  p.turn_slow_zone_deg =
      declare_parameter<double>("turn_slow_zone_deg", 20.0);
  p.turn_stable_samples = declare_parameter<int>("turn_stable_samples", 5);
  p.maximum_overshoot_deg =
      declare_parameter<double>("maximum_overshoot_deg", 8.0);
  p.final_yaw_acceptance_deg =
      declare_parameter<double>("final_yaw_acceptance_deg", 5.0);
  p.turn_longitudinal_compensation_vx = declare_parameter<double>(
      "turn_longitudinal_compensation_vx", 0.05);
  p.turn_compensation_max_abs_vx =
      declare_parameter<double>("turn_compensation_max_abs_vx", 0.06);
  p.turn_compensation_taper_deg =
      declare_parameter<double>("turn_compensation_taper_deg", 5.0);
  p.post_turn_zero_velocity_hold_sec =
      declare_parameter<double>("post_turn_zero_velocity_hold_sec", 1.0);
  p.post_turn_rollback_control_sec = declare_parameter<double>(
      "post_turn_rollback_control_sec", 3.0);
  p.post_turn_rollback_deadband_radps = declare_parameter<double>(
      "post_turn_rollback_deadband_radps", 0.03);
  p.post_turn_rollback_gain = declare_parameter<double>(
      "post_turn_rollback_gain", 0.8);
  p.post_turn_rollback_max_vx = declare_parameter<double>(
      "post_turn_rollback_max_vx", 0.10);
  p.post_turn_rollback_stable_samples = declare_parameter<int>(
      "post_turn_rollback_stable_samples", 8);
  p.wheel_radius_m = declare_parameter<double>("wheel_radius_m", 0.089);
  if (!std::isfinite(p.turn_longitudinal_compensation_vx) ||
      !std::isfinite(p.turn_compensation_max_abs_vx) ||
      p.turn_compensation_max_abs_vx <= 0.0 ||
      p.turn_compensation_max_abs_vx > p.goal_limits.max_abs_vx ||
      std::abs(p.turn_longitudinal_compensation_vx) >
          p.turn_compensation_max_abs_vx ||
      !std::isfinite(p.turn_compensation_taper_deg) ||
      p.turn_compensation_taper_deg <= p.turn_tolerance_deg ||
      !std::isfinite(p.post_turn_zero_velocity_hold_sec) ||
      p.post_turn_zero_velocity_hold_sec < 0.0 ||
      p.post_turn_zero_velocity_hold_sec > 3.0 ||
      !std::isfinite(p.post_turn_rollback_control_sec) ||
      p.post_turn_rollback_control_sec < 0.0 ||
      p.post_turn_rollback_control_sec > 6.0 ||
      !std::isfinite(p.post_turn_rollback_deadband_radps) ||
      p.post_turn_rollback_deadband_radps < 0.0 ||
      p.post_turn_rollback_deadband_radps > 0.2 ||
      !std::isfinite(p.post_turn_rollback_gain) ||
      p.post_turn_rollback_gain < 0.0 ||
      p.post_turn_rollback_gain > 3.0 ||
      !std::isfinite(p.post_turn_rollback_max_vx) ||
      p.post_turn_rollback_max_vx <= 0.0 ||
      p.post_turn_rollback_max_vx > p.goal_limits.max_abs_vx ||
      p.post_turn_rollback_stable_samples < 1 ||
      p.post_turn_rollback_stable_samples > 50 ||
      !std::isfinite(p.wheel_radius_m) ||
      p.wheel_radius_m <= 0.0 ||
      p.wheel_radius_m > 0.3) {
    throw std::invalid_argument(
        "invalid turn compensation, rollback control, or wheel radius "
        "parameters");
  }
  p.allow_reverse_correction =
      declare_parameter<bool>("allow_reverse_correction", false);
  p.yaw_command_sign = declare_parameter<int>("yaw_command_sign", 0);
  p.allowed_initial_modes = declare_parameter<std::vector<int64_t>>(
      "allowed_initial_modes", std::vector<int64_t>{1});
  const char *control_root = std::getenv("GO2W_CONTROL_ROOT");
  const std::string default_log_root =
      control_root ? std::string(control_root) + "/logs" : "logs";
  p.log_root = declare_parameter<std::string>("log_root", default_log_root);
  return p;
}

std::string MotionActionServer::GoalId(const rclcpp_action::GoalUUID &uuid) {
  std::ostringstream output;
  for (uint8_t byte : uuid) {
    output << std::hex << std::setw(2) << std::setfill('0')
           << static_cast<int>(byte);
  }
  return output.str();
}

rclcpp_action::GoalResponse MotionActionServer::HandleGoal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const MotionCommand::Goal> goal) {
  const auto state = state_monitor_->Snapshot();
  GoalContext context{!parameters_.require_arm || armed_.load(),
                      active_goal_.load() || goal_reserved_.load(),
                      sport_client_->LeaseAvailable(),
                      state_monitor_->StateFresh(parameters_.state_timeout_sec),
                      state.error_code, parameters_.yaw_command_sign};
  const auto validation = ValidateGoal(*goal, parameters_.goal_limits, context);
  if (!validation.valid) {
    RCLCPP_WARN(get_logger(), "Reject goal %s: %s", GoalId(uuid).c_str(),
                validation.message.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!MotionModeFreshAndExpected()) {
    RCLCPP_WARN(get_logger(), "Reject goal %s: MotionSwitcher is not fresh ai-w",
                GoalId(uuid).c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!InitialRobotModeAllowed(state.mode)) {
    RCLCPP_WARN(get_logger(), "Reject goal %s: robot mode %u is not allowed",
                GoalId(uuid).c_str(), static_cast<unsigned>(state.mode));
    return rclcpp_action::GoalResponse::REJECT;
  }
  bool expected = false;
  if (!goal_reserved_.compare_exchange_strong(expected, true)) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotionActionServer::HandleCancel(
    const std::shared_ptr<GoalHandle>) {
  stop_requested_.store(true);
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionActionServer::HandleAccepted(
    const std::shared_ptr<GoalHandle> goal_handle) {
  active_goal_.store(true);
  stop_requested_.store(false);
  armed_activity_time_ = std::chrono::steady_clock::now();
  const auto goal_id = GoalId(goal_handle->get_goal_id());
  std::lock_guard<std::mutex> lock(worker_mutex_);
  if (worker_.joinable()) worker_.join();
  worker_ = std::thread(
      [this, goal_handle, goal_id]() { Execute(goal_handle, goal_id); });
}

bool MotionActionServer::MotionModeFreshAndExpected() const {
  std::lock_guard<std::mutex> lock(mode_mutex_);
  if (motion_name_time_.time_since_epoch().count() == 0) return false;
  return motion_name_ == parameters_.expected_motion_name &&
         std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                       motion_name_time_)
                 .count() <= 2.0;
}

bool MotionActionServer::InitialRobotModeAllowed(uint8_t robot_mode) const {
  return IsAllowedInitialMode(robot_mode, parameters_.allowed_initial_modes);
}

void MotionActionServer::HandleArm(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
  if (request->data) {
    const auto state = state_monitor_->Snapshot();
    if (active_goal_.load() || goal_reserved_.load()) {
      response->message = "cannot arm while a goal is active";
    } else if (!sport_client_->LeaseAvailable()) {
      response->message = "Sport lease unavailable or stale";
    } else if (!MotionModeFreshAndExpected()) {
      response->message = "MotionSwitcher status is stale or not ai-w";
    } else if (!state_monitor_->StateFresh(parameters_.state_timeout_sec)) {
      response->message = "robot state is stale";
    } else if (state.error_code != 0) {
      response->message = "robot error_code is nonzero";
    } else if (!InitialRobotModeAllowed(state.mode)) {
      response->message = "robot mode " + std::to_string(state.mode) +
                          " is not allowed for arming";
    } else if (!state_monitor_->IsStationary(
                   state, parameters_.stationary_max_abs_vx,
                   parameters_.stationary_max_abs_vy,
                   parameters_.stationary_max_abs_yaw_rate)) {
      response->message = "robot is not stationary";
    } else {
      armed_.store(true);
      armed_activity_time_ = std::chrono::steady_clock::now();
      response->success = true;
      response->message = "motion armed for 60 seconds of idle time";
      return;
    }
    response->success = false;
    return;
  }

  stop_requested_.store(true);
  int32_t status = -9999;
  const bool stopped = StopAndVerify("disarm", &status, nullptr);
  armed_.store(false);
  response->success = stopped;
  response->message = stopped ? "stopped and disarmed"
                              : "disarmed; STOP verification failed";
}

void MotionActionServer::HandleEmergencyStop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  stop_requested_.store(true);
  armed_.store(false);
  int32_t status = -9999;
  const bool stopped = StopAndVerify("emergency_stop", &status, nullptr);
  response->success = stopped;
  response->message = stopped
                          ? "three STOP responses succeeded; robot stationary"
                          : "STOP verification failed; use remote emergency stop";
}

void MotionActionServer::CheckArmTimeout() {
  if (!armed_.load() || active_goal_.load()) return;
  if (armed_activity_time_.time_since_epoch().count() == 0) return;
  if (std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    armed_activity_time_)
          .count() > parameters_.arm_timeout_sec) {
    stop_requested_.store(true);
    int32_t ignored = -9999;
    StopAndVerify("arm_idle_timeout", &ignored, nullptr);
    armed_.store(false);
  }
}

bool MotionActionServer::StopAndVerify(
    const std::string &reason, int32_t *last_status,
    const std::shared_ptr<GoalLogger> &logger, int stop_count) {
  if (logger) logger->LogSafety("stop_and_verify", reason);
  const int count = stop_count > 0 ? stop_count : parameters_.final_stop_count;
  const bool responses_ok = sport_client_->StopRepeatedly(
      count,
      std::chrono::milliseconds(
          static_cast<int>(parameters_.stop_interval_sec * 1000.0)),
      std::chrono::milliseconds(
          static_cast<int>(parameters_.response_timeout_sec * 1000.0)),
      last_status);
  const bool stationary = state_monitor_->WaitForStationary(
      parameters_.final_stationary_timeout_sec, parameters_.state_timeout_sec,
      parameters_.stationary_stable_samples,
      parameters_.stationary_max_abs_vx, parameters_.stationary_max_abs_vy,
      parameters_.stationary_max_abs_yaw_rate);
  if (logger) {
    logger->LogSafety("stop_verification",
                      std::string("responses=") +
                          (responses_ok ? "ok" : "failed") +
                          " stationary=" + (stationary ? "true" : "false"));
  }
  return responses_ok && stationary;
}

void MotionActionServer::CheckRuntimeSafety(
    const std::shared_ptr<GoalHandle> &goal_handle,
    const std::shared_ptr<GoalLogger> &logger) {
  if (shutting_down_.load() || !rclcpp::ok()) {
    throw MotionFailure{MotionCommand::Result::ERROR_INTERNAL,
                        "node is shutting down", false};
  }
  if (goal_handle->is_canceling() || stop_requested_.load()) {
    if (logger) logger->LogSafety("cancel_or_stop", "stop requested");
    throw MotionFailure{MotionCommand::Result::ERROR_CANCELED,
                        "goal canceled or emergency stop requested", true};
  }
  if (!sport_client_->LeaseAvailable()) {
    throw MotionFailure{MotionCommand::Result::ERROR_LEASE_UNAVAILABLE,
                        "Sport lease became unavailable", false};
  }
  if (!state_monitor_->StateFresh(parameters_.state_timeout_sec)) {
    throw MotionFailure{MotionCommand::Result::ERROR_STATE_STALE,
                        "SportModeState or LowState became stale", false};
  }
  const auto state = state_monitor_->Snapshot();
  if (state.error_code != 0) {
    throw MotionFailure{MotionCommand::Result::ERROR_ROBOT_ERROR,
                        "robot error_code became nonzero", false};
  }
}

void MotionActionServer::PublishFeedback(
    const std::shared_ptr<GoalHandle> &goal_handle,
    const ExecutionState &execution, double target_yaw_deg,
    double current_yaw_deg, double yaw_error_deg,
    const std::shared_ptr<GoalLogger> &logger) {
  const auto state = state_monitor_->Snapshot();
  auto feedback = std::make_shared<MotionCommand::Feedback>();
  feedback->elapsed_sec = execution.elapsed_sec;
  feedback->current_vx = state.velocity_x;
  feedback->current_vy = state.velocity_y;
  feedback->current_yaw_rate = state.yaw_rate;
  feedback->estimated_distance_m = state_monitor_->EstimatedDistance();
  feedback->target_relative_yaw_deg = target_yaw_deg;
  feedback->current_relative_yaw_deg = current_yaw_deg;
  feedback->yaw_error_deg = yaw_error_deg;
  feedback->robot_mode = state.mode;
  feedback->robot_error_code = state.error_code;
  feedback->lease_alive = sport_client_->LeaseAvailable();
  feedback->state_fresh =
      state_monitor_->StateFresh(parameters_.state_timeout_sec);
  goal_handle->publish_feedback(feedback);
  if (logger) {
    logger->LogState(state, sport_client_->CurrentLeaseId());
    logger->LogFeedback({{"elapsed_sec", feedback->elapsed_sec},
                         {"current_vx", feedback->current_vx},
                         {"current_vy", feedback->current_vy},
                         {"current_yaw_rate", feedback->current_yaw_rate},
                         {"estimated_distance_m",
                          feedback->estimated_distance_m},
                         {"target_relative_yaw_deg",
                          feedback->target_relative_yaw_deg},
                         {"current_relative_yaw_deg",
                          feedback->current_relative_yaw_deg},
                         {"yaw_error_deg", feedback->yaw_error_deg},
                         {"robot_mode", feedback->robot_mode},
                         {"robot_error_code", feedback->robot_error_code},
                         {"lease_alive", feedback->lease_alive},
                         {"state_fresh", feedback->state_fresh}});
  }
}

void MotionActionServer::RunTimedVelocity(
    const std::shared_ptr<GoalHandle> &goal_handle, ExecutionState &execution,
    const std::shared_ptr<GoalLogger> &logger) {
  const auto goal = goal_handle->get_goal();
  const auto refresh = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / parameters_.timed_refresh_hz));
  auto next_request = execution.started;
  const double timeout_sec =
      goal->timeout_sec > 0.0 ? goal->timeout_sec : goal->duration_sec + 5.0;
  while (true) {
    CheckRuntimeSafety(goal_handle, logger);
    const auto now = std::chrono::steady_clock::now();
    execution.elapsed_sec =
        std::chrono::duration<float>(now - execution.started).count();
    if (execution.elapsed_sec >= goal->duration_sec) break;
    if (execution.elapsed_sec > timeout_sec) {
      throw MotionFailure{MotionCommand::Result::ERROR_TIMEOUT,
                          "timed velocity goal timed out", false};
    }
    if (now >= next_request) {
      const auto response = sport_client_->SendMove(
          goal->vx, goal->vy, goal->yaw_rate,
          std::chrono::milliseconds(
              static_cast<int>(parameters_.response_timeout_sec * 1000.0)));
      execution.last_move_status = response.status_code;
      if (!response.response_received || response.status_code != 0) {
        throw MotionFailure{MotionCommand::Result::ERROR_MOVE_REJECTED,
                            "Move 1008 timed out or returned nonzero", false};
      }
      next_request = now + refresh;
    }
    PublishFeedback(goal_handle, execution, 0.0, 0.0, 0.0, logger);
    std::this_thread::sleep_for(50ms);
  }
}

void MotionActionServer::RunRelativeYaw(
    const std::shared_ptr<GoalHandle> &goal_handle, ExecutionState &execution,
    const std::shared_ptr<GoalLogger> &logger) {
  const auto goal = goal_handle->get_goal();
  for (int frame = 0; frame < 5; ++frame) {
    CheckRuntimeSafety(goal_handle, logger);
    std::this_thread::sleep_for(50ms);
  }
  const double initial_yaw = state_monitor_->Snapshot().unwrapped_yaw;
  execution.initial_unwrapped_yaw = initial_yaw;
  execution.yaw_reference_initialized = true;
  const double target_rad = DegreesToRadians(goal->relative_yaw_deg);
  const double estimated = std::abs(target_rad) / goal->max_yaw_rate;
  const double timeout_sec = goal->timeout_sec > 0.0
                                 ? goal->timeout_sec
                                 : std::max(5.0, estimated * 3.0 + 3.0);
  YawControlParameters controller{parameters_.turn_kp,
                                  parameters_.turn_tolerance_deg,
                                  parameters_.turn_slow_zone_deg,
                                  parameters_.min_abs_yaw_rate_turn};
  const auto control_period =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(1.0 / parameters_.turn_control_hz));
  const auto publish_period =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(1.0 /
                                        parameters_.turn_publish_max_hz));
  auto next_publish = execution.started;
  double previous_error = target_rad;
  int stable = 0;

  while (true) {
    CheckRuntimeSafety(goal_handle, logger);
    const auto now = std::chrono::steady_clock::now();
    execution.elapsed_sec =
        std::chrono::duration<float>(now - execution.started).count();
    if (execution.elapsed_sec > timeout_sec) {
      throw MotionFailure{MotionCommand::Result::ERROR_TIMEOUT,
                          "relative yaw goal timed out", false};
    }
    const auto state = state_monitor_->Snapshot();
    const double current = state.unwrapped_yaw - initial_yaw;
    const double error = target_rad - current;
    const double current_deg = RadiansToDegrees(current);
    const double error_deg = RadiansToDegrees(error);
    execution.actual_relative_yaw_deg = current_deg;

    if (!parameters_.allow_reverse_correction &&
        ErrorCrossedTarget(previous_error, error)) {
      const double overshoot = std::abs(error_deg);
      if (overshoot <= parameters_.turn_tolerance_deg) break;
      throw MotionFailure{MotionCommand::Result::ERROR_TURN_OVERSHOOT,
                          overshoot <= parameters_.maximum_overshoot_deg
                              ? "turn crossed target outside tolerance"
                              : "turn exceeded maximum overshoot",
                          false};
    }

    const double logical_command =
        ComputeLogicalYawRate(error, goal->max_yaw_rate, controller);
    const double robot_command =
        parameters_.yaw_command_sign * logical_command;
    const double compensation_vx = ComputeTurnLongitudinalCompensation(
        error, parameters_.turn_tolerance_deg,
        parameters_.turn_compensation_taper_deg,
        parameters_.turn_longitudinal_compensation_vx,
        parameters_.turn_compensation_max_abs_vx);
    execution.last_turn_compensation_vx = compensation_vx;
    execution.peak_abs_turn_compensation_vx =
        std::max(execution.peak_abs_turn_compensation_vx,
                 std::abs(compensation_vx));
    const bool in_tolerance =
        std::abs(error_deg) <= parameters_.turn_tolerance_deg;
    if (in_tolerance) {
      if (state_monitor_->IsStationary(
              state, parameters_.stationary_max_abs_vx,
              parameters_.stationary_max_abs_vy,
              parameters_.stationary_max_abs_yaw_rate)) {
        if (++stable >= parameters_.turn_stable_samples) break;
      } else {
        stable = 0;
      }
    } else {
      stable = 0;
    }

    // Unitree Sport Move commands must be refreshed while the requested
    // velocity is unchanged. Publishing only when the P-controller output
    // changes lets a constant low-rate turn expire after its first pulse.
    if (now >= next_publish) {
      RequestResult response;
      if (in_tolerance) {
        // Keep the high-level wheel velocity controller engaged at zero while
        // yaw settles. Sending StopMove here released it immediately and was
        // observed as a small longitudinal rollback after otherwise clean
        // turns.
        response = sport_client_->SendMove(
            0.0, 0.0, 0.0,
            std::chrono::milliseconds(
                static_cast<int>(parameters_.response_timeout_sec * 1000.0)));
        execution.last_move_status = response.status_code;
      } else {
        response = sport_client_->SendMove(
            compensation_vx, 0.0, robot_command,
            std::chrono::milliseconds(
                static_cast<int>(parameters_.response_timeout_sec * 1000.0)));
        execution.last_move_status = response.status_code;
      }
      if (!response.response_received || response.status_code != 0) {
        throw MotionFailure{MotionCommand::Result::ERROR_MOVE_REJECTED,
                            "turn command timed out or returned nonzero", false};
      }
      next_publish = now + publish_period;
    }
    PublishFeedback(goal_handle, execution, goal->relative_yaw_deg, current_deg,
                    error_deg, logger);
    previous_error = error;
    std::this_thread::sleep_for(control_period);
  }

  if (parameters_.post_turn_rollback_control_sec > 0.0 &&
      state_monitor_->Snapshot().low_state_received) {
    logger->LogSafety("post_turn_rollback_control", "started");
    const auto rollback_started = std::chrono::steady_clock::now();
    auto next_rollback_publish = rollback_started;
    int rollback_stable = 0;
    execution.post_turn_rollback_peak_abs_dq = 0.0;
    execution.post_turn_rollback_peak_vx = 0.0;
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                         rollback_started)
               .count() < parameters_.post_turn_rollback_control_sec) {
      CheckRuntimeSafety(goal_handle, logger);
      const auto now = std::chrono::steady_clock::now();
      const auto state = state_monitor_->Snapshot();
      double mean_dq = 0.0;
      for (const double dq : state.wheel_dq) {
        mean_dq += dq;
      }
      mean_dq /= static_cast<double>(state.wheel_dq.size());
      const double abs_dq = std::abs(mean_dq);
      execution.post_turn_rollback_peak_abs_dq =
          std::max(execution.post_turn_rollback_peak_abs_dq, abs_dq);
      const double commanded =
          -parameters_.post_turn_rollback_gain * mean_dq *
          parameters_.wheel_radius_m;
      const double rollback_vx =
          std::clamp(commanded, -parameters_.post_turn_rollback_max_vx,
                     parameters_.post_turn_rollback_max_vx);
      execution.post_turn_rollback_peak_vx =
          std::max(execution.post_turn_rollback_peak_vx, std::abs(rollback_vx));
      if (abs_dq <= parameters_.post_turn_rollback_deadband_radps) {
        if (++rollback_stable >=
            parameters_.post_turn_rollback_stable_samples) {
          break;
        }
      } else {
        rollback_stable = 0;
      }
      if (now >= next_rollback_publish) {
        const auto response = sport_client_->SendMove(
            rollback_vx, 0.0, 0.0,
            std::chrono::milliseconds(
                static_cast<int>(parameters_.response_timeout_sec * 1000.0)));
        execution.last_move_status = response.status_code;
        if (!response.response_received || response.status_code != 0) {
          throw MotionFailure{MotionCommand::Result::ERROR_MOVE_REJECTED,
                              "post-turn rollback control command failed",
                              false};
        }
        next_rollback_publish = now + publish_period;
      }
      std::this_thread::sleep_for(control_period);
    }
    execution.post_turn_rollback_completed = true;
    logger->LogSafety("post_turn_rollback_control", "completed");
  } else if (parameters_.post_turn_rollback_control_sec > 0.0) {
    logger->LogSafety("post_turn_rollback_control",
                      "skipped: lowstate unavailable");
  }

  if (parameters_.post_turn_zero_velocity_hold_sec > 0.0) {
    logger->LogSafety("post_turn_zero_velocity_hold", "started");
    const auto hold_started = std::chrono::steady_clock::now();
    auto next_hold_publish = hold_started;
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                         hold_started)
               .count() < parameters_.post_turn_zero_velocity_hold_sec) {
      CheckRuntimeSafety(goal_handle, logger);
      const auto now = std::chrono::steady_clock::now();
      if (now >= next_hold_publish) {
        const auto response = sport_client_->SendMove(
            0.0, 0.0, 0.0,
            std::chrono::milliseconds(
                static_cast<int>(parameters_.response_timeout_sec * 1000.0)));
        execution.last_move_status = response.status_code;
        if (!response.response_received || response.status_code != 0) {
          throw MotionFailure{MotionCommand::Result::ERROR_MOVE_REJECTED,
                              "post-turn zero-velocity hold failed", false};
        }
        next_hold_publish = now + publish_period;
      }
      const auto state = state_monitor_->Snapshot();
      execution.elapsed_sec =
          std::chrono::duration<float>(now - execution.started).count();
      execution.actual_relative_yaw_deg = RadiansToDegrees(
          state.unwrapped_yaw - execution.initial_unwrapped_yaw);
      PublishFeedback(goal_handle, execution, goal->relative_yaw_deg,
                      execution.actual_relative_yaw_deg,
                      goal->relative_yaw_deg -
                          execution.actual_relative_yaw_deg,
                      logger);
      std::this_thread::sleep_for(control_period);
    }
    execution.post_turn_zero_velocity_hold_completed = true;
    logger->LogSafety("post_turn_zero_velocity_hold", "completed");
  }
}

void MotionActionServer::Execute(
    const std::shared_ptr<GoalHandle> goal_handle, const std::string &goal_id) {
  const auto goal = goal_handle->get_goal();
  nlohmann::json goal_json = {{"goal_id", goal_id},
                              {"mode", goal->mode},
                              {"vx", goal->vx},
                              {"vy", goal->vy},
                              {"yaw_rate", goal->yaw_rate},
                              {"duration_sec", goal->duration_sec},
                              {"relative_yaw_deg", goal->relative_yaw_deg},
                              {"max_yaw_rate", goal->max_yaw_rate},
                              {"timeout_sec", goal->timeout_sec},
                              {"dry_run", parameters_.dry_run}};
  auto logger =
      std::make_shared<GoalLogger>(session_log_directory_, goal_id, goal_json);
  {
    std::lock_guard<std::mutex> lock(logger_mutex_);
    active_logger_ = logger;
  }
  auto result = std::make_shared<MotionCommand::Result>();
  result->success = false;
  result->error_code = MotionCommand::Result::ERROR_INTERNAL;
  result->message = "uninitialized result";
  ExecutionState execution;
  execution.started = std::chrono::steady_clock::now();
  MotionFailure failure{MotionCommand::Result::ERROR_NONE, "", false};
  bool motion_completed = false;

  try {
    CheckRuntimeSafety(goal_handle, logger);
    if (!MotionModeFreshAndExpected()) {
      throw MotionFailure{MotionCommand::Result::ERROR_INVALID_GOAL,
                          "MotionSwitcher is not confirmed ai-w", false};
    }
    const auto initial_state = state_monitor_->Snapshot();
    if (!InitialRobotModeAllowed(initial_state.mode)) {
      throw MotionFailure{
          MotionCommand::Result::ERROR_INVALID_GOAL,
          "robot mode " + std::to_string(initial_state.mode) +
              " is not allowed as an initial motion state",
          false};
    }
    if (!StopAndVerify("pre_goal", &execution.last_stop_status, logger,
                       parameters_.pre_stop_count)) {
      throw MotionFailure{MotionCommand::Result::ERROR_STOP_FAILED,
                          "pre-goal STOP or stationary verification failed",
                          false};
    }
    state_monitor_->ResetDistance();
    state_monitor_->BeginEvidence();
    execution.started = std::chrono::steady_clock::now();
    if (goal->mode == MotionCommand::Goal::MODE_TIMED_VELOCITY) {
      RunTimedVelocity(goal_handle, execution, logger);
    } else {
      RunRelativeYaw(goal_handle, execution, logger);
    }
    motion_completed = true;
  } catch (const MotionFailure &caught) {
    failure = caught;
    logger->LogSafety("motion_failure", caught.message);
  } catch (const std::exception &caught) {
    failure = {MotionCommand::Result::ERROR_INTERNAL, caught.what(), false};
    logger->LogSafety("exception", caught.what());
  }

  const bool stopped =
      StopAndVerify("final", &execution.last_stop_status, logger);
  const auto final_state = state_monitor_->Snapshot();
  const auto evidence = state_monitor_->Evidence();
  execution.elapsed_sec = std::chrono::duration<float>(
                              std::chrono::steady_clock::now() -
                              execution.started)
                              .count();
  if (goal->mode == MotionCommand::Goal::MODE_RELATIVE_YAW &&
      execution.yaw_reference_initialized) {
    execution.actual_relative_yaw_deg = RadiansToDegrees(
        final_state.unwrapped_yaw - execution.initial_unwrapped_yaw);
  }

  if (!stopped && motion_completed) {
    failure = {MotionCommand::Result::ERROR_STATIONARY_VERIFY_FAILED,
               "final STOP response or stationary verification failed", false};
    motion_completed = false;
  } else if (!stopped && !failure.message.empty()) {
    failure.message += "; final stationary verification also failed";
  } else if (motion_completed &&
             goal->mode == MotionCommand::Goal::MODE_RELATIVE_YAW &&
             std::abs(goal->relative_yaw_deg -
                      execution.actual_relative_yaw_deg) >
                 parameters_.final_yaw_acceptance_deg) {
    failure = {MotionCommand::Result::ERROR_TURN_OVERSHOOT,
               "final relative yaw error exceeds acceptance limit", false};
    motion_completed = false;
  } else if (motion_completed && !parameters_.dry_run && !evidence.strong) {
    failure = {MotionCommand::Result::ERROR_MOVE_REJECTED,
               "RPC succeeded but strong wheel encoder motion was not observed",
               false};
    motion_completed = false;
  }

  result->success = motion_completed;
  result->error_code = motion_completed ? MotionCommand::Result::ERROR_NONE
                                        : failure.code;
  result->message = motion_completed ? "motion completed and robot stationary"
                                     : failure.message;
  result->elapsed_sec = execution.elapsed_sec;
  result->estimated_distance_m = state_monitor_->EstimatedDistance();
  result->actual_relative_yaw_deg = execution.actual_relative_yaw_deg;
  result->last_move_status_code = execution.last_move_status;
  result->last_stop_status_code = execution.last_stop_status;

  logger->LogResult({{"success", result->success},
                     {"error_code", result->error_code},
                     {"message", result->message},
                     {"elapsed_sec", result->elapsed_sec},
                     {"estimated_distance_m", result->estimated_distance_m},
                     {"actual_relative_yaw_deg",
                      result->actual_relative_yaw_deg},
                     {"last_move_status_code", result->last_move_status_code},
                     {"last_stop_status_code", result->last_stop_status_code},
                     {"configured_turn_longitudinal_compensation_vx",
                      parameters_.turn_longitudinal_compensation_vx},
                     {"last_turn_compensation_vx",
                      execution.last_turn_compensation_vx},
                     {"peak_abs_turn_compensation_vx",
                      execution.peak_abs_turn_compensation_vx},
                     {"post_turn_zero_velocity_hold_sec",
                      parameters_.post_turn_zero_velocity_hold_sec},
                     {"post_turn_zero_velocity_hold_completed",
                      execution.post_turn_zero_velocity_hold_completed},
                     {"post_turn_rollback_control_sec",
                      parameters_.post_turn_rollback_control_sec},
                     {"post_turn_rollback_deadband_radps",
                      parameters_.post_turn_rollback_deadband_radps},
                     {"post_turn_rollback_gain",
                      parameters_.post_turn_rollback_gain},
                     {"post_turn_rollback_max_vx",
                      parameters_.post_turn_rollback_max_vx},
                     {"wheel_radius_m", parameters_.wheel_radius_m},
                     {"post_turn_rollback_completed",
                      execution.post_turn_rollback_completed},
                     {"post_turn_rollback_peak_vx",
                      execution.post_turn_rollback_peak_vx},
                     {"post_turn_rollback_peak_abs_dq",
                      execution.post_turn_rollback_peak_abs_dq},
                     {"final_mode", final_state.mode},
                     {"final_error_code", final_state.error_code},
                     {"final_yaw_rate", final_state.yaw_rate},
                     {"require_low_state", parameters_.require_low_state},
                     {"low_state_received", final_state.low_state_received},
                     {"wheel_evidence_strong", evidence.strong},
                     {"wheel_sample_count", evidence.sample_count},
                     {"wheel_q_peak_to_peak", evidence.q_peak_to_peak},
                     {"wheel_dq_p95_abs", evidence.dq_p95_abs}});
  logger->LogProcessAudit("controller_pid=" + std::to_string(getpid()) +
                          " goal_thread_completed=true");

  if (goal_handle->is_canceling()) {
    goal_handle->canceled(result);
  } else if (result->success) {
    goal_handle->succeed(result);
  } else {
    goal_handle->abort(result);
  }
  active_goal_.store(false);
  goal_reserved_.store(false);
  stop_requested_.store(false);
  armed_activity_time_ = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(logger_mutex_);
    active_logger_.reset();
  }
}

}  // namespace go2w_motion_control
