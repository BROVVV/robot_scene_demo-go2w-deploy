#pragma once

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "go2w_motion_control/angle_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

namespace go2w_motion_control {

struct MotionStateSnapshot {
  bool sport_state_received{false};
  bool low_state_received{false};
  std::chrono::steady_clock::time_point sport_receive_time{};
  std::chrono::steady_clock::time_point low_receive_time{};
  uint64_t sport_sequence{0};
  uint64_t low_sequence{0};
  uint32_t error_code{0};
  uint8_t mode{0};
  double velocity_x{0.0};
  double velocity_y{0.0};
  double velocity_z{0.0};
  double yaw_rate{0.0};
  double raw_yaw{0.0};
  double unwrapped_yaw{0.0};
  double position_x{0.0};
  double position_y{0.0};
  std::array<double, 4> wheel_q{};
  std::array<double, 4> wheel_dq{};
};

struct MotionEvidence {
  bool strong{false};
  size_t sample_count{0};
  std::array<double, 4> q_peak_to_peak{};
  std::array<double, 4> dq_p95_abs{};
};

class MotionStateMonitor {
 public:
  MotionStateMonitor(rclcpp::Node *node, const std::string &sport_topic,
                     const std::string &low_topic,
                     bool require_low_state = true,
                     double wheel_radius_m = 0.089);
  ~MotionStateMonitor();

  MotionStateSnapshot Snapshot() const;
  bool StateFresh(double timeout_sec) const;
  bool IsStationary(const MotionStateSnapshot &snapshot, double max_vx,
                    double max_vy, double max_yaw_rate) const;
  bool WaitForStationary(double timeout_sec, double state_timeout_sec,
                         int stable_samples, double max_vx, double max_vy,
                         double max_yaw_rate);
  bool WaitForFreshState(double timeout_sec, double state_timeout_sec);

  void ResetDistance();
  double EstimatedDistance() const;
  void BeginEvidence();
  MotionEvidence Evidence() const;

 private:
  void OnSportState(const unitree_go::msg::SportModeState::SharedPtr msg);
  void OnLowState(const unitree_go::msg::LowState::SharedPtr msg);

  mutable std::mutex mutex_;
  std::condition_variable changed_;
  MotionStateSnapshot state_;
  YawUnwrapper yaw_unwrapper_;
  bool distance_active_{false};
  double estimated_distance_{0.0};
  double wheel_radius_m_{0.089};
  std::chrono::steady_clock::time_point previous_distance_time_{};
  double previous_speed_{0.0};
  bool previous_wheel_q_valid_{false};
  std::array<double, 4> previous_wheel_q_{};
  bool evidence_active_{false};
  std::vector<std::array<double, 4>> evidence_q_;
  std::vector<std::array<double, 4>> evidence_dq_;
  std::vector<double> evidence_yaw_;
  std::vector<double> evidence_position_x_;
  std::vector<double> evidence_position_y_;
  std::vector<double> evidence_speed_;
  bool require_low_state_{true};
  rclcpp::Node::SharedPtr state_node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> state_executor_;
  std::thread state_thread_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sport_sub_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_sub_;
};

}  // namespace go2w_motion_control
