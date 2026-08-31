#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int64.hpp"

namespace go2w_motion_control {

struct RequestResult {
  bool response_received{false};
  bool published{false};
  int32_t status_code{-9999};
  int64_t request_id{0};
  int64_t api_id{0};
  uint64_t lease_id{0};
  std::string response_data;
  double round_trip_ms{0.0};
};

class LeasedSportClient {
 public:
  using EventCallback = std::function<void(const std::string &kind,
                                           const RequestResult &result,
                                           const std::string &parameter)>;

  LeasedSportClient(rclcpp::Node *node, const std::string &request_topic,
                    const std::string &response_topic,
                    const std::string &lease_id_topic,
                    const std::string &lease_alive_topic,
                    const std::string &sdk_command_socket,
                    double lease_status_timeout_sec, bool dry_run);

  RequestResult SendMove(double vx, double vy, double yaw_rate,
                         std::chrono::milliseconds timeout);
  RequestResult SendStopMove(std::chrono::milliseconds timeout);
  bool StopRepeatedly(int count, std::chrono::milliseconds interval,
                      std::chrono::milliseconds response_timeout,
                      int32_t *last_status = nullptr);

  bool LeaseAvailable() const;
  uint64_t CurrentLeaseId() const;
  void SetEventCallback(EventCallback callback);
  void SetDryRun(bool dry_run);

 private:
  RequestResult SendRequest(int64_t api_id, const std::string &parameter,
                            std::chrono::milliseconds timeout);
  int64_t NextRequestId();

  rclcpp::Node *node_;
  std::string sdk_command_socket_;
  double lease_status_timeout_sec_;
  std::atomic_bool dry_run_;
  std::atomic<uint64_t> lease_id_{0};
  std::atomic_bool lease_alive_{false};
  mutable std::mutex lease_mutex_;
  std::chrono::steady_clock::time_point lease_update_time_{};
  std::atomic<int64_t> last_request_id_{0};
  std::mutex command_mutex_;
  mutable std::mutex callback_mutex_;
  EventCallback event_callback_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr lease_id_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lease_alive_sub_;
};

}  // namespace go2w_motion_control
