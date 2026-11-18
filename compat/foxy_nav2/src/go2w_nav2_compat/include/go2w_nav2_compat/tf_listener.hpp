// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <atomic>
#include <chrono>
#include <cstdint>
#include <exception>
#include <memory>
#include <string>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace go2w_foxy_nav2 {
// Foxy's unbounded executor spin can miss cancel during rapid startup and
// leave a destructor blocked in join. Each wait here is bounded independently
// of cancellation, including cancellation before the new thread first runs.
class TransformListener {
public:
  explicit TransformListener(tf2_ros::Buffer & buffer) : buffer_(buffer) {
    auto options = rclcpp::NodeOptions().use_global_arguments(false);
    node_ = std::make_shared<rclcpp::Node>(
      "go2w_foxy_tf_listener_" + std::to_string(reinterpret_cast<std::uintptr_t>(this)), options);
    listener_ = std::make_unique<tf2_ros::TransformListener>(buffer_, node_, false);
    executor_.add_node(node_);
    buffer_.setUsingDedicatedThread(true);
    thread_ = std::thread([this]() {
      try {
        while (rclcpp::ok() && !stopping_.load()) {
          executor_.spin_once(std::chrono::milliseconds(20));
        }
      } catch (const std::exception & error) {
        if (rclcpp::ok() && !stopping_.load()) {
          // Do not leave collision processing alive with a dead TF input
          // thread. Preserve the original thread-failure process exit so
          // downstream command freshness can detect the failed component.
          RCLCPP_FATAL(node_->get_logger(), "TF executor failed: %s", error.what());
          std::terminate();
        }
      }
    });
  }
  ~TransformListener() {
    stopping_.store(true);
    executor_.cancel();
    if (thread_.joinable()) {thread_.join();}
    executor_.remove_node(node_);
    listener_.reset();
    buffer_.setUsingDedicatedThread(false);
  }
  TransformListener(const TransformListener &) = delete;
  TransformListener & operator=(const TransformListener &) = delete;
private:
  tf2_ros::Buffer & buffer_;
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::atomic<bool> stopping_{false};
  std::thread thread_;
};
}  // namespace go2w_foxy_nav2
