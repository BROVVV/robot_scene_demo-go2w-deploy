// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GO2W_FOXY_BT__BT_ACTION_NODE_HPP_
#define GO2W_FOXY_BT__BT_ACTION_NODE_HPP_

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"

namespace go2w_foxy_bt
{
// Foxy lacks executor.add_callback_group. An owned IO node keeps callbacks
// progressing after halt/timeout, so a late accepted goal is still canceled.
class ActionIO
{
public:
  explicit ActionIO(rclcpp::Node::SharedPtr node) : node_(node)
  {
    executor_.add_node(node_);
    thread_ = std::thread([this]() {
      try {
        while (!stopping_ && rclcpp::ok()) {
          executor_.spin_once(std::chrono::milliseconds(20));
        }
      } catch (const std::exception & error) {
        if (rclcpp::ok() && !stopping_) {
          failed_ = true;
          RCLCPP_ERROR(node_->get_logger(), "Action IO failed: %s", error.what());
        }
      }
    });
  }
  ~ActionIO() {stop();}
  void stop()
  {
    stopping_ = true;
    try {executor_.cancel();}
    catch (const std::exception & error) {
      if (rclcpp::ok()) {
        failed_ = true;
        RCLCPP_ERROR(node_->get_logger(), "Action IO cancel failed: %s", error.what());
      }
    }
    if (thread_.joinable()) {thread_.join();}
  }
  bool failed() const {return failed_;}
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::atomic<bool> stopping_{false}, failed_{false};
  std::thread thread_;
};

template<class ActionT>
class BtActionNode : public BT::ActionNodeBase
{
  using Client = rclcpp_action::Client<ActionT>;
  using Handle = rclcpp_action::ClientGoalHandle<ActionT>;
  using WrappedResult = typename Handle::WrappedResult;
  using Feedback = typename ActionT::Feedback;
  struct Exchange
  {
    std::mutex mutex;
    bool acknowledged{false}, rejected{false}, terminal{false};
    bool cancel_requested{false}, cancel_sent{false};
    std::string error;
    typename Handle::SharedPtr handle;
    WrappedResult result;
    std::shared_ptr<const Feedback> feedback;
    std::chrono::steady_clock::time_point sent{std::chrono::steady_clock::now()};
  };
public:
  BtActionNode(const std::string & tag, const std::string & action,
    const BT::NodeConfiguration & conf)
  : BT::ActionNodeBase(tag, conf), action_name_(action)
  {
    node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");
    server_timeout_ = config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
    getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);
    if (server_timeout_.count() <= 0) {throw std::runtime_error("Invalid BT server timeout");}
    std::chrono::milliseconds discovery_timeout{1000};  // Pinned Nav2 default.
    config().blackboard->template get<std::chrono::milliseconds>("wait_for_service_timeout", discovery_timeout);
    static std::atomic<unsigned long> sequence{0};
    rclcpp::NodeOptions options; options.use_global_arguments(false);
    if (node_->has_parameter("use_sim_time")) {
      options.parameter_overrides({node_->get_parameter("use_sim_time")});
    }
    io_node_ = std::make_shared<rclcpp::Node>(node_->get_name()+std::string("_foxy_")+tag+
      "_"+std::to_string(++sequence), node_->get_namespace(), options);
    std::string remapped;
    if (getInput("server_name", remapped)) {action_name_ = remapped;}
    action_client_ = rclcpp_action::create_client<ActionT>(io_node_, action_name_);
    io_ = std::make_unique<ActionIO>(io_node_);
    if (!action_client_->wait_for_action_server(discovery_timeout)) {
      throw std::runtime_error("Action server not available: "+action_name_);
    }
    drain_timeout_ = discovery_timeout;
  }
  virtual ~BtActionNode()
  {
    cancel_all();
    // A canceled send may be waiting for its acceptance response. Keep IO
    // alive for the existing discovery bound, without blocking halt itself.
    const auto deadline = std::chrono::steady_clock::now()+drain_timeout_;
    while (rclcpp::ok() && std::chrono::steady_clock::now()<deadline) {
      bool pending = false;
      for (auto & exchange : exchanges_) {
        std::lock_guard<std::mutex> lock(exchange->mutex);
        pending = pending || (!exchange->acknowledged && !exchange->terminal);
      }
      if (!pending) {break;}
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    if (io_) {io_->stop();}
    action_client_.reset(); io_.reset(); io_node_.reset();
  }
  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {BT::InputPort<std::string>("server_name", "Action server name"),
      BT::InputPort<std::chrono::milliseconds>("server_timeout")};
    basic.insert(addition.begin(), addition.end()); return basic;
  }
  static BT::PortsList providedPorts() {return providedBasicPorts({});}
  virtual void on_tick() {}
  virtual void on_wait_for_result(std::shared_ptr<const Feedback>) {}
  virtual BT::NodeStatus on_success() {return BT::NodeStatus::SUCCESS;}
  virtual BT::NodeStatus on_aborted() {return BT::NodeStatus::FAILURE;}
  virtual BT::NodeStatus on_cancelled() {return BT::NodeStatus::SUCCESS;}

  BT::NodeStatus tick() override
  {
    if (!rclcpp::ok() || io_->failed()) {cancel_all(); return BT::NodeStatus::FAILURE;}
    if (status() == BT::NodeStatus::IDLE) {
      setStatus(BT::NodeStatus::RUNNING); should_send_goal_ = true; on_tick();
      if (!should_send_goal_) {return BT::NodeStatus::FAILURE;}
      send_new_goal();
    }
    auto exchange = current_;
    bool acknowledged, rejected, terminal, ack_expired;
    std::string error;
    std::shared_ptr<const Feedback> feedback;
    {
      std::lock_guard<std::mutex> lock(exchange->mutex);
      acknowledged = exchange->acknowledged; rejected = exchange->rejected;
      terminal = exchange->terminal; error = exchange->error;
      // Sample state and its deadline together. The IO thread may acknowledge
      // while the BT thread is descheduled; an earlier snapshot must not turn
      // that already acknowledged goal into a false transport timeout.
      ack_expired = !acknowledged && !terminal &&
        std::chrono::steady_clock::now()-exchange->sent >= server_timeout_;
      feedback = exchange->feedback; exchange->feedback.reset();
      if (terminal) {result_ = exchange->result;}
    }
    if (!error.empty() || rejected) {cancel_all(); return BT::NodeStatus::FAILURE;}
    if (!acknowledged && !terminal) {
      if (ack_expired) {
        RCLCPP_WARN(node_->get_logger(), "ACK deadline for %s; late cancellation latched", action_name_.c_str());
        cancel_all(); return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;
    }
    if (!terminal) {
      on_wait_for_result(feedback);
      if (goal_updated_) {goal_updated_ = false; send_new_goal();}
      return BT::NodeStatus::RUNNING;
    }
    switch (result_.code) {
      case rclcpp_action::ResultCode::SUCCEEDED: return on_success();
      case rclcpp_action::ResultCode::ABORTED: return on_aborted();
      case rclcpp_action::ResultCode::CANCELED: return on_cancelled();
      default: return BT::NodeStatus::FAILURE;
    }
  }
  void halt() override
  {
    cancel_all(); setStatus(BT::NodeStatus::IDLE);
  }
protected:
  static void cancel_exchange(const std::shared_ptr<Exchange> & exchange,
    const std::shared_ptr<Client> & client)
  {
    typename Handle::SharedPtr handle;
    {
      std::lock_guard<std::mutex> lock(exchange->mutex);
      exchange->cancel_requested = true;
      if (exchange->terminal || !exchange->handle || exchange->cancel_sent) {return;}
      exchange->cancel_sent = true; handle = exchange->handle;
    }
    try {client->async_cancel_goal(handle);}
    catch (const std::exception & error) {
      std::lock_guard<std::mutex> lock(exchange->mutex);
      exchange->cancel_sent = false; exchange->error = error.what();
      RCLCPP_ERROR(rclcpp::get_logger("go2w_foxy_bt"), "Cancel request failed: %s", error.what());
    }
  }
  void cancel_all()
  {
    if (!action_client_) {return;}
    for (auto & exchange : exchanges_) {cancel_exchange(exchange, action_client_);}
  }
  void send_new_goal()
  {
    exchanges_.erase(std::remove_if(exchanges_.begin(), exchanges_.end(), [](const auto & e) {
      std::lock_guard<std::mutex> lock(e->mutex); return e->terminal;
    }), exchanges_.end());
    auto exchange = std::make_shared<Exchange>();
    exchanges_.push_back(exchange); current_ = exchange;
    typename Client::SendGoalOptions options;
    std::weak_ptr<Client> weak_client = action_client_;
    const bool diagnostics = std::getenv("GO2W_BT_ACK_DIAGNOSTICS") != nullptr;
    const auto action = action_name_;
    options.goal_response_callback = [exchange, weak_client, diagnostics, action](std::shared_future<typename Handle::SharedPtr> future) {
      bool cancel = false;
      try {
        auto handle = future.get();
        {
          std::lock_guard<std::mutex> lock(exchange->mutex);
          exchange->acknowledged = true; exchange->rejected = !handle;
          if (!handle) {exchange->terminal = true;}
          if (!exchange->terminal) {exchange->handle = handle;}
          cancel = exchange->cancel_requested;
        }
        if (diagnostics) {
          const double ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now()-exchange->sent).count();
          RCLCPP_INFO(rclcpp::get_logger("go2w_foxy_bt"),
            "action=%s ack_ms=%.3f cancel_latched=%s", action.c_str(), ms, cancel ? "true" : "false");
        }
        if (cancel) {
          if (auto client = weak_client.lock()) {cancel_exchange(exchange, client);}
        }
      } catch (const std::exception & error) {
        std::lock_guard<std::mutex> lock(exchange->mutex);
        exchange->error = error.what(); exchange->terminal = true;
      }
    };
    options.result_callback = [exchange](const WrappedResult & result) {
      std::lock_guard<std::mutex> lock(exchange->mutex);
      exchange->result = result; exchange->terminal = true; exchange->handle.reset();
    };
    options.feedback_callback = [exchange](typename Handle::SharedPtr,
      const std::shared_ptr<const Feedback> feedback) {
      std::lock_guard<std::mutex> lock(exchange->mutex); exchange->feedback = feedback;
    };
    try {action_client_->async_send_goal(goal_, options);}
    catch (const std::exception & error) {
      std::lock_guard<std::mutex> lock(exchange->mutex);
      exchange->error = error.what(); exchange->terminal = true;
    }
  }
  void increment_recovery_count()
  {
    int count = 0; config().blackboard->template get<int>("number_recoveries", count);
    config().blackboard->template set<int>("number_recoveries", count+1);
  }
  std::string action_name_;
  rclcpp::Node::SharedPtr node_;
  typename ActionT::Goal goal_;
  WrappedResult result_;
  bool goal_updated_{false}, should_send_goal_{true};
private:
  rclcpp::Node::SharedPtr io_node_;
  std::shared_ptr<Client> action_client_;
  std::unique_ptr<ActionIO> io_;
  std::shared_ptr<Exchange> current_;
  std::vector<std::shared_ptr<Exchange>> exchanges_;
  std::chrono::milliseconds server_timeout_{20}, drain_timeout_{1000};
};
}  // namespace go2w_foxy_bt
#endif  // GO2W_FOXY_BT__BT_ACTION_NODE_HPP_
