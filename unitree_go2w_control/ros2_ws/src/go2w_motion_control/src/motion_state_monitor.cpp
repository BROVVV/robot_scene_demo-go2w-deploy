#include "go2w_motion_control/motion_state_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <thread>

using namespace std::chrono_literals;

namespace go2w_motion_control {

MotionStateMonitor::MotionStateMonitor(rclcpp::Node *node,
                                       const std::string &sport_topic,
                                       const std::string &low_topic,
                                       bool require_low_state,
                                       double wheel_radius_m)
    : wheel_radius_m_(wheel_radius_m), require_low_state_(require_low_state) {
  // Keep high-rate state subscriptions off the Action/Service executor.  The
  // latter is deliberately single-threaded to avoid a Foxy rclcpp_action
  // readiness race.  A dedicated state-only executor lets a blocking safety
  // service continue collecting the fresh samples required by
  // WaitForStationary without reintroducing concurrent Action dispatch.
  rclcpp::NodeOptions state_options;
  // The launch file remaps __node for the public Action node.  Do not apply
  // that process-wide remap to this private state node or the graph will
  // contain two nodes with the same name.
  state_options.use_global_arguments(false);
  state_node_ = std::make_shared<rclcpp::Node>(
      "go2w_motion_state_monitor", node->get_namespace(), state_options);
  // The Go2-W bare DDS publishers expose RELIABLE state streams on this
  // deployment.  BEST_EFFORT readers discover the topics but can remain
  // silent, which would make the motion gate incorrectly report stale state.
  const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(20)).reliable();
  sport_sub_ = state_node_->create_subscription<unitree_go::msg::SportModeState>(
      sport_topic, state_qos,
      [this](const unitree_go::msg::SportModeState::SharedPtr msg) {
        OnSportState(msg);
      });
  low_sub_ = state_node_->create_subscription<unitree_go::msg::LowState>(
      low_topic, state_qos,
      [this](const unitree_go::msg::LowState::SharedPtr msg) {
        OnLowState(msg);
      });
  state_executor_ =
      std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  state_executor_->add_node(state_node_);
  state_thread_ = std::thread([this]() { state_executor_->spin(); });
}

MotionStateMonitor::~MotionStateMonitor() {
  if (state_executor_) state_executor_->cancel();
  if (state_thread_.joinable()) state_thread_.join();
  if (state_executor_ && state_node_) state_executor_->remove_node(state_node_);
  sport_sub_.reset();
  low_sub_.reset();
  state_executor_.reset();
  state_node_.reset();
}

void MotionStateMonitor::OnSportState(
    const unitree_go::msg::SportModeState::SharedPtr msg) {
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex_);
  const double raw_yaw = static_cast<double>(msg->imu_state.rpy[2]);
  state_.sport_state_received = true;
  state_.sport_receive_time = now;
  ++state_.sport_sequence;
  state_.error_code = msg->error_code;
  state_.mode = msg->mode;
  state_.velocity_x = msg->velocity[0];
  state_.velocity_y = msg->velocity[1];
  state_.velocity_z = msg->velocity[2];
  state_.yaw_rate = msg->yaw_speed;
  state_.raw_yaw = raw_yaw;
  state_.unwrapped_yaw = yaw_unwrapper_.Update(raw_yaw);
  state_.position_x = static_cast<double>(msg->position[0]);
  state_.position_y = static_cast<double>(msg->position[1]);
  if (state_.sport_sequence == 1) {
    RCLCPP_INFO(state_node_->get_logger(),
                "received SportModeState: mode=%u error_code=%u",
                static_cast<unsigned>(state_.mode), state_.error_code);
  }
  if (evidence_active_) {
    evidence_yaw_.push_back(state_.unwrapped_yaw);
    evidence_position_x_.push_back(state_.position_x);
    evidence_position_y_.push_back(state_.position_y);
    evidence_speed_.push_back(
        std::hypot(state_.velocity_x, state_.velocity_y));
  }

  if (distance_active_) {
    const double speed = std::hypot(state_.velocity_x, state_.velocity_y);
    if (previous_distance_time_.time_since_epoch().count() != 0) {
      const double dt =
          std::chrono::duration<double>(now - previous_distance_time_).count();
      if (dt > 0.0 && dt < 0.5) {
        estimated_distance_ += 0.5 * (previous_speed_ + speed) * dt;
      }
    }
    previous_distance_time_ = now;
    previous_speed_ = speed;
  }
  changed_.notify_all();
}

void MotionStateMonitor::OnLowState(
    const unitree_go::msg::LowState::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  state_.low_state_received = true;
  state_.low_receive_time = std::chrono::steady_clock::now();
  ++state_.low_sequence;
  if (state_.low_sequence == 1) {
    RCLCPP_INFO(state_node_->get_logger(), "received LowState");
  }
  std::array<double, 4> q{};
  std::array<double, 4> dq{};
  for (size_t index = 0; index < 4; ++index) {
    q[index] = msg->motor_state[index + 12].q;
    dq[index] = msg->motor_state[index + 12].dq;
  }
  state_.wheel_q = q;
  state_.wheel_dq = dq;
  if (distance_active_) {
    if (previous_wheel_q_valid_) {
      double mean_delta = 0.0;
      for (size_t index = 0; index < q.size(); ++index) {
        mean_delta += q[index] - previous_wheel_q_[index];
      }
      mean_delta /= static_cast<double>(q.size());
      // SportModeState.velocity is zero on this Go2-W firmware even while
      // Move(1008) is active.  Once LowState is relayed locally, use the
      // four-wheel encoder increment as the action's distance estimate.
      if (std::isfinite(mean_delta)) {
        estimated_distance_ += std::abs(mean_delta) * wheel_radius_m_;
      }
    }
    previous_wheel_q_ = q;
    previous_wheel_q_valid_ = true;
  }
  if (evidence_active_) {
    evidence_q_.push_back(q);
    evidence_dq_.push_back(dq);
  }
  changed_.notify_all();
}

MotionStateSnapshot MotionStateMonitor::Snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

bool MotionStateMonitor::StateFresh(double timeout_sec) const {
  const auto snapshot = Snapshot();
  if (!snapshot.sport_state_received ||
      (require_low_state_ && !snapshot.low_state_received)) {
    return false;
  }
  const auto now = std::chrono::steady_clock::now();
  if (std::chrono::duration<double>(now - snapshot.sport_receive_time)
          .count() > timeout_sec) {
    return false;
  }
  return !require_low_state_ ||
         (snapshot.low_state_received &&
          std::chrono::duration<double>(now - snapshot.low_receive_time)
                  .count() <= timeout_sec);
}

bool MotionStateMonitor::IsStationary(const MotionStateSnapshot &snapshot,
                                      double max_vx, double max_vy,
                                      double max_yaw_rate) const {
  return std::abs(snapshot.velocity_x) < max_vx &&
         std::abs(snapshot.velocity_y) < max_vy &&
         std::abs(snapshot.yaw_rate) < max_yaw_rate &&
         (!require_low_state_ ||
          std::all_of(snapshot.wheel_dq.begin(), snapshot.wheel_dq.end(),
                      [](double value) { return std::abs(value) < 0.2; }));
}

bool MotionStateMonitor::WaitForFreshState(double timeout_sec,
                                           double state_timeout_sec) {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(timeout_sec);
  std::unique_lock<std::mutex> lock(mutex_);
  while (std::chrono::steady_clock::now() < deadline) {
    const auto now = std::chrono::steady_clock::now();
    if (state_.sport_state_received &&
        (!require_low_state_ || state_.low_state_received) &&
        std::chrono::duration<double>(now - state_.sport_receive_time).count() <=
            state_timeout_sec &&
        (!require_low_state_ ||
         std::chrono::duration<double>(now - state_.low_receive_time).count() <=
             state_timeout_sec)) {
      return true;
    }
    changed_.wait_for(lock, 50ms);
  }
  return false;
}

bool MotionStateMonitor::WaitForStationary(
    double timeout_sec, double state_timeout_sec, int stable_samples,
    double max_vx, double max_vy, double max_yaw_rate) {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(timeout_sec);
  uint64_t previous_sequence = 0;
  int stable = 0;
  std::unique_lock<std::mutex> lock(mutex_);
  while (std::chrono::steady_clock::now() < deadline) {
    changed_.wait_for(lock, 50ms, [&]() {
      return state_.sport_sequence != previous_sequence;
    });
    if (state_.sport_sequence == previous_sequence) {
      continue;
    }
    previous_sequence = state_.sport_sequence;
    const auto now = std::chrono::steady_clock::now();
    const bool fresh = state_.sport_state_received &&
        (!require_low_state_ || state_.low_state_received) &&
        std::chrono::duration<double>(now - state_.sport_receive_time).count() <=
            state_timeout_sec &&
        (!state_.low_state_received ||
         std::chrono::duration<double>(now - state_.low_receive_time).count() <=
             state_timeout_sec);
    const bool stationary = std::abs(state_.velocity_x) < max_vx &&
                            std::abs(state_.velocity_y) < max_vy &&
                            std::abs(state_.yaw_rate) < max_yaw_rate &&
                            (!require_low_state_ ||
                             std::all_of(
                                 state_.wheel_dq.begin(), state_.wheel_dq.end(),
                                 [](double value) {
                                   return std::abs(value) < 0.2;
                                 }));
    if (fresh && stationary) {
      if (++stable >= stable_samples) {
        return true;
      }
    } else {
      stable = 0;
    }
  }
  return false;
}

void MotionStateMonitor::ResetDistance() {
  std::lock_guard<std::mutex> lock(mutex_);
  estimated_distance_ = 0.0;
  previous_distance_time_ = {};
  previous_speed_ = 0.0;
  previous_wheel_q_valid_ = false;
  previous_wheel_q_.fill(0.0);
  distance_active_ = true;
}

double MotionStateMonitor::EstimatedDistance() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return estimated_distance_;
}

void MotionStateMonitor::BeginEvidence() {
  std::lock_guard<std::mutex> lock(mutex_);
  evidence_q_.clear();
  evidence_dq_.clear();
  evidence_yaw_.clear();
  evidence_position_x_.clear();
  evidence_position_y_.clear();
  evidence_speed_.clear();
  evidence_active_ = true;
}

MotionEvidence MotionStateMonitor::Evidence() const {
  std::lock_guard<std::mutex> lock(mutex_);
  MotionEvidence result;
  result.sample_count = evidence_q_.size();
  if (!require_low_state_ || evidence_q_.empty() || evidence_dq_.empty()) {
    // Go2-W publishes SportModeState but, unlike the legged Go2, does not
    // provide a usable /lf/lowstate stream.  Keep motion evidence fail-safe:
    // require an observable yaw/position change and/or non-zero reported
    // speed from the authoritative sport state instead of accepting RPC
    // success alone.
    result.sample_count = evidence_yaw_.size();
    if (evidence_yaw_.size() < 2) return result;
    const auto yaw_bounds = std::minmax_element(evidence_yaw_.begin(),
                                                evidence_yaw_.end());
    const auto x_bounds = std::minmax_element(evidence_position_x_.begin(),
                                              evidence_position_x_.end());
    const auto y_bounds = std::minmax_element(evidence_position_y_.begin(),
                                              evidence_position_y_.end());
    const double yaw_range = *yaw_bounds.second - *yaw_bounds.first;
    const double position_range = std::hypot(
        *x_bounds.second - *x_bounds.first,
        *y_bounds.second - *y_bounds.first);
    const double max_speed = evidence_speed_.empty()
                                 ? 0.0
                                 : *std::max_element(evidence_speed_.begin(),
                                                     evidence_speed_.end());
    result.strong = yaw_range >= 0.15 || position_range >= 0.05 ||
                    (max_speed >= 0.03 && evidence_speed_.size() >= 3);
    return result;
  }
  std::array<double, 4> minimum{};
  std::array<double, 4> maximum{};
  minimum.fill(std::numeric_limits<double>::infinity());
  maximum.fill(-std::numeric_limits<double>::infinity());
  std::array<std::vector<double>, 4> absolute_dq;
  for (size_t sample = 0; sample < evidence_q_.size(); ++sample) {
    for (size_t wheel = 0; wheel < 4; ++wheel) {
      minimum[wheel] = std::min(minimum[wheel], evidence_q_[sample][wheel]);
      maximum[wheel] = std::max(maximum[wheel], evidence_q_[sample][wheel]);
      absolute_dq[wheel].push_back(std::abs(evidence_dq_[sample][wheel]));
    }
  }
  int q_active = 0;
  int dq_active = 0;
  for (size_t wheel = 0; wheel < 4; ++wheel) {
    result.q_peak_to_peak[wheel] = maximum[wheel] - minimum[wheel];
    std::sort(absolute_dq[wheel].begin(), absolute_dq[wheel].end());
    const size_t index = std::min(
        absolute_dq[wheel].size() - 1,
        static_cast<size_t>(std::ceil(0.95 * absolute_dq[wheel].size()) - 1));
    result.dq_p95_abs[wheel] = absolute_dq[wheel][index];
    q_active += result.q_peak_to_peak[wheel] > 0.03 ? 1 : 0;
    dq_active += result.dq_p95_abs[wheel] > 0.12 ? 1 : 0;
  }
  result.strong = q_active >= 3 && dq_active >= 3;
  return result;
}

}  // namespace go2w_motion_control
