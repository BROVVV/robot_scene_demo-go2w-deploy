#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <map>
#include <limits>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_sport_client.h"
#include "unitree_api/msg/response.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

using namespace std::chrono_literals;

namespace {

constexpr const char *kSafetyConfirmation = "I_HAVE_CLEARED_THE_AREA";
std::atomic_bool interrupted{false};

struct Options {
  double vx = 0.05;
  double vy = 0.0;
  double vyaw = 0.0;
  double duration = 0.5;
  int64_t lease_id = 0;
  bool confirmed = false;
};

struct StateSample {
  std::chrono::steady_clock::time_point time;
  uint32_t error_code;
  uint8_t mode;
  std::array<float, 3> position;
  std::array<float, 3> velocity;
  float yaw_speed;
  std::array<float, 4> range_obstacle;
};

struct WheelSample {
  std::chrono::steady_clock::time_point time;
  std::array<float, 4> q;
  std::array<float, 4> dq;
};

void signal_handler(int) { interrupted.store(true); }

Options parse_args(int argc, char **argv) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string arg(argv[index]);
    auto value = [&]() -> std::string {
      if (index + 1 >= argc) {
        throw std::invalid_argument("missing value for " + arg);
      }
      return argv[++index];
    };
    if (arg == "--vx") {
      options.vx = std::stod(value());
    } else if (arg == "--vy") {
      options.vy = std::stod(value());
    } else if (arg == "--vyaw") {
      options.vyaw = std::stod(value());
    } else if (arg == "--duration") {
      options.duration = std::stod(value());
    } else if (arg == "--lease-id") {
      options.lease_id = std::stoll(value());
    } else if (arg == "--confirmed") {
      options.confirmed = true;
    } else {
      throw std::invalid_argument("unknown argument: " + arg);
    }
  }
  if (std::abs(options.vx) > 0.05) {
    throw std::invalid_argument("abs(vx) must be <= 0.05 m/s");
  }
  if (options.vy != 0.0) {
    throw std::invalid_argument("vy must be exactly 0 for the first recovery test");
  }
  if (std::abs(options.vyaw) > 0.08) {
    throw std::invalid_argument("abs(vyaw) must be <= 0.08 rad/s");
  }
  if (options.duration < 0.05 || options.duration > 0.6) {
    throw std::invalid_argument("duration must be between 0.05 and 0.6 seconds");
  }
  if (options.vx == 0.0 && options.vyaw == 0.0) {
    throw std::invalid_argument("at least one of vx or vyaw must be non-zero");
  }
  if (options.lease_id <= 0) {
    throw std::invalid_argument("a positive --lease-id is required");
  }
  return options;
}

nlohmann::json state_json(const char *event, const StateSample &sample) {
  return {
      {"event", event},
      {"error_code", sample.error_code},
      {"mode", sample.mode},
      {"position", sample.position},
      {"velocity", sample.velocity},
      {"yaw_speed", sample.yaw_speed},
      {"range_obstacle", sample.range_obstacle},
  };
}

bool stationary(const StateSample &sample) {
  return std::abs(sample.velocity[0]) < 0.02 &&
         std::abs(sample.velocity[1]) < 0.02 &&
         std::abs(sample.yaw_speed) < 0.03;
}

int64_t request_id() {
  static std::atomic<int64_t> sequence{0};
  const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                       std::chrono::system_clock::now().time_since_epoch())
                       .count();
  return now + sequence.fetch_add(1);
}

}  // namespace

int main(int argc, char **argv) {
  Options options;
  try {
    options = parse_args(argc, argv);
  } catch (const std::exception &error) {
    std::cerr << nlohmann::json({{"event", "argument_error"},
                                 {"message", error.what()}})
                      .dump()
              << std::endl;
    return 2;
  }

  if (!options.confirmed) {
    std::cout << "平整地面和周围 2 米已清空，遥控器可立即急停。\nType "
              << kSafetyConfirmation << ": " << std::flush;
    std::string confirmation;
    std::getline(std::cin, confirmation);
    if (confirmation != kSafetyConfirmation) {
      std::cerr << "safety confirmation rejected" << std::endl;
      return 2;
    }
  }

  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("safe_go2w_ros2_move");
  SportClient sport(node.get());

  std::mutex mutex;
  std::condition_variable changed;
  std::vector<StateSample> states;
  std::vector<WheelSample> wheel_states;
  std::map<int64_t, int32_t> responses;
  std::map<int64_t, int64_t> response_api_ids;

  auto state_sub = node->create_subscription<unitree_go::msg::SportModeState>(
      "/lf/sportmodestate", 10,
      [&](const unitree_go::msg::SportModeState::SharedPtr msg) {
        StateSample sample{
            std::chrono::steady_clock::now(), msg->error_code, msg->mode,
            msg->position, msg->velocity, msg->yaw_speed, msg->range_obstacle};
        {
          std::lock_guard<std::mutex> lock(mutex);
          states.push_back(sample);
        }
        changed.notify_all();
      });

  auto response_sub = node->create_subscription<unitree_api::msg::Response>(
      "/api/sport/response", 20,
      [&](const unitree_api::msg::Response::SharedPtr msg) {
        {
          std::lock_guard<std::mutex> lock(mutex);
          responses[msg->header.identity.id] = msg->header.status.code;
          response_api_ids[msg->header.identity.id] =
              msg->header.identity.api_id;
        }
        std::cout
            << nlohmann::json({
                   {"event", "api_response"},
                   {"request_id", msg->header.identity.id},
                   {"api_id", msg->header.identity.api_id},
                   {"status_code", msg->header.status.code},
                   {"data", msg->data},
               })
                   .dump()
            << std::endl;
        changed.notify_all();
      });

  auto low_state_sub = node->create_subscription<unitree_go::msg::LowState>(
      "/lf/lowstate", 20,
      [&](const unitree_go::msg::LowState::SharedPtr msg) {
        WheelSample sample;
        sample.time = std::chrono::steady_clock::now();
        for (size_t index = 0; index < 4; ++index) {
          sample.q[index] = msg->motor_state[index + 12].q;
          sample.dq[index] = msg->motor_state[index + 12].dq;
        }
        {
          std::lock_guard<std::mutex> lock(mutex);
          wheel_states.push_back(sample);
        }
        changed.notify_all();
      });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&]() { executor.spin(); });

  auto latest_state = [&](double timeout_seconds) -> StateSample {
    std::unique_lock<std::mutex> lock(mutex);
    const bool available = changed.wait_for(
        lock, std::chrono::duration<double>(timeout_seconds),
        [&]() { return !states.empty(); });
    if (!available) {
      throw std::runtime_error("no SportModeState received");
    }
    return states.back();
  };

  auto wait_response = [&](int64_t id, double timeout_seconds)
      -> std::pair<bool, int32_t> {
    std::unique_lock<std::mutex> lock(mutex);
    const bool received = changed.wait_for(
        lock, std::chrono::duration<double>(timeout_seconds),
        [&]() { return responses.find(id) != responses.end(); });
    return received ? std::make_pair(true, responses.at(id))
                    : std::make_pair(false, 0);
  };

  auto make_request = [&](int64_t api_id) {
    unitree_api::msg::Request request;
    request.header.identity.id = request_id();
    request.header.identity.api_id = api_id;
    request.header.lease.id = options.lease_id;
    return request;
  };

  int exit_code = 1;
  bool move_sent = false;
  std::chrono::steady_clock::time_point move_start;
  StateSample initial{};
  WheelSample initial_wheels{};
  std::vector<int32_t> stop_codes;
  try {
    initial = latest_state(5.0);
    std::cout << state_json("state_before", initial).dump() << std::endl;
    if (!stationary(initial)) {
      throw std::runtime_error("robot is not stationary before movement");
    }
    {
      std::unique_lock<std::mutex> lock(mutex);
      const bool wheels_available = changed.wait_for(
          lock, 5s, [&]() { return !wheel_states.empty(); });
      if (!wheels_available) {
        throw std::runtime_error("no /lf/lowstate wheel encoder sample");
      }
      initial_wheels = wheel_states.back();
    }
    std::cout << nlohmann::json({
                     {"event", "wheels_before"},
                     {"q", initial_wheels.q},
                     {"dq", initial_wheels.dq},
                 }).dump()
              << std::endl;

    auto move_request = make_request(ROBOT_SPORT_API_ID_MOVE);
    std::cout << nlohmann::json({
                     {"event", "move_request"},
                     {"request_id", move_request.header.identity.id},
                     {"api_id", move_request.header.identity.api_id},
                     {"lease_id", move_request.header.lease.id},
                     {"parameter", {{"x", options.vx},
                                    {"y", options.vy},
                                    {"z", options.vyaw}}},
                     {"duration", options.duration},
                 }).dump()
              << std::endl;
    move_start = std::chrono::steady_clock::now();
    sport.Move(move_request, options.vx, options.vy, options.vyaw);
    move_sent = true;

    const auto deadline = move_start + std::chrono::duration<double>(options.duration);
    while (std::chrono::steady_clock::now() < deadline && !interrupted.load()) {
      std::this_thread::sleep_for(10ms);
    }
  } catch (const std::exception &error) {
    std::cerr << nlohmann::json({{"event", "error"},
                                 {"message", error.what()}})
                      .dump()
              << std::endl;
  }

  for (int attempt = 1; attempt <= 3; ++attempt) {
    auto stop_request = make_request(ROBOT_SPORT_API_ID_STOPMOVE);
    sport.StopMove(stop_request);
    auto [received, code] = wait_response(stop_request.header.identity.id, 2.0);
    std::cout << nlohmann::json({
                     {"event", "stop_verification"},
                     {"attempt", attempt},
                     {"request_id", stop_request.header.identity.id},
                     {"api_id", stop_request.header.identity.api_id},
                     {"response_received", received},
                     {"status_code", received ? nlohmann::json(code)
                                               : nlohmann::json(nullptr)},
                 }).dump()
              << std::endl;
    stop_codes.push_back(received ? code : -9999);
    std::this_thread::sleep_for(100ms);
  }

  if (move_sent) {
    std::this_thread::sleep_for(2s);
    StateSample final_state{};
    WheelSample final_wheels{};
    std::vector<StateSample> movement_states;
    std::vector<WheelSample> movement_wheels;
    {
      std::lock_guard<std::mutex> lock(mutex);
      final_state = states.back();
      final_wheels = wheel_states.back();
      for (const auto &sample : states) {
        if (sample.time >= move_start) {
          movement_states.push_back(sample);
        }
      }
      for (const auto &sample : wheel_states) {
        if (sample.time >= move_start) {
          movement_wheels.push_back(sample);
        }
      }
    }
    std::cout << state_json("state_after", final_state).dump() << std::endl;
    double peak_vx = 0.0;
    double peak_yaw = 0.0;
    std::set<int> modes;
    for (const auto &sample : movement_states) {
      peak_vx = std::max(peak_vx, std::abs(static_cast<double>(sample.velocity[0])));
      peak_yaw = std::max(peak_yaw, std::abs(static_cast<double>(sample.yaw_speed)));
      modes.insert(sample.mode);
    }
    const bool state_changed = modes.count(3) != 0 || peak_vx > 0.02 || peak_yaw > 0.03;
    double peak_wheel_dq = 0.0;
    std::array<double, 4> wheel_q_min;
    std::array<double, 4> wheel_q_max;
    std::array<std::vector<double>, 4> wheel_abs_dq;
    wheel_q_min.fill(std::numeric_limits<double>::infinity());
    wheel_q_max.fill(-std::numeric_limits<double>::infinity());
    for (const auto &sample : movement_wheels) {
      for (size_t index = 0; index < 4; ++index) {
        const auto value = sample.dq[index];
        peak_wheel_dq = std::max(peak_wheel_dq,
                                 std::abs(static_cast<double>(value)));
        wheel_q_min[index] =
            std::min(wheel_q_min[index], static_cast<double>(sample.q[index]));
        wheel_q_max[index] =
            std::max(wheel_q_max[index], static_cast<double>(sample.q[index]));
        wheel_abs_dq[index].push_back(std::abs(static_cast<double>(value)));
      }
    }
    std::array<double, 4> wheel_q_peak_to_peak{};
    std::array<double, 4> wheel_dq_p95_abs{};
    for (size_t index = 0; index < 4; ++index) {
      if (!wheel_abs_dq[index].empty()) {
        wheel_q_peak_to_peak[index] = wheel_q_max[index] - wheel_q_min[index];
        std::sort(wheel_abs_dq[index].begin(), wheel_abs_dq[index].end());
        const size_t percentile_index = std::min(
            wheel_abs_dq[index].size() - 1,
            static_cast<size_t>(
                std::ceil(0.95 * wheel_abs_dq[index].size()) - 1));
        wheel_dq_p95_abs[index] = wheel_abs_dq[index][percentile_index];
      }
    }
    const std::array<double, 4> wheel_q_delta{
        final_wheels.q[0] - initial_wheels.q[0],
        final_wheels.q[1] - initial_wheels.q[1],
        final_wheels.q[2] - initial_wheels.q[2],
        final_wheels.q[3] - initial_wheels.q[3]};
    const auto q_activity = std::count_if(
        wheel_q_peak_to_peak.begin(), wheel_q_peak_to_peak.end(),
        [](double value) { return value > 0.03; });
    const auto dq_activity = std::count_if(
        wheel_dq_p95_abs.begin(), wheel_dq_p95_abs.end(),
        [](double value) { return value > 0.12; });
    const bool wheel_motion = q_activity >= 3 && dq_activity >= 3;
    const bool wheels_stopped =
        std::all_of(final_wheels.dq.begin(), final_wheels.dq.end(),
                    [](float value) { return std::abs(value) < 0.1f; });
    std::cout << nlohmann::json({
                     {"event", "wheels_after"},
                     {"q", final_wheels.q},
                     {"dq", final_wheels.dq},
                 }).dump()
              << std::endl;
    const bool stops_ok = stop_codes.size() == 3 &&
                          std::all_of(stop_codes.begin(), stop_codes.end(),
                                      [](int32_t code) { return code == 0; });
    const bool stopped = stationary(final_state);
    const std::array<double, 3> position_delta{
        final_state.position[0] - initial.position[0],
        final_state.position[1] - initial.position[1],
        final_state.position[2] - initial.position[2]};
    std::cout << nlohmann::json({
                     {"event", "verification"},
                     {"motion_state_changed", state_changed},
                     {"wheel_motion_observed", wheel_motion},
                     {"peak_abs_wheel_dq", peak_wheel_dq},
                     {"wheel_dq_p95_abs", wheel_dq_p95_abs},
                     {"wheel_q_peak_to_peak", wheel_q_peak_to_peak},
                     {"wheel_q_delta", wheel_q_delta},
                     {"wheels_stopped_after_stop", wheels_stopped},
                     {"peak_abs_vx", peak_vx},
                     {"peak_abs_yaw_speed", peak_yaw},
                     {"modes", modes},
                     {"position_delta", position_delta},
                     {"stop_returns_ok", stops_ok},
                     {"stationary_after_stop", stopped},
                     {"sample_count", movement_states.size()},
                     {"wheel_sample_count", movement_wheels.size()},
                 }).dump()
              << std::endl;
    exit_code = state_changed && wheel_motion && stops_ok && stopped &&
                        wheels_stopped
                    ? 0
                    : 1;
  }

  std::cout << nlohmann::json({
                   {"event", "result"},
                   {"ros2_move_test", exit_code == 0 ? "PASS" : "FAIL"},
               }).dump()
            << std::endl;

  executor.cancel();
  spin_thread.join();
  executor.remove_node(node);
  response_sub.reset();
  state_sub.reset();
  low_state_sub.reset();
  rclcpp::shutdown();
  return exit_code;
}
