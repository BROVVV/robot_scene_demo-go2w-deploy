#include "go2w_motion_control/goal_logger.hpp"

#include <chrono>
#include <iomanip>
#include <sstream>

namespace go2w_motion_control {

std::string WallTimestampForPath() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t value = std::chrono::system_clock::to_time_t(now);
  std::tm local{};
  localtime_r(&value, &local);
  std::ostringstream output;
  output << std::put_time(&local, "%Y%m%d_%H%M%S");
  return output.str();
}

std::string GoalLogger::Timestamp() {
  const auto now = std::chrono::system_clock::now();
  const auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(
                                now.time_since_epoch())
                                .count();
  return std::to_string(microseconds);
}

GoalLogger::GoalLogger(const std::filesystem::path &session_directory,
                       const std::string &goal_id,
                       const nlohmann::json &goal) {
  directory_ = session_directory / ("goal_" + goal_id);
  std::filesystem::create_directories(directory_);
  {
    std::ofstream output(directory_ / "goal.json");
    output << goal.dump(2) << '\n';
  }
  requests_.open(directory_ / "requests.jsonl");
  responses_.open(directory_ / "responses.jsonl");
  sport_state_.open(directory_ / "sport_state.csv");
  low_state_.open(directory_ / "low_state.csv");
  feedback_.open(directory_ / "feedback.jsonl");
  safety_.open(directory_ / "safety_events.jsonl");
  sport_state_ << "timestamp,error_code,mode,vx,vy,yaw_rate,raw_yaw,unwrapped_yaw,lease_id\n";
  low_state_ << "timestamp,q0,q1,q2,q3,dq0,dq1,dq2,dq3,lease_id\n";
}

GoalLogger::~GoalLogger() {
  requests_.flush();
  responses_.flush();
  sport_state_.flush();
  low_state_.flush();
  feedback_.flush();
  safety_.flush();
}

void GoalLogger::WriteJsonLine(std::ofstream &stream,
                               const nlohmann::json &value) {
  stream << value.dump() << '\n';
  stream.flush();
}

void GoalLogger::LogRequestEvent(const std::string &kind,
                                 const RequestResult &result,
                                 const std::string &parameter) {
  std::lock_guard<std::mutex> lock(mutex_);
  nlohmann::json value = {{"timestamp", Timestamp()},
                          {"request_id", result.request_id},
                          {"api_id", result.api_id},
                          {"lease_id", result.lease_id},
                          {"parameter", parameter},
                          {"purpose", result.api_id == 1008 ? "Move" :
                                      (result.api_id == 1003 ? "StopMove" :
                                                               "unknown")}};
  if (kind == "request") {
    WriteJsonLine(requests_, value);
  } else {
    value["response_received"] = result.response_received;
    value["status_code"] = result.status_code;
    value["raw_response"] = result.response_data;
    value["round_trip_ms"] = result.round_trip_ms;
    WriteJsonLine(responses_, value);
  }
}

void GoalLogger::LogState(const MotionStateSnapshot &state,
                          uint64_t lease_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto timestamp = Timestamp();
  sport_state_ << timestamp << ',' << state.error_code << ','
               << static_cast<int>(state.mode) << ',' << state.velocity_x << ','
               << state.velocity_y << ',' << state.yaw_rate << ','
               << state.raw_yaw << ',' << state.unwrapped_yaw << ',' << lease_id
               << '\n';
  low_state_ << timestamp;
  for (double value : state.wheel_q) low_state_ << ',' << value;
  for (double value : state.wheel_dq) low_state_ << ',' << value;
  low_state_ << ',' << lease_id << '\n';
  sport_state_.flush();
  low_state_.flush();
}

void GoalLogger::LogFeedback(const nlohmann::json &feedback) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto value = feedback;
  value["timestamp"] = Timestamp();
  WriteJsonLine(feedback_, value);
}

void GoalLogger::LogSafety(const std::string &event,
                           const std::string &message) {
  std::lock_guard<std::mutex> lock(mutex_);
  WriteJsonLine(safety_, {{"timestamp", Timestamp()},
                          {"event", event},
                          {"message", message}});
}

void GoalLogger::LogResult(const nlohmann::json &result) {
  std::lock_guard<std::mutex> lock(mutex_);
  std::ofstream output(directory_ / "result.json");
  output << result.dump(2) << '\n';
}

void GoalLogger::LogProcessAudit(const std::string &text) {
  std::lock_guard<std::mutex> lock(mutex_);
  std::ofstream output(directory_ / "process_audit.txt");
  output << text << '\n';
}

}  // namespace go2w_motion_control
