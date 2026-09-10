#pragma once

#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>

#include "go2w_motion_control/leased_sport_client.hpp"
#include "go2w_motion_control/motion_state_monitor.hpp"
#include "nlohmann/json.hpp"

namespace go2w_motion_control {

class GoalLogger {
 public:
  GoalLogger(const std::filesystem::path &session_directory,
             const std::string &goal_id, const nlohmann::json &goal);
  ~GoalLogger();

  const std::filesystem::path &Directory() const { return directory_; }
  void LogRequestEvent(const std::string &kind, const RequestResult &result,
                       const std::string &parameter);
  void LogState(const MotionStateSnapshot &state, uint64_t lease_id);
  void LogFeedback(const nlohmann::json &feedback);
  void LogSafety(const std::string &event, const std::string &message);
  void LogResult(const nlohmann::json &result);
  void LogProcessAudit(const std::string &text);

 private:
  static std::string Timestamp();
  void WriteJsonLine(std::ofstream &stream, const nlohmann::json &value);

  std::filesystem::path directory_;
  std::mutex mutex_;
  std::ofstream requests_;
  std::ofstream responses_;
  std::ofstream sport_state_;
  std::ofstream low_state_;
  std::ofstream feedback_;
  std::ofstream safety_;
};

std::string WallTimestampForPath();

}  // namespace go2w_motion_control
