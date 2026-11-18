#pragma once
#include <cmath>
#include <optional>
#include <string>

namespace go2w_motion_control {
struct StreamVelocity { double vx, vy, yaw; };
// Called under the server's stream mutex. Bad, replayed or foreign updates
// never refresh the local receive watchdog. No update can start an action.
class VelocityStream {
 public:
  void reset(const std::string &token) { token_=token; command_.reset(); stamp_=0; }
  bool update(const std::string &token, StreamVelocity v, double stamp,
              double ros_now, double received) {
    if (token_.empty() || token!=token_ || !std::isfinite(stamp) ||
        !std::isfinite(ros_now) || !std::isfinite(received) ||
        !std::isfinite(v.vx) || !std::isfinite(v.vy) || !std::isfinite(v.yaw) ||
        stamp<=stamp_ || ros_now-stamp<-.05 || ros_now-stamp>.20 ||
        std::abs(v.vx)>.300001 || v.vy!=0 || std::abs(v.yaw)>.200001) return false;
    stamp_=stamp; received_=received; command_=v; return true;
  }
  std::optional<StreamVelocity> current(double received_now, double ttl) const {
    if (!command_ || !std::isfinite(received_now) || !std::isfinite(ttl) ||
        ttl<=0 || ttl>.25 || received_now-received_<0 || received_now-received_>=ttl) return {};
    return command_;
  }
 private:
  std::string token_;
  std::optional<StreamVelocity> command_;
  double stamp_{0}, received_{0};
};
}  // namespace go2w_motion_control
