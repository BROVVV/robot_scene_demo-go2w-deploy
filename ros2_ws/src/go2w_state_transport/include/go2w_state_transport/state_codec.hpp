#pragma once
#include <array>
#include <optional>
#include <string>

namespace go2w_state_transport {
struct LowSample {
  double monotonic = 0., wall = 0.;
  std::string boot_id, topic = "/lf/lowstate";
  std::array<float, 16> q{}, dq{};
  std::array<float, 4> quaternion{};
  std::array<float, 3> gyroscope{}, accelerometer{}, rpy{};
  int temperature = 0, soc = 0;
};
double monotonic_now();
double wall_now();
std::string boot_identity();
std::string encode(const LowSample & sample);
std::optional<LowSample> decode(const std::string & text, double now_monotonic,
  double now_wall, double max_age, const std::string & boot);
void atomic_write(const std::string & path, const std::string & text);
std::optional<std::string> bounded_read(const std::string & path);
}  // namespace go2w_state_transport
