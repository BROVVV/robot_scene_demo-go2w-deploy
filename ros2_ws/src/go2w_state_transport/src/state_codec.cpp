#include "go2w_state_transport/state_codec.hpp"
#include "nlohmann/json.hpp"
#include <algorithm>
#include <cmath>
#include <cerrno>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <system_error>
#include <unistd.h>
#include <vector>

namespace go2w_state_transport {
using Json = nlohmann::json;
static double clock_seconds(clockid_t clock) {
  timespec t{};
  if (::clock_gettime(clock, &t) != 0) {throw std::system_error(errno, std::generic_category());}
  return static_cast<double>(t.tv_sec) + static_cast<double>(t.tv_nsec) * 1e-9;
}
double monotonic_now() {return clock_seconds(CLOCK_MONOTONIC);}
double wall_now() {return clock_seconds(CLOCK_REALTIME);}
std::string boot_identity() {
  std::ifstream stream("/proc/sys/kernel/random/boot_id");
  std::string result;
  std::getline(stream, result);
  if (result.empty()) {throw std::runtime_error("boot identity unavailable");}
  return result;
}
std::string encode(const LowSample & s) {
  return Json{{"schema", "go2w.network_low_state.v2"}, {"source_topic", s.topic},
    {"capture_monotonic", s.monotonic}, {"capture_wall_time", s.wall},
    {"capture_boot_id", s.boot_id}, {"motor_q", s.q}, {"motor_dq", s.dq},
    {"wheel_q", std::vector<float>(s.q.begin()+12, s.q.end())},
    {"wheel_dq", std::vector<float>(s.dq.begin()+12, s.dq.end())},
    {"imu_quaternion", s.quaternion}, {"imu_gyroscope", s.gyroscope},
    {"imu_accelerometer", s.accelerometer}, {"imu_rpy", s.rpy},
    {"imu_temperature", s.temperature}, {"battery_soc", s.soc}}.dump() + "\n";
}
template<std::size_t N>
static std::array<float, N> finite_array(const Json & object, const char * key) {
  const auto & a = object.at(key);
  if (!a.is_array() || a.size() != N) {throw std::invalid_argument(key);}
  std::array<float, N> result{};
  for (std::size_t i=0; i<N; ++i) {
    if (!a[i].is_number()) {throw std::invalid_argument(key);}
    const double value = a[i].get<double>();
    if (!std::isfinite(value) || std::abs(value)>std::numeric_limits<float>::max()) {
      throw std::invalid_argument(key);
    }
    result[i] = static_cast<float>(value);
  }
  return result;
}
std::optional<LowSample> decode(const std::string & text, double mono, double wall,
  double max_age, const std::string & boot)
{
  try {
    if (text.size()>65536 || boot.empty() || !std::isfinite(max_age) || max_age<=0) {return {};}
    const auto p = Json::parse(text);
    if (!p.at("capture_monotonic").is_number() || !p.at("capture_wall_time").is_number()) {
      return {};
    }
    LowSample s;
    s.monotonic = p.at("capture_monotonic").get<double>();
    s.wall = p.at("capture_wall_time").get<double>();
    const double ma = mono-s.monotonic, wa = wall-s.wall;
    if (!(0<=ma && ma<=max_age && 0<=wa && wa<=max_age)) {return {};}
    if (p.contains("capture_boot_id")) {
      if (!p["capture_boot_id"].is_string() || p["capture_boot_id"].get<std::string>()!=boot) {
        return {};
      }
    }
    s.boot_id = boot;
    if (p.contains("motor_q") || p.contains("motor_dq")) {
      s.q = finite_array<16>(p, "motor_q");
      s.dq = finite_array<16>(p, "motor_dq");
    } else {
      const auto q = finite_array<4>(p, "wheel_q"), dq = finite_array<4>(p, "wheel_dq");
      std::copy(q.begin(), q.end(), s.q.begin()+12);
      std::copy(dq.begin(), dq.end(), s.dq.begin()+12);
    }
    if (p.contains("imu_quaternion")) {s.quaternion=finite_array<4>(p,"imu_quaternion");}
    if (p.contains("imu_gyroscope")) {s.gyroscope=finite_array<3>(p,"imu_gyroscope");}
    if (p.contains("imu_accelerometer")) {s.accelerometer=finite_array<3>(p,"imu_accelerometer");}
    if (p.contains("imu_rpy")) {s.rpy=finite_array<3>(p,"imu_rpy");}
    if (p.contains("imu_temperature")) {
      if (!p["imu_temperature"].is_number_integer()) {return {};}
      const double value=p["imu_temperature"].get<double>();
      if (value < -128 || value > 127) {return {};}
      s.temperature=p["imu_temperature"].get<int>();
    }
    if (p.contains("battery_soc")) {
      if (!p["battery_soc"].is_number_integer()) {return {};}
      const double value=p["battery_soc"].get<double>();
      if (value < 0 || value > 255) {return {};}
      s.soc=p["battery_soc"].get<int>();
    }
    return s;
  } catch (const std::exception &) {return {};}
}
void atomic_write(const std::string & name, const std::string & text) {
  const auto parent=std::filesystem::path(name).parent_path();
  if (!parent.empty()) {std::filesystem::create_directories(parent);}
  std::string pattern=name+".XXXXXX";
  std::vector<char> temporary(pattern.begin(), pattern.end()); temporary.push_back('\0');
  int fd=::mkstemp(temporary.data());
  if (fd<0) {throw std::system_error(errno,std::generic_category());}
  try {
    std::size_t offset=0;
    while (offset<text.size()) {
      const auto n=::write(fd,text.data()+offset,text.size()-offset);
      if (n<0 && errno==EINTR) {continue;}
      if (n<=0) {throw std::system_error(errno,std::generic_category());}
      offset+=static_cast<std::size_t>(n);
    }
    if (::fsync(fd)!=0) {throw std::system_error(errno,std::generic_category());}
    if (::close(fd)!=0) {fd=-1; throw std::system_error(errno,std::generic_category());}
    fd=-1;
    if (::rename(temporary.data(),name.c_str())!=0) {
      throw std::system_error(errno,std::generic_category());
    }
  } catch (...) {
    if (fd>=0) {::close(fd);}
    ::unlink(temporary.data()); throw;
  }
}
std::optional<std::string> bounded_read(const std::string & name) {
  std::ifstream stream(name,std::ios::binary);
  if (!stream) {return {};}
  std::string text(65537,'\0');
  stream.read(text.data(),static_cast<std::streamsize>(text.size()));
  text.resize(static_cast<std::size_t>(stream.gcount()));
  if (text.size()>65536 || stream.bad()) {return {};}
  return text;
}
}  // namespace go2w_state_transport
