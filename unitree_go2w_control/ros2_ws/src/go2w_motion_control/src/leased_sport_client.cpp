#include "go2w_motion_control/leased_sport_client.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <thread>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "nlohmann/json.hpp"

namespace go2w_motion_control {
namespace {
constexpr int64_t kMoveApi = 1008;
constexpr int64_t kStopApi = 1003;
constexpr std::size_t kMaxResponseBytes = 64 * 1024;

class FileDescriptor {
 public:
  explicit FileDescriptor(int value) : value_(value) {}
  ~FileDescriptor() {
    if (value_ >= 0) close(value_);
  }
  FileDescriptor(const FileDescriptor &) = delete;
  FileDescriptor &operator=(const FileDescriptor &) = delete;
  int get() const { return value_; }

 private:
  int value_;
};

timeval TimeoutValue(std::chrono::milliseconds timeout) {
  timeval value{};
  value.tv_sec = timeout.count() / 1000;
  value.tv_usec = (timeout.count() % 1000) * 1000;
  if (value.tv_sec == 0 && value.tv_usec == 0) value.tv_usec = 1000;
  return value;
}

void SendAll(int socket_fd, const std::string &payload) {
  std::size_t offset = 0;
  while (offset < payload.size()) {
    const ssize_t written =
        send(socket_fd, payload.data() + offset, payload.size() - offset,
             MSG_NOSIGNAL);
    if (written < 0) {
      if (errno == EINTR) continue;
      throw std::runtime_error(std::string("SDK executor send failed: ") +
                               std::strerror(errno));
    }
    if (written == 0) throw std::runtime_error("SDK executor closed during send");
    offset += static_cast<std::size_t>(written);
  }
}

std::string ReceiveLine(int socket_fd) {
  std::string response;
  response.reserve(1024);
  char buffer[4096];
  while (response.size() <= kMaxResponseBytes) {
    const ssize_t received = recv(socket_fd, buffer, sizeof(buffer), 0);
    if (received < 0) {
      if (errno == EINTR) continue;
      throw std::runtime_error(std::string("SDK executor receive failed: ") +
                               std::strerror(errno));
    }
    if (received == 0) break;
    response.append(buffer, static_cast<std::size_t>(received));
    const auto newline = response.find('\n');
    if (newline != std::string::npos) {
      response.resize(newline);
      return response;
    }
  }
  if (response.size() > kMaxResponseBytes) {
    throw std::runtime_error("SDK executor response exceeds size limit");
  }
  if (response.empty()) throw std::runtime_error("SDK executor returned no response");
  return response;
}
}  // namespace

LeasedSportClient::LeasedSportClient(
    rclcpp::Node *node, const std::string & /*request_topic*/,
    const std::string & /*response_topic*/, const std::string &lease_id_topic,
    const std::string &lease_alive_topic,
    const std::string &sdk_command_socket, double lease_status_timeout_sec,
    bool dry_run)
    : node_(node),
      sdk_command_socket_(sdk_command_socket),
      lease_status_timeout_sec_(lease_status_timeout_sec),
      dry_run_(dry_run) {
  const auto lease_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  lease_id_sub_ = node_->create_subscription<std_msgs::msg::UInt64>(
      lease_id_topic, lease_qos,
      [this](const std_msgs::msg::UInt64::SharedPtr msg) {
        lease_id_.store(msg->data);
        std::lock_guard<std::mutex> lock(lease_mutex_);
        lease_update_time_ = std::chrono::steady_clock::now();
      });
  lease_alive_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      lease_alive_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        lease_alive_.store(msg->data);
        std::lock_guard<std::mutex> lock(lease_mutex_);
        lease_update_time_ = std::chrono::steady_clock::now();
      });
}

int64_t LeasedSportClient::NextRequestId() {
  const auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
                               std::chrono::system_clock::now().time_since_epoch())
                               .count();
  int64_t previous = last_request_id_.load();
  int64_t candidate = 0;
  do {
    candidate = std::max<int64_t>(nanoseconds, previous + 1);
  } while (!last_request_id_.compare_exchange_weak(previous, candidate));
  return candidate;
}

RequestResult LeasedSportClient::SendRequest(
    int64_t api_id, const std::string &parameter,
    std::chrono::milliseconds timeout) {
  RequestResult result;
  result.request_id = NextRequestId();
  result.api_id = api_id;
  result.lease_id = CurrentLeaseId();

  EventCallback callback;
  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    callback = event_callback_;
  }
  if (callback) callback("request", result, parameter);

  if (dry_run_.load()) {
    result.response_received = true;
    result.status_code = 0;
    result.response_data = "{\"dry_run\":true,\"transport\":\"sdk_direct\"}";
    if (callback) callback("response", result, parameter);
    return result;
  }

  if (!LeaseAvailable() || result.lease_id == 0) {
    result.status_code = -9998;
    result.response_data = "{\"error\":\"lease unavailable\"}";
    if (callback) callback("response", result, parameter);
    return result;
  }

  const auto started = std::chrono::steady_clock::now();
  try {
    std::lock_guard<std::mutex> command_lock(command_mutex_);
    FileDescriptor socket_fd(socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0));
    if (socket_fd.get() < 0) {
      throw std::runtime_error(std::string("cannot create SDK executor socket: ") +
                               std::strerror(errno));
    }
    const timeval timeout_value = TimeoutValue(timeout);
    if (setsockopt(socket_fd.get(), SOL_SOCKET, SO_RCVTIMEO, &timeout_value,
                   sizeof(timeout_value)) != 0 ||
        setsockopt(socket_fd.get(), SOL_SOCKET, SO_SNDTIMEO, &timeout_value,
                   sizeof(timeout_value)) != 0) {
      throw std::runtime_error(std::string("cannot set SDK socket timeout: ") +
                               std::strerror(errno));
    }
    sockaddr_un address{};
    if (sdk_command_socket_.size() >= sizeof(address.sun_path)) {
      throw std::runtime_error("SDK executor socket path is too long");
    }
    address.sun_family = AF_UNIX;
    std::strncpy(address.sun_path, sdk_command_socket_.c_str(),
                 sizeof(address.sun_path) - 1);
    if (connect(socket_fd.get(), reinterpret_cast<sockaddr *>(&address),
                sizeof(address)) != 0) {
      throw std::runtime_error(std::string("cannot connect SDK executor: ") +
                               std::strerror(errno));
    }

    nlohmann::json parameter_json;
    try {
      parameter_json = nlohmann::json::parse(parameter);
    } catch (const nlohmann::json::exception &) {
      parameter_json = parameter;
    }
    const std::string request =
        nlohmann::json({{"request_id", result.request_id},
                        {"api_id", api_id},
                        {"parameter", parameter_json}})
            .dump() +
        "\n";
    SendAll(socket_fd.get(), request);
    result.published = true;
    result.response_data = ReceiveLine(socket_fd.get());
    const auto response = nlohmann::json::parse(result.response_data);
    if (response.at("request_id").get<int64_t>() != result.request_id ||
        response.at("api_id").get<int64_t>() != api_id) {
      throw std::runtime_error("SDK executor response identity mismatch");
    }
    result.status_code = response.at("status_code").get<int32_t>();
    result.lease_id = response.value("lease_id", result.lease_id);
    result.response_received = true;
  } catch (const std::exception &caught) {
    result.status_code = -9997;
    result.response_data =
        nlohmann::json({{"error", caught.what()},
                        {"transport", "sdk_direct"}})
            .dump();
  }
  result.round_trip_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                               started)
          .count();
  if (callback) callback("response", result, parameter);
  return result;
}

RequestResult LeasedSportClient::SendMove(
    double vx, double vy, double yaw_rate, std::chrono::milliseconds timeout) {
  nlohmann::json parameter = {{"x", vx}, {"y", vy}, {"z", yaw_rate}};
  return SendRequest(kMoveApi, parameter.dump(), timeout);
}

RequestResult LeasedSportClient::SendStopMove(
    std::chrono::milliseconds timeout) {
  return SendRequest(kStopApi, "{}", timeout);
}

bool LeasedSportClient::StopRepeatedly(
    int count, std::chrono::milliseconds interval,
    std::chrono::milliseconds response_timeout, int32_t *last_status) {
  bool all_ok = true;
  int32_t status = -9999;
  for (int attempt = 0; attempt < count; ++attempt) {
    const auto result = SendStopMove(response_timeout);
    status = result.status_code;
    all_ok = all_ok && result.response_received && result.status_code == 0;
    if (attempt + 1 < count) std::this_thread::sleep_for(interval);
  }
  if (last_status != nullptr) *last_status = status;
  return all_ok;
}

bool LeasedSportClient::LeaseAvailable() const {
  if (!lease_alive_.load() || lease_id_.load() == 0) return false;
  std::lock_guard<std::mutex> lock(lease_mutex_);
  if (lease_update_time_.time_since_epoch().count() == 0) return false;
  return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                       lease_update_time_)
             .count() <= lease_status_timeout_sec_;
}

uint64_t LeasedSportClient::CurrentLeaseId() const { return lease_id_.load(); }

void LeasedSportClient::SetEventCallback(EventCallback callback) {
  std::lock_guard<std::mutex> lock(callback_mutex_);
  event_callback_ = std::move(callback);
}

void LeasedSportClient::SetDryRun(bool dry_run) { dry_run_.store(dry_run); }

}  // namespace go2w_motion_control
