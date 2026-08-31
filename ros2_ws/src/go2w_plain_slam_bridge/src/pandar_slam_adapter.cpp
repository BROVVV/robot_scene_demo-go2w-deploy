// Copyright 2026 robot_scene_demo maintainers
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
//
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// pandar_slam_adapter: /hesai/pandarxt16/points_raw -> /go2w/slam/pandar_points.
//
// Validates the PointCloud2 schema, filters NaN/Inf/zero/out-of-range points,
// resolves the per-point timestamp policy, and republishes the minimal plain_
// slam schema (x/y/z/intensity FLOAT32 + timestamp FLOAT64).  No coordinate
// rotation is applied here; extrinsics belong to the plain_slam parameters.
// The candidate Pandar transform is never published as a formal TF.

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/string.hpp>

#include <go2w_plain_slam_bridge/pointcloud_utils.hpp>
#include <go2w_plain_slam_bridge/timestamp_policy.hpp>

namespace go2w_plain_slam_bridge
{

inline float read_f32(const uint8_t * p)
{
  float value;
  std::memcpy(&value, p, sizeof(float));
  return value;
}

inline double read_f64(const uint8_t * p)
{
  double value;
  std::memcpy(&value, p, sizeof(double));
  return value;
}

class PandarSlamAdapter : public rclcpp::Node
{
public:
  PandarSlamAdapter()
  : Node("pandar_slam_adapter")
  {
    declare_parameter<std::string>("input_topic", "/hesai/pandarxt16/points_raw");
    declare_parameter<std::string>("output_topic", "/go2w/slam/pandar_points");
    declare_parameter<std::string>("point_status_topic", "/go2w/slam/point_status");
    declare_parameter<std::string>("expected_input_frame", "pandarxt16_link_unvalidated");
    declare_parameter<std::string>("output_frame", "pandarxt16_link_unvalidated");
    declare_parameter<double>("range_min_m", 0.30);
    declare_parameter<double>("range_max_m", 60.0);
    declare_parameter<bool>("drop_nan", true);
    declare_parameter<bool>("drop_inf", true);
    declare_parameter<std::string>("timestamp_mode", "auto");
    declare_parameter<double>("scan_period_s", 0.10);
    declare_parameter<double>("absolute_stamp_tolerance_s", 5.0);
    declare_parameter<std::string>("timestamp_fallback", "linear_scan");
    declare_parameter<bool>("self_filter_enabled", false);

    input_topic_ = get_parameter("input_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    point_status_topic_ = get_parameter("point_status_topic").as_string();
    expected_input_frame_ = get_parameter("expected_input_frame").as_string();
    output_frame_ = get_parameter("output_frame").as_string();
    range_min_m_ = get_parameter("range_min_m").as_double();
    range_max_m_ = get_parameter("range_max_m").as_double();
    drop_nan_ = get_parameter("drop_nan").as_bool();
    drop_inf_ = get_parameter("drop_inf").as_bool();
    scan_period_s_ = get_parameter("scan_period_s").as_double();
    absolute_tolerance_s_ = get_parameter("absolute_stamp_tolerance_s").as_double();

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, sensor_qos,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {on_cloud(msg);});
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, sensor_qos);
    status_publisher_ = create_publisher<std_msgs::msg::String>(
      point_status_topic_, rclcpp::QoS(10));

    RCLCPP_INFO(
      get_logger(), "pandar_slam_adapter: %s -> %s",
      input_topic_.c_str(), output_topic_.c_str());
  }

private:
  void publish_status(
    const std::string & mode,
    bool non_monotonic,
    std::size_t total,
    std::size_t dropped,
    const std::string & extra)
  {
    std_msgs::msg::String msg;
    msg.data = "timestamp_mode: " + mode +
      "; non_monotonic: " + (non_monotonic ? "1" : "0") +
      "; points_total: " + std::to_string(total) +
      "; points_dropped: " + std::to_string(dropped) +
      "; " + extra;
    status_publisher_->publish(msg);
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const CloudFieldOffsets offsets = find_field_offsets(*msg);
    if (!offsets.valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "input cloud schema incompatible (need x/y/z/intensity FLOAT32); "
        "type %s", msg->fields.empty() ? "?" : msg->fields[0].name.c_str());
      publish_status("SCHEMA_ERROR", false, 0, 0, "schema: incompatible");
      return;
    }
    if (!expected_input_frame_.empty() &&
      msg->header.frame_id != expected_input_frame_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "expected frame '%s' but cloud arrived in '%s' (continuing, "
        "mapping_assist best-effort)", expected_input_frame_.c_str(),
        msg->header.frame_id.c_str());
    }

    const std::size_t n_points =
      msg->width * msg->height * (msg->is_dense ? 1u : 1u);
    const std::size_t point_step = msg->point_step > 0 ? msg->point_step : 16u;
    const uint8_t * data = msg->data.data();
    const std::size_t data_size = msg->data.size();

    std::vector<double> raw_timestamps;
    raw_timestamps.reserve(n_points);
    std::vector<float> xs, ys, zs, ints;
    xs.reserve(n_points);
    ys.reserve(n_points);
    zs.reserve(n_points);
    ints.reserve(n_points);
    std::size_t dropped = 0;

    const double header_sec =
      static_cast<double>(msg->header.stamp.sec) +
      static_cast<double>(msg->header.stamp.nanosec) * 1e-9;

    for (std::size_t i = 0; i < n_points; ++i) {
      const std::size_t base = i * point_step;
      if (base + point_step > data_size) {
        break;
      }
      const float x = read_f32(data + base + static_cast<std::size_t>(offsets.x));
      const float y = read_f32(data + base + static_cast<std::size_t>(offsets.y));
      const float z = read_f32(data + base + static_cast<std::size_t>(offsets.z));
      const float intensity = offsets.intensity >= 0 ?
        read_f32(data + base + static_cast<std::size_t>(offsets.intensity)) : 0.0f;

      // Zero-return points (Hesai convention) are always dropped.
      if (x == 0.0f && y == 0.0f && z == 0.0f) {
        ++dropped;
        continue;
      }
      if (drop_nan_ &&
        (std::isnan(x) || std::isnan(y) || std::isnan(z) ||
        std::isnan(intensity)))
      {
        ++dropped;
        continue;
      }
      if (drop_inf_ &&
        (std::isinf(x) || std::isinf(y) || std::isinf(z) ||
        std::isinf(intensity)))
      {
        ++dropped;
        continue;
      }
      const double range = std::hypot(
        static_cast<double>(x), static_cast<double>(y));
      if (range < range_min_m_ || range > range_max_m_) {
        ++dropped;
        continue;
      }
      xs.push_back(x);
      ys.push_back(y);
      zs.push_back(z);
      ints.push_back(intensity);
      if (offsets.timestamp >= 0) {
        const double ts = read_f64(
          data + base + static_cast<std::size_t>(offsets.timestamp));
        raw_timestamps.push_back(ts);
      }
    }

    // Timestamp policy: even when per-point timestamps are missing we keep
    // the adapter functioning with an explicit SYNTHETIC flag.
    TimestampPolicyResult policy;
    if (offsets.timestamp < 0 || raw_timestamps.empty()) {
      policy.mode = TimestampMode::kSyntheticLinear;
    } else {
      policy = resolve_timestamps(
        raw_timestamps, header_sec, scan_period_s_, absolute_tolerance_s_);
    }
    if (policy.mode == TimestampMode::kError) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "timestamp classification failed for %zu points; not publishing "
        "(motion chain unaffected)", raw_timestamps.size());
      publish_status(
        "TIMESTAMP_ERROR", false, n_points, dropped,
        "schema: timestamp unclassifiable");
      return;
    }

    auto out = std::make_shared<sensor_msgs::msg::PointCloud2>();
    out->header = msg->header;
    out->header.frame_id = output_frame_;
    out->height = 1;
    out->width = xs.size();
    out->is_dense = true;
    out->is_bigendian = false;
    out->point_step = static_cast<uint32_t>(kOutputPointStep);
    out->row_step = out->point_step * out->width;
    build_output_cloud_fields(*out);
    out->data.resize(out->row_step);
    if (!xs.empty()) {
      uint8_t * out_data = out->data.data();
      for (std::size_t i = 0; i < xs.size(); ++i) {
        const std::size_t base = i * kOutputPointStep;
        std::memcpy(out_data + base, &xs[i], sizeof(float));
        std::memcpy(out_data + base + 4, &ys[i], sizeof(float));
        std::memcpy(out_data + base + 8, &zs[i], sizeof(float));
        std::memcpy(out_data + base + 12, &ints[i], sizeof(float));
        double ts = header_sec;
        if (policy.mode != TimestampMode::kSyntheticLinear ||
          !policy.timestamps.empty())
        {
          ts = policy.timestamps.empty() ? header_sec : policy.timestamps[i];
        }
        std::memcpy(out_data + base + 16, &ts, sizeof(double));
      }
    }
    publisher_->publish(*out);

    publish_status(
      timestamp_mode_name(policy.mode), policy.non_monotonic, n_points, dropped,
      "schema: compatible; output_points: " + std::to_string(xs.size()));
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string point_status_topic_;
  std::string expected_input_frame_;
  std::string output_frame_;
  double range_min_m_;
  double range_max_m_;
  bool drop_nan_;
  bool drop_inf_;
  double scan_period_s_;
  double absolute_tolerance_s_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
};

}  // namespace go2w_plain_slam_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<go2w_plain_slam_bridge::PandarSlamAdapter>());
  rclcpp::shutdown();
  return 0;
}
