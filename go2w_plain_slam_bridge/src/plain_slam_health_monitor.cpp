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

// plain_slam_health_monitor: pipeline freshness state machine + readiness.
//
// Publishes:
//   /go2w/slam/health  diagnostic_msgs/DiagnosticArray  (state + degrade flags)
//   /go2w/slam/ready   std_msgs/Bool                    (mapping readiness only)
//
// ready=true means "the mapping-assist pipeline is producing data" and is
// NEVER motion/safety authorization: the health array always carries
// mode=MAPPING_ASSIST, motion_authorized=false, safety_authorized=false.
// A mapping failure must never disturb /go2w/odom/fused or any control node —
// this monitor only reports.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

namespace go2w_plain_slam_bridge
{

using SteadyClock = std::chrono::steady_clock;
using SteadyTime = std::chrono::steady_clock::time_point;

class PlainSlamHealthMonitor : public rclcpp::Node
{
public:
  PlainSlamHealthMonitor()
  : Node("plain_slam_health_monitor")
  {
    declare_parameter<std::string>("adapted_cloud_topic", "/go2w/slam/pandar_points");
    declare_parameter<std::string>("imu_topic", "/go2w/slam/imu");
    declare_parameter<std::string>("odom_topic", "/go2w/slam/odom_base");
    declare_parameter<std::string>("aligned_scan_topic", "/go2w/slam/aligned_scan");
    declare_parameter<std::string>("map_2d_topic", "/go2w/slam/map_2d");
    declare_parameter<std::string>("map_3d_topic", "/go2w/slam/map_3d");
    declare_parameter<std::string>("point_status_topic", "/go2w/slam/point_status");
    declare_parameter<std::string>("occupancy_status_topic", "/go2w/slam/occupancy_status");
    declare_parameter<std::string>("imu_status_topic", "/go2w/slam/imu_status");
    declare_parameter<std::string>("health_topic", "/go2w/slam/health");
    declare_parameter<std::string>("ready_topic", "/go2w/slam/ready");
    declare_parameter<double>("adapted_cloud_stale_s", 1.0);
    declare_parameter<double>("imu_stale_s", 0.2);
    declare_parameter<double>("odom_stale_s", 1.0);
    declare_parameter<double>("aligned_scan_stale_s", 1.0);
    declare_parameter<double>("map_2d_stale_s", 2.0);
    declare_parameter<double>("map_3d_stale_s", 5.0);
    declare_parameter<double>("publish_rate_hz", 2.0);
    declare_parameter<double>("imu_min_freq_hz", 50.0);
    declare_parameter<double>("imu_max_accel_m_s2", 50.0);
    declare_parameter<double>("imu_min_accel_m_s2", 0.5);
    declare_parameter<int>("imu_bad_frame_grace", 10);

    adapted_cloud_stale_s_ = get_parameter("adapted_cloud_stale_s").as_double();
    imu_stale_s_ = get_parameter("imu_stale_s").as_double();
    odom_stale_s_ = get_parameter("odom_stale_s").as_double();
    aligned_scan_stale_s_ = get_parameter("aligned_scan_stale_s").as_double();
    map_2d_stale_s_ = get_parameter("map_2d_stale_s").as_double();
    map_3d_stale_s_ = get_parameter("map_3d_stale_s").as_double();
    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
    imu_min_freq_hz_ = get_parameter("imu_min_freq_hz").as_double();
    imu_max_accel_ = get_parameter("imu_max_accel_m_s2").as_double();
    imu_min_accel_ = get_parameter("imu_min_accel_m_s2").as_double();
    imu_bad_frame_grace_ = get_parameter("imu_bad_frame_grace").as_int();

    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      get_parameter("adapted_cloud_topic").as_string(), qos,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr) {
        cloud_received_ = SteadyClock::now();
      });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      get_parameter("imu_topic").as_string(), qos,
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {on_imu(msg);});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("odom_topic").as_string(), qos,
      [this](const nav_msgs::msg::Odometry::SharedPtr) {
        odom_received_ = SteadyClock::now();
      });
    scan_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      get_parameter("aligned_scan_topic").as_string(), qos,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr) {
        scan_received_ = SteadyClock::now();
      });
    map2d_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      get_parameter("map_2d_topic").as_string(), qos,
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr) {
        map2d_received_ = SteadyClock::now();
      });
    if (!get_parameter("map_3d_topic").as_string().empty()) {
      map3d_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        get_parameter("map_3d_topic").as_string(), qos,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr) {
          map3d_received_ = SteadyClock::now();
        });
    }
    point_status_sub_ = create_subscription<std_msgs::msg::String>(
      get_parameter("point_status_topic").as_string(), qos,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        point_status_data_ = msg->data;
      });
    occupancy_status_sub_ = create_subscription<std_msgs::msg::String>(
      get_parameter("occupancy_status_topic").as_string(), qos,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        occupancy_status_data_ = msg->data;
      });
    imu_status_sub_ = create_subscription<std_msgs::msg::String>(
      get_parameter("imu_status_topic").as_string(), qos,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        imu_status_data_ = msg->data;
      });

    health_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      get_parameter("health_topic").as_string(), rclcpp::QoS(10));
    ready_pub_ = create_publisher<std_msgs::msg::Bool>(
      get_parameter("ready_topic").as_string(), rclcpp::QoS(10).transient_local());

    const double period = 1.0 / std::max(0.1, publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(period), [this]() {tick();});
  }

private:
  static double steady_seconds()
  {
    return std::chrono::duration<double>(
      SteadyClock::now().time_since_epoch()).count();
  }

  static bool received(const SteadyTime & last)
  {
    return last != SteadyTime{};
  }

  bool fresh(const SteadyTime & last, double stale_s) const
  {
    if (!received(last)) {
      return false;
    }
    const double age = std::chrono::duration<double>(SteadyClock::now() - last).count();
    return age <= stale_s;
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    imu_received_ = SteadyClock::now();
    const auto & a = msg->linear_acceleration;
    const double ax = a.x, ay = a.y, az = a.z;
    const bool finite_acc = std::isfinite(ax) && std::isfinite(ay) && std::isfinite(az);
    const double magnitude = std::sqrt(ax * ax + ay * ay + az * az);
    const bool plausible =
      finite_acc && magnitude >= imu_min_accel_ && magnitude <= imu_max_accel_;
    if (!plausible) {
      ++imu_bad_frames_;
    } else {
      imu_bad_frames_ = std::max(0, imu_bad_frames_ - 1);
    }
    const double now_s = steady_seconds();
    if (last_imu_sample_s_ > 0.0) {
      const double dt = now_s - last_imu_sample_s_;
      if (dt > 1e-4) {
        imu_instant_freq_hz_ = 1.0 / dt;
      }
    }
    last_imu_sample_s_ = now_s;
  }

  void tick()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();

    const bool cloud_ok = fresh(cloud_received_, adapted_cloud_stale_s_);
    const bool imu_ok = fresh(imu_received_, imu_stale_s_);
    const bool odom_ok = fresh(odom_received_, odom_stale_s_);
    const bool scan_ok = fresh(scan_received_, aligned_scan_stale_s_);
    const bool map2d_ok = fresh(map2d_received_, map_2d_stale_s_);
    const bool map3d_ok = fresh(map3d_received_, map_3d_stale_s_);

    const bool imu_weak =
      imu_bad_frames_ > imu_bad_frame_grace_ ||
      (imu_ok && imu_instant_freq_hz_ > 0.0 && imu_instant_freq_hz_ < imu_min_freq_hz_);

    const bool ready = cloud_ok && imu_ok && odom_ok && scan_ok && map2d_ok;

    std::string pipeline = "STARTING";
    if (ready) {
      pipeline = "READY_MAPPING_ASSIST";
    } else if (map2d_ok) {
      pipeline = "MAP2D_OK";
    } else if (map3d_ok) {
      pipeline = "MAP3D_OK";
    } else if (scan_ok) {
      pipeline = "LIO_OK";
    } else if (imu_ok) {
      pipeline = "IMU_OK";
    } else if (cloud_ok) {
      pipeline = "POINTCLOUD_OK";
    }

    // Degrade flags mirrored from the adapters + local detectors.
    std::string flags;
    if (point_status_data_.find("TIMESTAMP_SYNTHETIC") != std::string::npos) {
      flags += " TIMESTAMP_SYNTHETIC";
    }
    if (point_status_data_.find("CONVERTED_UNITS") != std::string::npos) {
      flags += " TIMESTAMP_CONVERTED";
    }
    if (point_status_data_.find("SCHEMA_ERROR") != std::string::npos) {
      flags += " POINTCLOUD_SCHEMA_ERROR";
    }
    if (occupancy_status_data_.find("FALLBACK") != std::string::npos) {
      flags += " GROUND_ESTIMATE_FALLBACK";
    } else if (!occupancy_status_data_.empty()) {
      flags += " GROUND_ESTIMATE_OK";
    }
    if (imu_weak) {
      flags += " IMU_DEGRADED";
    }
    if (imu_status_data_.find("SYNTHETIC_STATIC") != std::string::npos) {
      flags += " IMU_DEGRADED IMU_SYNTHETIC";
    }
    if (!odom_ok && received(odom_received_)) {
      flags += " LIO_STALE";
    }
    if (!map2d_ok && received(map2d_received_)) {
      flags += " MAP2D_STALE";
    }
    // The candidates are always flagged until officially calibrated.
    flags += " EXTRINSIC_CANDIDATE";

    auto add_status = [&array](const std::string & name, uint8_t level,
        const std::string & message) {
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = name;
        status.level = level;
        status.message = message;
        array.status.push_back(status);
      };

    const uint8_t level = ready ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    add_status(
      "/go2w/slam pipeline", level,
      pipeline + " | mode=MAPPING_ASSIST motion_authorized=false "
      "safety_authorized=false" + flags);
    add_status(
      "/go2w/slam pandar_points", cloud_ok ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      cloud_ok ? "POINTCLOUD_OK" : "POINTCLOUD_STALE");
    add_status(
      "/go2w/slam/imu", (imu_ok && !imu_weak) ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      std::string(imu_ok ? "IMU_OK" : "IMU_STALE") +
      (imu_weak ? " IMU_DEGRADED" : "") +
      (imu_status_data_.find("SYNTHETIC_STATIC") != std::string::npos ?
      " IMU_SYNTHETIC" : "") +
      (imu_instant_freq_hz_ > 0.0 ?
      " freq=" + std::to_string(static_cast<int>(std::round(imu_instant_freq_hz_))) + "Hz" :
      " freq=n/a"));
    add_status(
      "/go2w/slam odom_base", odom_ok ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      odom_ok ? "LIO_OK" : "LIO_STALE");
    add_status(
      "/go2w/slam aligned_scan", scan_ok ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      scan_ok ? "ALIGNED_SCAN_OK" : "ALIGNED_SCAN_STALE");
    add_status(
      "/go2w/slam map_3d", map3d_ok ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      map3d_ok ? "MAP3D_OK" : (received(map3d_received_) ?
      "MAP3D_STALE" : "MAP3D_NO_DATA"));
    add_status(
      "/go2w/slam map_2d", map2d_ok ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      map2d_ok ? "MAP2D_OK" : "MAP2D_STALE");
    add_status(
      "authorization", diagnostic_msgs::msg::DiagnosticStatus::OK,
      "mode=MAPPING_ASSIST motion_authorized=false safety_authorized=false "
      "pandar_extrinsic=candidate_unconfirmed");

    health_pub_->publish(array);

    std_msgs::msg::Bool ready_msg;
    ready_msg.data = ready;
    ready_pub_->publish(ready_msg);
  }

  double adapted_cloud_stale_s_;
  double imu_stale_s_;
  double odom_stale_s_;
  double aligned_scan_stale_s_;
  double map_2d_stale_s_;
  double map_3d_stale_s_;
  double publish_rate_hz_;
  double imu_min_freq_hz_;
  double imu_max_accel_;
  double imu_min_accel_;
  int imu_bad_frame_grace_;

  SteadyTime cloud_received_;
  SteadyTime imu_received_;
  SteadyTime odom_received_;
  SteadyTime scan_received_;
  SteadyTime map2d_received_;
  SteadyTime map3d_received_;
  double last_imu_sample_s_ = 0.0;
  double imu_instant_freq_hz_ = 0.0;
  int imu_bad_frames_ = 0;
  std::string point_status_data_;
  std::string occupancy_status_data_;
  std::string imu_status_data_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map2d_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map3d_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr point_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr occupancy_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr imu_status_sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr health_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace go2w_plain_slam_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<go2w_plain_slam_bridge::PlainSlamHealthMonitor>());
  rclcpp::shutdown();
  return 0;
}
