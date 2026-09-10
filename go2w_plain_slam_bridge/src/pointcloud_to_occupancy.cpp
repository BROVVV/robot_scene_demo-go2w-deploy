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

// pointcloud_to_occupancy: /go2w/slam/aligned_scan + /go2w/slam/odom_base
//                         -> /go2w/slam/map_2d (real free/occupied/unknown).
//
// Every aligned scan is ray-traced in 2D: cells between the sensor and each
// endpoint become FREE, robot-height endpoints become OCCUPIED and untouched
// cells stay UNKNOWN.  Ground height is estimated automatically.  This map is
// consumed by FrontierExtractor / SemanticRoutePlanner as mapping*assist*
// input; it never feeds the motion safety chain.

#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <go2w_plain_slam_bridge/occupancy_grid.hpp>
#include <go2w_plain_slam_bridge/pointcloud_utils.hpp>
#include <go2w_plain_slam_bridge/transform_utils.hpp>

namespace go2w_plain_slam_bridge
{

inline float read_f32_p(const uint8_t * p)
{
  float value;
  std::memcpy(&value, p, sizeof(float));
  return value;
}

class PointcloudToOccupancy : public rclcpp::Node
{
public:
  PointcloudToOccupancy()
  : Node("pointcloud_to_occupancy")
  {
    declare_parameter<std::string>("scan_topic", "/go2w/slam/aligned_scan");
    declare_parameter<std::string>("odom_topic", "/go2w/slam/odom_base");
    declare_parameter<std::string>("map_topic", "/go2w/slam/map_2d");
    declare_parameter<std::string>("debug_cloud_topic", "/go2w/slam/map_debug_cloud");
    declare_parameter<bool>("publish_debug_cloud", false);
    declare_parameter<std::string>("occupancy_status_topic", "/go2w/slam/occupancy_status");
    declare_parameter<std::string>("map_frame", "pslam_odom");
    declare_parameter<std::string>("extrinsics_file", "");

    declare_parameter<double>("resolution_m", 0.10);
    declare_parameter<double>("width_m", 80.0);
    declare_parameter<double>("height_m", 80.0);
    declare_parameter<double>("origin_x_m", -40.0);
    declare_parameter<double>("origin_y_m", -40.0);
    declare_parameter<double>("publish_rate_hz", 2.0);
    declare_parameter<double>("max_scan_update_hz", 10.0);
    declare_parameter<double>("min_range_m", 0.35);
    declare_parameter<double>("max_range_m", 25.0);
    declare_parameter<std::string>("ground_mode", "auto");
    declare_parameter<double>("ground_search_radius_m", 3.0);
    declare_parameter<double>("ground_histogram_bin_m", 0.05);
    declare_parameter<double>("ground_update_interval_s", 1.0);
    declare_parameter<double>("ground_ema_alpha", 0.3);
    declare_parameter<double>("floor_tolerance_m", 0.08);
    declare_parameter<double>("obstacle_min_height_m", 0.10);
    declare_parameter<double>("obstacle_max_height_m", 1.60);
    declare_parameter<double>("log_odds_hit", 0.85);
    declare_parameter<double>("log_odds_miss", -0.40);
    declare_parameter<double>("log_odds_min", -2.0);
    declare_parameter<double>("log_odds_max", 3.5);
    declare_parameter<double>("free_probability_threshold", 0.35);
    declare_parameter<double>("occupied_probability_threshold", 0.65);
    declare_parameter<double>("angular_ray_bin_deg", 0.5);

    OccupancyParams params;
    params.resolution_m = get_parameter("resolution_m").as_double();
    params.width_m = get_parameter("width_m").as_double();
    params.height_m = get_parameter("height_m").as_double();
    params.origin_x_m = get_parameter("origin_x_m").as_double();
    params.origin_y_m = get_parameter("origin_y_m").as_double();
    params.min_range_m = get_parameter("min_range_m").as_double();
    params.max_range_m = get_parameter("max_range_m").as_double();
    params.obstacle_min_height_m = get_parameter("obstacle_min_height_m").as_double();
    params.obstacle_max_height_m = get_parameter("obstacle_max_height_m").as_double();
    params.log_odds_hit = get_parameter("log_odds_hit").as_double();
    params.log_odds_miss = get_parameter("log_odds_miss").as_double();
    params.log_odds_min = get_parameter("log_odds_min").as_double();
    params.log_odds_max = get_parameter("log_odds_max").as_double();
    params.free_probability_threshold = get_parameter("free_probability_threshold").as_double();
    params.occupied_probability_threshold =
      get_parameter("occupied_probability_threshold").as_double();
    params.angular_ray_bin_deg = get_parameter("angular_ray_bin_deg").as_double();
    grid_ = std::make_unique<OccupancyGrid2D>(params);

    scan_topic_ = get_parameter("scan_topic").as_string();
    odom_topic_ = get_parameter("odom_topic").as_string();
    map_topic_ = get_parameter("map_topic").as_string();
    occupancy_status_topic_ = get_parameter("occupancy_status_topic").as_string();
    map_frame_ = get_parameter("map_frame").as_string();
    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
    max_scan_update_hz_ = get_parameter("max_scan_update_hz").as_double();

    const std::string extrinsics_file = get_parameter("extrinsics_file").as_string();
    if (!load_pandar_candidate_xy(extrinsics_file, pandar_offset_x_, pandar_offset_y_)) {
      RCLCPP_WARN(
        get_logger(),
        "candidate extrinsics file '%s' not readable; using sensor offset "
        "(0,0) (mapping_assist best-effort)", extrinsics_file.c_str());
    }

    ground_estimator_ = std::make_unique<GroundEstimator>(
      get_parameter("ground_search_radius_m").as_double(),
      get_parameter("ground_histogram_bin_m").as_double(),
      get_parameter("ground_update_interval_s").as_double(),
      get_parameter("ground_ema_alpha").as_double());

    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    scan_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      scan_topic_, qos,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {on_scan(msg);});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, qos,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {on_odom(msg);});
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(10).transient_local());
    status_pub_ = create_publisher<std_msgs::msg::String>(
      occupancy_status_topic_, rclcpp::QoS(10));

    const double period = 1.0 / std::max(0.1, publish_rate_hz_);
    map_timer_ = create_wall_timer(
      std::chrono::duration<double>(period),
      [this]() {publish_map();});

    RCLCPP_INFO(
      get_logger(), "pointcloud_to_occupancy: %s -> %s",
      scan_topic_.c_str(), map_topic_.c_str());
  }

private:
  bool load_pandar_candidate_xy(
    const std::string & yaml_path, double & x, double & y)
  {
    if (yaml_path.empty()) {
      return false;
    }
    try {
      const YAML::Node root = YAML::LoadFile(yaml_path);
      const YAML::Node candidate = root["transform_candidate"];
      const YAML::Node translation = candidate["translation_m"];
      if (!translation || !translation.IsMap()) {
        return false;
      }
      x = translation["x"] ? translation["x"].as<double>() : 0.0;
      y = translation["y"] ? translation["y"].as<double>() : 0.0;
      return true;
    } catch (const std::exception &) {
      return false;
    }
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto & p = msg->pose.pose;
    const double qw = p.orientation.w, qx = p.orientation.x;
    const double qy = p.orientation.y, qz = p.orientation.z;
    base_x_ = p.position.x;
    base_y_ = p.position.y;
    base_yaw_ = std::atan2(
      2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    pose_valid_ = true;
  }

  void on_scan(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    RCLCPP_INFO(
      get_logger(), "on_scan: w=%u h=%u pts=%zu",
      msg->width, msg->height, msg->width * msg->height);
    if (!pose_valid_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "on_scan: waiting for odom");
      return;  // wait for the first shadow odometry
    }
    // Rate limit the heavy scan updates (steady clock: never mix clocks).
    if (max_scan_update_hz_ > 0.0) {
      const double min_interval = 1.0 / max_scan_update_hz_;
      const auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_scan_update_).count();
      if (elapsed < min_interval) {
        return;
      }
    }
    last_scan_update_ = std::chrono::steady_clock::now();

    const CloudFieldOffsets offsets = find_field_offsets(*msg);
    if (!offsets.valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "on_scan: schema invalid");
      return;
    }
    const std::size_t n_points = msg->width * msg->height;
    const std::size_t point_step = msg->point_step > 0 ? msg->point_step : 16u;
    const uint8_t * data = msg->data.data();
    const std::size_t data_size = msg->data.size();

    // Sensor world position: base pose rotated by the candidate Pandar offset
    // (mapping-assist best effort; never a formal TF claim).
    const double sensor_x = base_x_ + std::cos(base_yaw_) * pandar_offset_x_ -
      std::sin(base_yaw_) * pandar_offset_y_;
    const double sensor_y = base_y_ + std::sin(base_yaw_) * pandar_offset_x_ +
      std::cos(base_yaw_) * pandar_offset_y_;

    std::vector<Point3> points;
    points.reserve(n_points);
    for (std::size_t i = 0; i < n_points; ++i) {
      const std::size_t base = i * point_step;
      if (base + point_step > data_size) {
        break;
      }
      const Point3 p{
        static_cast<double>(read_f32_p(data + base + static_cast<std::size_t>(offsets.x))),
        static_cast<double>(read_f32_p(data + base + static_cast<std::size_t>(offsets.y))),
        static_cast<double>(read_f32_p(data + base + static_cast<std::size_t>(offsets.z)))};
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }
      if (p.x == 0.0 && p.y == 0.0 && p.z == 0.0) {
        continue;
      }
      points.push_back(p);
    }

    // Ground estimation (auto; operator never measures base-ground height).
    ground_estimator_->update(
      now().seconds(), points, sensor_x, sensor_y);
    if (!ground_estimator_->valid()) {
      return;
    }
    const double ground_z = ground_estimator_->ground_z();
    const bool ground_fallback = ground_estimator_->is_fallback();

    const double min_range = grid_->params().min_range_m;
    const double max_range = grid_->params().max_range_m;
    const double obstacle_min_h = grid_->params().obstacle_min_height_m;
    const double obstacle_max_h = grid_->params().obstacle_max_height_m;

    const std::vector<Point3> rays = polar_downsample(
      points, sensor_x, sensor_y, grid_->params().angular_ray_bin_deg);

    int sensor_cx = 0, sensor_cy = 0;
    grid_->world_to_cell(sensor_x, sensor_y, sensor_cx, sensor_cy);
    grid_->update_sensor_cell_free(sensor_x, sensor_y);

    for (const Point3 & p : rays) {
      const double dx = p.x - sensor_x;
      const double dy = p.y - sensor_y;
      const double range = std::hypot(dx, dy);
      if (range < min_range || range > max_range) {
        continue;
      }
      int cx = 0, cy = 0;
      grid_->world_to_cell(p.x, p.y, cx, cy);
      grid_->raytrace_free(sensor_cx, sensor_cy, cx, cy);
      const double relative_h = p.z - ground_z;
      if (relative_h >= obstacle_min_h && relative_h <= obstacle_max_h) {
        grid_->update_hit(cx, cy);
      }
      // Floor points only clear rays; ceiling points are ignored entirely.
    }

    std::string status = "ground: " + std::string(ground_fallback ? "FALLBACK" : "OK") +
      " z=" + std::to_string(ground_z);
    std_msgs::msg::String status_msg;
    status_msg.data = status;
    status_pub_->publish(status_msg);
    last_ground_fallback_ = ground_fallback;
  }

  void publish_map()
  {
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp = now();
    msg.header.frame_id = map_frame_;
    msg.info.resolution = grid_->resolution();
    msg.info.width = static_cast<uint32_t>(grid_->width());
    msg.info.height = static_cast<uint32_t>(grid_->height());
    msg.info.origin.position.x = grid_->origin_x();
    msg.info.origin.position.y = grid_->origin_y();
    msg.info.origin.orientation.w = 1.0;
    msg.data.resize(grid_->width() * grid_->height());
    for (std::size_t y = 0; y < grid_->height(); ++y) {
      for (std::size_t x = 0; x < grid_->width(); ++x) {
        msg.data[y * grid_->width() + x] =
          grid_->cell_value(static_cast<int>(x), static_cast<int>(y));
      }
    }
    map_pub_->publish(msg);
  }

  std::string scan_topic_;
  std::string odom_topic_;
  std::string map_topic_;
  std::string occupancy_status_topic_;
  std::string map_frame_;
  double publish_rate_hz_ = 2.0;
  double max_scan_update_hz_ = 10.0;
  double pandar_offset_x_ = 0.13;
  double pandar_offset_y_ = 0.015;
  double base_x_ = 0.0;
  double base_y_ = 0.0;
  double base_yaw_ = 0.0;
  bool pose_valid_ = false;
  bool last_ground_fallback_ = false;
  std::chrono::steady_clock::time_point last_scan_update_{};
  std::unique_ptr<OccupancyGrid2D> grid_;
  std::unique_ptr<GroundEstimator> ground_estimator_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr map_timer_;
};

}  // namespace go2w_plain_slam_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<go2w_plain_slam_bridge::PointcloudToOccupancy>());
  rclcpp::shutdown();
  return 0;
}
