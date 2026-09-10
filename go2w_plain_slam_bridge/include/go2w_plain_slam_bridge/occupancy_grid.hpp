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

// Ray-traced 2D occupancy grid for the plain_slam mapping pipeline (plan §9).
//
// This is NOT a flattened 3D bitmap: every aligned scan is projected with 2D
// ray casting so cells between the sensor and each endpoint become FREE,
// robot-height geometry endpoints become OCCUPIED and everything else remains
// UNKNOWN.  That free/occupied/unknown triple is what makes frontier
// extraction possible.
//
// Ground height is estimated automatically (histogram + EMA + low-percentile
// fallback); the operator never measures base-to-ground.

#ifndef GO2W_PLAIN_SLAM_BRIDGE__OCCUPANCY_GRID_HPP_
#define GO2W_PLAIN_SLAM_BRIDGE__OCCUPANCY_GRID_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace go2w_plain_slam_bridge
{

struct OccupancyParams
{
  double resolution_m = 0.10;
  double width_m = 80.0;
  double height_m = 80.0;
  double origin_x_m = -40.0;
  double origin_y_m = -40.0;
  double min_range_m = 0.35;
  double max_range_m = 25.0;
  double log_odds_hit = 0.85;
  double log_odds_miss = -0.40;
  double log_odds_min = -2.0;
  double log_odds_max = 3.5;
  double free_probability_threshold = 0.35;
  double occupied_probability_threshold = 0.65;
  double obstacle_min_height_m = 0.10;
  double obstacle_max_height_m = 1.60;
  double floor_tolerance_m = 0.08;
  double angular_ray_bin_deg = 0.5;
};

class OccupancyGrid2D
{
public:
  explicit OccupancyGrid2D(const OccupancyParams & params)
  : params_(params),
    width_(static_cast<std::size_t>(std::round(params.width_m / params.resolution_m))),
    height_(static_cast<std::size_t>(std::round(params.height_m / params.resolution_m))),
    log_odds_(width_ * height_, 0.0)
  {
    const double lo_free = std::log(
      params.free_probability_threshold /
      (1.0 - params.free_probability_threshold));
    const double lo_occ = std::log(
      params.occupied_probability_threshold /
      (1.0 - params.occupied_probability_threshold));
    lo_free_threshold_ = lo_free;
    lo_occ_threshold_ = lo_occ;
  }

  std::size_t width() const {return width_;}
  std::size_t height() const {return height_;}
  double resolution() const {return params_.resolution_m;}
  double origin_x() const {return params_.origin_x_m;}
  double origin_y() const {return params_.origin_y_m;}

  bool cell_in_bounds(int x, int y) const
  {
    return x >= 0 && y >= 0 &&
           static_cast<std::size_t>(x) < width_ &&
           static_cast<std::size_t>(y) < height_;
  }

  void world_to_cell(double wx, double wy, int & x, int & y) const
  {
    x = static_cast<int>(std::floor((wx - params_.origin_x_m) / params_.resolution_m));
    y = static_cast<int>(std::floor((wy - params_.origin_y_m) / params_.resolution_m));
  }

  double log_odds(int x, int y) const
  {
    return log_odds_[static_cast<std::size_t>(y) * width_ + static_cast<std::size_t>(x)];
  }

  void add_log_odds(int x, int y, double delta)
  {
    if (!cell_in_bounds(x, y)) {
      return;
    }
    double & value = log_odds_[static_cast<std::size_t>(y) * width_ + static_cast<std::size_t>(x)];
    // std::clamp is C++17; keep this header C++14-compatible (ROS Foxy/GCC9).
    value = std::min(std::max(value + delta, params_.log_odds_min), params_.log_odds_max);
  }

  // OccupancyGrid cell value: -1 unknown, 0 free, 100 occupied.
  int8_t cell_value(int x, int y) const
  {
    if (!cell_in_bounds(x, y)) {
      return -1;
    }
    const double lo = log_odds(x, y);
    if (lo >= lo_occ_threshold_) {
      return 100;
    }
    if (lo <= lo_free_threshold_) {
      return 0;
    }
    return -1;
  }

  // DDA (Amanatides-Woo) traversal: mark every cell crossed by the segment
  // (excluding start cell, stopping before the endpoint cell) as FREE, then
  // the caller may mark the endpoint OCCUPIED via update_hit.
  void raytrace_free(int x0, int y0, int x1, int y1)
  {
    if (!cell_in_bounds(x0, y0)) {
      // Sensor outside the map: start from the first inside cell on the ray.
      return;
    }
    const double start_x = static_cast<double>(x0);
    const double start_y = static_cast<double>(y0);
    const double end_x = static_cast<double>(x1);
    const double end_y = static_cast<double>(y1);
    const double dx = end_x - start_x;
    const double dy = end_y - start_y;

    int step_x = dx > 0.0 ? 1 : -1;
    int step_y = dy > 0.0 ? 1 : -1;
    const double t_delta_x = std::abs(dx) >
      1e-12 ? std::abs(1.0 / dx) : std::numeric_limits<double>::infinity();
    const double t_delta_y = std::abs(dy) >
      1e-12 ? std::abs(1.0 / dy) : std::numeric_limits<double>::infinity();

    // Distance (in t units) until the next voxel boundary on each axis.
    double t_max_x = t_delta_x;
    double t_max_y = t_delta_y;
    if (dx > 0.0) {
      t_max_x = (std::floor(start_x) + 1.0 - start_x) * t_delta_x;
    } else if (dx < 0.0) {
      t_max_x = (start_x - std::floor(start_x)) * t_delta_x;
    }
    if (dy > 0.0) {
      t_max_y = (std::floor(start_y) + 1.0 - start_y) * t_delta_y;
    } else if (dy < 0.0) {
      t_max_y = (start_y - std::floor(start_y)) * t_delta_y;
    }

    int vx = x0;
    int vy = y0;
    const int guard = static_cast<int>(width_ + height_) * 2 + 16;
    for (int step = 0; step < guard; ++step) {
      if (t_max_x < t_max_y) {
        vx += step_x;
        t_max_x += t_delta_x;
      } else {
        vy += step_y;
        t_max_y += t_delta_y;
      }
      if (vx == x1 && vy == y1) {
        break;  // endpoint cell: caller decides occupancy
      }
      if (!cell_in_bounds(vx, vy)) {
        break;
      }
      // Cells between start and endpoint become observed free.
      if (vx != x0 || vy != y0) {
        add_log_odds(vx, vy, params_.log_odds_miss);
      }
    }
  }

  void update_hit(int x, int y)
  {
    add_log_odds(x, y, params_.log_odds_hit);
  }

  void update_sensor_cell_free(double sensor_x, double sensor_y)
  {
    int x = 0, y = 0;
    world_to_cell(sensor_x, sensor_y, x, y);
    if (cell_in_bounds(x, y)) {
      add_log_odds(x, y, params_.log_odds_miss);
    }
  }

  const OccupancyParams & params() const {return params_;}

private:
  OccupancyParams params_;
  std::size_t width_;
  std::size_t height_;
  std::vector<double> log_odds_;
  double lo_free_threshold_ = -0.619;
  double lo_occ_threshold_ = 0.619;
};

struct Point3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

// Slow-but-simple polar downsampler: one representative point per angular bin
// (the closest one wins; conservative for free-space clearing).
inline std::vector<Point3> polar_downsample(
  const std::vector<Point3> & points,
  double sensor_x,
  double sensor_y,
  double bin_deg)
{
  const double bin_deg_clamped = std::max(0.05, bin_deg);
  const int num_bins = static_cast<int>(std::max(1.0, std::round(360.0 / bin_deg_clamped)));
  std::vector<Point3> best(num_bins);
  std::vector<double> best_range(num_bins, std::numeric_limits<double>::infinity());
  std::vector<bool> used(num_bins, false);
  for (const Point3 & p : points) {
    const double dx = p.x - sensor_x;
    const double dy = p.y - sensor_y;
    const double range = std::hypot(dx, dy);
    if (range <= 0.0) {
      continue;
    }
    double angle_deg = std::atan2(dy, dx) * 180.0 / M_PI;
    if (angle_deg < 0.0) {
      angle_deg += 360.0;
    }
    int bin = static_cast<int>(angle_deg / bin_deg_clamped) % num_bins;
    if (range < best_range[bin]) {
      best_range[bin] = range;
      best[bin] = p;
      used[bin] = true;
    }
  }
  std::vector<Point3> out;
  out.reserve(num_bins);
  for (int i = 0; i < num_bins; ++i) {
    if (used[i]) {
      out.push_back(best[i]);
    }
  }
  return out;
}

// Automatic ground-height estimator (plan §9.4).
class GroundEstimator
{
public:
  explicit GroundEstimator(
    double search_radius_m = 3.0,
    double histogram_bin_m = 0.05,
    double update_interval_s = 1.0,
    double ema_alpha = 0.3)
  : search_radius_m_(search_radius_m),
    histogram_bin_m_(histogram_bin_m),
    update_interval_s_(update_interval_s),
    ema_alpha_(ema_alpha)
  {}

  // now_s: seconds (steady clock).  points: aligned scan around the robot.
  // sensor position in world (x, y, z).  Returns true when a ground estimate
  // is currently available (either fresh or held from a previous frame).
  bool update(
    double now_s,
    const std::vector<Point3> & points,
    double sensor_x,
    double sensor_y)
  {
    if (now_s - last_update_s_ < update_interval_s_ && valid_) {
      return true;
    }
    last_update_s_ = now_s;

    std::vector<double> zs;
    zs.reserve(points.size());
    for (const Point3 & p : points) {
      const double dx = p.x - sensor_x;
      const double dy = p.y - sensor_y;
      if (std::hypot(dx, dy) <= search_radius_m_) {
        zs.push_back(p.z);
      }
    }
    if (zs.size() < 50) {
      fallback_ = true;
      return valid_;  // keep previous estimate
    }
    std::sort(zs.begin(), zs.end());
    const double min_z = zs.front();
    const double low_percentile = zs[static_cast<std::size_t>(zs.size() * 0.02)];

    // Histogram over low candidates (upward 1.0 m above min).
    const int num_bins = static_cast<int>(std::round(1.0 / histogram_bin_m_));
    std::vector<int> counts(static_cast<std::size_t>(num_bins), 0);
    for (const double z : zs) {
      if (z - min_z < 1.0) {
        const int bin = static_cast<int>((z - min_z) / histogram_bin_m_);
        if (bin >= 0 && bin < num_bins) {
          ++counts[static_cast<std::size_t>(bin)];
        }
      }
    }
    int best_bin = -1;
    int best_count = 0;
    for (int i = 0; i < num_bins; ++i) {
      if (counts[static_cast<std::size_t>(i)] > best_count) {
        best_count = counts[static_cast<std::size_t>(i)];
        best_bin = i;
      }
    }
    double candidate = low_percentile;
    if (best_count >= 5) {
      candidate = min_z + (static_cast<double>(best_bin) + 0.5) * histogram_bin_m_;
      fallback_ = false;
    } else {
      candidate = low_percentile;
      fallback_ = true;
    }

    if (!valid_) {
      ground_z_ = candidate;
      valid_ = true;
    } else {
      ground_z_ = ema_alpha_ * candidate + (1.0 - ema_alpha_) * ground_z_;
    }
    return true;
  }

  double ground_z() const {return ground_z_;}
  bool valid() const {return valid_;}
  bool is_fallback() const {return fallback_;}

private:
  double search_radius_m_;
  double histogram_bin_m_;
  double update_interval_s_;
  double ema_alpha_;
  double ground_z_ = 0.0;
  double last_update_s_ = -std::numeric_limits<double>::infinity();
  bool valid_ = false;
  bool fallback_ = false;
};

}  // namespace go2w_plain_slam_bridge

#endif  // GO2W_PLAIN_SLAM_BRIDGE__OCCUPANCY_GRID_HPP_
