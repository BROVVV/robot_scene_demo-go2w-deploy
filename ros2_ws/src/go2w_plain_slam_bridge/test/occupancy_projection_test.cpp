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

// Unit tests for the ray-traced occupancy projection (plan §18.3).
//
// Synthetic corridor: robot at (0.5, 2.0), wall at x=3.0.
// The cells between sensor and wall must be FREE, the wall endpoint
// OCCUPIED and untouched areas UNKNOWN.  Ground must be estimated
// automatically from the floor points.

#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include <go2w_plain_slam_bridge/occupancy_grid.hpp>

using go2w_plain_slam_bridge::GroundEstimator;
using go2w_plain_slam_bridge::OccupancyGrid2D;
using go2w_plain_slam_bridge::OccupancyParams;
using go2w_plain_slam_bridge::Point3;
using go2w_plain_slam_bridge::polar_downsample;

namespace
{

OccupancyParams corridor_params()
{
  OccupancyParams params;
  params.resolution_m = 0.10;
  params.width_m = 6.0;
  params.height_m = 6.0;
  params.origin_x_m = -2.0;
  params.origin_y_m = -2.0;
  params.min_range_m = 0.2;
  params.max_range_m = 25.0;
  return params;
}

std::vector<Point3> corridor_points(
  std::vector<Point3> & ground_out)
{
  // Floor: 120 points uniformly around the sensor (z ~ 0).
  std::vector<Point3> points;
  for (int i = 0; i < 120; ++i) {
    const double radius = 0.2 + 0.8 * static_cast<double>(i % 30) / 29.0;
    const double angle = 2.0 * M_PI * static_cast<double>(i) / 120.0;
    points.push_back(
      Point3{
        0.5 + radius * std::cos(angle),
        2.0 + radius * std::sin(angle),
        0.01 * static_cast<double>(i % 3)});
  }
  // Wall at x = 3.0 (obstacle height 0.5 m above ground).
  points.push_back(Point3{3.0, 2.0, 0.5});
  points.push_back(Point3{3.0, 2.1, 0.5});
  points.push_back(Point3{3.0, 1.9, 0.5});
  ground_out = points;
  return points;
}

}  // namespace

TEST(OccupancyProjection, CorridorFreeOccupiedUnknown)
{
  std::vector<Point3> ground;
  std::vector<Point3> points = corridor_points(ground);

  OccupancyGrid2D grid(corridor_params());
  const double sensor_x = 0.5, sensor_y = 2.0;

  // --- automatic ground estimation -------------------------------------
  GroundEstimator estimator(3.0, 0.05, 1.0, 0.3);
  ASSERT_TRUE(estimator.update(0.0, points, sensor_x, sensor_y));
  EXPECT_LT(estimator.ground_z(), 0.15);
  EXPECT_FALSE(estimator.is_fallback());

  // --- polar downsampling + ray tracing (multi-frame accumulation:
  //     10 Hz scans build log-odds over ~0.6 s before free is confirmed) ----
  const std::vector<Point3> rays =
    polar_downsample(points, sensor_x, sensor_y, 0.5);

  int sensor_cx = 0, sensor_cy = 0;
  grid.world_to_cell(sensor_x, sensor_y, sensor_cx, sensor_cy);
  grid.update_sensor_cell_free(sensor_x, sensor_y);

  for (int frame = 0; frame < 6; ++frame) {
    for (const Point3 & p : rays) {
      const double dx = p.x - sensor_x;
      const double dy = p.y - sensor_y;
      const double range = std::hypot(dx, dy);
      if (range < grid.params().min_range_m || range > grid.params().max_range_m) {
        continue;
      }
      int cx = 0, cy = 0;
      grid.world_to_cell(p.x, p.y, cx, cy);
      grid.raytrace_free(sensor_cx, sensor_cy, cx, cy);
      const double relative_h = p.z - estimator.ground_z();
      if (relative_h >= grid.params().obstacle_min_height_m &&
        relative_h <= grid.params().obstacle_max_height_m)
      {
        grid.update_hit(cx, cy);
      }
    }
  }

  // --- assertions --------------------------------------------------------
  int mid_cx = 0, mid_cy = 0, wall_cx = 0, wall_cy = 0, far_cx = 0, far_cy = 0;
  grid.world_to_cell(1.5, 2.0, mid_cx, mid_cy);   // between robot and wall
  grid.world_to_cell(3.0, 2.0, wall_cx, wall_cy);  // wall endpoint
  grid.world_to_cell(0.5, -1.5, far_cx, far_cy);   // never scanned area

  EXPECT_EQ(grid.cell_value(mid_cx, mid_cy), 0) << "corridor must become free";
  EXPECT_EQ(grid.cell_value(wall_cx, wall_cy), 100) << "wall must be occupied";
  EXPECT_EQ(grid.cell_value(far_cx, far_cy), -1) << "unseen area stays unknown";

  // Multiple visible wall rays must saturate the endpoint occupancy.
  const std::vector<Point3> wall_points = {
    Point3{3.0, 2.0, 0.5}, Point3{3.0, 2.1, 0.5}, Point3{3.0, 1.9, 0.5}};
  for (const Point3 & p : wall_points) {
    int cx = 0, cy = 0;
    grid.world_to_cell(p.x, p.y, cx, cy);
    grid.update_hit(cx, cy);
  }
  EXPECT_EQ(grid.cell_value(wall_cx, wall_cy), 100);
}

TEST(OccupancyProjection, GridGeometry)
{
  OccupancyGrid2D grid(corridor_params());
  EXPECT_EQ(grid.width(), 60u);
  EXPECT_EQ(grid.height(), 60u);
  EXPECT_DOUBLE_EQ(grid.resolution(), 0.10);
  int x = 0, y = 0;
  grid.world_to_cell(-1.95, -1.95, x, y);
  EXPECT_EQ(x, 0);
  EXPECT_EQ(y, 0);
  grid.world_to_cell(1.95, 1.95, x, y);
  EXPECT_EQ(x, 39);
  EXPECT_EQ(y, 39);
  EXPECT_EQ(grid.cell_value(39, 39), -1);  // untouched -> unknown
}

TEST(OccupancyProjection, GroundEstimatorFallbackKeepsPrior)
{
  GroundEstimator estimator(3.0, 0.05, 1.0, 0.3);
  std::vector<Point3> ground;
  const std::vector<Point3> points = corridor_points(ground);
  ASSERT_TRUE(estimator.update(0.0, points, 0.5, 2.0));
  const double first = estimator.ground_z();
  // Second update with no points: must keep the previous estimate.
  const std::vector<Point3> empty;
  EXPECT_TRUE(estimator.update(10.0, empty, 0.5, 2.0));
  EXPECT_DOUBLE_EQ(estimator.ground_z(), first);
  EXPECT_TRUE(estimator.is_fallback());
}
