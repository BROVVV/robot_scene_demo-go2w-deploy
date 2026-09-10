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

// PointCloud2 field utilities for the Pandar adapter (plan §5).
//
// The Hesai driver publishes: x/y/z/intensity FLOAT32, ring UINT16,
// timestamp FLOAT64.  plain_slam wants only x/y/z/intensity FLOAT32 +
// timestamp FLOAT64.  This adapter validates the schema, filters bad points
// and rebuilds a minimal output cloud.  It NEVER rotates the cloud and never
// publishes a formal base_link -> pandar transform.

#ifndef GO2W_PLAIN_SLAM_BRIDGE__POINTCLOUD_UTILS_HPP_
#define GO2W_PLAIN_SLAM_BRIDGE__POINTCLOUD_UTILS_HPP_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace go2w_plain_slam_bridge
{

struct CloudFieldOffsets
{
  bool valid = false;
  int32_t x = -1;
  int32_t y = -1;
  int32_t z = -1;
  int32_t intensity = -1;
  int32_t timestamp = -1;
  bool has_ring = false;
};

inline bool field_is_float32(const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
{
  for (const auto & field : cloud.fields) {
    if (field.name == name) {
      return field.datatype == sensor_msgs::msg::PointField::FLOAT32;
    }
  }
  return false;
}

inline CloudFieldOffsets find_field_offsets(const sensor_msgs::msg::PointCloud2 & cloud)
{
  CloudFieldOffsets offsets;
  for (const auto & field : cloud.fields) {
    const std::string & name = field.name;
    if (name == "x") {
      offsets.x = field.offset;
    } else if (name == "y") {
      offsets.y = field.offset;
    } else if (name == "z") {
      offsets.z = field.offset;
    } else if (name == "intensity") {
      offsets.intensity = field.offset;
    } else if (name == "timestamp") {
      offsets.timestamp = field.offset;
    } else if (name == "ring") {
      offsets.has_ring = true;
    }
  }
  offsets.valid =
    offsets.x >= 0 && offsets.y >= 0 && offsets.z >= 0 &&
    offsets.intensity >= 0 && field_is_float32(cloud, "x") &&
    field_is_float32(cloud, "y") && field_is_float32(cloud, "z") &&
    field_is_float32(cloud, "intensity");
  return offsets;
}

// Fixed layout of the adapter output: 4x FLOAT32 + 1x FLOAT64 = 24 bytes.
constexpr std::size_t kOutputPointStep = 24u;

inline void build_output_cloud_fields(sensor_msgs::msg::PointCloud2 & cloud)
{
  cloud.fields.clear();
  cloud.fields.reserve(5);
  sensor_msgs::msg::PointField field;
  field.name = "x";
  field.offset = 0;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;
  cloud.fields.push_back(field);
  field.name = "y";
  field.offset = 4;
  cloud.fields.push_back(field);
  field.name = "z";
  field.offset = 8;
  cloud.fields.push_back(field);
  field.name = "intensity";
  field.offset = 12;
  cloud.fields.push_back(field);
  field.name = "timestamp";
  field.offset = 16;
  field.datatype = sensor_msgs::msg::PointField::FLOAT64;
  cloud.fields.push_back(field);
}

}  // namespace go2w_plain_slam_bridge

#endif  // GO2W_PLAIN_SLAM_BRIDGE__POINTCLOUD_UTILS_HPP_
