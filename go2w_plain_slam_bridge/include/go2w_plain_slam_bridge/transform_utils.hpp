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

// Rigid-transform helpers for the odom adapter (plan §10).
//
// plain_slam outputs the IMU-frame pose; the project's navigation prefers the
// robot base pose.  T_base_imu is derived automatically from:
//
//   official_reference.yaml: frames.base_to_lidar + frames.lidar_to_lidar_imu
//
//   T_base_imu = T_base_lidar * T_lidar_imu
//   T_world_base = T_world_imu * inverse(T_base_imu)
//
// Rotation convention matches the Python config generator: fixed-axis RPY
// with R = Rz(yaw) * Ry(pitch) * Rx(roll).

#ifndef GO2W_PLAIN_SLAM_BRIDGE__TRANSFORM_UTILS_HPP_
#define GO2W_PLAIN_SLAM_BRIDGE__TRANSFORM_UTILS_HPP_

#include <yaml-cpp/yaml.h>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace go2w_plain_slam_bridge
{

struct RigidTransform
{
  // Row-major 4x4 homogeneous matrix (translation lives in the FOURTH
  // COLUMN: m[3], m[7], m[11]; m[15] = 1).
  double m[16] = {
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0};

  static RigidTransform from_rpy(
    const std::vector<double> & translation,
    const std::vector<double> & rpy_rad)
  {
    RigidTransform t;
    const double rx = rpy_rad.size() > 0 ? rpy_rad[0] : 0.0;
    const double ry = rpy_rad.size() > 1 ? rpy_rad[1] : 0.0;
    const double rz = rpy_rad.size() > 2 ? rpy_rad[2] : 0.0;
    const double cx = std::cos(rx), sx = std::sin(rx);
    const double cy = std::cos(ry), sy = std::sin(ry);
    const double cz = std::cos(rz), sz = std::sin(rz);
    // R = Rz * Ry * Rx
    t.m[0] = cz * cy;
    t.m[1] = cz * sy * sx - sz * cx;
    t.m[2] = cz * sy * cx + sz * sx;
    t.m[4] = sz * cy;
    t.m[5] = sz * sy * sx + cz * cx;
    t.m[6] = sz * sy * cx - cz * sx;
    t.m[8] = -sy;
    t.m[9] = cy * sx;
    t.m[10] = cy * cx;
    for (int i = 0; i < 3; ++i) {
      // fourth column carries the translation
      t.m[3 + 4 * i] = translation.size() > static_cast<std::size_t>(i) ?
        translation[i] : 0.0;
    }
    return t;
  }

  RigidTransform inverse() const
  {
    RigidTransform out;
    // R^T in the upper 3x3, t' = -R^T * t (fourth column).
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        out.m[i * 4 + j] = m[j * 4 + i];
      }
    }
    for (int i = 0; i < 3; ++i) {
      double acc = 0.0;
      for (int k = 0; k < 3; ++k) {
        acc += out.m[i * 4 + k] * m[3 + 4 * k];
      }
      out.m[3 + 4 * i] = -acc;
    }
    return out;
  }

  RigidTransform multiply(const RigidTransform & other) const
  {
    RigidTransform out;
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        double acc = 0.0;
        for (int k = 0; k < 4; ++k) {
          acc += m[i * 4 + k] * other.m[k * 4 + j];
        }
        out.m[i * 4 + j] = acc;
      }
    }
    return out;
  }

  void apply(double & x, double & y, double & z) const
  {
    const double nx =
      m[0] * x + m[1] * y + m[2] * z + m[3];
    const double ny =
      m[4] * x + m[5] * y + m[6] * z + m[7];
    const double nz =
      m[8] * x + m[9] * y + m[10] * z + m[11];
    x = nx;
    y = ny;
    z = nz;
  }
};

inline std::vector<double> yaml_vec3(const YAML::Node & node)
{
  std::vector<double> out;
  if (!node || !node.IsSequence()) {
    return out;
  }
  for (const auto & item : node) {
    out.push_back(item.as<double>());
  }
  return out;
}

// Load base_link -> utlidar_imu from the manufacturer reference YAML.
inline bool load_base_to_imu_transform(
  const std::string & yaml_path,
  RigidTransform & out,
  std::string & error)
{
  try {
    const YAML::Node root = YAML::LoadFile(yaml_path);
    const YAML::Node frames = root["frames"];
    if (!frames) {
      error = "missing top-level 'frames' in " + yaml_path;
      return false;
    }
    const YAML::Node base_to_lidar = frames["base_to_lidar"];
    const YAML::Node lidar_to_imu = frames["lidar_to_lidar_imu"];
    if (!base_to_lidar || !lidar_to_imu) {
      error = "missing frames.base_to_lidar / frames.lidar_to_lidar_imu";
      return false;
    }
    const RigidTransform t_base_lidar = RigidTransform::from_rpy(
      yaml_vec3(base_to_lidar["translation_m"]),
      yaml_vec3(base_to_lidar["rotation_rpy_rad"]));
    const RigidTransform t_lidar_imu = RigidTransform::from_rpy(
      yaml_vec3(lidar_to_imu["translation_m"]),
      yaml_vec3(lidar_to_imu["rotation_rpy_rad"]));
    out = t_base_lidar.multiply(t_lidar_imu);
    return true;
  } catch (const std::exception & exc) {
    error = std::string("yaml load failed: ") + exc.what();
    return false;
  }
}

}  // namespace go2w_plain_slam_bridge

#endif  // GO2W_PLAIN_SLAM_BRIDGE__TRANSFORM_UTILS_HPP_
