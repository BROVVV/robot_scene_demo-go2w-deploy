/*
 * plain_slam_ros2
 * 
 * Copyright (c) 2025 Naoki Akai
 * All rights reserved.
 *
 * This software is provided free of charge for academic and personal use only.
 * Commercial use is strictly prohibited without prior written permission from the author.
 *
 * Conditions:
 *   - Non-commercial use only
 *   - Attribution required in any academic or derivative work
 *   - Redistribution permitted with this license header intact
 *
 * Disclaimer:
 *   This software is provided "as is" without warranty of any kind.
 *   The author is not liable for any damages arising from its use.
 *
 * For commercial licensing inquiries, please contact:
 *   Naoki Akai
 *   Email: n.akai.goo[at]gmail.com   ([at] -> @)
 *   Subject: [plain_slam_ros2] Commercial License Inquiry
 */

#pragma once

#include <Eigen/Core>
#include <unordered_map>
#include <vector>
#include <cmath>

#include <plain_slam/types.hpp>

namespace pslam {

struct VoxelKey {
  int x{}, y{}, z{};

  bool operator==(const VoxelKey& other) const noexcept {
    return x == other.x && y == other.y && z == other.z;
  }
};

} // namespace pslam

namespace std {

template <>
struct hash<pslam::VoxelKey> {
  size_t operator()(const pslam::VoxelKey& k) const noexcept {
    size_t h = 0;
    h ^= std::hash<int>{}(k.x) + 0x9e3779b9 + (h << 6) + (h >> 2);
    h ^= std::hash<int>{}(k.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
    h ^= std::hash<int>{}(k.z) + 0x9e3779b9 + (h << 6) + (h >> 2);
    return h;
  }
};

} // namespace std

namespace pslam {

class VoxelGridFilter {
 public:
  VoxelGridFilter(float voxel_size);

  ~VoxelGridFilter();

  PointCloud3f filter(const PointCloud3f& input) const;

  void filter(
    const PointCloud3f& input_cloud,
    const std::vector<float>& input_intensities,
    PointCloud3f& out_cloud,
    std::vector<float>& out_intensities) const;

 private:
  float voxel_size_;
  float inv_size_;
};

} // namespace pslam
