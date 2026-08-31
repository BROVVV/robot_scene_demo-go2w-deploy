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

#include <plain_slam/voxel_grid_filter.hpp>

namespace pslam {

VoxelGridFilter::VoxelGridFilter(float voxel_size) {
  voxel_size_ = voxel_size;
  inv_size_ = 1.0f / voxel_size;
}

VoxelGridFilter::~VoxelGridFilter() {

}

PointCloud3f VoxelGridFilter::filter(const PointCloud3f& input) const {
  using Accum = std::pair<Point3f, int>;
  std::unordered_map<VoxelKey, Accum> voxels;

  // Assume the output cloud size is approximately one-fourth of the input size.
  voxels.reserve(input.size() / 4);

  for (const auto& p : input) {
    VoxelKey key{
      static_cast<int>(std::floor(p.x() * inv_size_)),
      static_cast<int>(std::floor(p.y() * inv_size_)),
      static_cast<int>(std::floor(p.z() * inv_size_)) };

    auto& [sum, cnt] = voxels[key];
    if (cnt == 0) {
      sum.setZero();
    }
    sum += p;
    cnt++;
  }

  PointCloud3f output;
  output.reserve(voxels.size());
  for (const auto& [k, acc] : voxels) {
    const auto& [sum, cnt] = acc;
    output.emplace_back(sum / static_cast<float>(cnt));
  }

  return output;
}

void VoxelGridFilter::filter(
  const PointCloud3f& input_cloud,
  const std::vector<float>& input_intensities,
  PointCloud3f& out_cloud,
  std::vector<float>& out_intensities) const {
  struct Accum {
    Point3f sum;
    float intensity_sum = 0.0f;
    int cnt = 0;
  };

  std::unordered_map<VoxelKey, Accum> voxels;
  voxels.reserve(input_cloud.size() / 4);

  for (size_t i = 0; i < input_cloud.size(); ++i) {
    const auto& p = input_cloud[i];
    const float intensity = input_intensities[i];

    VoxelKey key{
      static_cast<int>(std::floor(p.x() * inv_size_)),
      static_cast<int>(std::floor(p.y() * inv_size_)),
      static_cast<int>(std::floor(p.z() * inv_size_))};

    auto& acc = voxels[key];
    if (acc.cnt == 0) {
      acc.sum.setZero();
    }
    acc.sum += p;
    acc.intensity_sum += intensity;
    acc.cnt++;
  }

  out_cloud.clear();
  out_intensities.clear();
  out_cloud.reserve(voxels.size());
  out_intensities.reserve(voxels.size());

  for (const auto& [key, acc] : voxels) {
    const float inv_cnt = 1.0f / static_cast<float>(acc.cnt);
    out_cloud.emplace_back(acc.sum * inv_cnt);
    out_intensities.emplace_back(acc.intensity_sum * inv_cnt);
  }
}

} // namespace pslam
