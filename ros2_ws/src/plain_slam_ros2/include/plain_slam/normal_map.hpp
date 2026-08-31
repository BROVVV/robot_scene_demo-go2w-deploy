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

#include <boost/circular_buffer.hpp>

#include <Eigen/Dense>

#include <plain_slam/types.hpp>
#include <plain_slam/voxel_grid_filter.hpp>

namespace pslam {

class NormalMap {
 public:
  NormalMap();

  ~NormalMap();

  void AddKeyframe(
    const Sophus::SE3f& keyframe_pose,
    const PointCloud3f& aligned_scan_cloud);

  bool FindCorrespondence(
    const Eigen::Vector3f& query,
    float max_correspondence_dist,
    Eigen::Vector3f& target,
    Eigen::Vector3f& normal);

  void SetMaxKeyframeSize(size_t size) {
    keyframe_poses_ = boost::circular_buffer<Sophus::SE3f>(size);
    aligned_scan_clouds_ = boost::circular_buffer<PointCloud3f>(size);
  }

  void SetFilterSize(float size) {
    filter_size_ = size;
  }

  void SetNumNormalPoints(size_t num) {
    num_normal_points_ = num;
  }

  void SetNormalEigenValThreshhold(float th) {
    normal_eigen_val_thresh_ = th;
  }

  const PointCloud3f& GetMapCloud() const {
    return filtered_map_cloud_;
  }

  void GetActiveMapCloud(PointCloud3f& cloud) const;

 private:
  boost::circular_buffer<Sophus::SE3f> keyframe_poses_;
  boost::circular_buffer<PointCloud3f> aligned_scan_clouds_;

  PointCloud3f filtered_map_cloud_;
  std::unique_ptr<PointCloudAdaptor> adaptor_;
  std::unique_ptr<KDTree> kdtree_;

  std::vector<bool> normal_computed_flags_;
  std::vector<bool> normal_valid_flags_;
  std::vector<Eigen::Vector3f> normals_;

  float filter_size_;

  size_t num_normal_points_;
  float normal_eigen_val_thresh_;
};

} // namespace pslam
