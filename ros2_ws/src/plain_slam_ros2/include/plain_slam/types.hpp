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

#include <iostream>
#include <vector>
#include <memory>

#include <Eigen/Core>

#include <sophus/se3.hpp>

#include <nanoflann.hpp>

namespace pslam {

using Point3f = Eigen::Vector3f;
using PointCloud3f = std::vector<Point3f, Eigen::aligned_allocator<Point3f>>;

using Covariance3f = Eigen::Matrix3f;
using Covariances3f = std::vector<Covariance3f, Eigen::aligned_allocator<Covariance3f>>;

using StateCov = Eigen::Matrix<float, 24, 24>;

struct IMUMeasure {
  Eigen::Vector3f gyro;
  Eigen::Vector3f acc;
  double stamp = 0.0;
  double dt = 0.0;

  IMUMeasure()
    : gyro(Eigen::Vector3f::Zero()),
      acc(Eigen::Vector3f::Zero()),
      stamp(0.0),
      dt(0.0) {}
};

struct State {
  Sophus::SE3f T; // p_{odom} = T * p_{imu}
  Eigen::Vector3f v;
  Eigen::Vector3f gb;
  Eigen::Vector3f ab;
  Eigen::Vector3f gacc;
  Sophus::SE3f Til; // p_{imu} = Til * p_{lidar}

  State()
    : T(),
      v(Eigen::Vector3f::Zero()),
      gb(Eigen::Vector3f::Zero()),
      ab(Eigen::Vector3f::Zero()),
      gacc(Eigen::Vector3f(0.0f, 0.0f, 9.79f)),
      Til() {}
};

struct Edge {
  size_t source_idx;
  size_t target_idx;
  Sophus::SE3f relative;

  Edge()
    : source_idx(0),
      target_idx(0),
      relative() {}
};

struct PointCloudAdaptor {
  const PointCloud3f& pts;

  PointCloudAdaptor(const PointCloud3f& points)
    : pts(points) {}

  inline size_t kdtree_get_point_count() const {
    return pts.size();
  }

  inline float kdtree_get_pt(const size_t idx, const size_t dim) const {
    return pts[idx][dim];
  }

  template <class BBOX>
  bool kdtree_get_bbox(BBOX&) const {
    return false;
  }
};

using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
  nanoflann::L2_Simple_Adaptor<float, PointCloudAdaptor>,
  PointCloudAdaptor,
  3
>;

}  // namespace pslam
