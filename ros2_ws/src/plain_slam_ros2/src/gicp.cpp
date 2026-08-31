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

#include <plain_slam/gicp.hpp>

namespace pslam {

GICP::GICP() {
  filter_size_ = 0.1f;
  num_max_source_points_ = 10000000;
  num_max_iteration_ = 5;
  epsilon_ = 0.01f;
  num_cov_points_ = 10;
  max_correspondence_dist_ = 10.0f;
  max_correspondence_dist2_ = max_correspondence_dist_ * max_correspondence_dist_;
  huber_delta_ = 0.1f;

  T_ = Sophus::SE3f(Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero());
}

GICP::~GICP() {
  
}

void GICP::SetSourceCloud(
  const PointCloud3f& cloud,
  bool filter_cloud) {
  if (filter_cloud) {
    const VoxelGridFilter vgf(filter_size_);
    source_points_ = vgf.filter(cloud);
  } else {
    source_points_ = cloud;
  }
  source_means_.resize(source_points_.size());
  source_covs_.resize(source_points_.size());

  source_adaptor_ = std::make_unique<PointCloudAdaptor>(source_points_);
  source_kdtree_ = std::make_unique<KDTree>(3, *source_adaptor_, nanoflann::KDTreeSingleIndexAdaptorParams(10));
  source_kdtree_->buildIndex();

  source_point_computed_flags_.resize(source_points_.size());
  for (size_t i = 0; i < source_points_.size(); ++i) {
    source_point_computed_flags_[i] = false;
  }
}

void GICP::SetTargetCloud(
  const PointCloud3f& cloud,
  bool filter_cloud) {
  if (filter_cloud) {
    const VoxelGridFilter vgf(filter_size_);
    target_points_ = vgf.filter(cloud);
  } else {
    target_points_ = cloud;
  }
  target_means_.resize(target_points_.size());
  target_covs_.resize(target_points_.size());

  target_adaptor_ = std::make_unique<PointCloudAdaptor>(target_points_);
  target_kdtree_ = std::make_unique<KDTree>(3, *target_adaptor_, nanoflann::KDTreeSingleIndexAdaptorParams(10));
  target_kdtree_->buildIndex();

  target_point_computed_flags_.resize(target_points_.size());
  for (size_t i = 0; i < target_points_.size(); ++i) {
    target_point_computed_flags_[i] = false;
  }
}

void GICP::Align() {
  int step = source_means_.size() / num_max_source_points_;
  if (step < 1) {
    step = 1;
  }

  has_converged_ = false;

  for (size_t iter_num = 0; iter_num < num_max_iteration_; ++iter_num) {
    Eigen::Matrix<float, 6, 6> H = Eigen::Matrix<float, 6, 6>::Zero();
    Eigen::Matrix<float, 6, 1> b = Eigen::Matrix<float, 6, 1>::Zero();

    const Eigen::Matrix3f R = T_.rotationMatrix();
    const Eigen::Matrix3f RT = R.transpose();

    float e_sum = 0.0f;
    size_t total_num = 0;
    size_t num = 0;

    for (size_t i = 0; i < source_means_.size(); i += step) {
      total_num++;

      if (!source_point_computed_flags_[i]) {
        ComputeGICPPoint(source_points_[i], *source_kdtree_,
          *source_adaptor_, source_means_[i], source_covs_[i]);
        source_point_computed_flags_[i] = true;
      }

      std::vector<size_t> indices(1);
      std::vector<float> dists(1);
      nanoflann::KNNResultSet<float> result_set(1);
      result_set.init(indices.data(), dists.data());

      const Eigen::Vector3f query = T_ * source_means_[i];
      const float query_pt[3] = {query.x(), query.y(), query.z()};
      target_kdtree_->findNeighbors(result_set,
        query_pt, nanoflann::SearchParameters());

      if (dists[0] > max_correspondence_dist2_) {
        continue;
      }

      const size_t tidx = indices[0];
      if (!target_point_computed_flags_[tidx]) {
        ComputeGICPPoint(target_points_[tidx], *target_kdtree_,
          *target_adaptor_, target_means_[tidx], target_covs_[tidx]);
        target_point_computed_flags_[tidx] = true;
      }

      const Eigen::Vector3f e = target_means_[tidx] - query;
      const float abs_e = e.norm();
      e_sum += std::sqrt(dists[0]);

      Eigen::Matrix<float, 3, 6> J;
      J.block<3, 3>(0, 0) = -Eigen::Matrix3f::Identity();
      J.block<3, 3>(0, 3) = Sophus::SO3f::hat(query).matrix();
      Eigen::Matrix<float, 6, 3> JT = J.transpose();

      const Eigen::Matrix3f C = (target_covs_[tidx] + R * source_covs_[i] * RT).inverse();
      if (huber_delta_ > 0.0f) {
        const float w = (abs_e <= huber_delta_) ? 1.0f : (huber_delta_ / abs_e);
        H += w * JT * C * J;
        b += w * JT * C * e;
      } else {
        H += JT * C * J;
        b += JT * C * e;
      }

      num++;
    }

    if (num == 0) {
      std::cerr << "[WARN] No correspondences in GICP registration" << std::endl;
      active_points_rate_ = 0.0f;
      error_average_ = 10000.0f * std::sqrt(max_correspondence_dist2_);
      return;
    }

    const Eigen::LDLT<Eigen::Matrix<float, 6, 6>> solver(H);
    const Eigen::Matrix<float, 6, 1> d = solver.solve(-b);
    T_ = Sophus::SE3f::exp(d) * T_;
    const float epsilon = d.norm();
    active_points_rate_ = static_cast<float>(num)
                        / static_cast<float>(total_num);
    error_average_ = e_sum / num;
    // std::cout << T_.matrix() << std::endl;
    // std::cout << "epsilon = " << epsilon << std::endl;
    if (epsilon < epsilon_) {
      has_converged_ = true;
      break;
    }
  }
}

void GICP::ComputeGICPPoint(
  const Point3f& query,
  const KDTree& kdtree,
  const PointCloudAdaptor& adaptor,
  Point3f& mean,
  Covariance3f& cov)
{
  std::vector<size_t> indices(num_cov_points_);
  std::vector<float> dists(num_cov_points_);
  nanoflann::KNNResultSet<float> result_set(num_cov_points_);

  const float query_pt[3] = {query.x(), query.y(), query.z()};

  result_set.init(indices.data(), dists.data());
  kdtree.findNeighbors(result_set, query_pt, nanoflann::SearchParameters());

  mean = Eigen::Vector3f::Zero();
  for (size_t i = 0; i < num_cov_points_; ++i) {
    mean += adaptor.pts[indices[i]];
  }
  mean /= static_cast<float>(num_cov_points_);

  cov = Eigen::Matrix3f::Zero();
  for (size_t i = 0; i < num_cov_points_; ++i) {
    Eigen::Vector3f diff = adaptor.pts[indices[i]] - mean;
    cov += diff * diff.transpose();
  }
  cov /= static_cast<float>(num_cov_points_);
  cov += 1e-6f * Eigen::Matrix3f::Identity();
}

void GICP::Transform(
  const PointCloud3f& in,
  PointCloud3f& out) {
  out.resize(in.size());
  for (size_t i = 0; i < in.size(); ++i) {
    out[i] = T_ * in[i];
  }
}

} // namespace pslam
