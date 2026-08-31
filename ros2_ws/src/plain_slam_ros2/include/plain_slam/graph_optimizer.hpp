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
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>

#include <plain_slam/types.hpp>
#include <plain_slam/lie_algebra.hpp>

namespace pslam {

class GraphOptimizer {
 public:
  GraphOptimizer();

  ~GraphOptimizer();

  void Optimize(
    const std::vector<Edge>& odom_edges,
    const std::vector<Edge>& loop_edges,
    std::vector<Sophus::SE3f>& pose_graph);

  bool HasConverged() const {
    return has_converged_;
  }

  void SetMaxIterNum(size_t num) {
    max_iter_num_ = num;
  }

  void SetEpsilon(float epsilon) {
    epsilon_ = epsilon;
  }

  void SetHuberDelta(float delta) {
    huber_delta_ = delta;
  }

 private:
  bool has_converged_;
  size_t max_iter_num_;
  float epsilon_;
  float huber_delta_;

  void AddEdgeFactors(
    const std::vector<Sophus::SE3f>& pose_graph,
    const std::vector<Edge>& edges,
    std::vector<Eigen::Triplet<float>>& H_list,
    Eigen::MatrixXf& B);
};

} // namespace pslam
