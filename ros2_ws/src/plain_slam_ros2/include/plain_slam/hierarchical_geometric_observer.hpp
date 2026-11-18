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

#include <plain_slam/types.hpp>
#include <plain_slam/normal_map.hpp>

namespace pslam {

class HierarchicalGeometricObserver {
 public:
  HierarchicalGeometricObserver();

  ~HierarchicalGeometricObserver();

  bool Estimate(
    const State& pred_state,
    const PointCloud3f& scan_cloud,
    size_t max_iter_num,
    size_t num_max_matching_points,
    float max_correspondence_dist,
    float convergence_th,
    float preintegration_time,
    State& updated_state,
    NormalMap& normal_map);

  void SetHuberDelta(float delta) {
    huber_delta_ = delta;
  }

  float GetActivePointsRate() const {
    return active_points_rate_;
  }

  // [GO2W-DIAG-R10] 配准实际迭代次数与退出时的 se3 增量范数
  size_t GetLastIterNum() const {
    return last_iter_num_;
  }

  float GetLastEpsilon() const {
    return last_epsilon_;
  }

 private:
  float gamma1_; // k_q
  float gamma2_; // k_gb
  float gamma3_; // k_p
  float gamma4_; // k_v
  float gamma5_; // k_ab

  float huber_delta_;

  float active_points_rate_;

  size_t last_iter_num_ = 0;
  float last_epsilon_ = 0.0f;
};

} // namespace pslam
