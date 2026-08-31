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
#include <plain_slam/lie_algebra.hpp>

namespace pslam {

class JointOptimizer {
 public:
  JointOptimizer();

  ~JointOptimizer();

  bool Estimate(
    const State& prev_state,
    const State& pred_state,
    const StateCov& pred_state_cov,
    const PointCloud3f& scan_cloud,
    size_t max_iter_num,
    size_t num_max_matching_points,
    float max_correspondence_dist,
    float convergence_th,
    float preint_time,
    const std::vector<IMUMeasure>& imu_measures,
    State& updated_state,
    StateCov& updated_cov,
    NormalMap& normal_map);

  void SetHuberDelta(float delta) {
    huber_delta_ = delta;
  }

  float GetActivePointsRate() const {
    return active_points_rate_;
  }

 private:
  float huber_delta_;
  float active_points_rate_;
};

} // namespace pslam
