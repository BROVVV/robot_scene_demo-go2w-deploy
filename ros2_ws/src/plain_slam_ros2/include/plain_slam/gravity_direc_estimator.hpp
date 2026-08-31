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

#include <vector>

#include <plain_slam/types.hpp>

namespace pslam {

class GravityDirecEstimator {
 public:
  GravityDirecEstimator();

  ~GravityDirecEstimator();

  void EstimateGravityDirec(const IMUMeasure& measure);

  void SetEstimationTime(double time) {
    estimation_time_ = time;
  }

  bool HasEstimated() const {
    return has_estimated_;
  }

  Sophus::SO3f GetRotationMatrix() const {
    return R_;
  }

  void Reset() {
    has_estimated_ = false;
    is_first_stamp_initialized_ = false;
    num_measurements_ = 0;
    acc_sum_ = Eigen::Vector3f::Zero();
  }

 private:
  bool has_estimated_;
  double estimation_time_;

  size_t num_measurements_;
  Eigen::Vector3f acc_sum_;
  bool is_first_stamp_initialized_;

  double first_stamp_;
  Sophus::SO3f R_;

};

} // namespace pslam
