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
#include <plain_slam/lie_algebra.hpp>

namespace pslam {

class IMUPreintegrator {
 public:
  IMUPreintegrator();

  ~IMUPreintegrator();

  void Preintegration(
    const std::vector<IMUMeasure>& imu_measures,
    State& state,
    StateCov& state_cov);

  void Preintegration(
    const IMUMeasure imu_measure,
    State& state,
    StateCov& state_cov);

  void DeskewScanCloud(
    const State& state,
    const std::vector<IMUMeasure> imu_measures,
    const std::vector<double>& scan_stamps,
    PointCloud3f& scan_cloud);

  void SetProcessNoiseDiag(const std::vector<float>& diag) {
    assert(diag.size() == 12);
    for (size_t i = 0; i < 12; ++i) {
      process_noise_cov_(i, i) = diag[i];
    }
  }

  float GetPreintegrationTime() const {
    return preint_time_;
  }

  Sophus::SE3f GetDeltaT() const {
    return delta_T_;
  }

  Eigen::Vector3f GetDeltaV() const {
    return delta_v_;
  }

 private:
  std::vector<Sophus::SE3f> preint_Ts_;
  std::vector<Eigen::Vector3f> preint_vs_;
  Sophus::SE3f delta_T_;
  Eigen::Vector3f delta_v_;
  float preint_time_;

  Eigen::Matrix<float, 12, 12> process_noise_cov_;
};

} // namespace pslam
