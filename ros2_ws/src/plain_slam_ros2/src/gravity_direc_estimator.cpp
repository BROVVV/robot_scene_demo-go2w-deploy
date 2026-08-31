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

#include <plain_slam/gravity_direc_estimator.hpp>

namespace pslam {

GravityDirecEstimator::GravityDirecEstimator() {
  has_estimated_ = false;
  estimation_time_ = 1.0;

  num_measurements_ = 0;
  acc_sum_ = Eigen::Vector3f::Zero();
  is_first_stamp_initialized_ = false;
}

GravityDirecEstimator::~GravityDirecEstimator() {

}

void GravityDirecEstimator::EstimateGravityDirec(const IMUMeasure& measure) {
  if (has_estimated_) {
    return;
  }

  if (!is_first_stamp_initialized_) {
    first_stamp_ = measure.stamp;
    is_first_stamp_initialized_ = true;
  }

  const double dt = measure.stamp - first_stamp_;
  if (dt < estimation_time_) {
    num_measurements_++;
    acc_sum_ += measure.acc;
    return;
  }

  const Eigen::Vector3f acc_ave = acc_sum_ / num_measurements_;
  const Eigen::Vector3f g(0.0f, 0.0f, 1.0f);
  const Eigen::Vector3f a = acc_ave.normalized();

  const Eigen::Vector3f axis = a.cross(g);
  const float dot = std::clamp(a.dot(g), -1.0f, 1.0f);
  const float angle = std::acos(dot);

  if (axis.norm() < 1e-6 || std::isinf(angle) || std::isnan(angle)) {
    R_ = Sophus::SO3f();
  } else {
    const Eigen::Vector3f omega = axis.normalized() * angle;
    R_ = Sophus::SO3f::exp(omega);
  }

  // std::cout << "angle: " << angle << std::endl;
  // std::cout << "axis.norm(): " << axis.norm() << std::endl;
  // std::cout << "num_measurements_: " << num_measurements_ <<std::endl;
  // std::cout << "acc_sum_:" << std::endl << acc_sum_ << std::endl;
  // std::cout << R_.matrix() << std::endl;

  has_estimated_ = true;
}

} // namespace pslam
