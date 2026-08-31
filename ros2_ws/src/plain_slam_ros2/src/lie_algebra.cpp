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

#include <plain_slam/lie_algebra.hpp>

namespace pslam {

Eigen::Matrix<float, 6, 6> AdjointSE3(const Sophus::SE3f& T) {
  const Eigen::Matrix3f& R = T.rotationMatrix();
  const Eigen::Vector3f& t = T.translation();

  Eigen::Matrix<float, 6, 6> Adj = Eigen::Matrix<float, 6, 6>::Zero();
  Adj.block<3,3>(0,0) = R;
  Adj.block<3,3>(3,3) = R;
  Adj.block<3,3>(0,3) = Sophus::SO3f::hat(t) * R;

  return Adj;
}

Eigen::Matrix3f RightJacobianSO3(const Eigen::Vector3f& phi) {
  Eigen::Matrix3f Jr = Eigen::Matrix3f::Identity();
  const float t = phi.norm();

  const Eigen::Matrix3f K = Sophus::SO3f::hat(phi);
  if (t < 1e-5f) {
    Jr -= 0.5f * K + (1.0f / 6.0f) * K * K;
  } else {
    const float t2 = t * t;
    const float t3 = t2 * t;
    Jr -= (1.0f - std::cos(t)) / t2 * K + (t - std::sin(t)) / t3 * K * K;
  }

  return Jr;
}

Eigen::Matrix3f LeftJacobianInvSO3(const Eigen::Vector3f& phi) {
  const float t = phi.norm();
  const Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
  const Eigen::Matrix3f S = Sophus::SO3f::hat(phi);

  if (t < 1e-5f) {
    return I - 0.5f * S;
  }

  const float t2 = t * t;
  const float cot = 1.0f / tan(0.5f * t);

  return I - 0.5f * S + (1.0f - t * cot / 2.0f) / t2 * S * S;
}

Eigen::Matrix<float, 6, 6> LeftJacobianInvSE3(const Eigen::Matrix<float, 6, 1>& xi) {
  const Eigen::Vector3f v = xi.head<3>();
  const Eigen::Vector3f w = xi.tail<3>();

  const float t = w.norm();
  const Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
  const Eigen::Matrix3f W = Sophus::SO3f::hat(w);
  const Eigen::Matrix3f V = Sophus::SO3f::hat(v);

  Eigen::Matrix3f J_inv;
  if (t < 1e-5f) {
    J_inv = I - 0.5f * W;
  } else {
    const float t2 = t * t;
    const float cot = 1.0f / tan(0.5f * t);
    J_inv = I - 0.5f * W + (1.0f - t * cot / 2.0f) / t2 * W * W;
  }

  Eigen::Matrix3f Q = Eigen::Matrix3f::Zero();
  if (t >= 1e-5f) {
    const float a = 1.0f / (t * t) - (1.0f + std::cos(t)) / (2.0f * t * std::sin(t));
    Q = a * (W * V + V * W + W * V * W);
  }

  Eigen::Matrix<float, 6, 6> result = Eigen::Matrix<float, 6, 6>::Zero();
  result.block<3, 3>(0, 0) = J_inv;
  result.block<3, 3>(3, 3) = J_inv;
  result.block<3, 3>(0, 3) = -0.5f * V + Q;
  return result;
}

} // namespace pslam
