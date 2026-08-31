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

#include <sophus/se3.hpp>

namespace pslam {

Eigen::Matrix<float, 6, 6> AdjointSE3(const Sophus::SE3f& T);

Eigen::Matrix3f RightJacobianSO3(const Eigen::Vector3f& phi);

Eigen::Matrix3f LeftJacobianInvSO3(const Eigen::Vector3f& phi);

Eigen::Matrix<float, 6, 6> LeftJacobianInvSE3(const Eigen::Matrix<float, 6, 1>& xi);

} // namespace pslam
