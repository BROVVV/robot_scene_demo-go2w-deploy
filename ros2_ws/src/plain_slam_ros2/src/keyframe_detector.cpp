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

#include <plain_slam/keyframe_detector.hpp>

namespace pslam {

KeyframeDetector::KeyframeDetector() {
  dist_th_ = 5.0f;
  angle_th_ = 180.0f * M_PI / 180.0f;

  is_empty_ = true;
}

KeyframeDetector::~KeyframeDetector() {

}

bool KeyframeDetector::IsKeyframe(const Sophus::SE3f& pose) {
  if (is_empty_) {
    prev_pose_ = pose;
    is_empty_ = false;
    return true;
  }

  const Sophus::SE3f dT = prev_pose_.inverse() * pose;
  const float d_dist = dT.translation().norm();
  const float d_angle = dT.so3().log().norm();

  if (d_dist > dist_th_ || d_angle > angle_th_) {
    return true;
  }

  return false;
}

} // namespace pslam
