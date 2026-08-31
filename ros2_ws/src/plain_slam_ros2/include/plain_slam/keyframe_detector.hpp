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

#include <cmath>

#include <sophus/se3.hpp>

namespace pslam {

class KeyframeDetector {
 public:
  KeyframeDetector();

  ~KeyframeDetector();

  bool IsKeyframe(const Sophus::SE3f& pose);

  void SetDistanceThreshold(float th) {
    dist_th_ = th;
  }

  void SetAngleThreshold(float th) {
    angle_th_ = th;
  }

  void AddKeyframe(const Sophus::SE3f& pose) {
    prev_pose_ = pose;
    is_empty_ = false;
  }

  void UpdateKeyframe(const Sophus::SE3f& pose) {
    prev_pose_ = pose;
  }

  void Reset() {
    is_empty_ = true;
  }

 private:
  Sophus::SE3f prev_pose_;
  float dist_th_;
  float angle_th_;
  bool is_empty_;
};

} // namespace pslam
