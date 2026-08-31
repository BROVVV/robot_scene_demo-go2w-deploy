#pragma once

#include <cmath>

namespace go2w_motion_control {

inline double NormalizeAngle(double angle_rad) {
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kTwoPi = 2.0 * kPi;
  while (angle_rad > kPi) {
    angle_rad -= kTwoPi;
  }
  while (angle_rad <= -kPi) {
    angle_rad += kTwoPi;
  }
  return angle_rad;
}

inline double DegreesToRadians(double degrees) {
  return degrees * 3.14159265358979323846 / 180.0;
}

inline double RadiansToDegrees(double radians) {
  return radians * 180.0 / 3.14159265358979323846;
}

class YawUnwrapper {
 public:
  void Reset(double raw_yaw) {
    initialized_ = true;
    previous_raw_ = raw_yaw;
    unwrapped_ = raw_yaw;
  }

  double Update(double raw_yaw) {
    if (!initialized_) {
      Reset(raw_yaw);
      return unwrapped_;
    }
    unwrapped_ += NormalizeAngle(raw_yaw - previous_raw_);
    previous_raw_ = raw_yaw;
    return unwrapped_;
  }

  double Value() const { return unwrapped_; }
  bool Initialized() const { return initialized_; }

 private:
  bool initialized_{false};
  double previous_raw_{0.0};
  double unwrapped_{0.0};
};

}  // namespace go2w_motion_control
