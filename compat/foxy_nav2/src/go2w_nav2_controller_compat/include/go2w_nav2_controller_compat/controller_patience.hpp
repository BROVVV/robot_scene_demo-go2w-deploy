// SPDX-License-Identifier: Apache-2.0
// Controller-exception policy from Nav2 1.1.20, isolated from ROS transport.
#ifndef GO2W_NAV2_CONTROLLER_COMPAT__CONTROLLER_PATIENCE_HPP_
#define GO2W_NAV2_CONTROLLER_COMPAT__CONTROLLER_PATIENCE_HPP_

#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace go2w_foxy_controller
{
inline bool validFailureTolerance(double seconds)
{
  return std::isfinite(seconds) && (seconds >= 0.0 || seconds == -1.0);
}

// Only the controller action's execution thread owns this state. Pose lookup
// and progress checks stay outside this policy, as in the pinned Humble server.
class ControllerPatience
{
public:
  void reset(std::int64_t now_ns) {last_valid_ns_ = now_ns;}

  template<class PlannerError, class Compute, class Now, class Zero, class Warn>
  auto compute(double tolerance, Compute compute, Now now, Zero zero, Warn warn)
  -> decltype(compute())
  {
    if (!validFailureTolerance(tolerance)) {
      throw std::invalid_argument("failure_tolerance must be finite and >= 0 or -1");
    }
    try {
      auto command = compute();
      last_valid_ns_ = now();
      return command;
    } catch (const PlannerError & error) {
      if (!(tolerance > 0.0 || tolerance == -1.0)) {
        throw;
      }
      warn(error.what());
      auto command = zero();
      const double elapsed = static_cast<double>(now() - last_valid_ns_) / 1.0e9;
      if (tolerance != -1.0 && elapsed > tolerance) {
        throw PlannerError("Controller patience exceeded");
      }
      return command;
    }
  }

private:
  std::int64_t last_valid_ns_{0};
};
}  // namespace go2w_foxy_controller
#endif  // GO2W_NAV2_CONTROLLER_COMPAT__CONTROLLER_PATIENCE_HPP_
