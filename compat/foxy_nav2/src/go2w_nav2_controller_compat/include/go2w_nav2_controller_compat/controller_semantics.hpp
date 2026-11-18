// SPDX-License-Identifier: Apache-2.0
// Path feedback follows the pinned Nav2 1.1.20 nearest-pose / remaining-length
// policy. Keep these kernels independent of ROS execution for deterministic QA.
#ifndef GO2W_NAV2_CONTROLLER_COMPAT__CONTROLLER_SEMANTICS_HPP_
#define GO2W_NAV2_CONTROLLER_COMPAT__CONTROLLER_SEMANTICS_HPP_

#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

namespace go2w_foxy_controller
{
// A navigation target is fixed in its map frame, rather than a sensor sample.
// Transform it at the same time as the current robot pose used for comparison.
template<class Pose>
Pose endpointAtPoseTime(const Pose & endpoint, const Pose & robot_pose)
{
  Pose current = endpoint;
  current.header.stamp = robot_pose.header.stamp;
  return current;
}

template<class Path, class Pose>
double remainingPathLength(const Path & path, const Pose & robot_pose)
{
  if (path.header.frame_id != robot_pose.header.frame_id) {
    throw std::invalid_argument("path feedback requires a pose in the path frame");
  }
  std::size_t closest = 0;
  double minimum = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    const auto & point = path.poses[i].pose.position;
    const auto & robot = robot_pose.pose.position;
    const double distance = std::hypot(point.x - robot.x, point.y - robot.y);
    if (distance < minimum) {
      minimum = distance;
      closest = i;
    }
  }
  double length = 0.0;
  for (std::size_t i = closest; i + 1 < path.poses.size(); ++i) {
    const auto & before = path.poses[i].pose.position;
    const auto & after = path.poses[i + 1].pose.position;
    length += std::hypot(after.x - before.x, after.y - before.y);
  }
  return length;
}

enum class CostmapWait {Ready, Canceled, Unavailable};

// Recheck cancellation and liveness every bounded sleep, including when the
// costmap never becomes current. A stale map must not retain a nonzero command.
template<class Current, class Active, class Cancel, class Zero, class Sleep>
CostmapWait waitForCurrentCostmap(Current current, Active active, Cancel cancel,
  Zero zero, Sleep sleep)
{
  bool stopped = false;
  for (;;) {
    if (!active()) {
      zero();
      return CostmapWait::Unavailable;
    }
    if (cancel()) {
      zero();
      return CostmapWait::Canceled;
    }
    if (current()) {
      return CostmapWait::Ready;
    }
    if (!stopped) {
      zero();
      stopped = true;
    }
    sleep();
  }
}
}  // namespace go2w_foxy_controller
#endif  // GO2W_NAV2_CONTROLLER_COMPAT__CONTROLLER_SEMANTICS_HPP_
