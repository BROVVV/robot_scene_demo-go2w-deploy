// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GO2W_FOXY_SPIN__SPIN_HPP_
#define GO2W_FOXY_SPIN__SPIN_HPP_

#include <chrono>
#include <string>
#include <memory>

#include "nav2_recoveries/recovery.hpp"
#include "go2w_nav2_spin_compat/collision_checker.hpp"
#include "go2w_nav2_compat_msgs/action/spin.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace go2w_foxy_spin
{
using SpinAction = go2w_nav2_compat_msgs::action::Spin;
using Status = nav2_recoveries::Status;

/**
 * @class nav2_behaviors::Spin
 * @brief An action server behavior for spinning in
 */
class Spin : public nav2_recoveries::Recovery<SpinAction>
{
public:
  /**
   * @brief A constructor for nav2_behaviors::Spin
   */
  Spin();
  ~Spin();

  /**
   * @brief Initialization to run behavior
   * @param command Goal to execute
   * @return Status of behavior
   */
  Status onRun(const std::shared_ptr<const SpinAction::Goal> command) override;

  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override;

  /**
   * @brief Loop function to run behavior
   * @return Status of behavior
   */
  Status onCycleUpdate() override;

protected:
  /**
   * @brief Check if pose is collision free
   * @param distance Distance to check forward
   * @param cmd_vel current commanded velocity
   * @param pose2d Current pose
   * @return is collision free or not
   */
  bool isCollisionFree(
    const double & distance,
    geometry_msgs::msg::Twist * cmd_vel,
    geometry_msgs::msg::Pose2D & pose2d);

  SpinAction::Feedback::SharedPtr feedback_;
  std::unique_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber_;
  std::unique_ptr<go2w_foxy_nav2::FootprintSubscriber> footprint_subscriber_;
  std::unique_ptr<CostmapTopicCollisionChecker> compatible_collision_checker_;

  double min_rotational_vel_;
  double max_rotational_vel_;
  double rotational_acc_lim_;
  double cmd_yaw_;
  double prev_yaw_;
  double relative_yaw_;
  double simulate_ahead_time_;
  rclcpp::Duration command_time_allowance_{0, 0};
  rclcpp::Time end_time_;
};

}  // namespace go2w_foxy_spin

#endif  // GO2W_FOXY_SPIN__SPIN_HPP_
