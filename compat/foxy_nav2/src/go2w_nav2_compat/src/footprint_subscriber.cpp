// Copyright (c) 2019 Intel Corporation
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

#include <string>
#include <vector>
#include <memory>

#include "go2w_nav2_compat/footprint_subscriber.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

namespace go2w_foxy_nav2
{

FootprintSubscriber::FootprintSubscriber(
  const nav2_util::LifecycleNode::WeakPtr & parent,
  const std::string & topic_name,
  tf2_ros::Buffer & tf,
  std::string robot_base_frame,
  double transform_tolerance)
: tf_(tf),
  robot_base_frame_(robot_base_frame),
  transform_tolerance_(transform_tolerance)
{
  auto node = parent.lock();
  footprint_sub_ = node->create_subscription<geometry_msgs::msg::PolygonStamped>(
    topic_name, rclcpp::SystemDefaultsQoS(),
    std::bind(&FootprintSubscriber::footprint_callback, this, std::placeholders::_1));
}

FootprintSubscriber::FootprintSubscriber(
  const rclcpp::Node::WeakPtr & parent,
  const std::string & topic_name,
  tf2_ros::Buffer & tf,
  std::string robot_base_frame,
  double transform_tolerance)
: tf_(tf),
  robot_base_frame_(robot_base_frame),
  transform_tolerance_(transform_tolerance)
{
  auto node = parent.lock();
  footprint_sub_ = node->create_subscription<geometry_msgs::msg::PolygonStamped>(
    topic_name, rclcpp::SystemDefaultsQoS(),
    std::bind(&FootprintSubscriber::footprint_callback, this, std::placeholders::_1));
}

bool
FootprintSubscriber::getFootprintRaw(
  std::vector<geometry_msgs::msg::Point> & footprint,
  std_msgs::msg::Header & footprint_header)
{
  if (!footprint_received_) {
    return false;
  }

  auto current_footprint = std::atomic_load(&footprint_);
  footprint = nav2_costmap_2d::toPointVector(
    std::make_shared<geometry_msgs::msg::Polygon>(current_footprint->polygon));
  footprint_header = current_footprint->header;

  return true;
}

bool
FootprintSubscriber::getFootprintInRobotFrame(
  std::vector<geometry_msgs::msg::Point> & footprint,
  std_msgs::msg::Header & footprint_header)
{
  if (!getFootprintRaw(footprint, footprint_header)) {
    return false;
  }

  // Foxy's getCurrentPose has no timestamp argument. Preserve the upstream
  // footprint timestamp; substituting latest TF would mix two robot poses.
  geometry_msgs::msg::PoseStamped current_pose;
  try {
    const auto transform = tf_.lookupTransform(
      footprint_header.frame_id, robot_base_frame_,
      rclcpp::Time(footprint_header.stamp),
      rclcpp::Duration::from_seconds(transform_tolerance_));
    current_pose.header = transform.header;
    current_pose.pose.position.x = transform.transform.translation.x;
    current_pose.pose.position.y = transform.transform.translation.y;
    current_pose.pose.position.z = transform.transform.translation.z;
    current_pose.pose.orientation = transform.transform.rotation;
  } catch (const tf2::TransformException &) {
    return false;
  }

  double x = current_pose.pose.position.x;
  double y = current_pose.pose.position.y;
  double theta = tf2::getYaw(current_pose.pose.orientation);

  std::vector<geometry_msgs::msg::Point> temp;
  nav2_costmap_2d::transformFootprint(-x, -y, 0, footprint, temp);
  nav2_costmap_2d::transformFootprint(0, 0, -theta, temp, footprint);

  footprint_header.frame_id = robot_base_frame_;
  footprint_header.stamp = current_pose.header.stamp;

  return true;
}

void
FootprintSubscriber::footprint_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{
  std::atomic_store(&footprint_, msg);
  if (!footprint_received_) {
    footprint_received_ = true;
  }
}

}  // namespace go2w_foxy_nav2
