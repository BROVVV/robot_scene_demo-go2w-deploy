// Copyright 2026 robot_scene_demo maintainers
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
//
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// plain_slam_odom_adapter: IMU-frame plain_slam pose -> base-frame shadow odom.
//
// plain_slam's internal pose is in its IMU frame (pslam_imu).  The navigation
// stack prefers the robot base pose, so we convert with the manufacturer
// geometry automatically loaded from official_reference.yaml:
//
//   T_world_base = T_world_imu * inverse(T_base_imu)
//
// Outputs:
//   /go2w/slam/odom_base   nav_msgs/Odometry   (frame pslam_odom,
//                                               child base_link_mapping_assist)
//   /go2w/slam/base_pose   geometry_msgs/PoseStamped
//
// publish_tf defaults to false: this adapter is a shadow odometry source and
// must not fight the existing TF owners (odom -> base_link stays untouched).

#include <tf2_ros/transform_broadcaster.h>
#include <cmath>
#include <memory>
#include <string>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <go2w_plain_slam_bridge/transform_utils.hpp>

namespace go2w_plain_slam_bridge
{

inline RigidTransform rigid_from_pose(
  double x, double y, double z,
  double qx, double qy, double qz, double qw)
{
  RigidTransform t;
  const double x2 = qx + qx, y2 = qy + qy, z2 = qz + qz;
  const double xx = qx * x2, xy = qx * y2, xz = qx * z2;
  const double yy = qy * y2, yz = qy * z2, zz = qz * z2;
  const double wx = qw * x2, wy = qw * y2, wz = qw * z2;
  t.m[0] = 1.0 - (yy + zz);
  t.m[1] = xy - wz;
  t.m[2] = xz + wy;
  t.m[4] = xy + wz;
  t.m[5] = 1.0 - (xx + zz);
  t.m[6] = yz - wx;
  t.m[8] = xz - wy;
  t.m[9] = yz + wx;
  t.m[10] = 1.0 - (xx + yy);
  t.m[3] = x;
  t.m[7] = y;
  t.m[11] = z;
  return t;
}

class PlainSlamOdomAdapter : public rclcpp::Node
{
public:
  PlainSlamOdomAdapter()
  : Node("plain_slam_odom_adapter")
  {
    declare_parameter<std::string>("imu_pose_topic", "/go2w/slam/imu_pose_raw");
    declare_parameter<std::string>("imu_odom_topic", "/go2w/slam/imu_odom_raw");
    declare_parameter<std::string>("odom_base_topic", "/go2w/slam/odom_base");
    declare_parameter<std::string>("base_pose_topic", "/go2w/slam/base_pose");
    declare_parameter<std::string>("odom_frame", "pslam_odom");
    declare_parameter<std::string>("child_frame", "base_link_mapping_assist");
    declare_parameter<bool>("publish_tf", false);
    declare_parameter<std::string>("official_reference_file", "");

    imu_pose_topic_ = get_parameter("imu_pose_topic").as_string();
    imu_odom_topic_ = get_parameter("imu_odom_topic").as_string();
    odom_base_topic_ = get_parameter("odom_base_topic").as_string();
    base_pose_topic_ = get_parameter("base_pose_topic").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    child_frame_ = get_parameter("child_frame").as_string();
    publish_tf_ = get_parameter("publish_tf").as_bool();
    const std::string reference_file = get_parameter("official_reference_file").as_string();

    std::string error;
    if (!load_base_to_imu_transform(reference_file, t_base_imu_, error)) {
      RCLCPP_FATAL(
        get_logger(),
        "cannot load base->utlidar_imu from '%s': %s; adapter disabled "
        "(existing motion chain unaffected)", reference_file.c_str(),
        error.c_str());
      return;
    }
    t_base_imu_inv_ = t_base_imu_.inverse();
    RCLCPP_INFO(
      get_logger(),
      "loaded base->utlidar_imu: t=(%.4f, %.4f, %.4f); publish_tf=%s",
      t_base_imu_.m[3], t_base_imu_.m[7], t_base_imu_.m[11],
      publish_tf_ ? "true" : "false");

    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    if (!imu_odom_topic_.empty()) {
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        imu_odom_topic_, qos,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {on_odom(msg);});
    }
    if (!imu_pose_topic_.empty()) {
      pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        imu_pose_topic_, qos,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          on_pose_stamped(msg);
        });
    }
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_base_topic_, qos);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(base_pose_topic_, qos);
    if (publish_tf_) {
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }
  }

private:
  void publish_base_pose(
    const RigidTransform & t_world_base,
    const rclcpp::Time & stamp,
    const geometry_msgs::msg::Twist * twist)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = child_frame_;
    odom.pose.pose.position.x = t_world_base.m[3];
    odom.pose.pose.position.y = t_world_base.m[7];
    odom.pose.pose.position.z = t_world_base.m[11];
    // Recover quaternion from the rotation matrix (R = Rz*Ry*Rx).
    const double r00 = t_world_base.m[0], r11 = t_world_base.m[5];
    const double r22 = t_world_base.m[10];
    const double trace = r00 + r11 + r22;
    double qw, qx, qy, qz;
    if (trace > 0.0) {
      const double s = std::sqrt(trace + 1.0) * 2.0;
      qw = 0.25 * s;
      qx = (t_world_base.m[9] - t_world_base.m[6]) / s;
      qy = (t_world_base.m[2] - t_world_base.m[8]) / s;
      qz = (t_world_base.m[4] - t_world_base.m[1]) / s;
    } else if (r00 > r11 && r00 > r22) {
      const double s = std::sqrt(1.0 + r00 - r11 - r22) * 2.0;
      qw = (t_world_base.m[9] - t_world_base.m[6]) / s;
      qx = 0.25 * s;
      qy = (t_world_base.m[1] + t_world_base.m[4]) / s;
      qz = (t_world_base.m[2] + t_world_base.m[8]) / s;
    } else if (r11 > r22) {
      const double s = std::sqrt(1.0 + r11 - r00 - r22) * 2.0;
      qw = (t_world_base.m[2] - t_world_base.m[8]) / s;
      qx = (t_world_base.m[1] + t_world_base.m[4]) / s;
      qy = 0.25 * s;
      qz = (t_world_base.m[6] + t_world_base.m[9]) / s;
    } else {
      const double s = std::sqrt(1.0 + r22 - r00 - r11) * 2.0;
      qw = (t_world_base.m[4] - t_world_base.m[1]) / s;
      qx = (t_world_base.m[2] + t_world_base.m[8]) / s;
      qy = (t_world_base.m[6] + t_world_base.m[9]) / s;
      qz = 0.25 * s;
    }
    odom.pose.pose.orientation.w = qw;
    odom.pose.pose.orientation.x = qx;
    odom.pose.pose.orientation.y = qy;
    odom.pose.pose.orientation.z = qz;
    if (twist != nullptr) {
      // World-frame twist is frame invariant for a rigid body; the world
      // vector is the same whichever sensor frame measured it.
      odom.twist.twist = *twist;
    }
    odom_pub_->publish(odom);

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = odom.header;
    pose_msg.pose = odom.pose.pose;
    pose_pub_->publish(pose_msg);

    if (publish_tf_ && tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = odom_frame_;
      tf.child_frame_id = child_frame_;
      tf.transform.translation.x = odom.pose.pose.position.x;
      tf.transform.translation.y = odom.pose.pose.position.y;
      tf.transform.translation.z = odom.pose.pose.position.z;
      tf.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto & p = msg->pose.pose;
    const RigidTransform t_world_imu = rigid_from_pose(
      p.position.x, p.position.y, p.position.z,
      p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
    const RigidTransform t_world_base = t_world_imu.multiply(t_base_imu_inv_);
    publish_base_pose(t_world_base, msg->header.stamp, &msg->twist.twist);
  }

  void on_pose_stamped(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const auto & p = msg->pose;
    const RigidTransform t_world_imu = rigid_from_pose(
      p.position.x, p.position.y, p.position.z,
      p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
    const RigidTransform t_world_base = t_world_imu.multiply(t_base_imu_inv_);
    publish_base_pose(t_world_base, msg->header.stamp, nullptr);
  }

  std::string imu_pose_topic_;
  std::string imu_odom_topic_;
  std::string odom_base_topic_;
  std::string base_pose_topic_;
  std::string odom_frame_;
  std::string child_frame_;
  bool publish_tf_ = false;
  RigidTransform t_base_imu_;
  RigidTransform t_base_imu_inv_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace go2w_plain_slam_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<go2w_plain_slam_bridge::PlainSlamOdomAdapter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
