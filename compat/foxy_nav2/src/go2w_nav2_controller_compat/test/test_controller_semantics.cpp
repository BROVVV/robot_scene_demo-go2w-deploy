// SPDX-License-Identifier: Apache-2.0
// Exercise the production kernels with real ROS message/parameter types and
// an in-memory TF buffer. No ROS init, executor, DDS participant or SDK is used.
#include <cmath>
#include <future>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <thread>
#include <vector>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/buffer_core.h"
#if __has_include("tf2_geometry_msgs/tf2_geometry_msgs.hpp")
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#else
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#endif
#include "go2w_nav2_controller_compat/controller_parameters.hpp"
#include "go2w_nav2_controller_compat/controller_semantics.hpp"

using go2w_foxy_controller::ControllerParameters;
using go2w_foxy_controller::CostmapWait;
using go2w_foxy_controller::remainingPathLength;
using go2w_foxy_controller::waitForCurrentCostmap;
namespace
{
void expect(bool value, const char * message)
{
  if (!value) {throw std::runtime_error(message);}
}
void near(double actual, double expected)
{
  expect(std::abs(actual - expected) < 1.0e-10, "wrong geometric distance");
}
geometry_msgs::msg::PoseStamped pose(double x, double y, int second = 20)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map"; p.header.stamp.sec = second;
  p.pose.position.x = x; p.pose.position.y = y; p.pose.orientation.w = 1.;
  return p;
}
template<class Check>
void duringAction(ControllerParameters & parameters, Check check)
{
  std::promise<void> locked, release;
  auto done = release.get_future();
  std::thread action([&]() {
    std::lock_guard<std::mutex> guard(parameters.actionMutex());
    locked.set_value(); done.wait();
  });
  locked.get_future().wait();
  try {check();} catch (...) {
    release.set_value(); action.join(); throw;
  }
  release.set_value(); action.join();
}
}  // namespace

int main()
{
  int passed = 0;
  try {
    expect(!rclcpp::ok(), "test unexpectedly has an active ROS context");
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.poses = {pose(0, 0), pose(0, 2), pose(2, 2), pose(2, 0)};
    near(remainingPathLength(path, pose(0, 0)), 6.); ++passed;
    near(remainingPathLength(path, pose(1.7, 2)), 2.); ++passed;
    near(remainingPathLength(path, pose(1, 2)), 4.); ++passed;  // Equal minima choose first.
    {
      auto elevated = path;
      for (std::size_t i = 0; i < elevated.poses.size(); ++i) {
        elevated.poses[i].pose.position.z = i * 100.;
      }
      near(remainingPathLength(elevated, pose(0, 0)), 6.); ++passed;
    }
    {
      auto empty = path; empty.poses.clear();
      near(remainingPathLength(empty, pose(0, 0)), 0.);
      empty.poses.push_back(pose(0, 0));
      near(remainingPathLength(empty, pose(9, 9)), 0.); ++passed;
    }
    {
      auto wrong = pose(0, 0); wrong.header.frame_id = "odom_fused";
      bool rejected = false;
      try {remainingPathLength(path, wrong);} catch (const std::invalid_argument &) {rejected = true;}
      expect(rejected, "mixed-frame feedback was accepted"); ++passed;
    }
    {
      // Independent reference over nontrivial paths, without ROS computation.
      std::mt19937 random(20261106);
      std::uniform_real_distribution<double> coordinate(-5., 5.);
      for (int sample = 0; sample < 100; ++sample) {
        auto generated = path; generated.poses.clear();
        for (int i = 0; i < 20; ++i) {generated.poses.push_back(pose(coordinate(random), coordinate(random)));}
        auto robot = pose(coordinate(random), coordinate(random));
        std::size_t closest = 0; double minimum = std::numeric_limits<double>::max();
        for (std::size_t i = 0; i < generated.poses.size(); ++i) {
          const auto & p = generated.poses[i].pose.position;
          double d = std::pow(p.x - robot.pose.position.x, 2) + std::pow(p.y - robot.pose.position.y, 2);
          if (d < minimum) {minimum = d; closest = i;}
        }
        double reference = 0.;
        for (std::size_t i = closest + 1; i < generated.poses.size(); ++i) {
          const auto & a = generated.poses[i-1].pose.position;
          const auto & b = generated.poses[i].pose.position;
          reference += std::sqrt(std::pow(a.x-b.x, 2) + std::pow(a.y-b.y, 2));
        }
        near(remainingPathLength(generated, robot), reference);
      }
      ++passed;
    }
    {
      // A fixed map target must be transformed at current comparison time.
      // map->odom shifts between planning and checking; old TF yields x=4,
      // current TF yields x=1, matching the current robot in odom_fused.
      tf2::BufferCore buffer;
      geometry_msgs::msg::TransformStamped tf;
      tf.header.frame_id = "map"; tf.child_frame_id = "odom_fused";
      tf.transform.rotation.w = 1.;
      tf.header.stamp.sec = 10; tf.transform.translation.x = 1.;
      expect(buffer.setTransform(tf, "offline_test"), "old transform rejected");
      tf.header.stamp.sec = 20; tf.transform.translation.x = 4.;
      expect(buffer.setTransform(tf, "offline_test"), "new transform rejected");
      auto goal = pose(5, 0, 10), robot = pose(1, 0, 20);
      robot.header.frame_id = "odom_fused";
      const auto current = go2w_foxy_controller::endpointAtPoseTime(goal, robot);
      expect(goal.header.stamp.sec == 10 && current.header.stamp.sec == 20, "goal time policy or input mutated");
      geometry_msgs::msg::PoseStamped old_out, current_out;
      tf2::doTransform(goal, old_out, buffer.lookupTransform("odom_fused", "map", tf2::TimePoint(tf2::durationFromSec(10.))));
      tf2::doTransform(current, current_out, buffer.lookupTransform("odom_fused", "map", tf2::TimePoint(tf2::durationFromSec(20.))));
      near(old_out.pose.position.x, 4.); near(current_out.pose.position.x, robot.pose.position.x);
      expect(current_out.header.frame_id == "odom_fused", "goal check frame mismatch"); ++passed;
    }
    {
      int sleeps = 0, zeroes = 0;
      auto result = waitForCurrentCostmap([](){return true;}, [](){return true;},
        [](){return false;}, [&](){++zeroes;}, [&](){++sleeps;});
      expect(result == CostmapWait::Ready && sleeps == 0 && zeroes == 0, "current map altered command flow"); ++passed;
    }
    {
      int sleeps = 0, zeroes = 0;
      auto result = waitForCurrentCostmap([&](){return sleeps == 3;}, [](){return true;},
        [](){return false;}, [&](){++zeroes;}, [&](){++sleeps; expect(sleeps <= 3, "unbounded map wait");});
      expect(result == CostmapWait::Ready && sleeps == 3 && zeroes == 1, "stale map failed to stop then recover"); ++passed;
    }
    {
      int sleeps = 0, zeroes = 0;
      auto result = waitForCurrentCostmap([](){return false;}, [](){return true;},
        [&](){return sleeps == 3;}, [&](){++zeroes;}, [&](){++sleeps; expect(sleeps <= 3, "cancel blocked by stale map");});
      expect(result == CostmapWait::Canceled && zeroes == 2, "stale map lost cancellation"); ++passed;
    }
    {
      int sleeps = 0, zeroes = 0;
      auto result = waitForCurrentCostmap([](){return false;}, [&](){return sleeps < 3;},
        [](){return false;}, [&](){++zeroes;}, [&](){++sleeps; expect(sleeps <= 3, "shutdown blocked by stale map");});
      expect(result == CostmapWait::Unavailable && zeroes == 2, "inactive server resumed computation"); ++passed;
    }
    {
      int zeroes = 0;
      auto result = waitForCurrentCostmap([](){return true;}, [](){return true;},
        [](){return true;}, [&](){++zeroes;}, [](){throw std::runtime_error("cancel must not sleep");});
      expect(result == CostmapWait::Canceled && zeroes == 1, "ready map overrode cancel"); ++passed;
    }
    {
      ControllerParameters parameters;
      expect(parameters.validate({rclcpp::Parameter("failure_tolerance", .3),
        rclcpp::Parameter("controller_frequency", 20.),
        rclcpp::Parameter("min_x_velocity_threshold", 0.),
        rclcpp::Parameter("publish_zero_velocity", false)}).successful, "valid idle core update rejected"); ++passed;
      duringAction(parameters, [&]() {
        expect(!parameters.validate({rclcpp::Parameter("failure_tolerance", .4)}).successful,
          "active core parameter update was acknowledged");
        expect(!parameters.validate({rclcpp::Parameter("controller_frequency", 10.)}).successful,
          "active controller rate update was acknowledged");
        expect(!parameters.validate({rclcpp::Parameter("publish_zero_velocity", false)}).successful,
          "active completion policy update was acknowledged");
        expect(parameters.validate({rclcpp::Parameter("FollowPath.desired_linear_vel", .12)}).successful,
          "plugin speed update blocked by action mutex");
      }); ++passed;
      expect(parameters.validate({rclcpp::Parameter("failure_tolerance", -1.)}).successful,
        "core update stayed blocked after action ended"); ++passed;
      for (const auto & invalid : std::vector<rclcpp::Parameter>{
          rclcpp::Parameter("failure_tolerance", -.2), rclcpp::Parameter("failure_tolerance", false),
          rclcpp::Parameter("publish_zero_velocity", .5),
          rclcpp::Parameter("controller_frequency", 0.), rclcpp::Parameter("controller_frequency", -1.),
          rclcpp::Parameter("min_y_velocity_threshold", -.1),
          rclcpp::Parameter("min_theta_velocity_threshold", std::numeric_limits<double>::infinity()),
          rclcpp::Parameter("min_x_velocity_threshold", std::numeric_limits<double>::quiet_NaN())})
      {
        expect(!parameters.validate({invalid}).successful, "invalid core parameter accepted");
      }
      ++passed;
    }
    expect(!rclcpp::ok(), "kernels unexpectedly initialized a ROS context");
  } catch (const std::exception & error) {
    std::cerr << "FAILED after " << passed << " cases: " << error.what() << '\n'; return 1;
  }
  std::cout << "controller_semantics: " << passed << " cases passed; no ROS context\n";
  return 0;
}
