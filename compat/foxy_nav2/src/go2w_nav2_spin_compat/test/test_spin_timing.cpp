#include <chrono>
#include <thread>
#include "gtest/gtest.h"
#include "go2w_nav2_spin_compat/spin.hpp"

class SpinTiming : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node = std::make_shared<nav2_util::LifecycleNode>("spin_timing_test", "", false);
    node->declare_parameter("cycle_frequency", 10.0);
    node->declare_parameter("global_frame", std::string("odom_fused"));
    node->declare_parameter("robot_base_frame", std::string("base_link"));
    node->declare_parameter("transform_tolerance", 0.05);
    node->declare_parameter("costmap_topic", std::string("/acceptance/costmap"));
    node->declare_parameter("footprint_topic", std::string("/acceptance/footprint"));
    tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    received = false;
    subscription = node->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        received = true; last = *msg;
      });
    spin = std::make_unique<go2w_foxy_spin::Spin>();
    spin->configure(node, "acceptance_spin", tf, nullptr);
    spin->activate();
    const auto deadline = std::chrono::steady_clock::now()+std::chrono::seconds(1);
    while (node->count_publishers("cmd_vel") == 0 && std::chrono::steady_clock::now()<deadline) {
      rclcpp::spin_some(node->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  void TearDown() override
  {
    spin->deactivate(); spin->cleanup(); spin.reset();
    subscription.reset(); tf.reset(); node.reset();
  }
  void pose()
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "odom_fused";
    transform.child_frame_id = "base_link";
    transform.header.stamp = node->get_clock()->now();
    transform.transform.rotation.w = 1.;
    ASSERT_TRUE(tf->setTransform(transform, "acceptance", true));
  }
  void expect_stopped()
  {
    const auto deadline = std::chrono::steady_clock::now()+std::chrono::seconds(1);
    while (!received && std::chrono::steady_clock::now()<deadline) {
      rclcpp::spin_some(node->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    ASSERT_TRUE(received);
    EXPECT_DOUBLE_EQ(last.linear.x, 0.); EXPECT_DOUBLE_EQ(last.angular.z, 0.);
  }
  std::shared_ptr<nav2_util::LifecycleNode> node;
  std::shared_ptr<tf2_ros::Buffer> tf;
  std::unique_ptr<go2w_foxy_spin::Spin> spin;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription;
  geometry_msgs::msg::Twist last;
  bool received;
};

TEST_F(SpinTiming, missing_pose_rejects_and_publishes_zero)
{
  auto goal = std::make_shared<go2w_nav2_compat_msgs::action::Spin::Goal>();
  goal->target_yaw = 1.; goal->time_allowance.sec = 1;
  EXPECT_EQ(spin->onRun(goal), nav2_recoveries::Status::FAILED);
  expect_stopped();
}

TEST_F(SpinTiming, server_deadline_stops_without_a_client_cancel)
{
  pose();
  auto goal = std::make_shared<go2w_nav2_compat_msgs::action::Spin::Goal>();
  goal->target_yaw = 1.; goal->time_allowance.nanosec = 10000000;
  ASSERT_EQ(spin->onRun(goal), nav2_recoveries::Status::SUCCEEDED);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  EXPECT_EQ(spin->onCycleUpdate(), nav2_recoveries::Status::FAILED);
  expect_stopped();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
