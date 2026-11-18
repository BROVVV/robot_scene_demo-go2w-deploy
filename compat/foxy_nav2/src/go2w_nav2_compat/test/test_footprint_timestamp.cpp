#include <gtest/gtest.h>
#include <cmath>
#include "go2w_nav2_compat/footprint_subscriber.hpp"

class FootprintProbe : public go2w_foxy_nav2::FootprintSubscriber {
public:
  using go2w_foxy_nav2::FootprintSubscriber::FootprintSubscriber;
  using go2w_foxy_nav2::FootprintSubscriber::footprint_callback;
};

class FootprintTimestamp : public ::testing::Test {
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(FootprintTimestamp, UsesFootprintTimeInsteadOfLatestRobotPose) {
  auto node = std::make_shared<rclcpp::Node>("footprint_timestamp_test");
  tf2_ros::Buffer buffer(node->get_clock());
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "odom";
  transform.child_frame_id = "base_link";
  transform.header.stamp.sec = 10;
  transform.transform.translation.x = 1.;
  transform.transform.translation.y = 2.;
  transform.transform.rotation.z = std::sin(M_PI / 4.);
  transform.transform.rotation.w = std::cos(M_PI / 4.);
  ASSERT_TRUE(buffer.setTransform(transform, "test", false));
  transform.header.stamp.sec = 20;
  transform.transform.translation.x = 9.;
  ASSERT_TRUE(buffer.setTransform(transform, "test", false));
  FootprintProbe subscriber(rclcpp::Node::WeakPtr(node), "footprint", buffer, "base_link", 0.0);
  auto message = std::make_shared<geometry_msgs::msg::PolygonStamped>();
  message->header.frame_id = "odom";
  message->header.stamp.sec = 10;
  geometry_msgs::msg::Point32 point;
  point.x = 1.; point.y = 3.; message->polygon.points.push_back(point);
  point.x = 0.; point.y = 2.; message->polygon.points.push_back(point);
  subscriber.footprint_callback(message);
  std::vector<geometry_msgs::msg::Point> footprint;
  std_msgs::msg::Header header;
  ASSERT_TRUE(subscriber.getFootprintInRobotFrame(footprint, header));
  ASSERT_EQ(footprint.size(), 2u);
  EXPECT_NEAR(footprint[0].x, 1., 1e-6);
  EXPECT_NEAR(footprint[0].y, 0., 1e-6);
  EXPECT_NEAR(footprint[1].x, 0., 1e-6);
  EXPECT_NEAR(footprint[1].y, 1., 1e-6);
  EXPECT_EQ(header.frame_id, "base_link");
  EXPECT_EQ(header.stamp.sec, 10);
  // A newer footprint without matching TF must fail rather than use latest.
  message->header.stamp.sec = 30;
  subscriber.footprint_callback(message);
  EXPECT_FALSE(subscriber.getFootprintInRobotFrame(footprint, header));
}

TEST_F(FootprintTimestamp, NoFootprintIsRejected) {
  auto node = std::make_shared<rclcpp::Node>("missing_footprint_test");
  tf2_ros::Buffer buffer(node->get_clock());
  FootprintProbe subscriber(rclcpp::Node::WeakPtr(node), "footprint", buffer, "base_link", 0.0);
  std::vector<geometry_msgs::msg::Point> footprint;
  std_msgs::msg::Header header;
  EXPECT_FALSE(subscriber.getFootprintInRobotFrame(footprint, header));
}
