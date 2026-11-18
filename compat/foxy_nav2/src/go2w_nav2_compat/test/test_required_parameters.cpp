#include <gtest/gtest.h>
#include <limits>
#include "go2w_nav2_compat/compat.hpp"

class RequiredParameters : public ::testing::Test {
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(RequiredParameters, MissingGeometryIsRejectedAndCanBeCorrected) {
  auto node = std::make_shared<rclcpp::Node>("required_geometry_test");
  EXPECT_THROW(go2w_foxy_nav2::declare_parameter_if_not_declared(
    node, "radius", rclcpp::PARAMETER_DOUBLE), std::exception);
  ASSERT_TRUE(node->has_parameter("radius"));
  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("radius", 0.2)).successful);
  EXPECT_NO_THROW(go2w_foxy_nav2::declare_parameter_if_not_declared(
    node, "radius", rclcpp::PARAMETER_DOUBLE));
  EXPECT_DOUBLE_EQ(node->get_parameter("radius").as_double(), 0.2);
}

TEST_F(RequiredParameters, WrongTypeIsNotConvertedOrGivenADefault) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("radius", "0.2")});
  auto node = std::make_shared<rclcpp::Node>("wrong_geometry_test", options);
  EXPECT_THROW(go2w_foxy_nav2::declare_parameter_if_not_declared(
    node, "radius", rclcpp::PARAMETER_DOUBLE), std::exception);
}

TEST_F(RequiredParameters, ObservationSourcesAreRequired) {
  auto node = std::make_shared<rclcpp::Node>("required_sources_test");
  EXPECT_THROW(go2w_foxy_nav2::declare_parameter_if_not_declared(
    node, "observation_sources", rclcpp::PARAMETER_STRING_ARRAY), std::exception);
}

TEST_F(RequiredParameters, FiniteTwistValidationRejectsInvalidCommands) {
  geometry_msgs::msg::Twist command;
  EXPECT_TRUE(go2w_foxy_nav2::validateTwist(command));
  command.linear.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(go2w_foxy_nav2::validateTwist(command));
  command.linear.x = 0.;
  command.angular.z = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(go2w_foxy_nav2::validateTwist(command));
}
