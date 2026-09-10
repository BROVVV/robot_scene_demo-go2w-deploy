#include <gtest/gtest.h>

#include "go2w_motion_control/yaw_controller.hpp"

using namespace go2w_motion_control;

TEST(YawController, SaturatesFarTarget) {
  YawControlParameters parameters;
  EXPECT_NEAR(ComputeLogicalYawRate(DegreesToRadians(90.0), 0.2, parameters),
              0.2, 1e-9);
}

TEST(YawController, UsesMinimumRateNearTarget) {
  YawControlParameters parameters;
  EXPECT_NEAR(ComputeLogicalYawRate(DegreesToRadians(3.0), 0.2, parameters),
              0.05, 1e-9);
  EXPECT_NEAR(ComputeLogicalYawRate(DegreesToRadians(-3.0), 0.2, parameters),
              -0.05, 1e-9);
}

TEST(YawController, StopsInsideToleranceAndDetectsOvershoot) {
  YawControlParameters parameters;
  EXPECT_DOUBLE_EQ(
      ComputeLogicalYawRate(DegreesToRadians(1.0), 0.2, parameters), 0.0);
  EXPECT_TRUE(ErrorCrossedTarget(0.1, -0.01));
  EXPECT_FALSE(ErrorCrossedTarget(0.1, 0.01));
}

TEST(YawController, TapersAndLimitsLongitudinalTurnCompensation) {
  EXPECT_DOUBLE_EQ(ComputeTurnLongitudinalCompensation(
                       DegreesToRadians(10.0), 1.0, 5.0, 0.04, 0.06),
                   0.04);
  EXPECT_NEAR(ComputeTurnLongitudinalCompensation(
                  DegreesToRadians(3.0), 1.0, 5.0, 0.04, 0.06),
              0.02, 1e-9);
  EXPECT_DOUBLE_EQ(ComputeTurnLongitudinalCompensation(
                       DegreesToRadians(1.0), 1.0, 5.0, 0.04, 0.06),
                   0.0);
  EXPECT_DOUBLE_EQ(ComputeTurnLongitudinalCompensation(
                       DegreesToRadians(-20.0), 1.0, 5.0, 0.10, 0.06),
                   0.06);
  EXPECT_DOUBLE_EQ(ComputeTurnLongitudinalCompensation(
                       DegreesToRadians(20.0), 1.0, 5.0, -0.10, 0.06),
                   -0.06);
}
