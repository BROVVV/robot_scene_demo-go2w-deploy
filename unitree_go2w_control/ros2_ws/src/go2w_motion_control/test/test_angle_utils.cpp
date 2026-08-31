#include <gtest/gtest.h>

#include "go2w_motion_control/angle_utils.hpp"

using namespace go2w_motion_control;

TEST(AngleUtils, NormalizeAndConvert) {
  EXPECT_NEAR(NormalizeAngle(DegreesToRadians(181.0)), DegreesToRadians(-179.0),
              1e-9);
  EXPECT_NEAR(RadiansToDegrees(DegreesToRadians(-45.0)), -45.0, 1e-9);
}

TEST(AngleUtils, UnwrapAcrossPositivePi) {
  YawUnwrapper unwrap;
  unwrap.Reset(DegreesToRadians(179.0));
  EXPECT_NEAR(RadiansToDegrees(unwrap.Update(DegreesToRadians(-179.0))), 181.0,
              1e-6);
}

TEST(AngleUtils, UnwrapAcrossNegativePi) {
  YawUnwrapper unwrap;
  unwrap.Reset(DegreesToRadians(-179.0));
  EXPECT_NEAR(RadiansToDegrees(unwrap.Update(DegreesToRadians(179.0))), -181.0,
              1e-6);
}
