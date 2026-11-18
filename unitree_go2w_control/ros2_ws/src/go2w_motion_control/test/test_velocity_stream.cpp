#include <gtest/gtest.h>
#include <limits>
#include "go2w_motion_control/velocity_stream.hpp"
using namespace go2w_motion_control;
TEST(VelocityStream, RequiresCurrentMatchingTokenAndExpiresWithoutRefresh) {
  VelocityStream s;
  EXPECT_FALSE(s.update("a",{.05,0,0},100,100,1));
  s.reset("a");
  EXPECT_FALSE(s.current(1,.25));
  EXPECT_TRUE(s.update("a",{.05,0,0},100,100,1));
  ASSERT_TRUE(s.current(1.24,.25));
  EXPECT_DOUBLE_EQ(s.current(1.24,.25)->vx,.05);
  EXPECT_FALSE(s.current(1.25,.25));
  EXPECT_FALSE(s.update("b",{.05,0,0},100.3,100.3,1.3));
  EXPECT_FALSE(s.current(1.3,.25));
  s.reset("b");
  EXPECT_FALSE(s.current(1.3,.25));
}
TEST(VelocityStream, ReplayedStaleFutureOrUnsafeUpdatesDoNotRenew) {
  VelocityStream s;s.reset("a");
  EXPECT_TRUE(s.update("a",{.05,0,0},100,100,1));
  EXPECT_FALSE(s.update("a",{.05,0,0},100,100.2,1.2));
  EXPECT_FALSE(s.update("a",{.05,0,0},100.01,101,1.2));
  EXPECT_FALSE(s.update("a",{.05,0,0},102,101,1.2));
  EXPECT_FALSE(s.update("a",{.31,0,0},100.1,100.1,1.2));
  EXPECT_FALSE(s.update("a",{.05,.01,0},100.1,100.1,1.2));
  EXPECT_FALSE(s.update("a",{.05,0,.21},100.1,100.1,1.2));
  EXPECT_FALSE(s.update("a",{std::numeric_limits<double>::quiet_NaN(),0,0},100.1,100.1,1.2));
  EXPECT_FALSE(s.current(1.26,.25));
  EXPECT_FALSE(s.current(1.1,.26));
  EXPECT_TRUE(s.update("a",{.30,0,.1},100.3,100.3,1.3));
  EXPECT_DOUBLE_EQ(s.current(1.31,.25)->yaw,.1);
  s.reset("");
  EXPECT_FALSE(s.update("a",{.05,0,0},100.4,100.4,1.4));
}
