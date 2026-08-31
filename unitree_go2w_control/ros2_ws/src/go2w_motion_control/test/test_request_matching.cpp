#include <gtest/gtest.h>

#include "go2w_motion_control/request_matching.hpp"

using namespace go2w_motion_control;

TEST(RequestMatching, RequiresBothIdentityFields) {
  EXPECT_TRUE(ResponseMatches(10, 1008, 10, 1008));
  EXPECT_FALSE(ResponseMatches(10, 1008, 11, 1008));
  EXPECT_FALSE(ResponseMatches(10, 1008, 10, 1003));
}

TEST(RequestMatching, RejectsOldAndOutOfOrderResponses) {
  EXPECT_FALSE(ResponseMatches(200, 1003, 199, 1003));
  EXPECT_FALSE(ResponseMatches(200, 1003, 201, 1003));
}
