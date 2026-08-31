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

// Unit tests for the timestamp auto-policy (plan §18.2).

#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include <go2w_plain_slam_bridge/timestamp_policy.hpp>

using go2w_plain_slam_bridge::TimestampMode;
using go2w_plain_slam_bridge::resolve_timestamps;

namespace
{
constexpr double kHeader = 1700000000.0;  // arbitrary absolute epoch
}

TEST(TimestampPolicy, AbsoluteSeconds)
{
  const std::vector<double> raw = {kHeader, kHeader + 0.02, kHeader + 0.04};
  const auto result = resolve_timestamps(raw, kHeader, 0.1, 5.0);
  EXPECT_EQ(result.mode, TimestampMode::kAbsoluteSeconds);
  EXPECT_FALSE(result.non_monotonic);
  ASSERT_EQ(result.timestamps.size(), 3u);
  EXPECT_DOUBLE_EQ(result.timestamps[2], kHeader + 0.04);
}

TEST(TimestampPolicy, RelativeScan)
{
  const std::vector<double> raw = {0.0, 0.033, 0.066, 0.099};
  const auto result = resolve_timestamps(raw, kHeader, 0.1, 5.0);
  EXPECT_EQ(result.mode, TimestampMode::kRelativeScan);
  ASSERT_EQ(result.timestamps.size(), 4u);
  EXPECT_DOUBLE_EQ(result.timestamps[3], kHeader + 0.099);
}

TEST(TimestampPolicy, ZeroTimestampsSynthetic)
{
  const std::vector<double> raw = {0.0, 0.0, 0.0, 0.0};
  const auto result = resolve_timestamps(raw, kHeader, 0.1, 5.0);
  EXPECT_EQ(result.mode, TimestampMode::kSyntheticLinear);
  ASSERT_EQ(result.timestamps.size(), 4u);
  // Linear interpolation across the scan period.
  EXPECT_NEAR(result.timestamps[0], kHeader, 1e-9);
  EXPECT_NEAR(result.timestamps[3], kHeader + 0.1, 1e-9);
  EXPECT_FALSE(result.non_monotonic);
}

TEST(TimestampPolicy, EmptyTimestampsSynthetic)
{
  const std::vector<double> raw;
  const auto result = resolve_timestamps(raw, kHeader, 0.1, 5.0);
  EXPECT_EQ(result.mode, TimestampMode::kSyntheticLinear);
  EXPECT_TRUE(result.timestamps.empty());
}

TEST(TimestampPolicy, NaNIsErrorNeverSilent)
{
  const std::vector<double> raw = {kHeader, std::nan(""), kHeader};
  const auto result = resolve_timestamps(raw, kHeader, 0.1, 5.0);
  EXPECT_EQ(result.mode, TimestampMode::kError);
}

TEST(TimestampPolicy, InfIsErrorNeverSilent)
{
  const std::vector<double> raw = {kHeader, std::numeric_limits<double>::infinity()};
  const auto result = resolve_timestamps(raw, kHeader, 0.1, 5.0);
  EXPECT_EQ(result.mode, TimestampMode::kError);
}

TEST(TimestampPolicy, MicrosecondsConverted)
{
  // 1.7e15 us == 1.7e9 s, aligned with the header within tolerance.
  const double us_epoch = kHeader * 1e6;
  const std::vector<double> raw = {us_epoch, us_epoch + 20000.0};
  const auto result = resolve_timestamps(raw, kHeader, 0.1, 5.0);
  EXPECT_EQ(result.mode, TimestampMode::kConvertedUnits);
  ASSERT_EQ(result.timestamps.size(), 2u);
  EXPECT_NEAR(result.timestamps[1], kHeader + 0.02, 1e-6);
}

TEST(TimestampPolicy, NanosecondsConverted)
{
  const double ns_epoch = kHeader * 1e9;
  const std::vector<double> raw = {ns_epoch, ns_epoch + 50000000.0};
  const auto result = resolve_timestamps(raw, kHeader, 0.1, 5.0);
  EXPECT_EQ(result.mode, TimestampMode::kConvertedUnits);
  ASSERT_EQ(result.timestamps.size(), 2u);
  EXPECT_NEAR(result.timestamps[1], kHeader + 0.05, 1e-6);
}

TEST(TimestampPolicy, NonMonotonicOutlierFlagged)
{
  const std::vector<double> raw = {kHeader + 0.0, kHeader - 0.1, kHeader + 0.03};
  const auto result = resolve_timestamps(raw, kHeader, 0.1, 5.0);
  EXPECT_EQ(result.mode, TimestampMode::kAbsoluteSeconds);
  EXPECT_TRUE(result.non_monotonic);
}

TEST(TimestampPolicy, UnclassifiableIsError)
{
  // Large magnitude, no unit conversion aligns with the header, long span:
  // we must refuse instead of guessing.
  const std::vector<double> raw = {123456789.0, 123456790.0};
  const auto result = resolve_timestamps(raw, kHeader, 0.1, 5.0);
  EXPECT_EQ(result.mode, TimestampMode::kError);
}
