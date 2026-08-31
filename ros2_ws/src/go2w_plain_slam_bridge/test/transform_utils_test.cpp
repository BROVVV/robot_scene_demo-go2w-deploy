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

// Unit tests for the manufacturer-geometry transform derivation (plan §10).

#include <cmath>
#include <string>

#include <gtest/gtest.h>

#include <go2w_plain_slam_bridge/transform_utils.hpp>

using go2w_plain_slam_bridge::RigidTransform;
using go2w_plain_slam_bridge::load_base_to_imu_transform;

TEST(TransformUtils, LoadsBaseToImuFromOfficialReference)
{
  RigidTransform t_base_imu;
  std::string error;
  const std::string path =
    std::string(TEST_PROJECT_ROOT) + "/configs/go2w/official_reference.yaml";
  ASSERT_TRUE(load_base_to_imu_transform(path, t_base_imu, error)) << error;

  // Expected from the manufacturer geometry + plan §2.2 derivation:
  //   base->utlidar_lidar: t (0.28945, 0, -0.046825), rpy (0, -0.26339265, 0)
  //   utlidar_lidar->utlidar_imu: t (-0.007698, -0.014655, 0.00667), identity
  // (translation lives in the fourth column: m[3]/m[7]/m[11])
  EXPECT_NEAR(t_base_imu.m[3], 0.280281, 1e-4);
  EXPECT_NEAR(t_base_imu.m[7], -0.014655, 1e-4);
  EXPECT_NEAR(t_base_imu.m[11], -0.042390, 1e-4);
  // Rotation is Ry(-0.263392653559) in the reciprocal direction: cos/sin.
  EXPECT_NEAR(t_base_imu.m[0], 0.965505, 1e-4);
  EXPECT_NEAR(t_base_imu.m[2], -0.260386, 1e-4);
  EXPECT_NEAR(t_base_imu.m[8], 0.260386, 1e-4);
  EXPECT_NEAR(t_base_imu.m[10], 0.965505, 1e-4);
}

TEST(TransformUtils, InverseMultiplyRoundTrip)
{
  RigidTransform t;
  t.m[0] = 0.0;
  t.m[1] = -1.0;
  t.m[4] = 1.0;
  t.m[5] = 0.0;
  t.m[3] = 1.0;
  t.m[7] = 2.0;
  const RigidTransform identity = t.multiply(t.inverse());
  for (int i = 0; i < 16; ++i) {
    const double expected = (i % 5 == 0) ? 1.0 : 0.0;
    EXPECT_NEAR(identity.m[i], expected, 1e-9) << "m[" << i << "]";
  }
}