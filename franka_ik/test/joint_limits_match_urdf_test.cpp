// Copyright 2026 The multipanda_ros2 Authors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "franka_ik/panda_limits.hpp"
#include "franka_ik/robot_chains.hpp"

#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#ifndef PANDA_IK_SINGLE_TEST_URDF
#error "PANDA_IK_SINGLE_TEST_URDF must name the rendered single-arm wrapper"
#endif

#ifndef PANDA_IK_DUAL_TEST_URDF
#error "PANDA_IK_DUAL_TEST_URDF must name the rendered dual-arm wrapper"
#endif

namespace franka_ik {
namespace {

std::string readFile(const std::string& path) {
  std::ifstream stream(path);
  if (!stream) {
    return {};
  }
  return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

void expectLiteralLimits(const RobotChains& chains, const std::vector<std::string>& arm_ids) {
  for (const auto& arm_id : arm_ids) {
    const auto& arm = chains.arm(arm_id);
    for (std::size_t index = 0; index < kPandaJointCount; ++index) {
      EXPECT_DOUBLE_EQ(arm.position_lower()[index], kPandaPositionLowerLimits[index]);
      EXPECT_DOUBLE_EQ(arm.position_upper()[index], kPandaPositionUpperLimits[index]);
      EXPECT_DOUBLE_EQ(arm.velocity_limits()[index], kPandaVelocityLimits[index]);
      EXPECT_DOUBLE_EQ(arm.effort_limits()[index], kPandaEffortLimits[index]);
    }
  }
}

TEST(JointLimitsMatchUrdfTest, RenderedSingleWrapperMatchesLiteralPandaTable) {
  const auto urdf = readFile(PANDA_IK_SINGLE_TEST_URDF);
  ASSERT_FALSE(urdf.empty());
  const RobotChains chains(urdf, {"panda"});
  expectLiteralLimits(chains, {"panda"});
}

TEST(JointLimitsMatchUrdfTest, RenderedDualWrapperMatchesLiteralPandaTable) {
  const auto urdf = readFile(PANDA_IK_DUAL_TEST_URDF);
  ASSERT_FALSE(urdf.empty());
  const RobotChains chains(urdf, {"panda1", "panda2"});
  expectLiteralLimits(chains, {"panda1", "panda2"});
}

}  // namespace
}  // namespace franka_ik
