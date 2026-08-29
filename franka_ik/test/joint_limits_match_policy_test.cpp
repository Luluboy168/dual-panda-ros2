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

#include <array>
#include <cstddef>
#include <string>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#ifndef PANDA_JOINT_LIMIT_POLICY_FILE
#error "PANDA_JOINT_LIMIT_POLICY_FILE must name the test-only controller policy YAML"
#endif

namespace franka_ik {
namespace {

template <std::size_t Size>
void expectYamlArray(const YAML::Node& root,
                     const std::string& key,
                     const std::array<double, Size>& expected) {
  const auto values = root[key];
  ASSERT_TRUE(values.IsSequence()) << key;
  ASSERT_EQ(values.size(), Size) << key;
  for (std::size_t index = 0; index < Size; ++index) {
    EXPECT_DOUBLE_EQ(values[index].as<double>(), expected[index]) << key << "[" << index << "]";
  }
}

TEST(JointLimitsMatchPolicyTest, LiteralLimitsMatchReviewedControllerPolicy) {
  const auto policy = YAML::LoadFile(PANDA_JOINT_LIMIT_POLICY_FILE);
  ASSERT_EQ(policy["schema_version"].as<int>(), 1);
  expectYamlArray(policy, "position_lower", kPandaPositionLowerLimits);
  expectYamlArray(policy, "position_upper", kPandaPositionUpperLimits);
  expectYamlArray(policy, "urdf_velocity_ceiling", kPandaVelocityLimits);
  expectYamlArray(policy, "effort_ceiling", kPandaEffortLimits);
}

TEST(JointLimitsMatchPolicyTest, PolicyUsesCanonicalDualArmIdsAndJointSuffixes) {
  const auto policy = YAML::LoadFile(PANDA_JOINT_LIMIT_POLICY_FILE);
  const auto arm_ids = policy["arm_ids"];
  ASSERT_TRUE(arm_ids.IsSequence());
  ASSERT_EQ(arm_ids.size(), 2U);
  EXPECT_EQ(arm_ids[0].as<std::string>(), "panda1");
  EXPECT_EQ(arm_ids[1].as<std::string>(), "panda2");

  const auto joint_suffixes = policy["joint_suffixes"];
  ASSERT_TRUE(joint_suffixes.IsSequence());
  ASSERT_EQ(joint_suffixes.size(), kPandaJointCount);
  for (std::size_t index = 0; index < kPandaJointCount; ++index) {
    EXPECT_EQ(joint_suffixes[index].as<std::string>(), "joint" + std::to_string(index + 1));
  }
}

}  // namespace
}  // namespace franka_ik
