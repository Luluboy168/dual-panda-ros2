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

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cstddef>
#include <fstream>
#include <iterator>
#include <regex>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "franka_example_controllers/panda_joint_limits.hpp"

namespace {

using JointArray = std::array<double, franka_example_controllers::kPandaJointCount>;

JointArray readJointArray(const YAML::Node& root, const char* key) {
  const auto values = root[key];
  if (!values.IsSequence() || values.size() != franka_example_controllers::kPandaJointCount) {
    throw std::runtime_error(std::string("invalid policy array: ") + key);
  }
  JointArray result{};
  for (size_t joint = 0; joint < result.size(); ++joint) {
    result[joint] = values[joint].as<double>();
  }
  return result;
}

template <typename ExpectedArray>
void expectJointArrayEqual(const JointArray& actual, const ExpectedArray& expected) {
  for (size_t joint = 0; joint < actual.size(); ++joint) {
    EXPECT_DOUBLE_EQ(actual[joint], expected[joint]) << "joint " << joint + 1;
  }
}

double readAttribute(const std::string& attributes, const char* name) {
  const std::regex expression(std::string(name) + R"REGEX(="([^"]+)")REGEX");
  std::smatch match;
  if (!std::regex_search(attributes, match, expression)) {
    throw std::runtime_error(std::string("missing xacro limit attribute: ") + name);
  }
  return std::stod(match[1].str());
}

struct XacroLimits {
  JointArray effort{};
  JointArray lower{};
  JointArray upper{};
  JointArray velocity{};
};

XacroLimits readXacroLimits() {
  std::ifstream input(PANDA_ARM_XACRO_FILE);
  if (!input) {
    throw std::runtime_error("unable to open panda_arm.xacro");
  }
  const std::string text{std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
  const std::regex joint_expression(
      R"REGEX(<joint name="\$\{arm_id\}_joint([1-7])" type="revolute">([\s\S]*?)<limit ([^>]*)/>[\s\S]*?</joint>)REGEX");
  XacroLimits result;
  std::set<size_t> seen;
  for (std::sregex_iterator it(text.begin(), text.end(), joint_expression), end; it != end; ++it) {
    const size_t joint = static_cast<size_t>(std::stoul((*it)[1].str()) - 1U);
    if (joint >= franka_example_controllers::kPandaJointCount || !seen.insert(joint).second) {
      throw std::runtime_error("panda_arm.xacro contains an invalid or duplicate joint limit");
    }
    const std::string attributes = (*it)[3].str();
    result.effort[joint] = readAttribute(attributes, "effort");
    result.lower[joint] = readAttribute(attributes, "lower");
    result.upper[joint] = readAttribute(attributes, "upper");
    result.velocity[joint] = readAttribute(attributes, "velocity");
  }
  if (seen.size() != franka_example_controllers::kPandaJointCount) {
    throw std::runtime_error("panda_arm.xacro does not contain seven unique Panda joint limits");
  }
  return result;
}

TEST(PandaJointLimitsConsistencyTest, VersionedPolicyMatchesHeaderLibfrankaAndXacro) {
  const auto policy = YAML::LoadFile(PANDA_JOINT_LIMIT_POLICY_FILE);
  ASSERT_EQ(policy["schema_version"].as<int>(), 1);
  EXPECT_EQ(policy["libfranka_version"].as<std::string>(), PANDA_PINNED_LIBFRANKA_VERSION);

  const std::vector<std::string> expected_arms{"panda1", "panda2"};
  EXPECT_EQ(policy["arm_ids"].as<std::vector<std::string>>(), expected_arms);

  const auto policy_effort = readJointArray(policy, "effort_ceiling");
  const auto policy_lower = readJointArray(policy, "position_lower");
  const auto policy_upper = readJointArray(policy, "position_upper");
  const auto policy_urdf_velocity = readJointArray(policy, "urdf_velocity_ceiling");
  const auto policy_fci_velocity = readJointArray(policy, "libfranka_velocity_ceiling");
  const auto policy_fci_acceleration = readJointArray(policy, "libfranka_acceleration_ceiling");

  expectJointArrayEqual(policy_effort, franka_example_controllers::kPandaAbsoluteEffortCeilings);
  expectJointArrayEqual(policy_lower, franka_example_controllers::kPandaPositionLowerLimits);
  expectJointArrayEqual(policy_upper, franka_example_controllers::kPandaPositionUpperLimits);
  expectJointArrayEqual(policy_urdf_velocity,
                        franka_example_controllers::kPandaAbsoluteJointVelocityCeilings);
  expectJointArrayEqual(policy_fci_velocity,
                        franka_example_controllers::kPandaFciJointVelocityCeilings);
  expectJointArrayEqual(policy_fci_acceleration,
                        franka_example_controllers::kPandaFciJointAccelerationCeilings);

  const auto xacro = readXacroLimits();
  expectJointArrayEqual(policy_effort, xacro.effort);
  expectJointArrayEqual(policy_lower, xacro.lower);
  expectJointArrayEqual(policy_upper, xacro.upper);
  expectJointArrayEqual(policy_urdf_velocity, xacro.velocity);
}

}  // namespace
