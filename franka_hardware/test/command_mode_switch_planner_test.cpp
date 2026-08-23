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

#include "franka_hardware/real/command_mode_switch_planner.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

namespace franka_hardware
{
namespace
{

std::vector<std::string> jointInterfaces(
  const std::string & arm_name, const std::string & interface_name)
{
  std::vector<std::string> interfaces;
  for (int joint = 1; joint <= 7; ++joint) {
    interfaces.push_back(arm_name + "_joint" + std::to_string(joint) + "/" + interface_name);
  }
  return interfaces;
}

void append(std::vector<std::string> & destination, const std::vector<std::string> & source)
{
  destination.insert(destination.end(), source.begin(), source.end());
}

void expectFailure(const CommandModeSwitchPlanResult & result, CommandModeSwitchError error)
{
  EXPECT_FALSE(result.plan.has_value());
  EXPECT_EQ(result.error, error);
  EXPECT_FALSE(result.message.empty());
}

TEST(CommandModeSwitchPlannerTest, NoOpReturnsACompletePlanWithoutRequests)
{
  const std::vector<ArmCommandModeState> states{
    {"panda1", ControlMode::JointTorque}, {"panda2", ControlMode::None}};

  const auto result = CommandModeSwitchPlanner::makePlan(states, {}, {});

  ASSERT_TRUE(result);
  ASSERT_EQ(result.plan->arms.size(), 2U);
  EXPECT_EQ(result.plan->arms[0].arm_name, "panda1");
  EXPECT_EQ(result.plan->arms[0].requested_mode, ControlMode::JointTorque);
  EXPECT_FALSE(result.plan->arms[0].has_request);
  EXPECT_EQ(result.plan->arms[0].command_initialization, CommandInitialization::None);
  EXPECT_EQ(result.plan->arms[1].arm_name, "panda2");
  EXPECT_EQ(result.plan->arms[1].requested_mode, ControlMode::None);
  EXPECT_FALSE(result.plan->arms[1].has_request);
}

TEST(CommandModeSwitchPlannerTest, StartsTorqueWithSafeZeroEffortInitialization)
{
  const std::vector<ArmCommandModeState> states{{"panda1", ControlMode::None}};
  auto effort = jointInterfaces("panda1", "effort");
  std::reverse(effort.begin(), effort.end());

  const auto result = CommandModeSwitchPlanner::makePlan(states, effort, {});

  ASSERT_TRUE(result);
  ASSERT_EQ(result.plan->arms.size(), 1U);
  EXPECT_TRUE(result.plan->arms[0].has_request);
  EXPECT_EQ(result.plan->arms[0].requested_mode, ControlMode::JointTorque);
  EXPECT_EQ(result.plan->arms[0].command_initialization, CommandInitialization::ZeroJointEffort);
}

TEST(CommandModeSwitchPlannerTest, StartsMixedSupportedModesAcrossTwoArms)
{
  const std::vector<ArmCommandModeState> states{
    {"panda1", ControlMode::None}, {"panda2", ControlMode::None}};
  auto starts = jointInterfaces("panda1", "effort");
  append(starts, jointInterfaces("panda2", "velocity"));

  const auto result = CommandModeSwitchPlanner::makePlan(states, starts, {});

  ASSERT_TRUE(result);
  ASSERT_EQ(result.plan->arms.size(), 2U);
  EXPECT_EQ(result.plan->arms[0].requested_mode, ControlMode::JointTorque);
  EXPECT_EQ(result.plan->arms[0].command_initialization, CommandInitialization::ZeroJointEffort);
  EXPECT_EQ(result.plan->arms[1].requested_mode, ControlMode::JointVelocity);
  EXPECT_EQ(result.plan->arms[1].command_initialization, CommandInitialization::ZeroJointVelocity);
}

TEST(CommandModeSwitchPlannerTest, ResolvesCombinedStopAndStartTransactionForEachArm)
{
  const std::vector<ArmCommandModeState> states{
    {"panda1", ControlMode::JointTorque}, {"panda2", ControlMode::JointVelocity}};
  const auto original_states = states;
  auto stops = jointInterfaces("panda1", "effort");
  append(stops, jointInterfaces("panda2", "velocity"));
  auto starts = jointInterfaces("panda1", "velocity");
  append(starts, jointInterfaces("panda2", "effort"));

  const auto result = CommandModeSwitchPlanner::makePlan(states, starts, stops);

  ASSERT_TRUE(result);
  ASSERT_EQ(result.plan->arms.size(), 2U);
  EXPECT_EQ(result.plan->arms[0].requested_mode, ControlMode::JointVelocity);
  EXPECT_EQ(result.plan->arms[0].command_initialization, CommandInitialization::ZeroJointVelocity);
  EXPECT_EQ(result.plan->arms[1].requested_mode, ControlMode::JointTorque);
  EXPECT_EQ(result.plan->arms[1].command_initialization, CommandInitialization::ZeroJointEffort);
  ASSERT_EQ(states.size(), original_states.size());
  for (std::size_t index = 0; index < states.size(); ++index) {
    EXPECT_EQ(states[index].arm_name, original_states[index].arm_name);
    EXPECT_EQ(states[index].current_mode, original_states[index].current_mode);
  }
}

TEST(CommandModeSwitchPlannerTest, AllowsReplacingAControllerInTheSameMode)
{
  const std::vector<ArmCommandModeState> states{{"panda1", ControlMode::JointTorque}};
  const auto effort = jointInterfaces("panda1", "effort");

  const auto result = CommandModeSwitchPlanner::makePlan(states, effort, effort);

  ASSERT_TRUE(result);
  ASSERT_EQ(result.plan->arms.size(), 1U);
  EXPECT_TRUE(result.plan->arms[0].has_request);
  EXPECT_EQ(result.plan->arms[0].requested_mode, ControlMode::JointTorque);
  EXPECT_EQ(result.plan->arms[0].command_initialization, CommandInitialization::ZeroJointEffort);
}

TEST(CommandModeSwitchPlannerTest, StopsOnlyTheRequestedArmWhenModeMatches)
{
  const std::vector<ArmCommandModeState> states{
    {"panda1", ControlMode::JointTorque}, {"panda2", ControlMode::JointVelocity}};

  const auto result =
    CommandModeSwitchPlanner::makePlan(states, {}, jointInterfaces("panda1", "effort"));

  ASSERT_TRUE(result);
  ASSERT_EQ(result.plan->arms.size(), 2U);
  EXPECT_TRUE(result.plan->arms[0].has_request);
  EXPECT_EQ(result.plan->arms[0].requested_mode, ControlMode::None);
  EXPECT_EQ(result.plan->arms[0].command_initialization, CommandInitialization::ZeroJointEffort);
  EXPECT_FALSE(result.plan->arms[1].has_request);
  EXPECT_EQ(result.plan->arms[1].requested_mode, ControlMode::JointVelocity);
}

TEST(CommandModeSwitchPlannerTest, ExactArmMembershipHandlesPrefixArmNames)
{
  const std::vector<ArmCommandModeState> states{
    {"panda", ControlMode::None}, {"panda1", ControlMode::None}};

  const auto result =
    CommandModeSwitchPlanner::makePlan(states, jointInterfaces("panda1", "effort"), {});

  ASSERT_TRUE(result);
  ASSERT_EQ(result.plan->arms.size(), 2U);
  EXPECT_FALSE(result.plan->arms[0].has_request);
  EXPECT_EQ(result.plan->arms[0].requested_mode, ControlMode::None);
  EXPECT_TRUE(result.plan->arms[1].has_request);
  EXPECT_EQ(result.plan->arms[1].requested_mode, ControlMode::JointTorque);
}

TEST(CommandModeSwitchPlannerTest, RejectsAStaleStopModeWithoutReturningAPartialPlan)
{
  const std::vector<ArmCommandModeState> states{
    {"panda1", ControlMode::None}, {"panda2", ControlMode::JointVelocity}};
  const auto original_states = states;

  const auto result = CommandModeSwitchPlanner::makePlan(
    states, jointInterfaces("panda1", "effort"), jointInterfaces("panda2", "effort"));

  expectFailure(result, CommandModeSwitchError::StopModeMismatch);
  ASSERT_EQ(states.size(), original_states.size());
  for (std::size_t index = 0; index < states.size(); ++index) {
    EXPECT_EQ(states[index].arm_name, original_states[index].arm_name);
    EXPECT_EQ(states[index].current_mode, original_states[index].current_mode);
  }
}

TEST(CommandModeSwitchPlannerTest, RejectsStartingAnActiveArmWithoutAStop)
{
  const std::vector<ArmCommandModeState> states{{"panda1", ControlMode::JointTorque}};

  const auto result =
    CommandModeSwitchPlanner::makePlan(states, jointInterfaces("panda1", "velocity"), {});

  expectFailure(result, CommandModeSwitchError::StartRequiresStop);
}

TEST(CommandModeSwitchPlannerTest, RejectsPartialJointSets)
{
  const std::vector<ArmCommandModeState> states{{"panda1", ControlMode::None}};
  auto partial = jointInterfaces("panda1", "effort");
  partial.pop_back();

  const auto result = CommandModeSwitchPlanner::makePlan(states, partial, {});

  expectFailure(result, CommandModeSwitchError::IncompleteJointSet);
}

TEST(CommandModeSwitchPlannerTest, RejectsDuplicateInterfaces)
{
  const std::vector<ArmCommandModeState> states{{"panda1", ControlMode::None}};
  auto duplicate = jointInterfaces("panda1", "effort");
  duplicate.push_back(duplicate.front());

  const auto result = CommandModeSwitchPlanner::makePlan(states, duplicate, {});

  expectFailure(result, CommandModeSwitchError::DuplicateInterface);
}

TEST(CommandModeSwitchPlannerTest, RejectsMixedModesWithinOneArm)
{
  const std::vector<ArmCommandModeState> states{{"panda1", ControlMode::None}};
  auto mixed = jointInterfaces("panda1", "effort");
  mixed.back() = "panda1_joint7/velocity";

  const auto result = CommandModeSwitchPlanner::makePlan(states, mixed, {});

  expectFailure(result, CommandModeSwitchError::MixedJointModes);
}

TEST(CommandModeSwitchPlannerTest, RejectsAJointSetSplitAcrossArms)
{
  const std::vector<ArmCommandModeState> states{
    {"panda1", ControlMode::None}, {"panda2", ControlMode::None}};
  auto wrong_arm = jointInterfaces("panda1", "effort");
  wrong_arm.back() = "panda2_joint7/effort";

  const auto result = CommandModeSwitchPlanner::makePlan(states, wrong_arm, {});

  expectFailure(result, CommandModeSwitchError::IncompleteJointSet);
}

TEST(CommandModeSwitchPlannerTest, IgnoresForeignInterfacesWithoutSubstringOwnership)
{
  const std::vector<ArmCommandModeState> states{
    {"panda1", ControlMode::None}, {"panda2", ControlMode::None}};

  const auto result = CommandModeSwitchPlanner::makePlan(
    states, {"other_panda1_joint1/effort", "panda10_joint1/effort"}, {});

  ASSERT_TRUE(result);
  ASSERT_EQ(result.plan->arms.size(), 2U);
  EXPECT_FALSE(result.plan->arms[0].has_request);
  EXPECT_FALSE(result.plan->arms[1].has_request);
}

TEST(CommandModeSwitchPlannerTest, RejectsMalformedInterfaceClaimingConfiguredArm)
{
  const std::vector<ArmCommandModeState> states{{"panda1", ControlMode::None}};

  const auto result = CommandModeSwitchPlanner::makePlan(states, {"panda1_joint8/effort"}, {});

  expectFailure(result, CommandModeSwitchError::UnknownInterface);
}

TEST(CommandModeSwitchPlannerTest, RejectsJointPositionModeExplicitly)
{
  const std::vector<ArmCommandModeState> states{{"panda1", ControlMode::None}};

  const auto result =
    CommandModeSwitchPlanner::makePlan(states, jointInterfaces("panda1", "position"), {});

  expectFailure(result, CommandModeSwitchError::UnsupportedInterface);
}

TEST(CommandModeSwitchPlannerTest, RejectsCartesianPoseModeExplicitly)
{
  const std::vector<ArmCommandModeState> states{{"panda1", ControlMode::None}};

  const auto result =
    CommandModeSwitchPlanner::makePlan(states, {"panda1_ee_cartesian_position/00"}, {});

  expectFailure(result, CommandModeSwitchError::UnsupportedInterface);
}

TEST(CommandModeSwitchPlannerTest, RejectsCartesianVelocityModeExplicitly)
{
  const std::vector<ArmCommandModeState> states{{"panda1", ControlMode::None}};

  const auto result =
    CommandModeSwitchPlanner::makePlan(states, {"panda1_ee_cartesian_velocity/tx"}, {});

  expectFailure(result, CommandModeSwitchError::UnsupportedInterface);
}

TEST(CommandModeSwitchPlannerTest, RejectsUnsupportedCurrentModes)
{
  const std::vector<ArmCommandModeState> states{{"panda1", ControlMode::JointPosition}};

  const auto result = CommandModeSwitchPlanner::makePlan(states, {}, {});

  expectFailure(result, CommandModeSwitchError::UnsupportedCurrentMode);
}

TEST(CommandModeSwitchPlannerTest, RejectsDuplicateArmNames)
{
  const std::vector<ArmCommandModeState> states{
    {"panda1", ControlMode::None}, {"panda1", ControlMode::None}};

  const auto result = CommandModeSwitchPlanner::makePlan(states, {}, {});

  expectFailure(result, CommandModeSwitchError::InvalidArmConfiguration);
}

TEST(CommandModeSwitchPlannerTest, RejectsAnEmptyArmConfiguration)
{
  const auto result = CommandModeSwitchPlanner::makePlan({}, {}, {});

  expectFailure(result, CommandModeSwitchError::InvalidArmConfiguration);
}

}  // namespace
}  // namespace franka_hardware
