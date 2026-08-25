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
#include <array>
#include <cstdint>
#include <iostream>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace franka_hardware {
namespace {

std::vector<std::string> jointInterfaces(const std::string& arm_name,
                                         const std::string& interface_name) {
  std::vector<std::string> interfaces;
  for (int joint = 1; joint <= 7; ++joint) {
    interfaces.push_back(arm_name + "_joint" + std::to_string(joint) + "/" + interface_name);
  }
  return interfaces;
}

void append(std::vector<std::string>& destination, const std::vector<std::string>& source) {
  destination.insert(destination.end(), source.begin(), source.end());
}

void expectFailure(const CommandModeSwitchPlanResult& result, CommandModeSwitchError error) {
  EXPECT_FALSE(result.plan.has_value());
  EXPECT_EQ(result.error, error);
  EXPECT_FALSE(result.message.empty());
}

struct ReferenceInterfaceDescription {
  std::size_t arm_index;
  ControlMode mode;
  std::uint8_t joint_bit;
  bool supported;
};

struct ReferencePerArmRequest {
  ControlMode mode{ControlMode::None};
  std::uint8_t joint_mask{0};
  std::size_t count{0};
  bool present{false};
};

struct ReferencePlanResult {
  std::optional<CommandModeSwitchPlan> plan;
  CommandModeSwitchError error{CommandModeSwitchError::None};
};

CommandInitialization referenceInitialization(ControlMode mode) {
  if (mode == ControlMode::JointTorque) {
    return CommandInitialization::ZeroJointEffort;
  }
  if (mode == ControlMode::JointVelocity) {
    return CommandInitialization::ZeroJointVelocity;
  }
  return CommandInitialization::None;
}

bool referenceClaimsConfiguredArm(const std::string& interface_name,
                                  const std::vector<ArmCommandModeState>& states) {
  const auto slash = interface_name.find('/');
  const auto resource = interface_name.substr(0, slash);
  return std::any_of(states.begin(), states.end(), [&](const auto& state) {
    return resource.rfind(state.arm_name + "_", 0) == 0;
  });
}

ReferencePlanResult referencePlan(const std::vector<ArmCommandModeState>& states,
                                  const std::vector<std::string>& starts,
                                  const std::vector<std::string>& stops) {
  if (states.empty()) {
    return {std::nullopt, CommandModeSwitchError::InvalidArmConfiguration};
  }

  std::unordered_set<std::string> arm_names;
  std::unordered_map<std::string, ReferenceInterfaceDescription> lookup;
  for (std::size_t arm_index = 0; arm_index < states.size(); ++arm_index) {
    const auto& state = states[arm_index];
    if (state.arm_name.empty() || !arm_names.emplace(state.arm_name).second) {
      return {std::nullopt, CommandModeSwitchError::InvalidArmConfiguration};
    }
    if (state.current_mode != ControlMode::None && state.current_mode != ControlMode::JointTorque &&
        state.current_mode != ControlMode::JointVelocity) {
      return {std::nullopt, CommandModeSwitchError::UnsupportedCurrentMode};
    }
    for (std::size_t joint = 1; joint <= 7; ++joint) {
      const auto prefix = state.arm_name + "_joint" + std::to_string(joint) + "/";
      const auto bit = static_cast<std::uint8_t>(1U << (joint - 1));
      const std::array<std::pair<const char*, ReferenceInterfaceDescription>, 3> entries{{
          {"effort", {arm_index, ControlMode::JointTorque, bit, true}},
          {"velocity", {arm_index, ControlMode::JointVelocity, bit, true}},
          {"position", {arm_index, ControlMode::None, bit, false}},
      }};
      for (const auto& [suffix, description] : entries) {
        if (!lookup.emplace(prefix + suffix, description).second) {
          return {std::nullopt, CommandModeSwitchError::InvalidArmConfiguration};
        }
      }
    }
    for (std::size_t element = 0; element < 16; ++element) {
      std::ostringstream name;
      name << state.arm_name << "_ee_cartesian_position/";
      if (element < 10) {
        name << '0';
      }
      name << element;
      if (!lookup
               .emplace(name.str(),
                        ReferenceInterfaceDescription{arm_index, ControlMode::None, 0, false})
               .second) {
        return {std::nullopt, CommandModeSwitchError::InvalidArmConfiguration};
      }
    }
    for (const char* component : {"tx", "ty", "tz", "omega_x", "omega_y", "omega_z"}) {
      if (!lookup
               .emplace(state.arm_name + "_ee_cartesian_velocity/" + component,
                        ReferenceInterfaceDescription{arm_index, ControlMode::None, 0, false})
               .second) {
        return {std::nullopt, CommandModeSwitchError::InvalidArmConfiguration};
      }
    }
  }

  auto parse = [&](const std::vector<std::string>& interfaces,
                   std::vector<ReferencePerArmRequest>& requests) {
    std::unordered_set<std::string> seen;
    for (const auto& name : interfaces) {
      const auto found = lookup.find(name);
      if (found == lookup.end()) {
        if (referenceClaimsConfiguredArm(name, states)) {
          return CommandModeSwitchError::UnknownInterface;
        }
        continue;
      }
      if (!seen.emplace(name).second) {
        return CommandModeSwitchError::DuplicateInterface;
      }
      if (!found->second.supported) {
        return CommandModeSwitchError::UnsupportedInterface;
      }
      auto& request = requests[found->second.arm_index];
      if (request.present && request.mode != found->second.mode) {
        return CommandModeSwitchError::MixedJointModes;
      }
      request.present = true;
      request.mode = found->second.mode;
      request.joint_mask = static_cast<std::uint8_t>(request.joint_mask | found->second.joint_bit);
      ++request.count;
    }
    for (const auto& request : requests) {
      if (request.present && (request.count != 7U || request.joint_mask != 0x7fU)) {
        return CommandModeSwitchError::IncompleteJointSet;
      }
    }
    return CommandModeSwitchError::None;
  };

  std::vector<ReferencePerArmRequest> start_requests(states.size());
  std::vector<ReferencePerArmRequest> stop_requests(states.size());
  if (const auto error = parse(starts, start_requests); error != CommandModeSwitchError::None) {
    return {std::nullopt, error};
  }
  if (const auto error = parse(stops, stop_requests); error != CommandModeSwitchError::None) {
    return {std::nullopt, error};
  }

  CommandModeSwitchPlan plan;
  for (std::size_t index = 0; index < states.size(); ++index) {
    const auto& state = states[index];
    const auto& start = start_requests[index];
    const auto& stop = stop_requests[index];
    if (stop.present && stop.mode != state.current_mode) {
      return {std::nullopt, CommandModeSwitchError::StopModeMismatch};
    }
    const auto after_stop = stop.present ? ControlMode::None : state.current_mode;
    if (start.present && after_stop != ControlMode::None) {
      return {std::nullopt, CommandModeSwitchError::StartRequiresStop};
    }
    const auto requested =
        start.present ? start.mode : (stop.present ? ControlMode::None : state.current_mode);
    plan.arms.push_back({state.arm_name, requested, start.present || stop.present,
                         start.present ? referenceInitialization(start.mode)
                                       : (stop.present ? referenceInitialization(state.current_mode)
                                                       : CommandInitialization::None)});
  }
  return {std::move(plan), CommandModeSwitchError::None};
}

std::string describePlannerCase(const std::vector<ArmCommandModeState>& states,
                                const std::vector<std::string>& starts,
                                const std::vector<std::string>& stops) {
  std::ostringstream description;
  description << "states=[";
  for (const auto& state : states) {
    description << state.arm_name << ':' << static_cast<int>(state.current_mode) << ',';
  }
  description << "] starts=[";
  for (const auto& name : starts) {
    description << name << ',';
  }
  description << "] stops=[";
  for (const auto& name : stops) {
    description << name << ',';
  }
  return description.str() + ']';
}

void appendModeInterfaces(std::vector<std::string>& interfaces,
                          const std::string& arm,
                          ControlMode mode) {
  if (mode == ControlMode::JointTorque) {
    append(interfaces, jointInterfaces(arm, "effort"));
  } else if (mode == ControlMode::JointVelocity) {
    append(interfaces, jointInterfaces(arm, "velocity"));
  }
}

ControlMode randomSupportedMode(std::mt19937_64& engine) {
  constexpr std::array<ControlMode, 3> kModes{ControlMode::None, ControlMode::JointTorque,
                                              ControlMode::JointVelocity};
  return kModes[engine() % kModes.size()];
}

TEST(CommandModeSwitchPlannerTest, NoOpReturnsACompletePlanWithoutRequests) {
  const std::vector<ArmCommandModeState> states{{"panda1", ControlMode::JointTorque},
                                                {"panda2", ControlMode::None}};

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

TEST(CommandModeSwitchPlannerTest, StartsTorqueWithSafeZeroEffortInitialization) {
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

TEST(CommandModeSwitchPlannerTest, StartsMixedSupportedModesAcrossTwoArms) {
  const std::vector<ArmCommandModeState> states{{"panda1", ControlMode::None},
                                                {"panda2", ControlMode::None}};
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

TEST(CommandModeSwitchPlannerTest, ResolvesCombinedStopAndStartTransactionForEachArm) {
  const std::vector<ArmCommandModeState> states{{"panda1", ControlMode::JointTorque},
                                                {"panda2", ControlMode::JointVelocity}};
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

TEST(CommandModeSwitchPlannerTest, AllowsReplacingAControllerInTheSameMode) {
  const std::vector<ArmCommandModeState> states{{"panda1", ControlMode::JointTorque}};
  const auto effort = jointInterfaces("panda1", "effort");

  const auto result = CommandModeSwitchPlanner::makePlan(states, effort, effort);

  ASSERT_TRUE(result);
  ASSERT_EQ(result.plan->arms.size(), 1U);
  EXPECT_TRUE(result.plan->arms[0].has_request);
  EXPECT_EQ(result.plan->arms[0].requested_mode, ControlMode::JointTorque);
  EXPECT_EQ(result.plan->arms[0].command_initialization, CommandInitialization::ZeroJointEffort);
}

TEST(CommandModeSwitchPlannerTest, StopsOnlyTheRequestedArmWhenModeMatches) {
  const std::vector<ArmCommandModeState> states{{"panda1", ControlMode::JointTorque},
                                                {"panda2", ControlMode::JointVelocity}};

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

TEST(CommandModeSwitchPlannerTest, ExactArmMembershipHandlesPrefixArmNames) {
  const std::vector<ArmCommandModeState> states{{"panda", ControlMode::None},
                                                {"panda1", ControlMode::None}};

  const auto result =
      CommandModeSwitchPlanner::makePlan(states, jointInterfaces("panda1", "effort"), {});

  ASSERT_TRUE(result);
  ASSERT_EQ(result.plan->arms.size(), 2U);
  EXPECT_FALSE(result.plan->arms[0].has_request);
  EXPECT_EQ(result.plan->arms[0].requested_mode, ControlMode::None);
  EXPECT_TRUE(result.plan->arms[1].has_request);
  EXPECT_EQ(result.plan->arms[1].requested_mode, ControlMode::JointTorque);
}

TEST(CommandModeSwitchPlannerTest, RejectsAStaleStopModeWithoutReturningAPartialPlan) {
  const std::vector<ArmCommandModeState> states{{"panda1", ControlMode::None},
                                                {"panda2", ControlMode::JointVelocity}};
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

TEST(CommandModeSwitchPlannerTest, RejectsStartingAnActiveArmWithoutAStop) {
  const std::vector<ArmCommandModeState> states{{"panda1", ControlMode::JointTorque}};

  const auto result =
      CommandModeSwitchPlanner::makePlan(states, jointInterfaces("panda1", "velocity"), {});

  expectFailure(result, CommandModeSwitchError::StartRequiresStop);
}

TEST(CommandModeSwitchPlannerTest, RejectsPartialJointSets) {
  const std::vector<ArmCommandModeState> states{{"panda1", ControlMode::None}};
  auto partial = jointInterfaces("panda1", "effort");
  partial.pop_back();

  const auto result = CommandModeSwitchPlanner::makePlan(states, partial, {});

  expectFailure(result, CommandModeSwitchError::IncompleteJointSet);
}

TEST(CommandModeSwitchPlannerTest, RejectsDuplicateInterfaces) {
  const std::vector<ArmCommandModeState> states{{"panda1", ControlMode::None}};
  auto duplicate = jointInterfaces("panda1", "effort");
  duplicate.push_back(duplicate.front());

  const auto result = CommandModeSwitchPlanner::makePlan(states, duplicate, {});

  expectFailure(result, CommandModeSwitchError::DuplicateInterface);
}

TEST(CommandModeSwitchPlannerTest, RejectsMixedModesWithinOneArm) {
  const std::vector<ArmCommandModeState> states{{"panda1", ControlMode::None}};
  auto mixed = jointInterfaces("panda1", "effort");
  mixed.back() = "panda1_joint7/velocity";

  const auto result = CommandModeSwitchPlanner::makePlan(states, mixed, {});

  expectFailure(result, CommandModeSwitchError::MixedJointModes);
}

TEST(CommandModeSwitchPlannerTest, RejectsAJointSetSplitAcrossArms) {
  const std::vector<ArmCommandModeState> states{{"panda1", ControlMode::None},
                                                {"panda2", ControlMode::None}};
  auto wrong_arm = jointInterfaces("panda1", "effort");
  wrong_arm.back() = "panda2_joint7/effort";

  const auto result = CommandModeSwitchPlanner::makePlan(states, wrong_arm, {});

  expectFailure(result, CommandModeSwitchError::IncompleteJointSet);
}

TEST(CommandModeSwitchPlannerTest, IgnoresForeignInterfacesWithoutSubstringOwnership) {
  const std::vector<ArmCommandModeState> states{{"panda1", ControlMode::None},
                                                {"panda2", ControlMode::None}};

  const auto result = CommandModeSwitchPlanner::makePlan(
      states, {"other_panda1_joint1/effort", "panda10_joint1/effort"}, {});

  ASSERT_TRUE(result);
  ASSERT_EQ(result.plan->arms.size(), 2U);
  EXPECT_FALSE(result.plan->arms[0].has_request);
  EXPECT_FALSE(result.plan->arms[1].has_request);
}

TEST(CommandModeSwitchPlannerTest, RejectsMalformedInterfaceClaimingConfiguredArm) {
  const std::vector<ArmCommandModeState> states{{"panda1", ControlMode::None}};

  const auto result = CommandModeSwitchPlanner::makePlan(states, {"panda1_joint8/effort"}, {});

  expectFailure(result, CommandModeSwitchError::UnknownInterface);
}

TEST(CommandModeSwitchPlannerTest, RejectsJointPositionModeExplicitly) {
  const std::vector<ArmCommandModeState> states{{"panda1", ControlMode::None}};

  const auto result =
      CommandModeSwitchPlanner::makePlan(states, jointInterfaces("panda1", "position"), {});

  expectFailure(result, CommandModeSwitchError::UnsupportedInterface);
}

TEST(CommandModeSwitchPlannerTest, RejectsCartesianPoseModeExplicitly) {
  const std::vector<ArmCommandModeState> states{{"panda1", ControlMode::None}};

  const auto result =
      CommandModeSwitchPlanner::makePlan(states, {"panda1_ee_cartesian_position/00"}, {});

  expectFailure(result, CommandModeSwitchError::UnsupportedInterface);
}

TEST(CommandModeSwitchPlannerTest, RejectsCartesianVelocityModeExplicitly) {
  const std::vector<ArmCommandModeState> states{{"panda1", ControlMode::None}};

  const auto result =
      CommandModeSwitchPlanner::makePlan(states, {"panda1_ee_cartesian_velocity/tx"}, {});

  expectFailure(result, CommandModeSwitchError::UnsupportedInterface);
}

TEST(CommandModeSwitchPlannerTest, RejectsUnsupportedCurrentModes) {
  const std::vector<ArmCommandModeState> states{{"panda1", ControlMode::JointPosition}};

  const auto result = CommandModeSwitchPlanner::makePlan(states, {}, {});

  expectFailure(result, CommandModeSwitchError::UnsupportedCurrentMode);
}

TEST(CommandModeSwitchPlannerTest, RejectsDuplicateArmNames) {
  const std::vector<ArmCommandModeState> states{{"panda1", ControlMode::None},
                                                {"panda1", ControlMode::None}};

  const auto result = CommandModeSwitchPlanner::makePlan(states, {}, {});

  expectFailure(result, CommandModeSwitchError::InvalidArmConfiguration);
}

TEST(CommandModeSwitchPlannerTest, RejectsAnEmptyArmConfiguration) {
  const auto result = CommandModeSwitchPlanner::makePlan({}, {}, {});

  expectFailure(result, CommandModeSwitchError::InvalidArmConfiguration);
}

TEST(CommandModeSwitchPlannerTest, FixedSeedGeneratedCasesMatchIndependentReferenceModel) {
  constexpr std::array<std::uint64_t, 3> kSeeds{0x5a17c0deU, 0xd00df00dU, 0x7157a11ceULL};
  constexpr std::size_t kCasesPerSeed = 4000;

  for (const auto seed : kSeeds) {
    std::cout << "CommandModeSwitchPlanner property seed=" << seed << " cases=" << kCasesPerSeed
              << '\n';
    std::mt19937_64 engine(seed);
    for (std::size_t case_index = 0; case_index < kCasesPerSeed; ++case_index) {
      std::vector<ArmCommandModeState> states{{"panda1", randomSupportedMode(engine)},
                                              {"panda2", randomSupportedMode(engine)}};
      std::vector<std::string> starts;
      std::vector<std::string> stops;
      const auto category = case_index % 12U;

      if (category <= 1U || category == 11U) {
        for (const auto& state : states) {
          auto target = randomSupportedMode(engine);
          if (category == 11U) {
            target = state.current_mode;
          }
          if (target == state.current_mode) {
            if (target != ControlMode::None && (engine() & 1U) != 0U) {
              appendModeInterfaces(stops, state.arm_name, state.current_mode);
              appendModeInterfaces(starts, state.arm_name, target);
            }
          } else {
            appendModeInterfaces(stops, state.arm_name, state.current_mode);
            appendModeInterfaces(starts, state.arm_name, target);
          }
        }
        if (category == 1U) {
          std::shuffle(starts.begin(), starts.end(), engine);
          std::shuffle(stops.begin(), stops.end(), engine);
        }
      } else if (category == 2U) {
        starts = jointInterfaces("panda1", (engine() & 1U) != 0U ? "effort" : "velocity");
        starts.resize(1U + engine() % 6U);
        std::shuffle(starts.begin(), starts.end(), engine);
      } else if (category == 3U) {
        starts = jointInterfaces("panda1", "effort");
        starts.insert(starts.begin() + static_cast<std::ptrdiff_t>(engine() % 7U),
                      starts[engine() % 7U]);
        std::shuffle(starts.begin(), starts.end(), engine);
      } else if (category == 4U) {
        starts = jointInterfaces("panda1", "effort");
        starts[engine() % starts.size()] =
            "panda1_joint" + std::to_string(1U + engine() % 7U) + "/velocity";
        std::shuffle(starts.begin(), starts.end(), engine);
      } else if (category == 5U) {
        const std::array<std::string, 5> malformed{"panda1_joint0/effort", "panda1_joint8/velocity",
                                                   "panda1_joint1/efforts", "panda2_joint7",
                                                   "panda2_/velocity"};
        starts.push_back(malformed[engine() % malformed.size()]);
      } else if (category == 6U) {
        const std::array<std::string, 3> unsupported{"panda1_joint1/position",
                                                     "panda2_ee_cartesian_position/15",
                                                     "panda1_ee_cartesian_velocity/omega_z"};
        starts.push_back(unsupported[engine() % unsupported.size()]);
      } else if (category == 7U) {
        starts = {"foreign_joint1/effort", "panda10_joint1/effort", "other_panda1_joint2/velocity"};
        std::shuffle(starts.begin(), starts.end(), engine);
      } else if (category == 8U) {
        states[engine() % states.size()].current_mode =
            (engine() & 1U) != 0U ? ControlMode::JointPosition : ControlMode::CartesianVelocity;
      } else if (category == 9U) {
        if ((engine() & 1U) != 0U) {
          states[1].arm_name = states[0].arm_name;
        } else {
          states[engine() % states.size()].arm_name.clear();
        }
      } else {
        const std::array<std::string, 12> pool{"panda1_joint1/effort",
                                               "panda1_joint2/velocity",
                                               "panda1_joint3/position",
                                               "panda2_joint1/effort",
                                               "panda2_joint7/velocity",
                                               "panda2_joint8/effort",
                                               "panda1_ee_cartesian_position/00",
                                               "panda2_ee_cartesian_velocity/tx",
                                               "foreign_joint1/effort",
                                               "panda10_joint4/velocity",
                                               "panda1_joint4/effort",
                                               "panda2_joint3/velocity"};
        const auto start_count = engine() % 18U;
        const auto stop_count = engine() % 18U;
        for (std::size_t index = 0; index < start_count; ++index) {
          starts.push_back(pool[engine() % pool.size()]);
        }
        for (std::size_t index = 0; index < stop_count; ++index) {
          stops.push_back(pool[engine() % pool.size()]);
        }
        std::shuffle(starts.begin(), starts.end(), engine);
        std::shuffle(stops.begin(), stops.end(), engine);
      }

      const auto operation = describePlannerCase(states, starts, stops);
      SCOPED_TRACE("seed=" + std::to_string(seed) + " case=" + std::to_string(case_index) +
                   " operation=" + operation);
      const auto expected = referencePlan(states, starts, stops);
      const auto actual = CommandModeSwitchPlanner::makePlan(states, starts, stops);
      ASSERT_EQ(actual.plan.has_value(), expected.plan.has_value());
      ASSERT_EQ(actual.error, expected.error);
      if (!expected.plan.has_value()) {
        EXPECT_FALSE(actual.message.empty());
        continue;
      }
      ASSERT_TRUE(actual.message.empty());
      ASSERT_EQ(actual.plan->arms.size(), expected.plan->arms.size());
      for (std::size_t arm = 0; arm < expected.plan->arms.size(); ++arm) {
        EXPECT_EQ(actual.plan->arms[arm].arm_name, expected.plan->arms[arm].arm_name);
        EXPECT_EQ(actual.plan->arms[arm].requested_mode, expected.plan->arms[arm].requested_mode);
        EXPECT_EQ(actual.plan->arms[arm].has_request, expected.plan->arms[arm].has_request);
        EXPECT_EQ(actual.plan->arms[arm].command_initialization,
                  expected.plan->arms[arm].command_initialization);
      }
    }
  }
}

}  // namespace
}  // namespace franka_hardware
