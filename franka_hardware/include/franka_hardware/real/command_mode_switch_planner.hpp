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

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "franka_hardware/common/control_mode.h"

namespace franka_hardware
{

struct ArmCommandModeState
{
  std::string arm_name;
  ControlMode current_mode{ControlMode::None};
};

enum class CommandInitialization
{
  None,
  ZeroJointEffort,
  ZeroJointVelocity,
};

struct ArmCommandModeRequest
{
  std::string arm_name;
  ControlMode requested_mode{ControlMode::None};
  bool has_request{false};
  CommandInitialization command_initialization{CommandInitialization::None};
};

struct CommandModeSwitchPlan
{
  std::vector<ArmCommandModeRequest> arms;
};

enum class CommandModeSwitchError
{
  None,
  InvalidArmConfiguration,
  UnsupportedCurrentMode,
  DuplicateInterface,
  UnknownInterface,
  UnsupportedInterface,
  MixedJointModes,
  IncompleteJointSet,
  StopModeMismatch,
  StartRequiresStop,
};

struct CommandModeSwitchPlanResult
{
  std::optional<CommandModeSwitchPlan> plan;
  CommandModeSwitchError error{CommandModeSwitchError::None};
  std::string message;

  explicit operator bool() const noexcept { return plan.has_value(); }
};

class CommandModeSwitchPlanner
{
public:
  static CommandModeSwitchPlanResult makePlan(
    const std::vector<ArmCommandModeState> & arm_states,
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces)
  {
    InterfaceLookup interfaces;
    auto result = buildInterfaceLookup(arm_states, interfaces);
    if (result.error != CommandModeSwitchError::None) {
      return result;
    }

    std::vector<PerArmRequest> starts(arm_states.size());
    std::vector<PerArmRequest> stops(arm_states.size());
    result = parseInterfaces(start_interfaces, "start", arm_states, interfaces, starts);
    if (result.error != CommandModeSwitchError::None) {
      return result;
    }
    result = parseInterfaces(stop_interfaces, "stop", arm_states, interfaces, stops);
    if (result.error != CommandModeSwitchError::None) {
      return result;
    }

    CommandModeSwitchPlan plan;
    plan.arms.reserve(arm_states.size());

    for (std::size_t index = 0; index < arm_states.size(); ++index) {
      const auto & arm = arm_states[index];
      const auto & start = starts[index];
      const auto & stop = stops[index];

      if (stop.present && stop.mode != arm.current_mode) {
        return failure(
          CommandModeSwitchError::StopModeMismatch,
          "stop request for arm '" + arm.arm_name + "' does not match its current mode");
      }

      const auto mode_after_stop = stop.present ? ControlMode::None : arm.current_mode;
      if (start.present && mode_after_stop != ControlMode::None) {
        return failure(
          CommandModeSwitchError::StartRequiresStop,
          "arm '" + arm.arm_name + "' must stop its current mode before starting another mode");
      }

      const auto requested_mode =
        start.present ? start.mode : (stop.present ? ControlMode::None : arm.current_mode);
      plan.arms.push_back(ArmCommandModeRequest{
        arm.arm_name, requested_mode, start.present || stop.present,
        start.present
          ? initializationFor(start.mode)
          : (stop.present ? initializationFor(arm.current_mode) : CommandInitialization::None)});
    }

    return CommandModeSwitchPlanResult{std::move(plan), CommandModeSwitchError::None, {}};
  }

private:
  enum class InterfaceSupport
  {
    JointTorque,
    JointVelocity,
    Unsupported,
  };

  struct InterfaceDescription
  {
    std::size_t arm_index;
    InterfaceSupport support;
    std::uint8_t joint_bit;
  };

  struct PerArmRequest
  {
    ControlMode mode{ControlMode::None};
    std::uint8_t joint_mask{0};
    std::size_t count{0};
    bool present{false};
  };

  using InterfaceLookup = std::unordered_map<std::string, InterfaceDescription>;

  static constexpr std::uint8_t kCompleteJointMask = 0x7f;

  static bool isSupportedCurrentMode(ControlMode mode)
  {
    return mode == ControlMode::None || mode == ControlMode::JointTorque ||
           mode == ControlMode::JointVelocity;
  }

  static CommandInitialization initializationFor(ControlMode mode)
  {
    if (mode == ControlMode::JointTorque) {
      return CommandInitialization::ZeroJointEffort;
    }
    if (mode == ControlMode::JointVelocity) {
      return CommandInitialization::ZeroJointVelocity;
    }
    return CommandInitialization::None;
  }

  static ControlMode controlModeFor(InterfaceSupport support)
  {
    if (support == InterfaceSupport::JointTorque) {
      return ControlMode::JointTorque;
    }
    if (support == InterfaceSupport::JointVelocity) {
      return ControlMode::JointVelocity;
    }
    return ControlMode::None;
  }

  static CommandModeSwitchPlanResult failure(CommandModeSwitchError error, std::string message)
  {
    return CommandModeSwitchPlanResult{std::nullopt, error, std::move(message)};
  }

  static bool addInterface(
    InterfaceLookup & interfaces, std::string name, InterfaceDescription description)
  {
    return interfaces.emplace(std::move(name), description).second;
  }

  static CommandModeSwitchPlanResult buildInterfaceLookup(
    const std::vector<ArmCommandModeState> & arm_states, InterfaceLookup & interfaces)
  {
    if (arm_states.empty()) {
      return failure(
        CommandModeSwitchError::InvalidArmConfiguration, "at least one configured arm is required");
    }

    std::unordered_set<std::string> arm_names;
    for (std::size_t arm_index = 0; arm_index < arm_states.size(); ++arm_index) {
      const auto & arm = arm_states[arm_index];
      if (arm.arm_name.empty() || !arm_names.emplace(arm.arm_name).second) {
        return failure(
          CommandModeSwitchError::InvalidArmConfiguration,
          "arm names must be non-empty and unique");
      }
      if (!isSupportedCurrentMode(arm.current_mode)) {
        return failure(
          CommandModeSwitchError::UnsupportedCurrentMode,
          "arm '" + arm.arm_name + "' has a non-MVP current mode");
      }

      for (std::size_t joint = 1; joint <= 7; ++joint) {
        const auto prefix = arm.arm_name + "_joint" + std::to_string(joint) + "/";
        const auto joint_bit = static_cast<std::uint8_t>(1U << (joint - 1));
        if (
          !addInterface(
            interfaces, prefix + "effort", {arm_index, InterfaceSupport::JointTorque, joint_bit}) ||
          !addInterface(
            interfaces, prefix + "velocity",
            {arm_index, InterfaceSupport::JointVelocity, joint_bit}) ||
          !addInterface(
            interfaces, prefix + "position",
            {arm_index, InterfaceSupport::Unsupported, joint_bit})) {
          return failure(
            CommandModeSwitchError::InvalidArmConfiguration,
            "arm names generate colliding command interface names");
        }
      }

      static constexpr std::array<const char *, 16> kCartesianPoseNames{
        "00", "01", "02", "03", "04", "05", "06", "07",
        "08", "09", "10", "11", "12", "13", "14", "15"};
      for (const auto * name : kCartesianPoseNames) {
        if (!addInterface(
              interfaces, arm.arm_name + "_ee_cartesian_position/" + name,
              {arm_index, InterfaceSupport::Unsupported, 0})) {
          return failure(
            CommandModeSwitchError::InvalidArmConfiguration,
            "arm names generate colliding command interface names");
        }
      }

      static constexpr std::array<const char *, 6> kCartesianVelocityNames{
        "tx", "ty", "tz", "omega_x", "omega_y", "omega_z"};
      for (const auto * name : kCartesianVelocityNames) {
        if (!addInterface(
              interfaces, arm.arm_name + "_ee_cartesian_velocity/" + name,
              {arm_index, InterfaceSupport::Unsupported, 0})) {
          return failure(
            CommandModeSwitchError::InvalidArmConfiguration,
            "arm names generate colliding command interface names");
        }
      }
    }
    return CommandModeSwitchPlanResult{};
  }

  static CommandModeSwitchPlanResult parseInterfaces(
    const std::vector<std::string> & requested_interfaces, const char * request_kind,
    const std::vector<ArmCommandModeState> & arm_states, const InterfaceLookup & interfaces,
    std::vector<PerArmRequest> & per_arm_requests)
  {
    std::unordered_set<std::string> seen;
    for (const auto & interface_name : requested_interfaces) {
      const auto found = interfaces.find(interface_name);
      if (found == interfaces.end()) {
        if (claimsConfiguredArm(interface_name, arm_states)) {
          return failure(
            CommandModeSwitchError::UnknownInterface,
            std::string(request_kind) + " interface '" + interface_name +
              "' does not exactly match a configured arm interface");
        }
        continue;
      }
      if (!seen.emplace(interface_name).second) {
        return failure(
          CommandModeSwitchError::DuplicateInterface,
          std::string(request_kind) + " interface '" + interface_name + "' is duplicated");
      }
      const auto & description = found->second;
      if (description.support == InterfaceSupport::Unsupported) {
        return failure(
          CommandModeSwitchError::UnsupportedInterface,
          std::string(request_kind) + " interface '" + interface_name +
            "' requests a mode excluded from the MVP");
      }

      auto & request = per_arm_requests[description.arm_index];
      const auto mode = controlModeFor(description.support);
      if (request.present && request.mode != mode) {
        return failure(
          CommandModeSwitchError::MixedJointModes,
          std::string(request_kind) + " request for one arm mixes joint modes");
      }
      request.present = true;
      request.mode = mode;
      request.joint_mask = static_cast<std::uint8_t>(request.joint_mask | description.joint_bit);
      ++request.count;
    }

    for (const auto & request : per_arm_requests) {
      if (request.present && (request.count != 7 || request.joint_mask != kCompleteJointMask)) {
        return failure(
          CommandModeSwitchError::IncompleteJointSet,
          std::string(request_kind) + " request must contain joint1 through joint7 exactly once");
      }
    }
    return CommandModeSwitchPlanResult{};
  }

  static bool claimsConfiguredArm(
    const std::string & interface_name, const std::vector<ArmCommandModeState> & arm_states)
  {
    const auto separator = interface_name.find('/');
    const auto resource = interface_name.substr(0, separator);
    for (const auto & arm : arm_states) {
      if (resource.rfind(arm.arm_name + "_", 0) == 0) {
        return true;
      }
    }
    return false;
  }
};

}  // namespace franka_hardware
