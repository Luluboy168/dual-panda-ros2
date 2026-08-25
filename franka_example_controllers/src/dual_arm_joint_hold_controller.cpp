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

#include "franka_example_controllers/dual_arm_joint_hold_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <exception>
#include <type_traits>
#include <utility>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>

#include "franka_example_controllers/panda_joint_limits.hpp"

namespace {

constexpr size_t kExpectedCommandInterfaceCount = 14;
constexpr size_t kExpectedStateInterfaceCount = 32;

bool isAsciiArmId(const std::string& arm_id) {
  if (arm_id.empty() || arm_id.size() > franka_example_controllers::kPandaArmIdMaxLength) {
    return false;
  }
  const auto first = static_cast<unsigned char>(arm_id.front());
  if (!((first >= 'A' && first <= 'Z') || (first >= 'a' && first <= 'z'))) {
    return false;
  }
  return std::all_of(arm_id.begin() + 1, arm_id.end(), [](const char value) {
    const auto character = static_cast<unsigned char>(value);
    return (character >= 'A' && character <= 'Z') || (character >= 'a' && character <= 'z') ||
           (character >= '0' && character <= '9') || character == '_';
  });
}

bool validGains(const std::vector<double>& gains) {
  return gains.size() == 7 && std::all_of(gains.begin(), gains.end(), [](const double gain) {
           return std::isfinite(gain) && gain >= 0.0;
         });
}

bool validEffortBounds(const std::vector<double>& bounds) {
  if (bounds.size() != franka_example_controllers::kPandaAbsoluteEffortCeilings.size()) {
    return false;
  }
  for (size_t joint = 0; joint < bounds.size(); ++joint) {
    if (!std::isfinite(bounds[joint]) || bounds[joint] <= 0.0 ||
        bounds[joint] > franka_example_controllers::kPandaAbsoluteEffortCeilings[joint]) {
      return false;
    }
  }
  return true;
}

template <typename Pointer>
Pointer decodePointer(const double encoded) noexcept {
  static_assert(std::is_pointer<Pointer>::value, "decoded value must be a pointer");
  static_assert(sizeof(Pointer) == sizeof(encoded), "pointer interface must fit in a double");
  Pointer pointer = nullptr;
  std::memcpy(&pointer, &encoded, sizeof(pointer));
  return pointer;
}

template <typename Pointer>
bool decodeStablePointer(const hardware_interface::LoanedStateInterface& interface,
                         Pointer& pointer) noexcept {
  try {
    const auto first_value = interface.get_optional<double>(1);
    const auto second_value = interface.get_optional<double>(1);
    if (!first_value || !second_value) {
      return false;
    }
    const auto first_pointer = decodePointer<Pointer>(*first_value);
    const auto second_pointer = decodePointer<Pointer>(*second_value);
    if (first_pointer == nullptr || first_pointer != second_pointer) {
      return false;
    }
    pointer = first_pointer;
    return true;
  } catch (...) {
    return false;
  }
}

template <typename Pointer>
bool pointerStillMatches(const hardware_interface::LoanedStateInterface* interface,
                         const Pointer expected) noexcept {
  if (interface == nullptr || expected == nullptr) {
    return false;
  }
  try {
    const auto value = interface->get_optional<double>(1);
    return value && decodePointer<Pointer>(*value) == expected;
  } catch (...) {
    return false;
  }
}

bool finiteModelInput(const franka::RobotState& state) {
  const auto finite_array = [](const auto& values) {
    return std::all_of(values.begin(), values.end(),
                       [](const double value) { return std::isfinite(value); });
  };
  return finite_array(state.q) && finite_array(state.dq) && finite_array(state.I_total) &&
         finite_array(state.F_x_Ctotal) && std::isfinite(state.m_total);
}

bool readFinite(const hardware_interface::LoanedStateInterface* interface, double& value) noexcept {
  if (interface == nullptr) {
    return false;
  }
  try {
    const auto result = interface->get_optional<double>(1);
    if (!result || !std::isfinite(*result)) {
      return false;
    }
    value = *result;
    return true;
  } catch (...) {
    return false;
  }
}

bool writeEffort(hardware_interface::LoanedCommandInterface* interface,
                 const double effort) noexcept {
  if (interface == nullptr) {
    return false;
  }
  try {
    return interface->set_value(effort, 1);
  } catch (...) {
    return false;
  }
}

std::string jointInterfaceName(const std::string& arm_id,
                               const size_t joint_index,
                               const char* interface_name) {
  return arm_id + "_joint" + std::to_string(joint_index + 1) + "/" + interface_name;
}

}  // namespace

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
DualArmJointHoldController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  if (!configured_) {
    return configuration;
  }
  configuration.names.reserve(kExpectedCommandInterfaceCount);
  for (const auto& arm : arms_) {
    for (size_t joint = 0; joint < kJointCount; ++joint) {
      configuration.names.push_back(
          jointInterfaceName(arm.arm_id, joint, hardware_interface::HW_IF_EFFORT));
    }
  }
  return configuration;
}

controller_interface::InterfaceConfiguration
DualArmJointHoldController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  if (!configured_) {
    return configuration;
  }
  configuration.names.reserve(kExpectedStateInterfaceCount);
  for (const auto& arm : arms_) {
    for (size_t joint = 0; joint < kJointCount; ++joint) {
      configuration.names.push_back(
          jointInterfaceName(arm.arm_id, joint, hardware_interface::HW_IF_POSITION));
      configuration.names.push_back(
          jointInterfaceName(arm.arm_id, joint, hardware_interface::HW_IF_VELOCITY));
    }
    configuration.names.push_back(arm.arm_id + "/robot_state");
    configuration.names.push_back(arm.arm_id + "/robot_model");
  }
  return configuration;
}

void DualArmJointHoldController::release_interfaces() {
  active_ = false;
  if (interfaces_bound_ && zero_required_ && !attemptRequiredZero()) {
    release_zero_failed_ = true;
  }
  resetBindings();
  controller_interface::ControllerInterface::release_interfaces();
}

controller_interface::return_type DualArmJointHoldController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  if (!active_ || !interfaces_bound_) {
    if (interfaces_bound_) {
      attemptRequiredZero();
    }
    return controller_interface::return_type::ERROR;
  }

  std::array<std::array<double, kJointCount>, kArmCount> efforts{};
  std::array<std::array<double, kJointCount>, kArmCount> next_filtered_velocity{};
  if (!computeCommands(efforts, next_filtered_velocity)) {
    attemptRequiredZero();
    return controller_interface::return_type::ERROR;
  }

  for (size_t arm = 0; arm < kArmCount; ++arm) {
    arms_[arm].filtered_velocity = next_filtered_velocity[arm];
  }
  zero_required_ = true;
  if (!writeCommands(efforts)) {
    attemptRequiredZero();
    return controller_interface::return_type::ERROR;
  }
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn DualArmJointHoldController::on_init() {
  try {
    for (size_t arm = 0; arm < kArmCount; ++arm) {
      const auto prefix = "arm_" + std::to_string(arm + 1) + ".";
      auto_declare<std::string>(prefix + "arm_id", "");
      auto_declare<std::vector<double>>(prefix + "k_gains", {});
      auto_declare<std::vector<double>>(prefix + "d_gains", {});
      auto_declare<std::vector<double>>(prefix + "max_effort", {});
    }
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare hold parameters: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DualArmJointHoldController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  configured_ = false;
  active_ = false;
  if (interfaces_bound_) {
    return controller_interface::CallbackReturn::ERROR;
  }
  release_zero_failed_ = false;

  for (size_t arm = 0; arm < kArmCount; ++arm) {
    const auto prefix = "arm_" + std::to_string(arm + 1) + ".";
    const auto arm_id = get_node()->get_parameter(prefix + "arm_id").as_string();
    const auto k_gains = get_node()->get_parameter(prefix + "k_gains").as_double_array();
    const auto d_gains = get_node()->get_parameter(prefix + "d_gains").as_double_array();
    const auto max_effort = get_node()->get_parameter(prefix + "max_effort").as_double_array();
    if (!isAsciiArmId(arm_id)) {
      RCLCPP_ERROR(
          get_node()->get_logger(),
          "%sarm_id must contain 1..64 ASCII characters, start with a letter, and then contain "
          "only letters, digits, or '_'",
          prefix.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    if (!validGains(k_gains) || !validGains(d_gains)) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "%sk_gains and d_gains must each contain seven finite nonnegative values",
                   prefix.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    if (!validEffortBounds(max_effort)) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "%smax_effort must contain seven finite positive values no greater than "
                   "[87, 87, 87, 87, 12, 12, 12]",
                   prefix.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    arms_[arm].arm_id = arm_id;
    std::copy(k_gains.begin(), k_gains.end(), arms_[arm].k_gains.begin());
    std::copy(d_gains.begin(), d_gains.end(), arms_[arm].d_gains.begin());
    std::copy(max_effort.begin(), max_effort.end(), arms_[arm].max_effort.begin());
  }
  if (arms_[0].arm_id == arms_[1].arm_id) {
    RCLCPP_ERROR(get_node()->get_logger(), "The two hold-controller arm IDs must be unique");
    return controller_interface::CallbackReturn::FAILURE;
  }

  configured_ = true;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DualArmJointHoldController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  active_ = false;
  if (!configured_ || release_zero_failed_ || !bindInterfaces()) {
    return controller_interface::CallbackReturn::FAILURE;
  }
  const bool activation_state_valid = captureActivationState();
  const bool zeroed = attemptRequiredZero();
  if (!activation_state_valid || !zeroed) {
    return controller_interface::CallbackReturn::FAILURE;
  }
  active_ = true;
  // While active, conservatively require a final lifecycle/release zero even before the first
  // update. This also covers command storage changed below the controller after activation.
  zero_required_ = true;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DualArmJointHoldController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  active_ = false;
  const bool zeroed = attemptRequiredZero();
  return zeroed ? controller_interface::CallbackReturn::SUCCESS
                : controller_interface::CallbackReturn::ERROR;
}

controller_interface::CallbackReturn DualArmJointHoldController::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  active_ = false;
  configured_ = false;
  if (interfaces_bound_ && !attemptRequiredZero()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  release_zero_failed_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DualArmJointHoldController::on_error(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  active_ = false;
  const bool zeroed = attemptRequiredZero();
  return zeroed ? controller_interface::CallbackReturn::SUCCESS
                : controller_interface::CallbackReturn::ERROR;
}

controller_interface::CallbackReturn DualArmJointHoldController::on_shutdown(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  active_ = false;
  configured_ = false;
  const bool zeroed = attemptRequiredZero();
  return zeroed ? controller_interface::CallbackReturn::SUCCESS
                : controller_interface::CallbackReturn::ERROR;
}

bool DualArmJointHoldController::bindInterfaces() {
  resetBindings();
  if (command_interfaces_.size() != kExpectedCommandInterfaceCount ||
      state_interfaces_.size() != kExpectedStateInterfaceCount) {
    return false;
  }
  for (auto& arm : arms_) {
    if (!bindArmInterfaces(arm)) {
      resetBindings();
      return false;
    }
  }
  if (arms_[0].robot_state == arms_[1].robot_state ||
      arms_[0].robot_model == arms_[1].robot_model) {
    resetBindings();
    return false;
  }
  interfaces_bound_ = true;
  zero_required_ = true;
  return true;
}

bool DualArmJointHoldController::bindArmInterfaces(Arm& arm) {
  for (size_t joint = 0; joint < kJointCount; ++joint) {
    arm.position_interfaces[joint] = findUniqueStateInterface(
        jointInterfaceName(arm.arm_id, joint, hardware_interface::HW_IF_POSITION));
    arm.velocity_interfaces[joint] = findUniqueStateInterface(
        jointInterfaceName(arm.arm_id, joint, hardware_interface::HW_IF_VELOCITY));
    arm.effort_interfaces[joint] = findUniqueCommandInterface(
        jointInterfaceName(arm.arm_id, joint, hardware_interface::HW_IF_EFFORT));
    if (arm.position_interfaces[joint] == nullptr || arm.velocity_interfaces[joint] == nullptr ||
        arm.effort_interfaces[joint] == nullptr) {
      return false;
    }
  }

  arm.robot_state_interface = findUniqueStateInterface(arm.arm_id + "/robot_state");
  arm.robot_model_interface = findUniqueStateInterface(arm.arm_id + "/robot_model");
  if (arm.robot_state_interface == nullptr || arm.robot_model_interface == nullptr ||
      !decodeStablePointer(*arm.robot_state_interface, arm.robot_state) ||
      !decodeStablePointer(*arm.robot_model_interface, arm.robot_model)) {
    return false;
  }
  return true;
}

bool DualArmJointHoldController::captureActivationState() {
  for (auto& arm : arms_) {
    if (arm.robot_state == nullptr || arm.robot_model == nullptr ||
        !finiteModelInput(*arm.robot_state)) {
      return false;
    }
    for (size_t joint = 0; joint < kJointCount; ++joint) {
      double position = 0.0;
      double velocity = 0.0;
      if (!readFinite(arm.position_interfaces[joint], position) ||
          !readFinite(arm.velocity_interfaces[joint], velocity)) {
        return false;
      }
      arm.hold_position[joint] = position;
      arm.filtered_velocity[joint] = velocity;
    }
    try {
      const auto coriolis = arm.robot_model->coriolis(*arm.robot_state);
      if (!std::all_of(coriolis.begin(), coriolis.end(),
                       [](const double value) { return std::isfinite(value); })) {
        return false;
      }
    } catch (...) {
      return false;
    }
  }
  return true;
}

bool DualArmJointHoldController::computeCommands(
    std::array<std::array<double, kJointCount>, kArmCount>& efforts,
    std::array<std::array<double, kJointCount>, kArmCount>& next_filtered_velocity) const {
  for (size_t arm_index = 0; arm_index < kArmCount; ++arm_index) {
    const auto& arm = arms_[arm_index];
    if (arm.robot_state == nullptr || arm.robot_model == nullptr ||
        !pointerStillMatches(arm.robot_state_interface, arm.robot_state) ||
        !pointerStillMatches(arm.robot_model_interface, arm.robot_model) ||
        !finiteModelInput(*arm.robot_state)) {
      return false;
    }

    std::array<double, kJointCount> positions{};
    std::array<double, kJointCount> velocities{};
    for (size_t joint = 0; joint < kJointCount; ++joint) {
      if (!readFinite(arm.position_interfaces[joint], positions[joint]) ||
          !readFinite(arm.velocity_interfaces[joint], velocities[joint])) {
        return false;
      }
      next_filtered_velocity[arm_index][joint] =
          (1.0 - kVelocityFilterAlpha) * arm.filtered_velocity[joint] +
          kVelocityFilterAlpha * velocities[joint];
    }

    std::array<double, kJointCount> coriolis{};
    try {
      coriolis = arm.robot_model->coriolis(*arm.robot_state);
    } catch (...) {
      return false;
    }
    for (size_t joint = 0; joint < kJointCount; ++joint) {
      if (!std::isfinite(coriolis[joint])) {
        return false;
      }
      efforts[arm_index][joint] =
          arm.k_gains[joint] * (arm.hold_position[joint] - positions[joint]) -
          arm.d_gains[joint] * next_filtered_velocity[arm_index][joint] + coriolis[joint];
      if (!std::isfinite(efforts[arm_index][joint])) {
        return false;
      }
      if (std::abs(efforts[arm_index][joint]) > arm.max_effort[joint]) {
        return false;
      }
    }
  }
  return true;
}

bool DualArmJointHoldController::writeCommands(
    const std::array<std::array<double, kJointCount>, kArmCount>& efforts) noexcept {
  bool all_written = true;
  for (size_t arm = 0; arm < kArmCount; ++arm) {
    for (size_t joint = 0; joint < kJointCount; ++joint) {
      const bool written = writeEffort(arms_[arm].effort_interfaces[joint], efforts[arm][joint]);
      all_written = written && all_written;
    }
  }
  return all_written;
}

bool DualArmJointHoldController::writeZeroEffort() noexcept {
  bool all_written = true;
  for (auto& arm : arms_) {
    for (auto* interface : arm.effort_interfaces) {
      const bool written = writeEffort(interface, 0.0);
      all_written = written && all_written;
    }
  }
  return all_written;
}

bool DualArmJointHoldController::attemptRequiredZero() noexcept {
  if (!interfaces_bound_ || !zero_required_) {
    return true;
  }
  if (!writeZeroEffort()) {
    return false;
  }
  zero_required_ = false;
  return true;
}

void DualArmJointHoldController::resetBindings() noexcept {
  interfaces_bound_ = false;
  zero_required_ = false;
  for (auto& arm : arms_) {
    arm.position_interfaces.fill(nullptr);
    arm.velocity_interfaces.fill(nullptr);
    arm.effort_interfaces.fill(nullptr);
    arm.robot_state_interface = nullptr;
    arm.robot_model_interface = nullptr;
    arm.robot_state = nullptr;
    arm.robot_model = nullptr;
  }
}

hardware_interface::LoanedStateInterface* DualArmJointHoldController::findUniqueStateInterface(
    const std::string& name) noexcept {
  hardware_interface::LoanedStateInterface* match = nullptr;
  for (auto& interface : state_interfaces_) {
    if (interface.get_name() == name) {
      if (match != nullptr) {
        return nullptr;
      }
      match = &interface;
    }
  }
  return match;
}

hardware_interface::LoanedCommandInterface* DualArmJointHoldController::findUniqueCommandInterface(
    const std::string& name) noexcept {
  hardware_interface::LoanedCommandInterface* match = nullptr;
  for (auto& interface : command_interfaces_) {
    if (interface.get_name() == name) {
      if (match != nullptr) {
        return nullptr;
      }
      match = &interface;
    }
  }
  return match;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::DualArmJointHoldController,
                       controller_interface::ControllerInterface)
