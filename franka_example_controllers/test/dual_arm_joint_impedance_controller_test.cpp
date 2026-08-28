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

#include "franka_example_controllers/dual_arm_joint_impedance_controller.hpp"

#include <franka/rate_limiting.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <controller_interface/test_utils.hpp>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <future>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <pluginlib/class_loader.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "dual_arm_joint_impedance_controller_core.hpp"
#include "franka_example_controllers/panda_joint_limits.hpp"
#include "franka_hardware/common/model_base.hpp"

namespace franka_example_controllers {

class DualArmJointImpedanceControllerTestAccess {
 public:
  static JointTargetValidationResult accept(DualArmJointImpedanceController& controller,
                                            const size_t arm,
                                            const trajectory_msgs::msg::JointTrajectory& message,
                                            const int64_t ros_now_ns,
                                            const int64_t steady_receive_ns) {
    return controller.core_->acceptTarget(arm, message, ros_now_ns, steady_receive_ns);
  }

  static bool enable(DualArmJointImpedanceController& controller,
                     const size_t arm,
                     const bool enabled,
                     const int64_t steady_now_ns,
                     const int64_t ros_now_ns = 0) {
    return controller.core_->setArmEnabled(arm, enabled, steady_now_ns, ros_now_ns);
  }

  static bool enabled(const DualArmJointImpedanceController& controller, const size_t arm) {
    return controller.core_->armEnabled(arm);
  }

  static std::string topic(const DualArmJointImpedanceController& controller, const size_t arm) {
    return controller.core_->subscriptionTopic(arm);
  }

  static std::string service(const DualArmJointImpedanceController& controller, const size_t arm) {
    return controller.core_->enableServiceName(arm);
  }

  static std::array<double, kImpedanceJointCount> target(
      const DualArmJointImpedanceController& controller,
      const size_t arm) {
    return controller.core_->internalTarget(arm);
  }

  static uint64_t callbackEntries(const DualArmJointImpedanceController& controller) {
    return controller.core_->non_rt_callback_entries_.load(std::memory_order_acquire);
  }

  static void forceGenerationNearWrap(DualArmJointImpedanceController& controller,
                                      const size_t arm) {
    constexpr uint64_t kNearWrap = std::numeric_limits<uint64_t>::max() - 1U;
    controller.core_->arms_.at(arm).inbox.enable_generation_.store(kNearWrap,
                                                                   std::memory_order_release);
    controller.core_->arms_.at(arm).observed_enable_generation = kNearWrap;
  }

  static uint64_t enableGeneration(const DualArmJointImpedanceController& controller,
                                   const size_t arm) {
    return controller.core_->arms_.at(arm).inbox.enableGeneration();
  }

  static bool bufferedTargetValid(const DualArmJointImpedanceController& controller,
                                  const size_t arm) {
    return controller.core_->arms_.at(arm).inbox.nonRealtimeTarget().valid;
  }

  static bool bindingsCleared(const DualArmJointImpedanceController& controller) {
    if (controller.core_->interfaces_bound_) {
      return false;
    }
    return std::all_of(
        controller.core_->arms_.begin(), controller.core_->arms_.end(), [](const auto& arm) {
          return arm.robot_state_interface == nullptr && arm.robot_model_interface == nullptr &&
                 arm.robot_state == nullptr && arm.robot_model == nullptr &&
                 std::all_of(arm.position_interfaces.begin(), arm.position_interfaces.end(),
                             [](const auto* value) { return value == nullptr; }) &&
                 std::all_of(arm.velocity_interfaces.begin(), arm.velocity_interfaces.end(),
                             [](const auto* value) { return value == nullptr; }) &&
                 std::all_of(arm.effort_interfaces.begin(), arm.effort_interfaces.end(),
                             [](const auto* value) { return value == nullptr; });
        });
  }
};

namespace {

constexpr size_t kArmCount = 2;
constexpr size_t kJointCount = 7;
constexpr size_t kCommandCount = kArmCount * kJointCount;
constexpr int64_t kRosNowNs = 10000000000LL;

using JointArray = std::array<double, kJointCount>;
using JointNames = std::array<std::string, kJointCount>;

JointArray basePose(const double offset = 0.0) {
  return JointArray{{0.0 + offset, -0.5 + offset, 0.0 + offset, -1.0 + offset, 0.0 + offset,
                     1.0 + offset, 0.0 + offset}};
}

JointNames makeJointNames(const std::string& arm_id) {
  JointNames names;
  for (size_t joint = 0; joint < kJointCount; ++joint) {
    names[joint] = arm_id + "_joint" + std::to_string(joint + 1);
  }
  return names;
}

std::vector<std::string> asVector(const JointNames& names) {
  return {names.begin(), names.end()};
}

std::vector<double> asVector(const JointArray& values) {
  return {values.begin(), values.end()};
}

class FixedCoriolisModel final : public franka_hardware::ModelBase {
 public:
  JointArray coriolis(const franka::RobotState& /*robot_state*/) const override {
    if (throw_on_coriolis_) {
      throw std::runtime_error("injected model failure");
    }
    return coriolis_;
  }

  void setCoriolis(const JointArray& values) { coriolis_ = values; }
  void setThrow(bool value) noexcept { throw_on_coriolis_ = value; }

 private:
  std::array<double, 16> poseImpl(franka::Frame /*frame*/,
                                  const JointArray& /*q*/,
                                  const std::array<double, 16>& /*f_t_ee*/,
                                  const std::array<double, 16>& /*ee_t_k*/) const override {
    return {};
  }

  std::array<double, 42> bodyJacobianImpl(franka::Frame /*frame*/,
                                          const JointArray& /*q*/,
                                          const std::array<double, 16>& /*f_t_ee*/,
                                          const std::array<double, 16>& /*ee_t_k*/) const override {
    return {};
  }

  std::array<double, 42> zeroJacobianImpl(franka::Frame /*frame*/,
                                          const JointArray& /*q*/,
                                          const std::array<double, 16>& /*f_t_ee*/,
                                          const std::array<double, 16>& /*ee_t_k*/) const override {
    return {};
  }

  std::array<double, 49> massImpl(const JointArray& /*q*/,
                                  const std::array<double, 9>& /*i_total*/,
                                  double /*m_total*/,
                                  const std::array<double, 3>& /*f_x_ctotal*/) const override {
    return {};
  }

  JointArray coriolisImpl(const JointArray& /*q*/,
                          const JointArray& /*dq*/,
                          const std::array<double, 9>& /*i_total*/,
                          double /*m_total*/,
                          const std::array<double, 3>& /*f_x_ctotal*/) const override {
    return coriolis_;
  }

  JointArray gravityImpl(const JointArray& /*q*/,
                         double /*m_total*/,
                         const std::array<double, 3>& /*f_x_ctotal*/,
                         const std::array<double, 3>& /*gravity_earth*/) const override {
    return {};
  }

  JointArray coriolis_{};
  bool throw_on_coriolis_{false};
};

class ImpedanceHardwareFixture {
 public:
  explicit ImpedanceHardwareFixture(const std::array<JointNames, kArmCount>& joint_names,
                                    const std::array<std::string, kArmCount>& arm_ids)
      : arm_ids_(arm_ids) {
    state_handles_.reserve(32);
    command_handles_.reserve(kCommandCount);
    for (size_t arm = 0; arm < kArmCount; ++arm) {
      state_pointers_[arm] = &robot_states_[arm];
      model_pointers_[arm] = &models_[arm];
      setJointState(arm, basePose(0.05 * static_cast<double>(arm)), {});
      JointArray coriolis{};
      for (size_t joint = 0; joint < kJointCount; ++joint) {
        coriolis[joint] = 0.1 * static_cast<double>(arm + 1) + 0.01 * static_cast<double>(joint);
        state_handles_.push_back(std::make_shared<hardware_interface::StateInterface>(
            joint_names[arm][joint], "position", &positions_[arm][joint]));
        state_handles_.push_back(std::make_shared<hardware_interface::StateInterface>(
            joint_names[arm][joint], "velocity", &velocities_[arm][joint]));
        const size_t command_index = command_handles_.size();
        command_handles_.push_back(std::make_shared<hardware_interface::CommandInterface>(
            joint_names[arm][joint], "effort", &commands_[arm][joint]));
        command_handles_.back()->set_on_set_command_limiter(
            [this, command_index](const double value, bool& limited) {
              ++write_counts_[command_index];
              if (write_hook_) {
                write_hook_(command_index);
              }
              limited = false;
              return value;
            });
      }
      models_[arm].setCoriolis(coriolis);
      state_handles_.push_back(std::make_shared<hardware_interface::StateInterface>(
          arm_ids_[arm], "robot_state",
          reinterpret_cast<double*>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
              &state_pointers_[arm])));
      state_handles_.push_back(std::make_shared<hardware_interface::StateInterface>(
          arm_ids_[arm], "robot_model",
          reinterpret_cast<double*>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
              &model_pointers_[arm])));
    }
  }

  void setJointState(const size_t arm, const JointArray& positions, const JointArray& velocities) {
    positions_[arm] = positions;
    velocities_[arm] = velocities;
    robot_states_[arm].q = positions;
    robot_states_[arm].dq = velocities;
  }

  void setPosition(const size_t arm, const size_t joint, const double position) {
    positions_[arm][joint] = position;
    robot_states_[arm].q[joint] = position;
  }

  void setVelocity(const size_t arm, const size_t joint, const double velocity) {
    velocities_[arm][joint] = velocity;
    robot_states_[arm].dq[joint] = velocity;
  }

  void setCoriolis(const size_t arm, const JointArray& coriolis) {
    models_[arm].setCoriolis(coriolis);
  }

  void setModelThrow(const size_t arm, const bool value) { models_[arm].setThrow(value); }
  franka::RobotState& robotState(const size_t arm) { return robot_states_[arm]; }
  void setNullStatePointer(const size_t arm) { state_pointers_[arm] = nullptr; }
  void setNullModelPointer(const size_t arm) { model_pointers_[arm] = nullptr; }
  void aliasModelPointer(const size_t destination_arm, const size_t source_arm) {
    model_pointers_[destination_arm] = model_pointers_[source_arm];
  }

  void replaceStateInterface(const std::string& expected_name,
                             const std::string& replacement_prefix,
                             const std::string& replacement_interface) {
    const auto iterator =
        std::find_if(state_handles_.begin(), state_handles_.end(),
                     [&](const auto& interface) { return interface->get_name() == expected_name; });
    if (iterator == state_handles_.end()) {
      throw std::runtime_error("state interface to replace was not found");
    }
    *iterator = std::make_shared<hardware_interface::StateInterface>(
        replacement_prefix, replacement_interface, &replacement_state_value_);
  }

  void duplicateStateInterface(const std::string& missing_name, const std::string& duplicate_name) {
    const auto missing =
        std::find_if(state_handles_.begin(), state_handles_.end(),
                     [&](const auto& interface) { return interface->get_name() == missing_name; });
    const auto duplicate = std::find_if(
        state_handles_.begin(), state_handles_.end(),
        [&](const auto& interface) { return interface->get_name() == duplicate_name; });
    if (missing == state_handles_.end() || duplicate == state_handles_.end()) {
      throw std::runtime_error("state interface for duplicate injection was not found");
    }
    *missing = *duplicate;
  }

  void replaceCommandInterface(const std::string& expected_name,
                               const std::string& replacement_prefix) {
    const auto iterator =
        std::find_if(command_handles_.begin(), command_handles_.end(),
                     [&](const auto& interface) { return interface->get_name() == expected_name; });
    if (iterator == command_handles_.end()) {
      throw std::runtime_error("command interface to replace was not found");
    }
    *iterator = std::make_shared<hardware_interface::CommandInterface>(replacement_prefix, "effort",
                                                                       &replacement_command_value_);
  }

  void assignTo(DualArmJointImpedanceController& controller, const bool shuffled) const {
    auto command_order = command_handles_;
    auto state_order = state_handles_;
    if (shuffled) {
      std::rotate(command_order.begin(), command_order.begin() + 5, command_order.end());
      std::reverse(command_order.begin(), command_order.end());
      std::rotate(state_order.begin(), state_order.begin() + 11, state_order.end());
      std::reverse(state_order.begin(), state_order.end());
    }
    std::vector<hardware_interface::LoanedCommandInterface> commands;
    std::vector<hardware_interface::LoanedStateInterface> states;
    commands.reserve(command_order.size());
    states.reserve(state_order.size());
    for (const auto& handle : command_order) {
      commands.emplace_back(handle, hardware_interface::LoanedCommandInterface::Deleter{});
    }
    for (const auto& handle : state_order) {
      states.emplace_back(handle);
    }
    controller.assign_interfaces(std::move(commands), std::move(states));
  }

  void fillCommands(const double value) {
    for (auto& arm : commands_) {
      arm.fill(value);
    }
  }

  double command(const size_t arm, const size_t joint) const { return commands_[arm][joint]; }
  bool allCommandsEqual(const double expected) const {
    return std::all_of(commands_.begin(), commands_.end(), [&](const JointArray& arm) {
      return std::all_of(arm.begin(), arm.end(),
                         [&](const double value) { return value == expected; });
    });
  }

  void resetWriteCounts() { write_counts_.fill(0); }
  bool everyInterfaceWritten(const size_t count) const {
    return std::all_of(write_counts_.begin(), write_counts_.end(),
                       [&](const size_t value) { return value == count; });
  }
  size_t writeCount(const size_t command) const { return write_counts_.at(command); }
  void setWriteHook(std::function<void(size_t)> hook) { write_hook_ = std::move(hook); }
  void clearWriteHook() { write_hook_ = {}; }

  hardware_interface::CommandInterface::SharedPtr commandHandle(const std::string& name) const {
    const auto iterator =
        std::find_if(command_handles_.begin(), command_handles_.end(),
                     [&](const auto& interface) { return interface->get_name() == name; });
    if (iterator == command_handles_.end()) {
      throw std::runtime_error("command interface was not found");
    }
    return *iterator;
  }

 private:
  std::array<std::string, kArmCount> arm_ids_;
  std::array<JointArray, kArmCount> positions_{};
  std::array<JointArray, kArmCount> velocities_{};
  std::array<JointArray, kArmCount> commands_{};
  std::array<franka::RobotState, kArmCount> robot_states_{};
  std::array<FixedCoriolisModel, kArmCount> models_{};
  std::array<franka::RobotState*, kArmCount> state_pointers_{};
  std::array<franka_hardware::ModelBase*, kArmCount> model_pointers_{};
  std::vector<hardware_interface::StateInterface::SharedPtr> state_handles_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_handles_;
  std::array<size_t, kCommandCount> write_counts_{};
  std::function<void(size_t)> write_hook_{};
  double replacement_state_value_{0.0};
  double replacement_command_value_{0.0};
};

struct ControllerParameters {
  std::array<std::string, kArmCount> arm_ids{{"arm", "arm_extra"}};
  std::array<JointNames, kArmCount> joint_names{
      {makeJointNames("arm"), makeJointNames("arm_extra")}};
  std::array<std::vector<double>, kArmCount> k_gains{
      {std::vector<double>(kJointCount, 10.0), std::vector<double>(kJointCount, 12.0)}};
  std::array<std::vector<double>, kArmCount> d_gains{
      {std::vector<double>(kJointCount, 1.0), std::vector<double>(kJointCount, 1.5)}};
  std::array<std::vector<double>, kArmCount> max_effort{
      {asVector(kPandaAbsoluteEffortCeilings), asVector(kPandaAbsoluteEffortCeilings)}};
  std::array<std::vector<double>, kArmCount> position_lower{
      {asVector(kPandaPositionLowerLimits), asVector(kPandaPositionLowerLimits)}};
  std::array<std::vector<double>, kArmCount> position_upper{
      {asVector(kPandaPositionUpperLimits), asVector(kPandaPositionUpperLimits)}};
  std::array<std::vector<double>, kArmCount> max_target_velocity{
      {std::vector<double>(kJointCount, 0.5), std::vector<double>(kJointCount, 0.5)}};
  double watchdog_timeout{10.0};
  double max_header_age{1.0};
  double future_tolerance{0.1};
};

std::unique_ptr<DualArmJointImpedanceController> makeController(
    const ControllerParameters& parameters) {
  rclcpp::NodeOptions node_options;
  node_options.enable_rosout(false);
  node_options.start_parameter_event_publisher(false);
  node_options.start_parameter_services(false);
  std::vector<rclcpp::Parameter> overrides{
      rclcpp::Parameter("watchdog_timeout", parameters.watchdog_timeout),
      rclcpp::Parameter("max_header_age", parameters.max_header_age),
      rclcpp::Parameter("future_tolerance", parameters.future_tolerance),
  };
  for (size_t arm = 0; arm < kArmCount; ++arm) {
    const auto prefix = "arm_" + std::to_string(arm + 1) + ".";
    overrides.emplace_back(prefix + "arm_id", parameters.arm_ids[arm]);
    overrides.emplace_back(prefix + "joint_names", asVector(parameters.joint_names[arm]));
    overrides.emplace_back(prefix + "k_gains", parameters.k_gains[arm]);
    overrides.emplace_back(prefix + "d_gains", parameters.d_gains[arm]);
    overrides.emplace_back(prefix + "max_effort", parameters.max_effort[arm]);
    overrides.emplace_back(prefix + "position_lower", parameters.position_lower[arm]);
    overrides.emplace_back(prefix + "position_upper", parameters.position_upper[arm]);
    overrides.emplace_back(prefix + "max_target_velocity", parameters.max_target_velocity[arm]);
  }
  node_options.parameter_overrides(overrides);

  controller_interface::ControllerInterfaceParams controller_parameters;
  controller_parameters.controller_name = "dual_arm_joint_impedance_controller_test";
  controller_parameters.update_rate = 1000;
  controller_parameters.controller_manager_update_rate = 1000;
  controller_parameters.node_options = node_options;

  auto controller = std::make_unique<DualArmJointImpedanceController>();
  if (controller->init(controller_parameters) != controller_interface::return_type::OK) {
    throw std::runtime_error("controller initialization failed");
  }
  return controller;
}

bool configure(const std::unique_ptr<DualArmJointImpedanceController>& controller) {
  return controller_interface::configure_succeeds(controller);
}

bool activate(const std::unique_ptr<DualArmJointImpedanceController>& controller) {
  return controller_interface::activate_succeeds(controller);
}

controller_interface::return_type update(DualArmJointImpedanceController& controller,
                                         const double seconds = 0.1) {
  return controller.update(rclcpp::Time(0, 0, RCL_ROS_TIME),
                           rclcpp::Duration::from_seconds(seconds));
}

trajectory_msgs::msg::JointTrajectory makeMessage(const JointNames& names,
                                                  const JointArray& positions,
                                                  const int64_t stamp_ns = kRosNowNs,
                                                  const bool reverse = false) {
  trajectory_msgs::msg::JointTrajectory message;
  message.header.stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
  message.header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);
  message.points.resize(1);
  for (size_t index = 0; index < kJointCount; ++index) {
    const size_t source = reverse ? kJointCount - index - 1 : index;
    message.joint_names.push_back(names[source]);
    message.points.front().positions.push_back(positions[source]);
  }
  return message;
}

ImpedanceTargetPolicy makePolicy(const JointNames& names) {
  ImpedanceTargetPolicy policy;
  policy.joint_names = names;
  policy.position_lower = kPandaPositionLowerLimits;
  policy.position_upper = kPandaPositionUpperLimits;
  policy.max_header_age_ns = 1000000000LL;
  policy.future_tolerance_ns = 100000000LL;
  return policy;
}

struct ReferenceJointTargetResult {
  JointTargetValidationResult result{JointTargetValidationResult::Accepted};
  JointArray positions{};
};

ReferenceJointTargetResult referenceJointTargetValidation(
    const trajectory_msgs::msg::JointTrajectory& message,
    const JointNames& names,
    const JointArray& lower,
    const JointArray& upper,
    const int64_t ros_now_ns,
    const int64_t maximum_age_ns,
    const int64_t future_tolerance_ns) {
  if (!message.header.frame_id.empty()) {
    return {JointTargetValidationResult::InvalidFrame, {}};
  }
  if (message.header.stamp.sec < 0 || message.header.stamp.nanosec >= 1000000000U ||
      (message.header.stamp.sec == 0 && message.header.stamp.nanosec == 0)) {
    return {JointTargetValidationResult::InvalidStamp, {}};
  }
  const int64_t stamp_ns = static_cast<int64_t>(message.header.stamp.sec) * 1000000000LL +
                           static_cast<int64_t>(message.header.stamp.nanosec);
  const long double age = static_cast<long double>(ros_now_ns) - static_cast<long double>(stamp_ns);
  if (age > static_cast<long double>(maximum_age_ns)) {
    return {JointTargetValidationResult::HeaderTooOld, {}};
  }
  if (-age > static_cast<long double>(future_tolerance_ns)) {
    return {JointTargetValidationResult::HeaderTooFarInFuture, {}};
  }
  if (message.joint_names.size() != kJointCount) {
    return {JointTargetValidationResult::InvalidNameCount, {}};
  }
  if (message.points.size() != 1U) {
    return {JointTargetValidationResult::InvalidPointCount, {}};
  }
  const auto& point = message.points.front();
  if (point.positions.size() != kJointCount) {
    return {JointTargetValidationResult::InvalidPositionCount, {}};
  }
  if (!point.velocities.empty()) {
    return {JointTargetValidationResult::VelocityCommandNotAllowed, {}};
  }
  if (!point.accelerations.empty()) {
    return {JointTargetValidationResult::AccelerationCommandNotAllowed, {}};
  }
  if (!point.effort.empty()) {
    return {JointTargetValidationResult::EffortCommandNotAllowed, {}};
  }
  if (point.time_from_start.sec != 0 || point.time_from_start.nanosec != 0U) {
    return {JointTargetValidationResult::InvalidTimeFromStart, {}};
  }

  ReferenceJointTargetResult expected;
  std::array<bool, kJointCount> matched{};
  for (std::size_t message_index = 0; message_index < kJointCount; ++message_index) {
    const auto found = std::find(names.begin(), names.end(), message.joint_names[message_index]);
    if (found == names.end()) {
      return {JointTargetValidationResult::DuplicateOrUnknownJoint, {}};
    }
    const auto joint = static_cast<std::size_t>(std::distance(names.begin(), found));
    if (matched[joint]) {
      return {JointTargetValidationResult::DuplicateOrUnknownJoint, {}};
    }
    const auto position = point.positions[message_index];
    if (!std::isfinite(position)) {
      return {JointTargetValidationResult::NonfinitePosition, {}};
    }
    if (position < lower[joint] || position > upper[joint]) {
      return {JointTargetValidationResult::PositionLimitExceeded, {}};
    }
    matched[joint] = true;
    expected.positions[joint] = position;
  }
  return expected;
}

std::string mutateGeneratedJointTarget(trajectory_msgs::msg::JointTrajectory& message,
                                       const std::size_t category,
                                       std::mt19937_64& engine) {
  const auto joint = static_cast<std::size_t>(engine() % kJointCount);
  switch (category) {
    case 0:
      return "valid canonical";
    case 1:
      message.header.frame_id = "base";
      return "nonempty frame";
    case 2:
      message.header.stamp.sec = 0;
      message.header.stamp.nanosec = 0;
      return "zero stamp";
    case 3:
      message.header.stamp.sec = -1;
      return "negative stamp";
    case 4:
      message.header.stamp.nanosec = 1000000000U;
      return "invalid stamp nanoseconds";
    case 5:
      message.header.stamp.sec = 8;
      message.header.stamp.nanosec = 999999999U;
      return "header one nanosecond too old";
    case 6:
      message.header.stamp.sec = 10;
      message.header.stamp.nanosec = 100000001U;
      return "header one nanosecond too far in future";
    case 7:
      message.joint_names.erase(message.joint_names.begin() + static_cast<std::ptrdiff_t>(joint));
      return "missing joint name index=" + std::to_string(joint);
    case 8:
      message.points.clear();
      return "no trajectory point";
    case 9:
      message.points.push_back(message.points.front());
      return "two trajectory points";
    case 10:
      message.points.front().positions.erase(message.points.front().positions.begin() +
                                             static_cast<std::ptrdiff_t>(joint));
      return "missing position index=" + std::to_string(joint);
    case 11:
      message.points.front().velocities.push_back(0.0);
      return "velocity field present";
    case 12:
      message.points.front().accelerations.push_back(0.0);
      return "acceleration field present";
    case 13:
      message.points.front().effort.push_back(0.0);
      return "effort field present";
    case 14:
      message.points.front().time_from_start.sec = (engine() & 1U) != 0U ? 1 : -1;
      return "nonzero time_from_start seconds";
    case 15:
      message.points.front().time_from_start.nanosec = 1U;
      return "nonzero time_from_start nanoseconds";
    case 16:
      message.joint_names[joint] = message.joint_names[(joint + 1U) % kJointCount];
      return "duplicate joint name index=" + std::to_string(joint);
    case 17:
      message.joint_names[joint] = "unknown_joint";
      return "unknown joint name index=" + std::to_string(joint);
    case 18:
      message.points.front().positions[joint] = std::numeric_limits<double>::quiet_NaN();
      return "nan position index=" + std::to_string(joint);
    case 19:
      message.points.front().positions[joint] = (engine() & 1U) != 0U
                                                    ? std::numeric_limits<double>::infinity()
                                                    : -std::numeric_limits<double>::infinity();
      return "infinite position index=" + std::to_string(joint);
    case 20:
      message.points.front().positions[joint] = std::nextafter(
          kPandaPositionLowerLimits[joint], -std::numeric_limits<double>::infinity());
      return "position below lower bound index=" + std::to_string(joint);
    case 21:
      message.points.front().positions[joint] =
          std::nextafter(kPandaPositionUpperLimits[joint], std::numeric_limits<double>::infinity());
      return "position above upper bound index=" + std::to_string(joint);
    case 22:
      message.points.front().positions[joint] = (engine() & 1U) != 0U
                                                    ? kPandaPositionLowerLimits[joint]
                                                    : kPandaPositionUpperLimits[joint];
      return "exact position bound index=" + std::to_string(joint);
    default: {
      std::vector<std::size_t> order(kJointCount);
      for (std::size_t index = 0; index < kJointCount; ++index) {
        order[index] = index;
      }
      std::shuffle(order.begin(), order.end(), engine);
      const auto original_names = message.joint_names;
      const auto original_positions = message.points.front().positions;
      for (std::size_t index = 0; index < kJointCount; ++index) {
        message.joint_names[index] = original_names[order[index]];
        message.points.front().positions[index] = original_positions[order[index]];
      }
      return "valid permutation";
    }
  }
}

void expectArrayNear(const JointArray& actual, const JointArray& expected, const double tolerance) {
  for (size_t joint = 0; joint < kJointCount; ++joint) {
    EXPECT_NEAR(actual[joint], expected[joint], tolerance) << "joint " << joint;
  }
}

bool waitForCallbackEntries(const DualArmJointImpedanceController& controller,
                            const uint64_t expected,
                            const std::chrono::milliseconds timeout = std::chrono::seconds(2)) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (DualArmJointImpedanceControllerTestAccess::callbackEntries(controller) >= expected) {
      return true;
    }
    std::this_thread::yield();
  }
  return DualArmJointImpedanceControllerTestAccess::callbackEntries(controller) >= expected;
}

class DualArmJointImpedanceControllerTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(DualArmJointImpedanceControllerTest, CentralPandaLimitsMatchReviewedValues) {
  EXPECT_EQ(kPandaAbsoluteEffortCeilings, (JointArray{{87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0}}));
  EXPECT_EQ(kPandaPositionLowerLimits,
            (JointArray{{-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973}}));
  EXPECT_EQ(kPandaPositionUpperLimits,
            (JointArray{{2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973}}));
  EXPECT_EQ(kPandaAbsoluteJointVelocityCeilings,
            (JointArray{{2.175, 2.175, 2.175, 2.175, 2.610, 2.610, 2.610}}));
  EXPECT_EQ(kPandaFciJointVelocityCeilings, franka::kMaxJointVelocity);
  EXPECT_EQ(kPandaFciJointAccelerationCeilings, franka::kMaxJointAcceleration);
}

TEST_F(DualArmJointImpedanceControllerTest, DeclaresExactInterfacesTopicsAndServices) {
  ControllerParameters parameters;
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  const auto commands = controller->command_interface_configuration();
  const auto states = controller->state_interface_configuration();
  ASSERT_EQ(commands.names.size(), 14U);
  // 32 = 2 arms * (2*7 joint interfaces + 2 [robot_state, robot_model]).
  ASSERT_EQ(states.names.size(), 32U);
  for (size_t arm = 0; arm < kArmCount; ++arm) {
    for (const auto& joint_name : parameters.joint_names[arm]) {
      EXPECT_EQ(std::count(commands.names.begin(), commands.names.end(), joint_name + "/effort"),
                1);
      EXPECT_EQ(std::count(states.names.begin(), states.names.end(), joint_name + "/position"), 1);
      EXPECT_EQ(std::count(states.names.begin(), states.names.end(), joint_name + "/velocity"), 1);
    }
    EXPECT_EQ(std::count(states.names.begin(), states.names.end(),
                         parameters.arm_ids[arm] + "/robot_state"),
              1);
    EXPECT_EQ(std::count(states.names.begin(), states.names.end(),
                         parameters.arm_ids[arm] + "/robot_model"),
              1);
    EXPECT_EQ(DualArmJointImpedanceControllerTestAccess::topic(*controller, arm),
              "/dual_arm_joint_impedance_controller_test/arm_" + std::to_string(arm + 1) +
                  "/joint_target");
    EXPECT_EQ(
        DualArmJointImpedanceControllerTestAccess::service(*controller, arm),
        "/dual_arm_joint_impedance_controller_test/arm_" + std::to_string(arm + 1) + "/enable");
    EXPECT_FALSE(DualArmJointImpedanceControllerTestAccess::enabled(*controller, arm));
  }
}

TEST_F(DualArmJointImpedanceControllerTest, RejectsInvalidArmAndConfigurationJointMappings) {
  for (size_t scenario = 0; scenario < 9; ++scenario) {
    SCOPED_TRACE(scenario);
    ControllerParameters parameters;
    switch (scenario) {
      case 0:
        parameters.arm_ids[0] = parameters.arm_ids[1];
        break;
      case 1:
        parameters.arm_ids[0] = "1arm";
        break;
      case 2:
        parameters.arm_ids[0] = "bad/arm";
        break;
      case 3:
        std::swap(parameters.joint_names[0], parameters.joint_names[1]);
        break;
      case 4:
        std::swap(parameters.joint_names[0][0], parameters.joint_names[0][1]);
        break;
      case 5:
        parameters.joint_names[0][6] = "arm_joint8";
        break;
      case 6:
        parameters.joint_names[1][0] = "wrong_joint1";
        break;
      case 7:
        parameters.joint_names[1][6].clear();
        break;
      case 8:
        parameters.arm_ids[0] = "a" + std::string(kPandaArmIdMaxLength, 'x');
        parameters.joint_names[0] = makeJointNames(parameters.arm_ids[0]);
        break;
      default:
        FAIL() << "unhandled scenario";
    }
    EXPECT_FALSE(configure(makeController(parameters)));
  }

  ControllerParameters maximum_length;
  maximum_length.arm_ids[0] = "a" + std::string(kPandaArmIdMaxLength - 1U, 'x');
  maximum_length.joint_names[0] = makeJointNames(maximum_length.arm_ids[0]);
  EXPECT_TRUE(configure(makeController(maximum_length)));
}

TEST_F(DualArmJointImpedanceControllerTest, RejectsInvalidGainsEffortBoundsAndTiming) {
  for (size_t scenario = 0; scenario < 14; ++scenario) {
    SCOPED_TRACE(scenario);
    ControllerParameters parameters;
    switch (scenario) {
      case 0:
        parameters.k_gains[0].pop_back();
        break;
      case 1:
        parameters.d_gains[1].push_back(1.0);
        break;
      case 2:
        parameters.k_gains[0][2] = -0.1;
        break;
      case 3:
        parameters.d_gains[1][3] = std::numeric_limits<double>::infinity();
        break;
      case 4:
        parameters.max_effort[0].pop_back();
        break;
      case 5:
        parameters.max_effort[1][0] = 0.0;
        break;
      case 6:
        parameters.max_effort[0][3] = std::numeric_limits<double>::quiet_NaN();
        break;
      case 7:
        parameters.max_effort[1][0] = std::nextafter(87.0, 88.0);
        break;
      case 8:
        parameters.max_effort[0][6] = std::nextafter(12.0, 13.0);
        break;
      case 9:
        parameters.watchdog_timeout = 0.0;
        break;
      case 10:
        parameters.max_header_age = -1.0;
        break;
      case 11:
        parameters.future_tolerance = std::numeric_limits<double>::infinity();
        break;
      case 12:
        parameters.watchdog_timeout = std::numeric_limits<double>::denorm_min();
        break;
      case 13:
        parameters.future_tolerance = std::numeric_limits<double>::quiet_NaN();
        break;
      default:
        FAIL() << "unhandled scenario";
    }
    EXPECT_FALSE(configure(makeController(parameters)));
  }
}

TEST_F(DualArmJointImpedanceControllerTest, RejectsLoosePositionAndTargetVelocityLimits) {
  for (size_t scenario = 0; scenario < 13; ++scenario) {
    SCOPED_TRACE(scenario);
    ControllerParameters parameters;
    switch (scenario) {
      case 0:
        parameters.position_lower[0].pop_back();
        break;
      case 1:
        parameters.position_upper[1].push_back(1.0);
        break;
      case 2:
        parameters.position_lower[0][0] = std::nextafter(kPandaPositionLowerLimits[0], -4.0);
        break;
      case 3:
        parameters.position_upper[1][3] = std::nextafter(kPandaPositionUpperLimits[3], 1.0);
        break;
      case 4:
        parameters.position_lower[0][2] = parameters.position_upper[0][2];
        break;
      case 5:
        parameters.position_upper[0][4] = std::numeric_limits<double>::infinity();
        break;
      case 6:
        parameters.position_lower[1][5] = std::numeric_limits<double>::quiet_NaN();
        break;
      case 7:
        parameters.max_target_velocity[0].pop_back();
        break;
      case 8:
        parameters.max_target_velocity[1][0] = 0.0;
        break;
      case 9:
        parameters.max_target_velocity[0][2] = -0.1;
        break;
      case 10:
        parameters.max_target_velocity[1][4] = std::numeric_limits<double>::infinity();
        break;
      case 11:
        parameters.max_target_velocity[0][0] =
            std::nextafter(kPandaAbsoluteJointVelocityCeilings[0], 3.0);
        break;
      case 12:
        parameters.max_target_velocity[1][6] =
            std::nextafter(kPandaAbsoluteJointVelocityCeilings[6], 3.0);
        break;
      default:
        FAIL() << "unhandled scenario";
    }
    EXPECT_FALSE(configure(makeController(parameters)));
  }

  ControllerParameters exact;
  for (size_t arm = 0; arm < kArmCount; ++arm) {
    exact.max_target_velocity[arm] = asVector(kPandaAbsoluteJointVelocityCeilings);
  }
  EXPECT_TRUE(configure(makeController(exact)));
}

TEST_F(DualArmJointImpedanceControllerTest, RejectsEveryJointTrajectoryContractViolation) {
  const auto names = makeJointNames("arm");
  const auto positions = basePose();
  ArmImpedanceTargetInbox inbox;
  inbox.configure(makePolicy(names));

  for (size_t scenario = 0; scenario < 21; ++scenario) {
    SCOPED_TRACE(scenario);
    auto message = makeMessage(names, positions);
    JointTargetValidationResult expected = JointTargetValidationResult::Accepted;
    switch (scenario) {
      case 0:
        message.header.frame_id = "base";
        expected = JointTargetValidationResult::InvalidFrame;
        break;
      case 1:
        message.header.stamp.sec = 0;
        message.header.stamp.nanosec = 0;
        expected = JointTargetValidationResult::InvalidStamp;
        break;
      case 2:
        message.header.stamp.sec = -1;
        expected = JointTargetValidationResult::InvalidStamp;
        break;
      case 3:
        message.header.stamp.nanosec = 1000000000U;
        expected = JointTargetValidationResult::InvalidStamp;
        break;
      case 4:
        message.joint_names.pop_back();
        expected = JointTargetValidationResult::InvalidNameCount;
        break;
      case 5:
        message.points.clear();
        expected = JointTargetValidationResult::InvalidPointCount;
        break;
      case 6:
        message.points.push_back(message.points.front());
        expected = JointTargetValidationResult::InvalidPointCount;
        break;
      case 7:
        message.points.front().positions.pop_back();
        expected = JointTargetValidationResult::InvalidPositionCount;
        break;
      case 8:
        message.points.front().velocities.push_back(0.0);
        expected = JointTargetValidationResult::VelocityCommandNotAllowed;
        break;
      case 9:
        message.points.front().accelerations.push_back(0.0);
        expected = JointTargetValidationResult::AccelerationCommandNotAllowed;
        break;
      case 10:
        message.points.front().effort.push_back(0.0);
        expected = JointTargetValidationResult::EffortCommandNotAllowed;
        break;
      case 11:
        message.points.front().time_from_start.sec = 1;
        expected = JointTargetValidationResult::InvalidTimeFromStart;
        break;
      case 12:
        message.points.front().time_from_start.sec = -1;
        expected = JointTargetValidationResult::InvalidTimeFromStart;
        break;
      case 13:
        message.points.front().time_from_start.nanosec = 1U;
        expected = JointTargetValidationResult::InvalidTimeFromStart;
        break;
      case 14:
        message.joint_names[1] = message.joint_names[0];
        expected = JointTargetValidationResult::DuplicateOrUnknownJoint;
        break;
      case 15:
        message.joint_names[3] = "unknown_joint";
        expected = JointTargetValidationResult::DuplicateOrUnknownJoint;
        break;
      case 16:
        message.points.front().positions[2] = std::numeric_limits<double>::quiet_NaN();
        expected = JointTargetValidationResult::NonfinitePosition;
        break;
      case 17:
        message.points.front().positions[4] = std::numeric_limits<double>::infinity();
        expected = JointTargetValidationResult::NonfinitePosition;
        break;
      case 18:
        message.points.front().positions[0] = std::nextafter(kPandaPositionLowerLimits[0], -4.0);
        expected = JointTargetValidationResult::PositionLimitExceeded;
        break;
      case 19:
        message.points.front().positions[6] = std::nextafter(kPandaPositionUpperLimits[6], 4.0);
        expected = JointTargetValidationResult::PositionLimitExceeded;
        break;
      case 20:
        message.points.front().positions.push_back(0.0);
        expected = JointTargetValidationResult::InvalidPositionCount;
        break;
      default:
        FAIL() << "unhandled scenario";
    }
    EXPECT_EQ(inbox.accept(message, kRosNowNs, 100 + static_cast<int64_t>(scenario)), expected);
    EXPECT_FALSE(inbox.nonRealtimeTarget().valid);
  }
}

TEST_F(DualArmJointImpedanceControllerTest, EnforcesHeaderWindowAndReordersNamedPositions) {
  const auto names = makeJointNames("arm");
  const auto positions = basePose();
  ArmImpedanceTargetInbox inbox;
  inbox.configure(makePolicy(names));

  EXPECT_EQ(inbox.accept(makeMessage(names, positions, kRosNowNs - 1000000000LL), kRosNowNs, 1),
            JointTargetValidationResult::Accepted);
  EXPECT_EQ(inbox.accept(makeMessage(names, positions, kRosNowNs - 1000000001LL), kRosNowNs, 2),
            JointTargetValidationResult::HeaderTooOld);
  EXPECT_EQ(inbox.accept(makeMessage(names, positions, kRosNowNs + 100000000LL), kRosNowNs, 3),
            JointTargetValidationResult::Accepted);
  EXPECT_EQ(inbox.accept(makeMessage(names, positions, kRosNowNs + 100000001LL), kRosNowNs, 4),
            JointTargetValidationResult::HeaderTooFarInFuture);
  EXPECT_EQ(inbox.accept(makeMessage(names, positions, kRosNowNs, true), kRosNowNs, 5),
            JointTargetValidationResult::Accepted);
  EXPECT_EQ(inbox.nonRealtimeTarget().positions, positions);

  auto tighter_policy = makePolicy(names);
  tighter_policy.position_lower[0] = -0.1;
  tighter_policy.position_upper[0] = 0.1;
  inbox.configure(tighter_policy);
  auto outside_configured = positions;
  outside_configured[0] = 0.2;
  EXPECT_EQ(inbox.accept(makeMessage(names, outside_configured), kRosNowNs, 6),
            JointTargetValidationResult::PositionLimitExceeded);
}

TEST_F(DualArmJointImpedanceControllerTest,
       FixedSeedTargetsMatchReferenceValidationAndRejectedInputsFreezeInternalState) {
  constexpr std::array<std::uint64_t, 3> kSeeds{0x494d5045U, 0x71a6e7U, 0x667265657a6573ULL};
  constexpr std::size_t kCasesPerSeed = 4000;
  constexpr double kPeriodSeconds = 0.001;
  constexpr double kMaximumDelta = 0.5 * kPeriodSeconds;

  ControllerParameters parameters;
  parameters.k_gains = {std::vector<double>(kJointCount, 1.0),
                        std::vector<double>(kJointCount, 1.0)};
  parameters.d_gains = {std::vector<double>(kJointCount, 0.0),
                        std::vector<double>(kJointCount, 0.0)};
  parameters.max_target_velocity = {std::vector<double>(kJointCount, 0.5),
                                    std::vector<double>(kJointCount, 0.5)};
  ImpedanceHardwareFixture hardware(parameters.joint_names, parameters.arm_ids);
  hardware.setCoriolis(0, {});
  hardware.setCoriolis(1, {});
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, true);
  ASSERT_TRUE(activate(controller));
  // First-update capture (F-10c): onActivate() only posts a request now -- captureActivationState()
  // and the resulting zero-effort write, and the first publication of a stable active_epoch_ (the
  // gate enable()/accept() below check), all happen on this first update() cycle.
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);
  ASSERT_TRUE(DualArmJointImpedanceControllerTestAccess::enable(
      *controller, 0, true, impedanceSteadyNowNanoseconds() - 1, kRosNowNs - 1));

  for (const auto seed : kSeeds) {
    std::cout << "Dual-arm impedance property seed=" << seed << " cases=" << kCasesPerSeed << '\n';
    std::mt19937_64 engine(seed);
    for (std::size_t case_index = 0; case_index < kCasesPerSeed; ++case_index) {
      JointArray generated{};
      for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        const auto units = static_cast<double>(engine() % 1000001U) / 1000000.0;
        generated[joint] =
            kPandaPositionLowerLimits[joint] +
            units * (kPandaPositionUpperLimits[joint] - kPandaPositionLowerLimits[joint]);
      }
      auto message = makeMessage(parameters.joint_names[0], generated);
      const auto category = case_index % 24U;
      const auto operation = mutateGeneratedJointTarget(message, category, engine);
      SCOPED_TRACE("seed=" + std::to_string(seed) + " case=" + std::to_string(case_index) +
                   " operation=" + operation);

      const auto before = DualArmJointImpedanceControllerTestAccess::target(*controller, 0);
      const bool enabled_before =
          DualArmJointImpedanceControllerTestAccess::enabled(*controller, 0);
      const auto expected = referenceJointTargetValidation(
          message, parameters.joint_names[0], kPandaPositionLowerLimits, kPandaPositionUpperLimits,
          kRosNowNs, 1000000000LL, 100000000LL);
      const auto actual = DualArmJointImpedanceControllerTestAccess::accept(
          *controller, 0, message, kRosNowNs, impedanceSteadyNowNanoseconds());
      ASSERT_EQ(actual, expected.result);

      EXPECT_EQ(DualArmJointImpedanceControllerTestAccess::enabled(*controller, 0), enabled_before);
      expectArrayNear(DualArmJointImpedanceControllerTestAccess::target(*controller, 0), before,
                      0.0);

      ASSERT_EQ(update(*controller, kPeriodSeconds), controller_interface::return_type::OK);
      JointArray expected_target = before;
      if (expected.result == JointTargetValidationResult::Accepted) {
        for (std::size_t joint = 0; joint < kJointCount; ++joint) {
          expected_target[joint] =
              std::clamp(expected.positions[joint], before[joint] - kMaximumDelta,
                         before[joint] + kMaximumDelta);
        }
      }
      expectArrayNear(DualArmJointImpedanceControllerTestAccess::target(*controller, 0),
                      expected_target, 1e-15);
      expectArrayNear(DualArmJointImpedanceControllerTestAccess::target(*controller, 1),
                      basePose(0.05), 0.0);
    }
  }

  ASSERT_TRUE(controller_interface::deactivate_succeeds(controller));
  controller->release_interfaces();
}

TEST_F(DualArmJointImpedanceControllerTest,
       ActivationCapturesMeasuredTargetsDisablesBothAndWritesZero) {
  ControllerParameters parameters;
  ImpedanceHardwareFixture hardware(parameters.joint_names, parameters.arm_ids);
  hardware.fillCommands(99.0);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, true);
  ASSERT_TRUE(activate(controller));
  EXPECT_FALSE(DualArmJointImpedanceControllerTestAccess::enabled(*controller, 0));
  EXPECT_FALSE(DualArmJointImpedanceControllerTestAccess::enabled(*controller, 1));
  // First-update capture (F-10c): onActivate() only posts a request now -- bindInterfaces()/
  // captureActivationState() and the resulting zero-effort write, immediately followed by the
  // real command computed from the just-captured target, all happen inside this first update()
  // call (both writes land within this one call, so only the final, real-valued state is
  // externally observable here -- see the hold controller's identical comment for the same
  // pattern).
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);
  EXPECT_EQ(DualArmJointImpedanceControllerTestAccess::target(*controller, 0), basePose());
  EXPECT_EQ(DualArmJointImpedanceControllerTestAccess::target(*controller, 1), basePose(0.05));
  for (size_t arm = 0; arm < kArmCount; ++arm) {
    for (size_t joint = 0; joint < kJointCount; ++joint) {
      EXPECT_DOUBLE_EQ(hardware.command(arm, joint),
                       0.1 * static_cast<double>(arm + 1) + 0.01 * static_cast<double>(joint));
    }
  }
}

TEST_F(DualArmJointImpedanceControllerTest,
       RequestsQueuedDuringActivationCannotMutateTheNewActiveEpoch) {
  ControllerParameters parameters;
  ImpedanceHardwareFixture hardware(parameters.joint_names, parameters.arm_ids);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, true);

  std::promise<void> zero_entered;
  auto zero_entered_future = zero_entered.get_future();
  std::promise<void> allow_zero;
  const auto allow_zero_future = allow_zero.get_future().share();
  std::atomic<bool> blocked{false};
  hardware.setWriteHook([&](const size_t command_index) {
    if (command_index == 0U && !blocked.exchange(true, std::memory_order_acq_rel)) {
      zero_entered.set_value();
      allow_zero_future.wait();
    }
  });

  // First-update capture (F-10c): onActivate() itself no longer performs any write -- it only
  // posts a request that the owner thread's first update() call services (bindInterfaces() +
  // captureActivationState() + the zero-effort write the barrier below blocks inside). Activation
  // itself is therefore fast and synchronous now; it is this first update() call that plays the
  // role the async activate() call used to.
  ASSERT_TRUE(activate(controller));
  auto first_update = std::async(std::launch::async, [&] { return update(*controller); });
  if (zero_entered_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
    allow_zero.set_value();
    EXPECT_EQ(first_update.wait_for(std::chrono::seconds(2)), std::future_status::ready);
    FAIL() << "the first post-activation update() did not reach the deterministic zero-write "
              "barrier";
  }

  const uint64_t callback_entries =
      DualArmJointImpedanceControllerTestAccess::callbackEntries(*controller);
  const uint64_t enable_generation =
      DualArmJointImpedanceControllerTestAccess::enableGeneration(*controller, 0);
  const int64_t steady_now = impedanceSteadyNowNanoseconds();
  const auto message = makeMessage(parameters.joint_names[0], basePose(), kRosNowNs + 1);
  auto enable_request = std::async(std::launch::async, [&] {
    return DualArmJointImpedanceControllerTestAccess::enable(*controller, 0, true, steady_now,
                                                             kRosNowNs);
  });
  auto target_request = std::async(std::launch::async, [&] {
    return DualArmJointImpedanceControllerTestAccess::accept(*controller, 0, message, kRosNowNs + 1,
                                                             steady_now + 1);
  });
  const bool both_callbacks_queued = waitForCallbackEntries(*controller, callback_entries + 2U);
  // F-10c: acceptTarget()/setArmEnabled() take non_rt_mutex_, not any lock update() ever holds --
  // update()'s own blocked zero write (the barrier above) no longer serializes them the way the
  // old design's onActivate(), which held non_rt_mutex_ for its own zero write, incidentally did.
  // They are therefore not expected to block here; both_callbacks_queued (via
  // non_rt_callback_entries_, incremented before either takes that mutex) is what proves they ran
  // concurrently with the still-in-flight first update() -- the epoch-gated result each returns,
  // checked below, is the actual property this test verifies.
  allow_zero.set_value();

  ASSERT_TRUE(both_callbacks_queued);
  ASSERT_EQ(first_update.get(), controller_interface::return_type::OK);
  EXPECT_FALSE(enable_request.get());
  EXPECT_EQ(target_request.get(), JointTargetValidationResult::ControllerInactive);
  EXPECT_FALSE(DualArmJointImpedanceControllerTestAccess::enabled(*controller, 0));
  EXPECT_EQ(DualArmJointImpedanceControllerTestAccess::enableGeneration(*controller, 0),
            enable_generation);
  EXPECT_FALSE(DualArmJointImpedanceControllerTestAccess::bufferedTargetValid(*controller, 0));
  EXPECT_EQ(DualArmJointImpedanceControllerTestAccess::target(*controller, 0), basePose());
}

TEST_F(DualArmJointImpedanceControllerTest,
       RequestsRacingDeactivationCannotMutateTheInactiveEpochAndAreNeverBlockedByIt) {
  // Two properties, both changed by F-10c amendment A and both asserted here.
  //
  // 1. onDeactivate() closes the active_epoch_ gate before it returns, so a topic-callback-thread
  //    enable()/accept() that arrives while the owner thread is still mid-cycle is rejected as
  //    ControllerInactive and mutates nothing.
  // 2. onDeactivate() no longer holds non_rt_mutex_ across a bounded wait -- there is no bounded
  //    wait left to hold it across. Before the amendment this callback could sit on that mutex
  //    for the whole ~100-220 ms handshake ceiling, stalling every topic callback with it; the
  //    latency assertions below fail if that is ever reintroduced.
  ControllerParameters parameters;
  ImpedanceHardwareFixture hardware(parameters.joint_names, parameters.arm_ids);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, false);
  ASSERT_TRUE(activate(controller));
  // First-update capture (F-10c): binds and publishes the first stable active_epoch_.
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);

  // Park the owner thread inside update()'s post-deactivation zero write, so the requests below
  // really do race a control cycle that has not finished yet.
  std::promise<void> zero_entered;
  auto zero_entered_future = zero_entered.get_future();
  std::promise<void> allow_zero;
  const auto allow_zero_future = allow_zero.get_future().share();
  std::atomic<bool> blocked{false};
  hardware.setWriteHook([&](const size_t command_index) {
    if (command_index == 0U && !blocked.exchange(true, std::memory_order_acq_rel)) {
      zero_entered.set_value();
      allow_zero_future.wait();
    }
  });

  const auto deactivation_started = std::chrono::steady_clock::now();
  ASSERT_TRUE(controller_interface::deactivate_succeeds(controller));
  const auto deactivation_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                   std::chrono::steady_clock::now() - deactivation_started)
                                   .count();
  EXPECT_LT(deactivation_ms, 20) << "onDeactivate() blocked for " << deactivation_ms << " ms";

  auto owner_update = std::async(std::launch::async, [&] { return update(*controller); });
  if (zero_entered_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
    allow_zero.set_value();
    EXPECT_EQ(owner_update.wait_for(std::chrono::seconds(2)), std::future_status::ready);
    FAIL() << "the owner thread never reached the post-deactivation zero-write barrier";
  }

  const uint64_t callback_entries =
      DualArmJointImpedanceControllerTestAccess::callbackEntries(*controller);
  const uint64_t enable_generation =
      DualArmJointImpedanceControllerTestAccess::enableGeneration(*controller, 0);
  const int64_t steady_now = impedanceSteadyNowNanoseconds();
  const auto message = makeMessage(parameters.joint_names[0], basePose(), kRosNowNs + 1);
  const auto requests_started = std::chrono::steady_clock::now();
  auto enable_request = std::async(std::launch::async, [&] {
    return DualArmJointImpedanceControllerTestAccess::enable(*controller, 0, true, steady_now,
                                                             kRosNowNs);
  });
  auto target_request = std::async(std::launch::async, [&] {
    return DualArmJointImpedanceControllerTestAccess::accept(*controller, 0, message, kRosNowNs + 1,
                                                             steady_now + 1);
  });
  ASSERT_TRUE(waitForCallbackEntries(*controller, callback_entries + 2U));
  const bool enable_finished =
      enable_request.wait_for(std::chrono::seconds(2)) == std::future_status::ready;
  const bool target_finished =
      target_request.wait_for(std::chrono::seconds(2)) == std::future_status::ready;
  const auto requests_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::steady_clock::now() - requests_started)
                               .count();
  allow_zero.set_value();
  ASSERT_EQ(owner_update.get(), controller_interface::return_type::ERROR);

  // Property 2: nothing on the lifecycle side is holding non_rt_mutex_ any more.
  EXPECT_TRUE(enable_finished);
  EXPECT_TRUE(target_finished);
  EXPECT_LT(requests_ms, 200) << "topic callbacks were blocked for " << requests_ms << " ms";

  // Property 1: both were rejected by the closed epoch gate and mutated nothing.
  EXPECT_FALSE(enable_request.get());
  EXPECT_EQ(target_request.get(), JointTargetValidationResult::ControllerInactive);
  EXPECT_FALSE(DualArmJointImpedanceControllerTestAccess::enabled(*controller, 0));
  EXPECT_EQ(DualArmJointImpedanceControllerTestAccess::enableGeneration(*controller, 0),
            enable_generation);
  EXPECT_FALSE(DualArmJointImpedanceControllerTestAccess::bufferedTargetValid(*controller, 0));
}

TEST_F(DualArmJointImpedanceControllerTest,
       HeaderAndReceiptMustBothBeStrictlyNewerThanTheEnableEpoch) {
  ControllerParameters parameters;
  ImpedanceHardwareFixture hardware(parameters.joint_names, parameters.arm_ids);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, false);
  ASSERT_TRUE(activate(controller));
  // First-update capture (F-10c): captureActivationState() and the first stable active_epoch_
  // publication (the gate accept()/enable() below check) both happen on this first update() cycle.
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);

  JointArray endpoint = basePose();
  for (double& position : endpoint) {
    position += 0.2;
  }

  int64_t steady_epoch = impedanceSteadyNowNanoseconds() - 3;
  ASSERT_TRUE(DualArmJointImpedanceControllerTestAccess::enable(*controller, 0, true, steady_epoch,
                                                                kRosNowNs));
  EXPECT_EQ(DualArmJointImpedanceControllerTestAccess::accept(
                *controller, 0, makeMessage(parameters.joint_names[0], endpoint, kRosNowNs - 1),
                kRosNowNs, steady_epoch + 1),
            JointTargetValidationResult::Accepted);
  ASSERT_EQ(update(*controller, 0.1), controller_interface::return_type::OK);
  EXPECT_EQ(DualArmJointImpedanceControllerTestAccess::target(*controller, 0), basePose());

  steady_epoch = impedanceSteadyNowNanoseconds() - 2;
  ASSERT_TRUE(DualArmJointImpedanceControllerTestAccess::enable(*controller, 0, true, steady_epoch,
                                                                kRosNowNs));
  EXPECT_EQ(DualArmJointImpedanceControllerTestAccess::accept(
                *controller, 0, makeMessage(parameters.joint_names[0], endpoint, kRosNowNs),
                kRosNowNs, steady_epoch + 1),
            JointTargetValidationResult::Accepted);
  ASSERT_EQ(update(*controller, 0.1), controller_interface::return_type::OK);
  EXPECT_EQ(DualArmJointImpedanceControllerTestAccess::target(*controller, 0), basePose());

  steady_epoch = impedanceSteadyNowNanoseconds() - 2;
  ASSERT_TRUE(DualArmJointImpedanceControllerTestAccess::enable(*controller, 0, true, steady_epoch,
                                                                kRosNowNs));
  EXPECT_EQ(DualArmJointImpedanceControllerTestAccess::accept(
                *controller, 0, makeMessage(parameters.joint_names[0], endpoint, kRosNowNs + 1),
                kRosNowNs + 1, steady_epoch + 1),
            JointTargetValidationResult::Accepted);
  ASSERT_EQ(update(*controller, 0.1), controller_interface::return_type::OK);
  JointArray expected = basePose();
  for (double& position : expected) {
    position += 0.05;
  }
  expectArrayNear(DualArmJointImpedanceControllerTestAccess::target(*controller, 0), expected,
                  1e-12);
}

TEST_F(DualArmJointImpedanceControllerTest,
       FreshTargetsRateLimitInvalidAndStaleInputFreezeLastInternalTarget) {
  ControllerParameters parameters;
  parameters.max_target_velocity = {std::vector<double>(kJointCount, 0.5),
                                    std::vector<double>(kJointCount, 0.5)};
  parameters.k_gains = {std::vector<double>(kJointCount, 1.0),
                        std::vector<double>(kJointCount, 1.0)};
  parameters.d_gains = {std::vector<double>(kJointCount, 0.0),
                        std::vector<double>(kJointCount, 0.0)};
  ImpedanceHardwareFixture hardware(parameters.joint_names, parameters.arm_ids);
  hardware.setCoriolis(0, {});
  hardware.setCoriolis(1, {});
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, false);
  ASSERT_TRUE(activate(controller));
  // First-update capture (F-10c): captureActivationState() and the first stable active_epoch_
  // publication (the gate accept()/enable() below check) both happen on this first update() cycle.
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);

  JointArray endpoint = basePose();
  for (double& position : endpoint) {
    position += 0.3;
  }
  int64_t now = impedanceSteadyNowNanoseconds();
  ASSERT_EQ(
      DualArmJointImpedanceControllerTestAccess::accept(
          *controller, 0, makeMessage(parameters.joint_names[0], endpoint), kRosNowNs, now - 2),
      JointTargetValidationResult::Accepted);
  DualArmJointImpedanceControllerTestAccess::enable(*controller, 0, true, now - 1);
  ASSERT_EQ(update(*controller, 0.1), controller_interface::return_type::OK);
  EXPECT_EQ(DualArmJointImpedanceControllerTestAccess::target(*controller, 0), basePose());

  now = impedanceSteadyNowNanoseconds();
  ASSERT_EQ(DualArmJointImpedanceControllerTestAccess::accept(
                *controller, 0, makeMessage(parameters.joint_names[0], endpoint, kRosNowNs, true),
                kRosNowNs, now - 1),
            JointTargetValidationResult::Accepted);
  ASSERT_EQ(update(*controller, 0.1), controller_interface::return_type::OK);
  JointArray first_limited = basePose();
  for (double& position : first_limited) {
    position += 0.05;
  }
  expectArrayNear(DualArmJointImpedanceControllerTestAccess::target(*controller, 0), first_limited,
                  1e-12);

  auto invalid = makeMessage(parameters.joint_names[0], endpoint);
  invalid.points.front().velocities.push_back(0.0);
  EXPECT_EQ(DualArmJointImpedanceControllerTestAccess::accept(*controller, 0, invalid, kRosNowNs,
                                                              impedanceSteadyNowNanoseconds()),
            JointTargetValidationResult::VelocityCommandNotAllowed);
  ASSERT_EQ(update(*controller, 0.1), controller_interface::return_type::OK);
  expectArrayNear(DualArmJointImpedanceControllerTestAccess::target(*controller, 0), first_limited,
                  1e-12);

  now = impedanceSteadyNowNanoseconds();
  ASSERT_EQ(DualArmJointImpedanceControllerTestAccess::accept(
                *controller, 0, makeMessage(parameters.joint_names[0], endpoint), kRosNowNs,
                now - 20000000000LL),
            JointTargetValidationResult::Accepted);
  ASSERT_EQ(update(*controller, 0.1), controller_interface::return_type::OK);
  expectArrayNear(DualArmJointImpedanceControllerTestAccess::target(*controller, 0), first_limited,
                  1e-12);

  now = impedanceSteadyNowNanoseconds();
  ASSERT_EQ(
      DualArmJointImpedanceControllerTestAccess::accept(
          *controller, 0, makeMessage(parameters.joint_names[0], endpoint), kRosNowNs, now - 1),
      JointTargetValidationResult::Accepted);
  ASSERT_EQ(update(*controller, 0.1), controller_interface::return_type::OK);
  JointArray second_limited = basePose();
  for (double& position : second_limited) {
    position += 0.1;
  }
  expectArrayNear(DualArmJointImpedanceControllerTestAccess::target(*controller, 0), second_limited,
                  1e-12);
  EXPECT_EQ(DualArmJointImpedanceControllerTestAccess::target(*controller, 1), basePose(0.05));
}

TEST_F(DualArmJointImpedanceControllerTest,
       DisableIsObservedInUpdateAndSamplesMeasuredPositionExactlyOnce) {
  ControllerParameters parameters;
  parameters.k_gains = {std::vector<double>(kJointCount, 1.0),
                        std::vector<double>(kJointCount, 1.0)};
  parameters.d_gains = {std::vector<double>(kJointCount, 0.0),
                        std::vector<double>(kJointCount, 0.0)};
  ImpedanceHardwareFixture hardware(parameters.joint_names, parameters.arm_ids);
  hardware.setCoriolis(0, {});
  hardware.setCoriolis(1, {});
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, true);
  ASSERT_TRUE(activate(controller));
  // First-update capture (F-10c): captureActivationState() and the first stable active_epoch_
  // publication (the gate accept()/enable() below check) both happen on this first update() cycle.
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);

  JointArray endpoint = basePose();
  for (double& position : endpoint) {
    position += 0.2;
  }
  int64_t now = impedanceSteadyNowNanoseconds();
  DualArmJointImpedanceControllerTestAccess::enable(*controller, 0, true, now - 2);
  ASSERT_EQ(
      DualArmJointImpedanceControllerTestAccess::accept(
          *controller, 0, makeMessage(parameters.joint_names[0], endpoint), kRosNowNs, now - 1),
      JointTargetValidationResult::Accepted);
  ASSERT_EQ(update(*controller, 0.1), controller_interface::return_type::OK);

  JointArray disabled_pose = basePose();
  disabled_pose[0] = 0.02;
  disabled_pose[3] = -0.98;
  hardware.setJointState(0, disabled_pose, {});
  DualArmJointImpedanceControllerTestAccess::enable(*controller, 0, false,
                                                    impedanceSteadyNowNanoseconds());
  ASSERT_EQ(update(*controller, 0.1), controller_interface::return_type::OK);
  EXPECT_EQ(DualArmJointImpedanceControllerTestAccess::target(*controller, 0), disabled_pose);

  JointArray later_pose = disabled_pose;
  later_pose[0] = 0.04;
  later_pose[3] = -0.96;
  hardware.setJointState(0, later_pose, {});
  ASSERT_EQ(update(*controller, 0.1), controller_interface::return_type::OK);
  EXPECT_EQ(DualArmJointImpedanceControllerTestAccess::target(*controller, 0), disabled_pose);
  EXPECT_NE(hardware.command(0, 0), 0.0);
}

TEST_F(DualArmJointImpedanceControllerTest,
       DisableAfterFinalGenerationReadIsConsumedOnTheNextUpdate) {
  ControllerParameters parameters;
  parameters.k_gains = {std::vector<double>(kJointCount, 1.0),
                        std::vector<double>(kJointCount, 1.0)};
  parameters.d_gains = {std::vector<double>(kJointCount, 0.0),
                        std::vector<double>(kJointCount, 0.0)};
  ImpedanceHardwareFixture hardware(parameters.joint_names, parameters.arm_ids);
  hardware.setCoriolis(0, {});
  hardware.setCoriolis(1, {});
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, false);
  ASSERT_TRUE(activate(controller));
  // First-update capture (F-10c): captureActivationState() and the first stable active_epoch_
  // publication (the gate accept()/enable() below check) both happen on this first update() cycle.
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);

  JointArray endpoint = basePose();
  for (double& position : endpoint) {
    position += 0.2;
  }
  const int64_t now = impedanceSteadyNowNanoseconds();
  ASSERT_TRUE(DualArmJointImpedanceControllerTestAccess::enable(*controller, 0, true, now - 2));
  ASSERT_EQ(
      DualArmJointImpedanceControllerTestAccess::accept(
          *controller, 0, makeMessage(parameters.joint_names[0], endpoint), kRosNowNs, now - 1),
      JointTargetValidationResult::Accepted);
  ASSERT_EQ(update(*controller, 0.1), controller_interface::return_type::OK);

  JointArray disabled_pose = basePose();
  disabled_pose[0] += 0.02;
  disabled_pose[3] += 0.02;
  hardware.setJointState(0, disabled_pose, {});

  std::promise<void> command_write_entered;
  auto command_write_entered_future = command_write_entered.get_future();
  std::promise<void> allow_command_write;
  const auto allow_command_write_future = allow_command_write.get_future().share();
  std::atomic<bool> blocked{false};
  hardware.setWriteHook([&](const size_t command_index) {
    if (command_index == 0U && !blocked.exchange(true, std::memory_order_acq_rel)) {
      command_write_entered.set_value();
      allow_command_write_future.wait();
    }
  });

  auto in_flight_update = std::async(std::launch::async, [&] { return update(*controller, 0.1); });
  if (command_write_entered_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
    allow_command_write.set_value();
    EXPECT_EQ(in_flight_update.wait_for(std::chrono::seconds(2)), std::future_status::ready);
    FAIL() << "update did not reach the post-generation-read command-write barrier";
  }
  const bool disable_accepted = DualArmJointImpedanceControllerTestAccess::enable(
      *controller, 0, false, impedanceSteadyNowNanoseconds());
  allow_command_write.set_value();
  ASSERT_TRUE(disable_accepted);
  ASSERT_EQ(in_flight_update.get(), controller_interface::return_type::OK);
  hardware.clearWriteHook();

  const JointArray before_disable_is_consumed =
      DualArmJointImpedanceControllerTestAccess::target(*controller, 0);
  EXPECT_NE(before_disable_is_consumed, disabled_pose);
  ASSERT_EQ(update(*controller, 0.1), controller_interface::return_type::OK);
  EXPECT_EQ(DualArmJointImpedanceControllerTestAccess::target(*controller, 0), disabled_pose);

  JointArray later_pose = disabled_pose;
  later_pose[0] += 0.02;
  hardware.setJointState(0, later_pose, {});
  ASSERT_EQ(update(*controller, 0.1), controller_interface::return_type::OK);
  EXPECT_EQ(DualArmJointImpedanceControllerTestAccess::target(*controller, 0), disabled_pose);
}

TEST_F(DualArmJointImpedanceControllerTest, EnableGenerationWrapRetainsOddEvenSnapshotProtocol) {
  ControllerParameters parameters;
  ImpedanceHardwareFixture hardware(parameters.joint_names, parameters.arm_ids);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, true);
  ASSERT_TRUE(activate(controller));
  // First-update capture (F-10c): captureActivationState() -- which snapshots
  // arm.observed_enable_generation, the value forceGenerationNearWrap() below deliberately
  // leaves stale -- and the first stable active_epoch_ publication both happen on this first
  // update() cycle, exactly as captureActivationState() did synchronously inside onActivate()
  // before this fix.
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);

  DualArmJointImpedanceControllerTestAccess::forceGenerationNearWrap(*controller, 0);
  const int64_t now = impedanceSteadyNowNanoseconds();
  ASSERT_TRUE(DualArmJointImpedanceControllerTestAccess::enable(*controller, 0, true, now - 2,
                                                                kRosNowNs - 1));
  EXPECT_EQ(DualArmJointImpedanceControllerTestAccess::enableGeneration(*controller, 0), 0U);
  JointArray endpoint = basePose();
  for (double& position : endpoint) {
    position += 0.2;
  }
  ASSERT_EQ(
      DualArmJointImpedanceControllerTestAccess::accept(
          *controller, 0, makeMessage(parameters.joint_names[0], endpoint), kRosNowNs, now - 1),
      JointTargetValidationResult::Accepted);
  ASSERT_EQ(update(*controller, 0.1), controller_interface::return_type::OK);
  JointArray expected = basePose();
  for (double& position : expected) {
    position += 0.05;
  }
  expectArrayNear(DualArmJointImpedanceControllerTestAccess::target(*controller, 0), expected,
                  1e-12);
}

TEST_F(DualArmJointImpedanceControllerTest,
       RapidDisableEnableResetsToMeasuredBeforeApplyingFreshEndpoint) {
  ControllerParameters parameters;
  parameters.k_gains = {std::vector<double>(kJointCount, 1.0),
                        std::vector<double>(kJointCount, 1.0)};
  parameters.d_gains = {std::vector<double>(kJointCount, 0.0),
                        std::vector<double>(kJointCount, 0.0)};
  ImpedanceHardwareFixture hardware(parameters.joint_names, parameters.arm_ids);
  hardware.setCoriolis(0, {});
  hardware.setCoriolis(1, {});
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, false);
  ASSERT_TRUE(activate(controller));
  // First-update capture (F-10c): captureActivationState() and the first stable active_epoch_
  // publication (the gate accept()/enable() below check) both happen on this first update() cycle.
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);

  JointArray endpoint = basePose();
  for (double& position : endpoint) {
    position += 0.2;
  }
  int64_t now = impedanceSteadyNowNanoseconds();
  DualArmJointImpedanceControllerTestAccess::enable(*controller, 0, true, now - 1);
  ASSERT_EQ(DualArmJointImpedanceControllerTestAccess::accept(
                *controller, 0, makeMessage(parameters.joint_names[0], endpoint), kRosNowNs, now),
            JointTargetValidationResult::Accepted);
  ASSERT_EQ(update(*controller, 0.1), controller_interface::return_type::OK);

  JointArray measured = basePose();
  for (double& position : measured) {
    position -= 0.1;
  }
  hardware.setJointState(0, measured, {});
  now = impedanceSteadyNowNanoseconds();
  DualArmJointImpedanceControllerTestAccess::enable(*controller, 0, false, now - 2);
  DualArmJointImpedanceControllerTestAccess::enable(*controller, 0, true, now - 1);
  ASSERT_EQ(DualArmJointImpedanceControllerTestAccess::accept(
                *controller, 0, makeMessage(parameters.joint_names[0], endpoint), kRosNowNs, now),
            JointTargetValidationResult::Accepted);
  ASSERT_EQ(update(*controller, 0.1), controller_interface::return_type::OK);

  JointArray expected = measured;
  for (double& position : expected) {
    position += 0.05;
  }
  expectArrayNear(DualArmJointImpedanceControllerTestAccess::target(*controller, 0), expected,
                  1e-12);
}

TEST_F(DualArmJointImpedanceControllerTest, BothArmsAcceptIndependentTargetsInOneUpdate) {
  ControllerParameters parameters;
  parameters.max_target_velocity = {std::vector<double>(kJointCount, 1.0),
                                    std::vector<double>(kJointCount, 0.5)};
  parameters.k_gains = {std::vector<double>(kJointCount, 1.0),
                        std::vector<double>(kJointCount, 1.0)};
  parameters.d_gains = {std::vector<double>(kJointCount, 0.0),
                        std::vector<double>(kJointCount, 0.0)};
  ImpedanceHardwareFixture hardware(parameters.joint_names, parameters.arm_ids);
  hardware.setCoriolis(0, {});
  hardware.setCoriolis(1, {});
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, true);
  ASSERT_TRUE(activate(controller));
  // First-update capture (F-10c): captureActivationState() and the first stable active_epoch_
  // publication (the gate accept()/enable() below check) both happen on this first update() cycle.
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);

  JointArray first_endpoint = basePose();
  JointArray second_endpoint = basePose(0.05);
  for (double& position : first_endpoint) {
    position += 0.2;
  }
  for (double& position : second_endpoint) {
    position -= 0.2;
  }
  const int64_t now = impedanceSteadyNowNanoseconds();
  DualArmJointImpedanceControllerTestAccess::enable(*controller, 0, true, now - 3);
  DualArmJointImpedanceControllerTestAccess::enable(*controller, 1, true, now - 3);
  ASSERT_EQ(
      DualArmJointImpedanceControllerTestAccess::accept(
          *controller, 0, makeMessage(parameters.joint_names[0], first_endpoint, kRosNowNs, true),
          kRosNowNs, now - 2),
      JointTargetValidationResult::Accepted);
  ASSERT_EQ(DualArmJointImpedanceControllerTestAccess::accept(
                *controller, 1, makeMessage(parameters.joint_names[1], second_endpoint), kRosNowNs,
                now - 1),
            JointTargetValidationResult::Accepted);
  ASSERT_EQ(update(*controller, 0.1), controller_interface::return_type::OK);

  JointArray first_expected = basePose();
  JointArray second_expected = basePose(0.05);
  for (double& position : first_expected) {
    position += 0.1;
  }
  for (double& position : second_expected) {
    position -= 0.05;
  }
  expectArrayNear(DualArmJointImpedanceControllerTestAccess::target(*controller, 0), first_expected,
                  1e-12);
  expectArrayNear(DualArmJointImpedanceControllerTestAccess::target(*controller, 1),
                  second_expected, 1e-12);
}

TEST_F(DualArmJointImpedanceControllerTest, FiltersVelocityAndCommandsArmsIndependently) {
  ControllerParameters parameters;
  parameters.k_gains = {std::vector<double>(kJointCount, 0.0),
                        std::vector<double>(kJointCount, 0.0)};
  parameters.d_gains = {std::vector<double>(kJointCount, 2.0),
                        std::vector<double>(kJointCount, 3.0)};
  ImpedanceHardwareFixture hardware(parameters.joint_names, parameters.arm_ids);
  JointArray first_velocity{{0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7}};
  JointArray second_velocity{{-0.7, 0.6, -0.5, 0.4, -0.3, 0.2, -0.1}};
  hardware.setJointState(0, basePose(), first_velocity);
  hardware.setJointState(1, basePose(0.05), second_velocity);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, true);
  ASSERT_TRUE(activate(controller));
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);
  for (size_t joint = 0; joint < kJointCount; ++joint) {
    EXPECT_DOUBLE_EQ(hardware.command(0, joint),
                     0.1 + 0.01 * static_cast<double>(joint) - 2.0 * first_velocity[joint]);
    EXPECT_DOUBLE_EQ(hardware.command(1, joint),
                     0.2 + 0.01 * static_cast<double>(joint) - 3.0 * second_velocity[joint]);
  }

  hardware.setVelocity(0, 0, 0.2);
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);
  const double filtered = 0.01 * first_velocity[0] + 0.99 * 0.2;
  EXPECT_NEAR(hardware.command(0, 0), 0.1 - 2.0 * filtered, 1e-12);
}

TEST_F(DualArmJointImpedanceControllerTest, RejectsMissingDuplicateAndAliasedBindings) {
  for (size_t scenario = 0; scenario < 5; ++scenario) {
    SCOPED_TRACE(scenario);
    ControllerParameters parameters;
    ImpedanceHardwareFixture hardware(parameters.joint_names, parameters.arm_ids);
    if (scenario == 0) {
      hardware.replaceStateInterface("arm_joint1/position", "arm_extra_joint1", "effort");
    } else if (scenario == 1) {
      hardware.duplicateStateInterface("arm/robot_model", "arm/robot_state");
    } else if (scenario == 2) {
      hardware.replaceCommandInterface("arm_extra_joint7/effort", "arm_joint7");
    } else if (scenario == 3) {
      hardware.setNullStatePointer(0);
    } else {
      hardware.aliasModelPointer(1, 0);
    }
    auto controller = makeController(parameters);
    ASSERT_TRUE(configure(controller));
    hardware.assignTo(*controller, true);
    // F-10c amendment A.4: the name-level wiring faults (scenarios 0-2) are rejected by
    // onActivate()'s restored read-only validation, externally visible to controller_manager.
    // The pointer-level faults (scenarios 3-4) can only be found by decoding the robot_state/
    // robot_model pointers, which is a *read of interface values* the owner thread writes, so it
    // stays on the owner thread's first update() cycle where design 3.2 put it.
    const bool name_level_fault = scenario < 3;
    EXPECT_EQ(activate(controller), !name_level_fault);
    EXPECT_EQ(update(*controller), controller_interface::return_type::ERROR);
  }
}

TEST_F(DualArmJointImpedanceControllerTest,
       NonfiniteStateModelChangedPointerAndOverLimitEffortFailToAllZero) {
  for (size_t scenario = 0; scenario < 7; ++scenario) {
    SCOPED_TRACE(scenario);
    ControllerParameters parameters;
    if (scenario == 5) {
      parameters.max_effort[0] = std::vector<double>(kJointCount, 0.05);
    } else if (scenario == 6) {
      parameters.k_gains[0][0] = std::numeric_limits<double>::max();
    }
    ImpedanceHardwareFixture hardware(parameters.joint_names, parameters.arm_ids);
    auto controller = makeController(parameters);
    ASSERT_TRUE(configure(controller));
    hardware.assignTo(*controller, true);
    ASSERT_TRUE(activate(controller));
    // First-update capture (F-10c): capture the target from the *pre-corruption* state --
    // scenario 4 in particular depends on the bind (which now also runs on this first update()
    // cycle) having already succeeded before its model pointer is corrupted, exactly like the old
    // synchronous on_activate() used to guarantee. Not asserted OK: scenarios 5/6 bake their fault
    // into the controller's own parameters (an extreme k_gain / a near-zero max_effort), so this
    // very first cycle already reproduces their failure -- the second, post-corruption update()
    // below is what every scenario is actually asserted against.
    (void)update(*controller);
    // Scenario 5's fault (an unattainably tight max_effort) is already violated by this first
    // cycle's coriolis-only steady-state effort -- unlike every other scenario, that first cycle
    // never reaches a *successful* real-command write, so zero_required_ is never re-armed after
    // the capture-time zero already satisfied it (this is the pre-existing, F-10c-unrelated
    // attemptRequiredZero() early-guard: a cycle that never owed a fresh write does not force
    // one). Injecting stale garbage via fillCommands() here would therefore not be re-zeroed by
    // the second update() below -- that is expected, not a regression, so scenario 5 skips it.
    if (scenario != 5) {
      hardware.fillCommands(77.0);
    }
    hardware.resetWriteCounts();

    if (scenario == 0) {
      hardware.setPosition(0, 2, std::numeric_limits<double>::quiet_NaN());
    } else if (scenario == 1) {
      hardware.robotState(1).m_total = std::numeric_limits<double>::infinity();
    } else if (scenario == 2) {
      JointArray coriolis{};
      coriolis[5] = std::numeric_limits<double>::quiet_NaN();
      hardware.setCoriolis(0, coriolis);
    } else if (scenario == 3) {
      hardware.setModelThrow(1, true);
    } else if (scenario == 4) {
      hardware.setNullModelPointer(0);
    } else if (scenario == 6) {
      hardware.setPosition(0, 0, std::numeric_limits<double>::max());
    }

    EXPECT_EQ(update(*controller), controller_interface::return_type::ERROR);
    // Scenario 5: see the comment above -- this second cycle does not owe a fresh write (the
    // first cycle's capture-time zero already satisfied zero_required_, and no real command was
    // ever successfully written since), so no write happens here; the command storage still
    // holds the first cycle's zero, untouched.
    if (scenario != 5) {
      EXPECT_TRUE(hardware.everyInterfaceWritten(1));
    }
    EXPECT_TRUE(hardware.allCommandsEqual(0.0));
  }
}

TEST_F(DualArmJointImpedanceControllerTest, AggregatesNormalWritesThenAttemptsAllZerosOnFailure) {
  ControllerParameters parameters;
  ImpedanceHardwareFixture hardware(parameters.joint_names, parameters.arm_ids);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, true);
  ASSERT_TRUE(activate(controller));
  hardware.fillCommands(77.0);
  hardware.resetWriteCounts();

  auto failing_handle = hardware.commandHandle("arm_joint4/effort");
  std::unique_lock<std::shared_mutex> lock(failing_handle->get_mutex());
  EXPECT_EQ(update(*controller), controller_interface::return_type::ERROR);
  EXPECT_TRUE(hardware.everyInterfaceWritten(2));
  lock.unlock();
  for (size_t arm = 0; arm < kArmCount; ++arm) {
    for (size_t joint = 0; joint < kJointCount; ++joint) {
      if (arm == 0 && joint == 3) {
        EXPECT_DOUBLE_EQ(hardware.command(arm, joint), 77.0);
      } else {
        EXPECT_DOUBLE_EQ(hardware.command(arm, joint), 0.0);
      }
    }
  }
}

TEST_F(DualArmJointImpedanceControllerTest,
       ActivationFailureReleaseRequiresCleanupReconfigureAndFreshAssignment) {
  ControllerParameters parameters;
  ImpedanceHardwareFixture hardware(parameters.joint_names, parameters.arm_ids);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, true);
  hardware.fillCommands(4.0);
  hardware.resetWriteCounts();

  auto failing_handle = hardware.commandHandle("arm_joint4/effort");
  std::unique_lock<std::shared_mutex> lock(failing_handle->get_mutex());
  // First-update capture (F-10c): onActivate()'s restored validation is read-only (amendment
  // A.4), so a *locked* handle -- which only fails writes -- does not affect it and activation
  // still succeeds; the locked joint4 write failure surfaces once the first update() cycle
  // attempts the post-bind zero effort (twice: once from serviceFirstUpdateActivation()'s own
  // attemptRequiredZero(), once more from update()'s interfaces_bound_-but-inactive
  // fallthrough).
  ASSERT_TRUE(activate(controller));
  EXPECT_EQ(update(*controller), controller_interface::return_type::ERROR);
  EXPECT_TRUE(hardware.everyInterfaceWritten(2));
  // F-10c amendment A: release_interfaces() writes nothing -- no third attempt.
  controller->release_interfaces();
  EXPECT_TRUE(hardware.everyInterfaceWritten(2));
  lock.unlock();
  EXPECT_EQ(update(*controller), controller_interface::return_type::ERROR);
  // The lifecycle node is still ACTIVE at this point (onActivate() never detected the failure
  // synchronously) -- return it to inactive, exactly as controller_manager would once it noticed
  // update() returning ERROR, before re-attempting activation.
  ASSERT_TRUE(controller_interface::deactivate_succeeds(controller));
  EXPECT_FALSE(activate(controller));

  ASSERT_TRUE(controller_interface::cleanup_succeeds(controller));
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, false);
  ASSERT_TRUE(activate(controller));
  // First-update capture (F-10c): this call binds fresh (joint4 is no longer locked), captures
  // the (default) pose as the target, and -- since position error is zero -- writes the
  // coriolis-only steady-state command, exactly like
  // ActivationCapturesMeasuredTargetsDisablesBothAndWritesZero above.
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);
  for (size_t arm = 0; arm < kArmCount; ++arm) {
    for (size_t joint = 0; joint < kJointCount; ++joint) {
      EXPECT_DOUBLE_EQ(hardware.command(arm, joint),
                       0.1 * static_cast<double>(arm + 1) + 0.01 * static_cast<double>(joint));
    }
  }
  ASSERT_TRUE(controller_interface::deactivate_succeeds(controller));
  controller->release_interfaces();
}

TEST_F(DualArmJointImpedanceControllerTest, ReleaseWritesNothingAndClearsEveryRawBinding) {
  ControllerParameters parameters;
  ImpedanceHardwareFixture hardware(parameters.joint_names, parameters.arm_ids);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, true);
  ASSERT_TRUE(activate(controller));
  // First-update bind (F-10c): establish a fully bound controller (rt_ever_bound_ == true)
  // before deactivating, otherwise there is nothing for either callback below to zero.
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);
  ASSERT_TRUE(controller_interface::deactivate_succeeds(controller));
  hardware.fillCommands(7.0);
  hardware.resetWriteCounts();

  controller->release_interfaces();

  // F-10c amendment A: no lifecycle-thread command write. The bindings are still cleared before
  // the base class frees the loans, which is what this test is really about.
  EXPECT_TRUE(hardware.everyInterfaceWritten(0));
  EXPECT_TRUE(hardware.allCommandsEqual(7.0));
  EXPECT_TRUE(DualArmJointImpedanceControllerTestAccess::bindingsCleared(*controller));
  EXPECT_EQ(update(*controller), controller_interface::return_type::ERROR);
}

TEST_F(DualArmJointImpedanceControllerTest,
       ReleaseWithALockedHandleStillClearsBindingsAndRequiresFreshAssignment) {
  ControllerParameters parameters;
  ImpedanceHardwareFixture hardware(parameters.joint_names, parameters.arm_ids);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, false);
  ASSERT_TRUE(activate(controller));
  // First-update bind (F-10c): establish a fully bound controller (rt_ever_bound_ == true)
  // before deactivating, otherwise there is nothing for either callback below to zero.
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);
  ASSERT_TRUE(controller_interface::deactivate_succeeds(controller));
  hardware.fillCommands(8.0);
  hardware.resetWriteCounts();

  auto failing_handle = hardware.commandHandle("arm_joint4/effort");
  std::unique_lock<std::shared_mutex> lock(failing_handle->get_mutex());
  controller->release_interfaces();
  // F-10c amendment A: release writes nothing, so a locked handle changes nothing about it.
  EXPECT_TRUE(hardware.everyInterfaceWritten(0));
  EXPECT_TRUE(DualArmJointImpedanceControllerTestAccess::bindingsCleared(*controller));
  lock.unlock();
  // The loans are gone, so the restored activation-time wiring validation rejects this attempt.
  EXPECT_FALSE(activate(controller));

  ASSERT_TRUE(controller_interface::cleanup_succeeds(controller));
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, true);
  ASSERT_TRUE(activate(controller));
  // First-update capture (F-10c): this call binds fresh (joint4 is no longer locked), captures
  // the (default) pose as the target, and -- since position error is zero -- writes the
  // coriolis-only steady-state command, exactly like
  // ActivationCapturesMeasuredTargetsDisablesBothAndWritesZero above.
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);
  for (size_t arm = 0; arm < kArmCount; ++arm) {
    for (size_t joint = 0; joint < kJointCount; ++joint) {
      EXPECT_DOUBLE_EQ(hardware.command(arm, joint),
                       0.1 * static_cast<double>(arm + 1) + 0.01 * static_cast<double>(joint));
    }
  }
}

TEST_F(DualArmJointImpedanceControllerTest, DeactivationCascadeWritesNothingAndUsesNoReleasedLoan) {
  ControllerParameters parameters;
  ImpedanceHardwareFixture hardware(parameters.joint_names, parameters.arm_ids);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, true);
  ASSERT_TRUE(activate(controller));
  // First-update capture (F-10c): establish a fully bound, active controller before exercising
  // the failing-deactivation cascade below.
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);
  hardware.fillCommands(5.0);
  hardware.resetWriteCounts();

  auto failing_handle = hardware.commandHandle("arm_joint4/effort");
  std::unique_lock<std::shared_mutex> lock(failing_handle->get_mutex());
  const auto state = controller->get_node()->deactivate();
  // F-10c amendment A: onDeactivate() writes nothing, so it can no longer fail on a locked
  // handle; the node simply reaches INACTIVE instead of cascading into on_error and FINALIZED.
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_TRUE(hardware.everyInterfaceWritten(0));
  EXPECT_FALSE(DualArmJointImpedanceControllerTestAccess::topic(*controller, 0).empty());
  controller->release_interfaces();
  EXPECT_TRUE(hardware.everyInterfaceWritten(0));
  EXPECT_TRUE(DualArmJointImpedanceControllerTestAccess::bindingsCleared(*controller));
  lock.unlock();
  EXPECT_EQ(update(*controller), controller_interface::return_type::ERROR);
}

TEST_F(DualArmJointImpedanceControllerTest, ShutdownErrorClearsBindingsBeforeLoansDie) {
  ControllerParameters parameters;
  auto hardware =
      std::make_unique<ImpedanceHardwareFixture>(parameters.joint_names, parameters.arm_ids);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware->assignTo(*controller, true);
  ASSERT_TRUE(activate(controller));
  // First-update capture (F-10c): establish a fully bound, active controller before exercising
  // the failing-shutdown cascade below.
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);
  hardware->fillCommands(6.0);
  hardware->resetWriteCounts();

  auto failing_handle = hardware->commandHandle("arm_joint4/effort");
  std::unique_lock<std::shared_mutex> lock(failing_handle->get_mutex());
  const auto state = controller->get_node()->shutdown();
  // F-10c amendment A: onShutdown() writes nothing, so it can no longer fail on a locked handle
  // and the node finalizes cleanly rather than routing through on_error.
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
  EXPECT_TRUE(hardware->everyInterfaceWritten(0));
  EXPECT_TRUE(DualArmJointImpedanceControllerTestAccess::topic(*controller, 0).empty());
  EXPECT_TRUE(DualArmJointImpedanceControllerTestAccess::service(*controller, 1).empty());
  controller->release_interfaces();
  EXPECT_TRUE(hardware->everyInterfaceWritten(0));
  EXPECT_TRUE(DualArmJointImpedanceControllerTestAccess::bindingsCleared(*controller));
  lock.unlock();
  failing_handle.reset();
  hardware.reset();

  EXPECT_EQ(update(*controller), controller_interface::return_type::ERROR);
  controller->release_interfaces();
}

TEST_F(DualArmJointImpedanceControllerTest, CleanupAfterManagerStyleReleaseAllowsFreshConfigure) {
  ControllerParameters parameters;
  ImpedanceHardwareFixture hardware(parameters.joint_names, parameters.arm_ids);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, true);
  ASSERT_TRUE(activate(controller));
  ASSERT_TRUE(controller_interface::deactivate_succeeds(controller));
  controller->release_interfaces();
  ASSERT_TRUE(controller_interface::cleanup_succeeds(controller));
  EXPECT_TRUE(DualArmJointImpedanceControllerTestAccess::topic(*controller, 0).empty());
  EXPECT_TRUE(DualArmJointImpedanceControllerTestAccess::service(*controller, 1).empty());
  ASSERT_TRUE(configure(controller));
  EXPECT_FALSE(DualArmJointImpedanceControllerTestAccess::topic(*controller, 0).empty());
  EXPECT_FALSE(DualArmJointImpedanceControllerTestAccess::service(*controller, 1).empty());
}

TEST_F(DualArmJointImpedanceControllerTest, InvalidPeriodAttemptsAllZerosAndReturnsError) {
  ControllerParameters parameters;
  ImpedanceHardwareFixture hardware(parameters.joint_names, parameters.arm_ids);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, false);
  ASSERT_TRUE(activate(controller));
  hardware.fillCommands(8.0);
  hardware.resetWriteCounts();
  EXPECT_EQ(update(*controller, 0.0), controller_interface::return_type::ERROR);
  EXPECT_TRUE(hardware.everyInterfaceWritten(1));
  EXPECT_TRUE(hardware.allCommandsEqual(0.0));
}

TEST_F(DualArmJointImpedanceControllerTest, PluginHasDistinctLoadableName) {
  pluginlib::ClassLoader<controller_interface::ControllerInterface> loader(
      "controller_interface", "controller_interface::ControllerInterface");
  EXPECT_TRUE(
      loader.isClassAvailable("franka_example_controllers/DualArmJointImpedanceController"));
  auto controller =
      loader.createUniqueInstance("franka_example_controllers/DualArmJointImpedanceController");
  EXPECT_NE(controller, nullptr);
}

}  // namespace
}  // namespace franka_example_controllers
