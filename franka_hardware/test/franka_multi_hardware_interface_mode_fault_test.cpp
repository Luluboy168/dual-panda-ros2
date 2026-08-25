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

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rcutils/logging.h>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "franka_hardware/real/franka_multi_hardware_interface.hpp"
#include "support/synthetic_franka_arm_backend.hpp"

namespace franka_hardware {
namespace {

using test_support::SyntheticFailurePoint;
using test_support::SyntheticFrankaArmBackend;
using test_support::SyntheticFrankaArmBackendConfig;

hardware_interface::InterfaceInfo makeInterface(const std::string& name) {
  hardware_interface::InterfaceInfo interface{};
  interface.name = name;
  interface.data_type = "double";
  return interface;
}

hardware_interface::HardwareInfo makeHardwareInfo(const std::vector<std::string>& arm_names) {
  hardware_interface::HardwareInfo info{};
  info.name = "FrankaMultiHardwareInterface";
  info.type = "system";
  info.hardware_plugin_name = "franka_hardware/FrankaMultiHardwareInterface";
  info.hardware_parameters["robot_count"] = std::to_string(arm_names.size());
  for (size_t arm_index = 0; arm_index < arm_names.size(); ++arm_index) {
    const auto slot = std::to_string(arm_index + 1);
    const auto& arm_name = arm_names.at(arm_index);
    info.hardware_parameters["ns_" + slot] = arm_name;
    info.hardware_parameters["robot_ip_" + slot] = "offline-placeholder-" + slot;
    for (size_t joint_index = 1; joint_index <= FrankaMultiHardwareInterface::kNumberOfJoints;
         ++joint_index) {
      hardware_interface::ComponentInfo joint{};
      joint.name = arm_name + "_joint" + std::to_string(joint_index);
      joint.type = "joint";
      joint.command_interfaces = {makeInterface("effort"), makeInterface("position"),
                                  makeInterface("velocity")};
      joint.state_interfaces = {makeInterface("position"), makeInterface("velocity"),
                                makeInterface("effort")};
      info.joints.push_back(std::move(joint));
    }
  }
  return info;
}

class RclcppScope {
 public:
  RclcppScope() {
    if (!rclcpp::ok()) {
      int argc = 0;
      char** argv = nullptr;
      rclcpp::init(argc, argv);
      owns_context_ = true;
    }
  }

  RclcppScope(const RclcppScope&) = delete;
  RclcppScope& operator=(const RclcppScope&) = delete;

  ~RclcppScope() {
    if (owns_context_ && rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

 private:
  bool owns_context_{false};
};

enum class TraceKind : uint8_t { Publish, Request };

struct TraceEvent {
  TraceKind kind{TraceKind::Publish};
  uint8_t arm_slot{0};
  ControlMode mode{ControlMode::None};
  RobotCommand command{};
  bool result{false};
};

struct BackendControl {
  bool no_command_capacity{false};
  bool fail_all_publishes{false};
  size_t fail_publish_call{0};
  size_t fail_request_call{0};
  size_t throw_read_call{0};
  size_t publish_calls{0};
  size_t request_calls{0};
  size_t read_calls{0};
  bool pause_can_request{false};
  bool can_request_entered{false};
  bool release_can_request{false};
  mutable std::mutex can_request_mutex;
  mutable std::condition_variable can_request_cv;

  bool waitForCanRequest(std::chrono::milliseconds timeout = std::chrono::seconds(1)) {
    std::unique_lock<std::mutex> lock(can_request_mutex);
    return can_request_cv.wait_for(lock, timeout, [this]() { return can_request_entered; });
  }

  void releaseCanRequest() {
    const std::lock_guard<std::mutex> lock(can_request_mutex);
    release_can_request = true;
    can_request_cv.notify_all();
  }
};

class TracedBackend final : public FrankaArmBackend {
 public:
  TracedBackend(uint8_t arm_slot,
                std::shared_ptr<SyntheticFrankaArmBackend> backend,
                std::shared_ptr<BackendControl> control,
                std::vector<TraceEvent>* trace)
      : arm_slot_(arm_slot),
        backend_(std::move(backend)),
        control_(std::move(control)),
        trace_(trace) {}

  bool startStateReading() override { return backend_->startStateReading(); }
  bool stop() override { return backend_->stop(); }
  franka::RobotState readLatestState() override {
    ++control_->read_calls;
    if (control_->throw_read_call != 0 && control_->read_calls == control_->throw_read_call) {
      throw std::runtime_error("injected read exception");
    }
    return backend_->readLatestState();
  }
  ModelBase* model() noexcept override { return backend_->model(); }

  bool canPublishCommand() const noexcept override {
    return !control_->no_command_capacity && backend_->canPublishCommand();
  }
  bool publishCommand(const RobotCommand& command) noexcept override {
    ++control_->publish_calls;
    const bool injected_failure =
        control_->fail_all_publishes || (control_->fail_publish_call != 0 &&
                                         control_->publish_calls == control_->fail_publish_call);
    const bool result = !injected_failure && backend_->publishCommand(command);
    trace_->push_back({TraceKind::Publish, arm_slot_, ControlMode::None, command, result});
    return result;
  }
  bool canRequestControlMode(ControlMode mode) const noexcept override {
    {
      std::unique_lock<std::mutex> lock(control_->can_request_mutex);
      if (control_->pause_can_request) {
        control_->pause_can_request = false;
        control_->can_request_entered = true;
        control_->can_request_cv.notify_all();
        (void)control_->can_request_cv.wait_for(lock, std::chrono::seconds(2),
                                                [this]() { return control_->release_can_request; });
        control_->release_can_request = false;
        control_->can_request_entered = false;
      }
    }
    return backend_->canRequestControlMode(mode);
  }
  bool requestControlMode(ControlMode mode) noexcept override {
    ++control_->request_calls;
    const bool injected_failure =
        control_->fail_request_call != 0 && control_->request_calls == control_->fail_request_call;
    const bool result = !injected_failure && backend_->requestControlMode(mode);
    trace_->push_back({TraceKind::Request, arm_slot_, mode, {}, result});
    return result;
  }
  ControlMode requestedControlMode() const noexcept override {
    return backend_->requestedControlMode();
  }
  ControlMode activeControlMode() const noexcept override { return backend_->activeControlMode(); }

  bool hasFault() const noexcept override { return backend_->hasFault(); }
  bool recoverToReading() override { return backend_->recoverToReading(); }
  FrankaArmBackendDiagnostics diagnostics() const noexcept override {
    return backend_->diagnostics();
  }

  void setJointStiffness(
      const franka_msgs::srv::SetJointStiffness::Request::SharedPtr& request) override {
    backend_->setJointStiffness(request);
  }
  void setCartesianStiffness(
      const franka_msgs::srv::SetCartesianStiffness::Request::SharedPtr& request) override {
    backend_->setCartesianStiffness(request);
  }
  void setLoad(const franka_msgs::srv::SetLoad::Request::SharedPtr& request) override {
    backend_->setLoad(request);
  }
  void setTCPFrame(const franka_msgs::srv::SetTCPFrame::Request::SharedPtr& request) override {
    backend_->setTCPFrame(request);
  }
  void setStiffnessFrame(
      const franka_msgs::srv::SetStiffnessFrame::Request::SharedPtr& request) override {
    backend_->setStiffnessFrame(request);
  }
  void setForceTorqueCollisionBehavior(
      const franka_msgs::srv::SetForceTorqueCollisionBehavior::Request::SharedPtr& request)
      override {
    backend_->setForceTorqueCollisionBehavior(request);
  }
  void setFullCollisionBehavior(
      const franka_msgs::srv::SetFullCollisionBehavior::Request::SharedPtr& request) override {
    backend_->setFullCollisionBehavior(request);
  }

 private:
  uint8_t arm_slot_;
  std::shared_ptr<SyntheticFrankaArmBackend> backend_;
  std::shared_ptr<BackendControl> control_;
  std::vector<TraceEvent>* trace_;
};

struct BackendHarness {
  std::vector<std::string> arm_names;
  std::map<std::string, SyntheticFrankaArmBackendConfig> configurations;
  std::map<std::string, std::shared_ptr<BackendControl>> controls;
  std::map<std::string, std::shared_ptr<SyntheticFrankaArmBackend>> backends;
  std::vector<TraceEvent> trace;

  explicit BackendHarness(std::vector<std::string> names = {"panda1", "panda2"})
      : arm_names(std::move(names)) {
    for (size_t arm_index = 0; arm_index < arm_names.size(); ++arm_index) {
      configurations.emplace(arm_names.at(arm_index), SyntheticFrankaArmBackendConfig::forArm(
                                                          static_cast<uint8_t>(arm_index + 1)));
      controls.emplace(arm_names.at(arm_index), std::make_shared<BackendControl>());
    }
  }

  BackendFactory factory() {
    return [this](const std::string& arm_name, const std::string&,
                  const rclcpp::Logger&) -> std::shared_ptr<FrankaArmBackend> {
      const auto found = std::find(arm_names.begin(), arm_names.end(), arm_name);
      if (found == arm_names.end()) {
        throw std::runtime_error("unknown arm name");
      }
      const auto arm_slot = static_cast<uint8_t>(std::distance(arm_names.begin(), found) + 1);
      auto backend = std::make_shared<SyntheticFrankaArmBackend>(configurations.at(arm_name));
      backends[arm_name] = backend;
      return std::make_shared<TracedBackend>(arm_slot, std::move(backend), controls.at(arm_name),
                                             &trace);
    };
  }

  std::shared_ptr<SyntheticFrankaArmBackend> backend(const std::string& arm_name) const {
    return backends.at(arm_name);
  }

  std::shared_ptr<BackendControl> control(const std::string& arm_name) const {
    return controls.at(arm_name);
  }
};

std::vector<std::string> jointModeInterfaces(const std::string& arm_name,
                                             const std::string& interface_name) {
  std::vector<std::string> interfaces;
  for (size_t joint = 1; joint <= FrankaMultiHardwareInterface::kNumberOfJoints; ++joint) {
    interfaces.push_back(arm_name + "_joint" + std::to_string(joint) + "/" + interface_name);
  }
  return interfaces;
}

std::vector<std::string> concatenate(std::vector<std::string> first,
                                     const std::vector<std::string>& second) {
  first.insert(first.end(), second.begin(), second.end());
  return first;
}

std::vector<std::string> foreignInterfaces(size_t count, const std::string& prefix) {
  std::vector<std::string> interfaces;
  interfaces.reserve(count);
  for (size_t index = 0; index < count; ++index) {
    interfaces.push_back(prefix + std::to_string(index) + "/effort");
  }
  return interfaces;
}

void initializeAndActivate(FrankaMultiHardwareInterface& hardware, BackendHarness& harness) {
  ASSERT_EQ(hardware.on_init(makeHardwareInfo(harness.arm_names)), CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware.write(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            hardware_interface::return_type::OK);
  harness.trace.clear();
}

void expectSafeCommand(const RobotCommand& command, const franka::RobotState& state) {
  for (size_t joint = 0; joint < FrankaMultiHardwareInterface::kNumberOfJoints; ++joint) {
    EXPECT_DOUBLE_EQ(command.efforts.at(joint), 0.0);
    EXPECT_DOUBLE_EQ(command.joint_positions.at(joint), state.q.at(joint));
    EXPECT_DOUBLE_EQ(command.joint_velocities.at(joint), 0.0);
  }
  for (size_t index = 0; index < command.cartesian_positions.size(); ++index) {
    EXPECT_DOUBLE_EQ(command.cartesian_positions.at(index), state.O_T_EE.at(index));
  }
  for (const double velocity : command.cartesian_velocities) {
    EXPECT_DOUBLE_EQ(velocity, 0.0);
  }
}

void expectDiagnostic(const FrankaMultiHardwareInterface& hardware,
                      uint8_t origin_arm_slot,
                      GlobalFaultCause cause,
                      uint8_t unsafe_safe_publish_mask,
                      uint8_t unsafe_none_request_mask) {
  const auto diagnostic = hardware.globalFaultDiagnostic();
  EXPECT_TRUE(diagnostic.latched());
  EXPECT_EQ(diagnostic.origin_arm_slot, origin_arm_slot);
  EXPECT_EQ(diagnostic.cause, cause);
  EXPECT_EQ(diagnostic.unsafe_safe_publish_mask, unsafe_safe_publish_mask);
  EXPECT_EQ(diagnostic.unsafe_none_request_mask, unsafe_none_request_mask);
}

void setCommandInterfaceValue(std::vector<hardware_interface::CommandInterface>& interfaces,
                              const std::string& full_name,
                              double value) {
  const auto found = std::find_if(
      interfaces.begin(), interfaces.end(),
      [&full_name](const auto& interface) { return interface.get_name() == full_name; });
  ASSERT_NE(found, interfaces.end());
  ASSERT_TRUE(found->template set_value<double>(value));
}

class PreparedCheckpointBarrier {
 public:
  ModeSwitchCheckpointHook hook() {
    return [this](ModeSwitchCheckpoint checkpoint) {
      if (checkpoint != ModeSwitchCheckpoint::PreparedPayloadWritten) {
        return;
      }
      std::unique_lock<std::mutex> lock(mutex_);
      entered_ = true;
      condition_.notify_all();
      (void)condition_.wait_for(lock, std::chrono::seconds(2), [this]() { return released_; });
    };
  }

  bool waitUntilEntered(std::chrono::milliseconds timeout = std::chrono::seconds(1)) {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, timeout, [this]() { return entered_; });
  }

  void release() noexcept {
    const std::lock_guard<std::mutex> lock(mutex_);
    released_ = true;
    condition_.notify_all();
  }

 private:
  std::mutex mutex_{};
  std::condition_variable condition_{};
  bool entered_{false};
  bool released_{false};
};

franka::RobotState makeState(uint8_t marker, uint64_t timestamp, double base) {
  auto state = test_support::makeSyntheticRobotState(marker, timestamp);
  for (size_t index = 0; index < state.q.size(); ++index) {
    state.q.at(index) = base + static_cast<double>(index);
  }
  for (size_t index = 0; index < state.O_T_EE.size(); ++index) {
    state.O_T_EE.at(index) = base + 20.0 + static_cast<double>(index);
  }
  return state;
}

class ScopedModeTestLoggerLevel {
 public:
  ScopedModeTestLoggerLevel(const char* logger_name, int temporary_level)
      : logger_name_(logger_name),
        previous_level_(rcutils_logging_get_logger_effective_level(logger_name)) {
    if (rcutils_logging_set_logger_level(logger_name_, temporary_level) != RCUTILS_RET_OK) {
      throw std::runtime_error("failed to set mode property logger level");
    }
  }

  ScopedModeTestLoggerLevel(const ScopedModeTestLoggerLevel&) = delete;
  ScopedModeTestLoggerLevel& operator=(const ScopedModeTestLoggerLevel&) = delete;

  ~ScopedModeTestLoggerLevel() {
    [[maybe_unused]] const auto restored =
        rcutils_logging_set_logger_level(logger_name_, previous_level_);
  }

 private:
  const char* logger_name_;
  int previous_level_;
};

constexpr std::array<ControlMode, 3> kModelModes{ControlMode::None, ControlMode::JointTorque,
                                                 ControlMode::JointVelocity};

struct ModelTransition {
  std::vector<std::string> starts;
  std::vector<std::string> stops;
  std::array<ControlMode, 2> target_modes{};
};

struct PreparedModelTransition {
  ModelTransition transition;
  uint64_t generation{0};
};

struct TwoArmModeModel {
  std::array<ControlMode, 2> modes{ControlMode::None, ControlMode::None};
  std::optional<PreparedModelTransition> prepared;
  uint64_t next_generation{1};
};

const char* modelModeName(ControlMode mode) {
  switch (mode) {
    case ControlMode::None:
      return "none";
    case ControlMode::JointTorque:
      return "joint_torque";
    case ControlMode::JointVelocity:
      return "joint_velocity";
    default:
      return "unsupported";
  }
}

void appendModelModeInterfaces(std::vector<std::string>& interfaces,
                               const std::string& arm_name,
                               ControlMode mode) {
  if (mode == ControlMode::JointTorque) {
    interfaces = concatenate(std::move(interfaces), jointModeInterfaces(arm_name, "effort"));
  } else if (mode == ControlMode::JointVelocity) {
    interfaces = concatenate(std::move(interfaces), jointModeInterfaces(arm_name, "velocity"));
  }
}

ModelTransition makeModelTransition(const std::array<ControlMode, 2>& current_modes,
                                    const std::array<ControlMode, 2>& target_modes,
                                    const bool replace_same_mode) {
  ModelTransition result;
  result.target_modes = target_modes;
  for (std::size_t arm = 0; arm < 2; ++arm) {
    const auto arm_name = "panda" + std::to_string(arm + 1);
    if (current_modes[arm] == target_modes[arm]) {
      if (replace_same_mode && current_modes[arm] != ControlMode::None) {
        appendModelModeInterfaces(result.stops, arm_name, current_modes[arm]);
        appendModelModeInterfaces(result.starts, arm_name, target_modes[arm]);
      }
      continue;
    }
    appendModelModeInterfaces(result.stops, arm_name, current_modes[arm]);
    appendModelModeInterfaces(result.starts, arm_name, target_modes[arm]);
  }
  return result;
}

std::string describeModelOperation(const std::string& kind,
                                   const TwoArmModeModel& model,
                                   const ModelTransition* transition = nullptr) {
  std::ostringstream description;
  description << kind << " current=[" << modelModeName(model.modes[0]) << ','
              << modelModeName(model.modes[1]) << "] prepared_generation="
              << (model.prepared.has_value() ? model.prepared->generation : 0U);
  if (transition != nullptr) {
    description << " target=[" << modelModeName(transition->target_modes[0]) << ','
                << modelModeName(transition->target_modes[1]) << "] start=[";
    for (const auto& name : transition->starts) {
      description << name << ',';
    }
    description << "] stop=[";
    for (const auto& name : transition->stops) {
      description << name << ',';
    }
    description << ']';
  }
  return description.str();
}

void expectModelModes(const BackendHarness& harness, const TwoArmModeModel& model) {
  for (std::size_t arm = 0; arm < 2; ++arm) {
    const auto& name = harness.arm_names[arm];
    EXPECT_EQ(harness.backend(name)->requestedControlMode(), model.modes[arm]);
    EXPECT_EQ(harness.backend(name)->activeControlMode(), model.modes[arm]);
  }
}

TEST(FrankaMultiHardwareInterfaceModeTest,
     EffortVelocityStartStopCombinedAndBothArmTransactionsAreExact) {
  RclcppScope rclcpp_scope;
  BackendHarness harness;
  FrankaMultiHardwareInterface hardware(harness.factory());
  initializeAndActivate(hardware, harness);

  const auto effort1 = jointModeInterfaces("panda1", "effort");
  const auto velocity1 = jointModeInterfaces("panda1", "velocity");
  const auto effort2 = jointModeInterfaces("panda2", "effort");
  const auto velocity2 = jointModeInterfaces("panda2", "velocity");

  ASSERT_EQ(hardware.prepare_command_mode_switch(effort1, {}), hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.perform_command_mode_switch(effort1, {}), hardware_interface::return_type::OK);
  ASSERT_EQ(harness.trace.size(), 2U);
  EXPECT_EQ(harness.trace.at(0).kind, TraceKind::Publish);
  EXPECT_EQ(harness.trace.at(0).arm_slot, 1);
  expectSafeCommand(harness.trace.at(0).command, harness.configurations.at("panda1").initial_state);
  EXPECT_EQ(harness.trace.at(1).kind, TraceKind::Request);
  EXPECT_EQ(harness.trace.at(1).arm_slot, 1);
  EXPECT_EQ(harness.trace.at(1).mode, ControlMode::JointTorque);
  EXPECT_EQ(harness.backend("panda1")->requestedControlMode(), ControlMode::JointTorque);
  EXPECT_EQ(harness.backend("panda2")->requestedControlMode(), ControlMode::None);

  harness.trace.clear();
  ASSERT_EQ(hardware.prepare_command_mode_switch({}, effort1), hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.perform_command_mode_switch({}, effort1), hardware_interface::return_type::OK);
  ASSERT_EQ(harness.trace.size(), 2U);
  EXPECT_EQ(harness.trace.at(0).arm_slot, 1);
  EXPECT_EQ(harness.trace.at(1).mode, ControlMode::None);

  harness.trace.clear();
  ASSERT_EQ(hardware.prepare_command_mode_switch(velocity2, {}),
            hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.perform_command_mode_switch(velocity2, {}),
            hardware_interface::return_type::OK);
  EXPECT_EQ(harness.backend("panda1")->requestedControlMode(), ControlMode::None);
  EXPECT_EQ(harness.backend("panda2")->requestedControlMode(), ControlMode::JointVelocity);

  harness.trace.clear();
  ASSERT_EQ(hardware.prepare_command_mode_switch(effort2, velocity2),
            hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.perform_command_mode_switch(effort2, velocity2),
            hardware_interface::return_type::OK);
  EXPECT_EQ(harness.backend("panda2")->requestedControlMode(), ControlMode::JointTorque);

  const auto both_velocity = concatenate(velocity1, velocity2);
  harness.trace.clear();
  ASSERT_EQ(hardware.prepare_command_mode_switch(both_velocity, effort2),
            hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.perform_command_mode_switch(both_velocity, effort2),
            hardware_interface::return_type::OK);
  ASSERT_EQ(harness.trace.size(), 4U);
  EXPECT_EQ(harness.trace.at(0).kind, TraceKind::Publish);
  EXPECT_EQ(harness.trace.at(0).arm_slot, 1);
  EXPECT_EQ(harness.trace.at(1).kind, TraceKind::Publish);
  EXPECT_EQ(harness.trace.at(1).arm_slot, 2);
  EXPECT_EQ(harness.trace.at(2).kind, TraceKind::Request);
  EXPECT_EQ(harness.trace.at(2).arm_slot, 1);
  EXPECT_EQ(harness.trace.at(2).mode, ControlMode::JointVelocity);
  EXPECT_EQ(harness.trace.at(3).kind, TraceKind::Request);
  EXPECT_EQ(harness.trace.at(3).arm_slot, 2);
  EXPECT_EQ(harness.trace.at(3).mode, ControlMode::JointVelocity);
  EXPECT_FALSE(hardware.globalFaultDiagnostic().latched());

  const auto both_stops = concatenate(velocity1, velocity2);
  ASSERT_EQ(hardware.prepare_command_mode_switch({}, both_stops),
            hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.perform_command_mode_switch({}, both_stops),
            hardware_interface::return_type::OK);
  EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST(FrankaMultiHardwareInterfaceModeTest,
     TransitionSnapshotsUseLastValidStateInSlotOrderAndSkipUnaffectedArms) {
  RclcppScope rclcpp_scope;
  BackendHarness harness({"zeta", "alpha"});
  const auto activation1 = makeState(1, 1001, 100.0);
  const auto activation2 = makeState(2, 2001, 200.0);
  const auto latest1 = makeState(1, 1002, 300.0);
  const auto latest2 = makeState(2, 2002, 400.0);
  harness.configurations.at("zeta").replay_states = {activation1, latest1};
  harness.configurations.at("alpha").replay_states = {activation2, latest2};
  FrankaMultiHardwareInterface hardware(harness.factory());
  initializeAndActivate(hardware, harness);
  ASSERT_EQ(hardware.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            hardware_interface::return_type::OK);

  const auto effort1 = jointModeInterfaces("zeta", "effort");
  const auto effort2 = jointModeInterfaces("alpha", "effort");
  const auto both = concatenate(effort1, effort2);
  harness.trace.clear();
  ASSERT_EQ(hardware.prepare_command_mode_switch(both, {}), hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.perform_command_mode_switch(both, {}), hardware_interface::return_type::OK);
  ASSERT_EQ(harness.trace.size(), 4U);
  EXPECT_EQ(harness.trace.at(0).kind, TraceKind::Publish);
  EXPECT_EQ(harness.trace.at(0).arm_slot, 1);
  expectSafeCommand(harness.trace.at(0).command, latest1);
  EXPECT_EQ(harness.trace.at(1).kind, TraceKind::Publish);
  EXPECT_EQ(harness.trace.at(1).arm_slot, 2);
  expectSafeCommand(harness.trace.at(1).command, latest2);
  EXPECT_EQ(harness.trace.at(2).kind, TraceKind::Request);
  EXPECT_EQ(harness.trace.at(2).arm_slot, 1);
  EXPECT_EQ(harness.trace.at(3).kind, TraceKind::Request);
  EXPECT_EQ(harness.trace.at(3).arm_slot, 2);

  ASSERT_EQ(hardware.prepare_command_mode_switch({}, both), hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.perform_command_mode_switch({}, both), hardware_interface::return_type::OK);
  harness.trace.clear();
  ASSERT_EQ(hardware.prepare_command_mode_switch(effort1, {}), hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.perform_command_mode_switch(effort1, {}), hardware_interface::return_type::OK);
  ASSERT_EQ(harness.trace.size(), 2U);
  EXPECT_EQ(harness.trace.at(0).arm_slot, 1);
  EXPECT_EQ(harness.trace.at(1).arm_slot, 1);
  EXPECT_EQ(harness.backend("alpha")->requestedControlMode(), ControlMode::None);
  EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST(FrankaMultiHardwareInterfaceModeTest,
     EachModeSwitchVectorAcceptsExactlyTwentyEightEntriesAndRejectsTwentyNine) {
  RclcppScope rclcpp_scope;
  BackendHarness harness;
  FrankaMultiHardwareInterface hardware(harness.factory());
  initializeAndActivate(hardware, harness);

  const auto both_effort =
      concatenate(jointModeInterfaces("panda1", "effort"), jointModeInterfaces("panda2", "effort"));
  const auto fourteen_foreign = foreignInterfaces(14, "foreign_start_");
  const auto fifteen_foreign = foreignInterfaces(15, "foreign_overflow_");
  const auto start_at_limit = concatenate(both_effort, fourteen_foreign);
  const auto start_over_limit = concatenate(both_effort, fifteen_foreign);
  ASSERT_EQ(start_at_limit.size(), FrankaMultiHardwareInterface::kMaximumModeSwitchInterfaceCount);
  ASSERT_EQ(start_over_limit.size(),
            FrankaMultiHardwareInterface::kMaximumModeSwitchInterfaceCount + 1);

  EXPECT_EQ(hardware.prepare_command_mode_switch(start_over_limit, {}),
            hardware_interface::return_type::ERROR);
  EXPECT_TRUE(harness.trace.empty());
  ASSERT_EQ(hardware.prepare_command_mode_switch(start_at_limit, {}),
            hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.perform_command_mode_switch(start_at_limit, {}),
            hardware_interface::return_type::OK);
  EXPECT_EQ(harness.backend("panda1")->requestedControlMode(), ControlMode::JointTorque);
  EXPECT_EQ(harness.backend("panda2")->requestedControlMode(), ControlMode::JointTorque);

  harness.trace.clear();
  const auto stop_at_limit = concatenate(both_effort, foreignInterfaces(14, "foreign_stop_"));
  const auto stop_over_limit =
      concatenate(both_effort, foreignInterfaces(15, "foreign_stop_overflow_"));
  ASSERT_EQ(stop_at_limit.size(), FrankaMultiHardwareInterface::kMaximumModeSwitchInterfaceCount);
  ASSERT_EQ(stop_over_limit.size(),
            FrankaMultiHardwareInterface::kMaximumModeSwitchInterfaceCount + 1);
  EXPECT_EQ(hardware.prepare_command_mode_switch({}, stop_over_limit),
            hardware_interface::return_type::ERROR);
  EXPECT_TRUE(harness.trace.empty());
  ASSERT_EQ(hardware.prepare_command_mode_switch({}, stop_at_limit),
            hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.perform_command_mode_switch({}, stop_at_limit),
            hardware_interface::return_type::OK);
  EXPECT_EQ(harness.backend("panda1")->requestedControlMode(), ControlMode::None);
  EXPECT_EQ(harness.backend("panda2")->requestedControlMode(), ControlMode::None);
  EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST(FrankaMultiHardwareInterfaceModeTest,
     ForeignInterfacesAreIgnoredForPreparedSetEquivalenceButConfiguredMalformedAndDuplicatesFail) {
  RclcppScope rclcpp_scope;
  BackendHarness harness;
  FrankaMultiHardwareInterface hardware(harness.factory());
  initializeAndActivate(hardware, harness);
  const auto effort = jointModeInterfaces("panda1", "effort");

  auto duplicated = effort;
  duplicated.push_back(effort.front());
  EXPECT_EQ(hardware.prepare_command_mode_switch(duplicated, {}),
            hardware_interface::return_type::ERROR);
  EXPECT_EQ(hardware.prepare_command_mode_switch({"panda1_joint8/effort"}, {}),
            hardware_interface::return_type::ERROR);
  const auto overlong_configured_name =
      std::string("panda1_") +
      std::string(FrankaMultiHardwareInterface::kMaximumJointModeInterfaceNameLength, 'x');
  ASSERT_GT(overlong_configured_name.size(),
            FrankaMultiHardwareInterface::kMaximumJointModeInterfaceNameLength);
  EXPECT_EQ(hardware.prepare_command_mode_switch({overlong_configured_name}, {}),
            hardware_interface::return_type::ERROR);
  EXPECT_TRUE(harness.trace.empty());

  auto prepared_start = effort;
  prepared_start.push_back("foreign_alpha_joint1/effort");
  const std::vector<std::string> prepared_stop{"foreign_beta_joint1/velocity"};
  auto equivalent_start = effort;
  std::reverse(equivalent_start.begin(), equivalent_start.end());
  equivalent_start.push_back("different_foreign_joint7/position");
  const std::vector<std::string> equivalent_stop{
      std::string(FrankaMultiHardwareInterface::kMaximumDerivedInterfaceNameLength + 32, 'z')};

  ASSERT_EQ(hardware.prepare_command_mode_switch(prepared_start, prepared_stop),
            hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.perform_command_mode_switch(equivalent_start, equivalent_stop),
            hardware_interface::return_type::OK);
  EXPECT_EQ(harness.backend("panda1")->requestedControlMode(), ControlMode::JointTorque);
  ASSERT_EQ(hardware.prepare_command_mode_switch({}, effort), hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.perform_command_mode_switch({}, effort), hardware_interface::return_type::OK);
  EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST(FrankaMultiHardwareInterfaceModeTest,
     PreparedTransactionsAreOneShotAndEveryFailedPrepareOrPerformInvalidatesThem) {
  RclcppScope rclcpp_scope;
  BackendHarness harness;
  FrankaMultiHardwareInterface hardware(harness.factory());
  initializeAndActivate(hardware, harness);
  const auto effort = jointModeInterfaces("panda1", "effort");
  const auto velocity = jointModeInterfaces("panda1", "velocity");
  const auto position = jointModeInterfaces("panda1", "position");

  EXPECT_EQ(hardware.perform_command_mode_switch(effort, {}),
            hardware_interface::return_type::ERROR);
  ASSERT_EQ(hardware.prepare_command_mode_switch(effort, {}), hardware_interface::return_type::OK);
  EXPECT_EQ(hardware.perform_command_mode_switch(velocity, {}),
            hardware_interface::return_type::ERROR);
  EXPECT_EQ(hardware.perform_command_mode_switch(effort, {}),
            hardware_interface::return_type::ERROR);

  ASSERT_EQ(hardware.prepare_command_mode_switch(effort, {}), hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.perform_command_mode_switch(effort, {}), hardware_interface::return_type::OK);
  EXPECT_EQ(hardware.perform_command_mode_switch(effort, {}),
            hardware_interface::return_type::ERROR);

  ASSERT_EQ(hardware.prepare_command_mode_switch({}, effort), hardware_interface::return_type::OK);
  EXPECT_EQ(hardware.prepare_command_mode_switch(position, {}),
            hardware_interface::return_type::ERROR);
  EXPECT_EQ(hardware.perform_command_mode_switch({}, effort),
            hardware_interface::return_type::ERROR);
  EXPECT_EQ(hardware.prepare_command_mode_switch({}, velocity),
            hardware_interface::return_type::ERROR);
  EXPECT_EQ(hardware.perform_command_mode_switch({}, velocity),
            hardware_interface::return_type::ERROR);

  ASSERT_EQ(hardware.prepare_command_mode_switch({}, effort), hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.perform_command_mode_switch({}, effort), hardware_interface::return_type::OK);
  EXPECT_FALSE(hardware.globalFaultDiagnostic().latched());
  EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST(FrankaMultiHardwareInterfaceModeTest,
     RejectedOffOwnerPerformInvalidatesPreparedTransactionAndCannotBeReplayed) {
  RclcppScope rclcpp_scope;
  BackendHarness harness;
  FrankaMultiHardwareInterface hardware(harness.factory());
  initializeAndActivate(hardware, harness);
  const auto effort = jointModeInterfaces("panda1", "effort");

  ASSERT_EQ(hardware.prepare_command_mode_switch(effort, {}), hardware_interface::return_type::OK);
  const auto panda1_reads_before = harness.control("panda1")->read_calls;
  const auto panda2_reads_before = harness.control("panda2")->read_calls;
  auto rejected_read = hardware_interface::return_type::OK;
  auto rejected_write = hardware_interface::return_type::OK;
  auto rejected = hardware_interface::return_type::OK;
  std::thread off_owner([&]() {
    rejected_read = hardware.read(rclcpp::Time(0), rclcpp::Duration(0, 0));
    rejected_write = hardware.write(rclcpp::Time(0), rclcpp::Duration(0, 0));
    rejected = hardware.perform_command_mode_switch(effort, {});
  });
  off_owner.join();

  EXPECT_EQ(rejected_read, hardware_interface::return_type::ERROR);
  EXPECT_EQ(rejected_write, hardware_interface::return_type::ERROR);
  EXPECT_EQ(rejected, hardware_interface::return_type::ERROR);
  EXPECT_EQ(harness.control("panda1")->read_calls, panda1_reads_before);
  EXPECT_EQ(harness.control("panda2")->read_calls, panda2_reads_before);
  EXPECT_TRUE(harness.trace.empty());
  EXPECT_EQ(hardware.perform_command_mode_switch(effort, {}),
            hardware_interface::return_type::ERROR);
  EXPECT_TRUE(harness.trace.empty());

  ASSERT_EQ(hardware.prepare_command_mode_switch(effort, {}), hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.perform_command_mode_switch(effort, {}), hardware_interface::return_type::OK);
  EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST(FrankaMultiHardwareInterfaceModeTest,
     PreparedGenerationPackingUsesTheFullNonzeroSixtyOneBitDomain) {
  EXPECT_EQ(preparedTransactionGenerationCandidate(1), 1U);
  EXPECT_EQ(preparedTransactionGenerationCandidate(kPreparedTransactionGenerationMask),
            kPreparedTransactionGenerationMask);
  EXPECT_EQ(preparedTransactionGenerationCandidate(uint64_t{1} << 61U), 0U);
  EXPECT_EQ(preparedTransactionGenerationCandidate((uint64_t{1} << 61U) + 1U), 1U);
  EXPECT_EQ(preparedTransactionGenerationCandidate(std::numeric_limits<uint64_t>::max()),
            kPreparedTransactionGenerationMask);
}

TEST(FrankaMultiHardwareInterfaceModeFaultTest,
     FixedSeedTwoArmSequencesMatchIndependentTransactionAndFaultModel) {
  RclcppScope rclcpp_scope;
  ScopedModeTestLoggerLevel quiet_expected_rejections{"FrankaMultiHardwareInterface",
                                                      RCUTILS_LOG_SEVERITY_FATAL};
  constexpr std::array<std::uint64_t, 3> kSeeds{0x4d4f4445U, 0xfa017a11U, 0x7472616e73616374ULL};
  constexpr std::size_t kCasesPerSeed = 4000;
  std::array<std::array<std::array<std::size_t, 3>, 3>, 2> transition_coverage{};

  BackendHarness harness;
  FrankaMultiHardwareInterface hardware(harness.factory());
  initializeAndActivate(hardware, harness);
  TwoArmModeModel model;

  const auto mode_index = [](const ControlMode mode) {
    const auto found = std::find(kModelModes.begin(), kModelModes.end(), mode);
    if (found == kModelModes.end()) {
      throw std::runtime_error("unsupported mode in reference model");
    }
    return static_cast<std::size_t>(std::distance(kModelModes.begin(), found));
  };
  const auto prepare = [&](const ModelTransition& transition) {
    const auto result = hardware.prepare_command_mode_switch(transition.starts, transition.stops);
    if (result == hardware_interface::return_type::OK) {
      model.prepared = PreparedModelTransition{transition, model.next_generation++};
    } else {
      model.prepared.reset();
    }
    return result;
  };
  const auto perform_matching = [&](std::mt19937_64& engine, const bool add_foreign) {
    if (!model.prepared.has_value()) {
      return hardware.perform_command_mode_switch({}, {});
    }
    auto starts = model.prepared->transition.starts;
    auto stops = model.prepared->transition.stops;
    std::shuffle(starts.begin(), starts.end(), engine);
    std::shuffle(stops.begin(), stops.end(), engine);
    if (add_foreign) {
      starts.push_back("foreign_joint1/effort");
    }
    const auto target = model.prepared->transition.target_modes;
    const auto result = hardware.perform_command_mode_switch(starts, stops);
    model.prepared.reset();
    if (result == hardware_interface::return_type::OK) {
      model.modes = target;
    }
    return result;
  };
  const auto complete_valid = [&](const ModelTransition& transition, std::mt19937_64& engine,
                                  const bool add_foreign) {
    if (prepare(transition) != hardware_interface::return_type::OK) {
      return false;
    }
    return perform_matching(engine, add_foreign) == hardware_interface::return_type::OK;
  };

  for (const auto seed : kSeeds) {
    std::cout << "Franka two-arm mode/fault property seed=" << seed << " cases=" << kCasesPerSeed
              << '\n';
    std::mt19937_64 engine(seed);
    for (std::size_t case_index = 0; case_index < kCasesPerSeed; ++case_index) {
      // A real control loop drains accepted backend commands between mode transactions. Mirror
      // that independent environment fact so the fixed synthetic queue does not manufacture
      // capacity faults unrelated to the transition model under test.
      ASSERT_EQ(hardware.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
                hardware_interface::return_type::OK);
      harness.trace.clear();
      SCOPED_TRACE("seed=" + std::to_string(seed) + " case=" + std::to_string(case_index));

      if (case_index < 18U) {
        const auto arm = case_index / 9U;
        const auto from = kModelModes[(case_index / 3U) % 3U];
        const auto target_mode = kModelModes[case_index % 3U];
        auto drive_modes = model.modes;
        drive_modes[arm] = from;
        auto drive = makeModelTransition(model.modes, drive_modes, false);
        SCOPED_TRACE("coverage drive=" + describeModelOperation("valid", model, &drive));
        ASSERT_TRUE(complete_valid(drive, engine, false));
        auto target_modes = model.modes;
        target_modes[arm] = target_mode;
        auto transition = makeModelTransition(model.modes, target_modes, true);
        SCOPED_TRACE("coverage transition=" + describeModelOperation("valid", model, &transition));
        ASSERT_TRUE(complete_valid(transition, engine, (case_index & 1U) != 0U));
        ++transition_coverage[arm][mode_index(from)][mode_index(target_mode)];
      } else {
        const auto operation = static_cast<std::size_t>(engine() % 8U);
        if (operation == 0U || operation == 5U) {
          const auto target_modes = operation == 5U
                                        ? model.modes
                                        : std::array<ControlMode, 2>{kModelModes[engine() % 3U],
                                                                     kModelModes[engine() % 3U]};
          const auto before = model.modes;
          auto transition = makeModelTransition(model.modes, target_modes, operation == 5U);
          SCOPED_TRACE(describeModelOperation(
              operation == 5U ? "same-mode replacement" : "valid transition", model, &transition));
          ASSERT_TRUE(complete_valid(transition, engine, (engine() & 1U) != 0U));
          for (std::size_t arm = 0; arm < 2; ++arm) {
            ++transition_coverage[arm][mode_index(before[arm])][mode_index(target_modes[arm])];
          }
        } else if (operation == 1U) {
          const std::array<ControlMode, 2> target_modes{kModelModes[engine() % 3U],
                                                        kModelModes[engine() % 3U]};
          auto transition = makeModelTransition(model.modes, target_modes, (engine() & 1U) != 0U);
          SCOPED_TRACE(describeModelOperation("prepare only", model, &transition));
          ASSERT_EQ(prepare(transition), hardware_interface::return_type::OK);
        } else if (operation == 2U) {
          const bool had_prepared = model.prepared.has_value();
          SCOPED_TRACE(describeModelOperation("matching perform", model));
          const auto result = perform_matching(engine, (engine() & 1U) != 0U);
          ASSERT_EQ(result, had_prepared ? hardware_interface::return_type::OK
                                         : hardware_interface::return_type::ERROR);
        } else if (operation == 3U) {
          SCOPED_TRACE(describeModelOperation("mismatched perform", model));
          const auto before = model.modes;
          EXPECT_EQ(hardware.perform_command_mode_switch({"panda1_joint8/effort"}, {}),
                    hardware_interface::return_type::ERROR);
          model.prepared.reset();
          EXPECT_EQ(model.modes, before);
        } else if (operation == 4U) {
          SCOPED_TRACE(describeModelOperation("invalid partial prepare", model));
          const auto before = model.modes;
          auto partial = jointModeInterfaces("panda1", "effort");
          partial.resize(1U + engine() % 6U);
          EXPECT_EQ(hardware.prepare_command_mode_switch(partial, {}),
                    hardware_interface::return_type::ERROR);
          model.prepared.reset();
          EXPECT_EQ(model.modes, before);
        } else if (operation == 6U) {
          const auto fault_mask = static_cast<uint8_t>(1U + engine() % 3U);
          SCOPED_TRACE(
              describeModelOperation("fault/recovery mask=" + std::to_string(fault_mask), model));
          for (std::size_t arm = 0; arm < 2; ++arm) {
            if ((fault_mask & (1U << arm)) != 0U) {
              harness.backend(harness.arm_names[arm])->injectFaultForTest();
            }
          }
          EXPECT_EQ(hardware.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
                    hardware_interface::return_type::ERROR);
          const auto fault = hardware.globalFaultDiagnostic();
          EXPECT_TRUE(fault.latched());
          EXPECT_EQ(fault.cause, GlobalFaultCause::BackendFault);
          EXPECT_EQ(fault.origin_arm_slot, (fault_mask & 1U) != 0U ? 1U : 2U);
          model.modes = {ControlMode::None, ControlMode::None};
          model.prepared.reset();
          EXPECT_EQ(hardware.prepare_command_mode_switch({}, {}),
                    hardware_interface::return_type::ERROR);
          EXPECT_EQ(hardware.perform_command_mode_switch({}, {}),
                    hardware_interface::return_type::ERROR);
          for (std::size_t arm = 0; arm < 2; ++arm) {
            if ((fault_mask & (1U << arm)) != 0U) {
              ASSERT_TRUE(harness.backend(harness.arm_names[arm])->recoverToReading());
            }
          }
          EXPECT_EQ(hardware.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
                    hardware_interface::return_type::OK);
          EXPECT_FALSE(hardware.globalFaultDiagnostic().latched());
        } else {
          const auto arm = static_cast<std::size_t>(engine() % 2U);
          SCOPED_TRACE(describeModelOperation(
              "healthy recovery rejection arm=" + std::to_string(arm + 1), model));
          const auto before = model.modes;
          EXPECT_FALSE(harness.backend(harness.arm_names[arm])->recoverToReading());
          EXPECT_EQ(model.modes, before);
          EXPECT_FALSE(hardware.globalFaultDiagnostic().latched());
        }
      }

      expectModelModes(harness, model);
      EXPECT_EQ(hardware.globalFaultDiagnostic().latched(), false);
    }
  }

  for (std::size_t arm = 0; arm < 2; ++arm) {
    for (std::size_t from = 0; from < 3; ++from) {
      for (std::size_t to = 0; to < 3; ++to) {
        EXPECT_GT(transition_coverage[arm][from][to], 0U)
            << "missing arm=" << arm + 1 << " transition=" << modelModeName(kModelModes[from])
            << "->" << modelModeName(kModelModes[to]);
      }
    }
  }
  EXPECT_GT(model.next_generation, 1U);
  EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST(FrankaMultiHardwareInterfaceModeFaultTest,
     FaultWhilePreparedPayloadIsPublishingInvalidatesItWithoutReadWriteOverlapOrMotion) {
  RclcppScope rclcpp_scope;
  BackendHarness harness;
  PreparedCheckpointBarrier barrier;
  FrankaMultiHardwareInterface hardware(harness.factory(), InitializationCheckpointHook{},
                                        barrier.hook());
  initializeAndActivate(hardware, harness);
  auto command_interfaces = hardware.export_command_interfaces();
  const auto effort = jointModeInterfaces("panda1", "effort");
  auto prepare_result = hardware_interface::return_type::OK;
  std::thread prepare_thread(
      [&]() { prepare_result = hardware.prepare_command_mode_switch(effort, {}); });

  const bool reached_barrier = barrier.waitUntilEntered();
  if (!reached_barrier) {
    barrier.release();
    prepare_thread.join();
  }
  ASSERT_TRUE(reached_barrier);
  setCommandInterfaceValue(command_interfaces, "panda1_joint1/effort",
                           std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(hardware.write(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            hardware_interface::return_type::ERROR);
  barrier.release();
  prepare_thread.join();

  EXPECT_EQ(prepare_result, hardware_interface::return_type::ERROR);
  expectDiagnostic(hardware, 1, GlobalFaultCause::InvalidCommand, 0, 0);
  EXPECT_EQ(std::count_if(harness.trace.begin(), harness.trace.end(),
                          [](const auto& event) {
                            return event.kind == TraceKind::Request &&
                                   event.mode != ControlMode::None;
                          }),
            0);
  const auto trace_size = harness.trace.size();
  EXPECT_EQ(hardware.perform_command_mode_switch(effort, {}),
            hardware_interface::return_type::ERROR);
  EXPECT_EQ(harness.trace.size(), trace_size);
  EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST(FrankaMultiHardwareInterfaceModeTest,
     ServiceGatePreflightIsTransientUnlatchedAndOnlyChecksAffectedArms) {
  RclcppScope rclcpp_scope;
  BackendHarness harness;
  FrankaMultiHardwareInterface hardware(harness.factory());
  initializeAndActivate(hardware, harness);
  const auto effort1 = jointModeInterfaces("panda1", "effort");

  ASSERT_TRUE(
      harness.backend("panda1")->holdServiceOperationForTest(BackendServiceOperation::Parameter));
  ASSERT_EQ(hardware.prepare_command_mode_switch(effort1, {}), hardware_interface::return_type::OK);
  EXPECT_EQ(hardware.perform_command_mode_switch(effort1, {}),
            hardware_interface::return_type::ERROR);
  EXPECT_TRUE(harness.trace.empty());
  EXPECT_FALSE(hardware.globalFaultDiagnostic().latched());
  harness.backend("panda1")->releaseServiceOperationForTest();
  EXPECT_EQ(hardware.perform_command_mode_switch(effort1, {}),
            hardware_interface::return_type::ERROR);

  ASSERT_TRUE(
      harness.backend("panda2")->holdServiceOperationForTest(BackendServiceOperation::Parameter));
  ASSERT_EQ(hardware.prepare_command_mode_switch(effort1, {}), hardware_interface::return_type::OK);
  EXPECT_EQ(hardware.perform_command_mode_switch(effort1, {}), hardware_interface::return_type::OK);
  EXPECT_EQ(harness.backend("panda1")->requestedControlMode(), ControlMode::JointTorque);
  EXPECT_EQ(harness.backend("panda2")->requestedControlMode(), ControlMode::None);
  EXPECT_FALSE(hardware.globalFaultDiagnostic().latched());
  harness.backend("panda2")->releaseServiceOperationForTest();
  EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST(FrankaMultiHardwareInterfaceModeFaultTest,
     FaultInsertedAfterPreflightStartsInvalidatesPreparedMotionBeforeAnyMotionRequest) {
  RclcppScope rclcpp_scope;
  BackendHarness harness;
  FrankaMultiHardwareInterface hardware(harness.factory());
  initializeAndActivate(hardware, harness);
  const auto starts =
      concatenate(jointModeInterfaces("panda1", "effort"), jointModeInterfaces("panda2", "effort"));
  ASSERT_EQ(hardware.prepare_command_mode_switch(starts, {}), hardware_interface::return_type::OK);

  auto panda2_control = harness.control("panda2");
  panda2_control->pause_can_request = true;
  bool reached_precommit_barrier = false;
  std::thread fault_injector([&]() {
    reached_precommit_barrier = panda2_control->waitForCanRequest();
    if (reached_precommit_barrier) {
      harness.backend("panda1")->injectFaultForTest();
    }
    panda2_control->releaseCanRequest();
  });

  EXPECT_EQ(hardware.perform_command_mode_switch(starts, {}),
            hardware_interface::return_type::ERROR);
  fault_injector.join();
  ASSERT_TRUE(reached_precommit_barrier);
  expectDiagnostic(hardware, 1, GlobalFaultCause::BackendFault, 0b01, 0b01);
  EXPECT_EQ(harness.backend("panda1")->requestedControlMode(), ControlMode::None);
  EXPECT_EQ(harness.backend("panda2")->requestedControlMode(), ControlMode::None);
  EXPECT_EQ(std::count_if(harness.trace.begin(), harness.trace.end(),
                          [](const auto& event) {
                            return event.kind == TraceKind::Request &&
                                   event.mode != ControlMode::None;
                          }),
            0);
  EXPECT_EQ(hardware.perform_command_mode_switch(starts, {}),
            hardware_interface::return_type::ERROR);
  EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST(FrankaMultiHardwareInterfaceModeFaultTest,
     SnapshotCapacityAndPublishFailuresLatchExactArmCauseAndUnsafeMask) {
  RclcppScope rclcpp_scope;
  struct Scenario {
    uint8_t arm_slot;
    bool capacity_failure;
  };
  for (const auto scenario :
       {Scenario{1, true}, Scenario{2, true}, Scenario{1, false}, Scenario{2, false}}) {
    SCOPED_TRACE(static_cast<int>(scenario.arm_slot));
    BackendHarness harness;
    FrankaMultiHardwareInterface hardware(harness.factory());
    initializeAndActivate(hardware, harness);
    const auto arm_name = "panda" + std::to_string(scenario.arm_slot);
    if (scenario.capacity_failure) {
      harness.control(arm_name)->no_command_capacity = true;
    } else {
      harness.control(arm_name)->fail_all_publishes = true;
    }
    const auto starts = concatenate(jointModeInterfaces("panda1", "effort"),
                                    jointModeInterfaces("panda2", "effort"));
    ASSERT_EQ(hardware.prepare_command_mode_switch(starts, {}),
              hardware_interface::return_type::OK);
    EXPECT_EQ(hardware.perform_command_mode_switch(starts, {}),
              hardware_interface::return_type::ERROR);
    expectDiagnostic(hardware, scenario.arm_slot,
                     scenario.capacity_failure ? GlobalFaultCause::CommandCapacity
                                               : GlobalFaultCause::CommandPublish,
                     static_cast<uint8_t>(1U << (scenario.arm_slot - 1)), 0);
    EXPECT_TRUE(std::none_of(harness.trace.begin(), harness.trace.end(), [](const auto& event) {
      return event.kind == TraceKind::Request && event.mode != ControlMode::None;
    }));
    const auto trace_size = harness.trace.size();
    EXPECT_EQ(hardware.write(rclcpp::Time(0), rclcpp::Duration(0, 0)),
              hardware_interface::return_type::ERROR);
    EXPECT_EQ(hardware.prepare_command_mode_switch(starts, {}),
              hardware_interface::return_type::ERROR);
    EXPECT_EQ(harness.trace.size(), trace_size);
    EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  }
}

TEST(FrankaMultiHardwareInterfaceModeFaultTest,
     ArmLocalModeRequestFailuresPublishEverySnapshotFirstAndRollbackToNone) {
  RclcppScope rclcpp_scope;
  for (const uint8_t failing_slot : {1, 2}) {
    SCOPED_TRACE(static_cast<int>(failing_slot));
    BackendHarness harness;
    FrankaMultiHardwareInterface hardware(harness.factory());
    initializeAndActivate(hardware, harness);
    harness.control("panda" + std::to_string(failing_slot))->fail_request_call = 1;
    const auto starts = concatenate(jointModeInterfaces("panda1", "effort"),
                                    jointModeInterfaces("panda2", "effort"));
    ASSERT_EQ(hardware.prepare_command_mode_switch(starts, {}),
              hardware_interface::return_type::OK);
    EXPECT_EQ(hardware.perform_command_mode_switch(starts, {}),
              hardware_interface::return_type::ERROR);
    expectDiagnostic(hardware, failing_slot, GlobalFaultCause::ModeRequest, 0, 0);
    const auto first_request =
        std::find_if(harness.trace.begin(), harness.trace.end(),
                     [](const auto& event) { return event.kind == TraceKind::Request; });
    ASSERT_NE(first_request, harness.trace.end());
    ASSERT_GE(std::distance(harness.trace.begin(), first_request), 2);
    EXPECT_EQ(harness.trace.at(0).kind, TraceKind::Publish);
    EXPECT_EQ(harness.trace.at(0).arm_slot, 1);
    EXPECT_EQ(harness.trace.at(1).kind, TraceKind::Publish);
    EXPECT_EQ(harness.trace.at(1).arm_slot, 2);
    EXPECT_EQ(harness.backend("panda1")->requestedControlMode(), ControlMode::None);
    EXPECT_EQ(harness.backend("panda2")->requestedControlMode(), ControlMode::None);
    EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  }
}

TEST(FrankaMultiHardwareInterfaceModeFaultTest,
     RarePartialRequestAndFailedNoneRollbackExposeExactUnsafeMaskAndBlockMotion) {
  RclcppScope rclcpp_scope;
  BackendHarness harness;
  FrankaMultiHardwareInterface hardware(harness.factory());
  initializeAndActivate(hardware, harness);
  harness.control("panda1")->fail_request_call = 2;
  harness.control("panda2")->fail_request_call = 1;
  const auto starts =
      concatenate(jointModeInterfaces("panda1", "effort"), jointModeInterfaces("panda2", "effort"));
  ASSERT_EQ(hardware.prepare_command_mode_switch(starts, {}), hardware_interface::return_type::OK);
  EXPECT_EQ(hardware.perform_command_mode_switch(starts, {}),
            hardware_interface::return_type::ERROR);
  expectDiagnostic(hardware, 2, GlobalFaultCause::ModeRequest, 0, 0x01);
  EXPECT_EQ(harness.backend("panda1")->requestedControlMode(), ControlMode::JointTorque);
  EXPECT_EQ(harness.backend("panda2")->requestedControlMode(), ControlMode::None);
  const auto trace_size = harness.trace.size();
  EXPECT_EQ(hardware.write(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            hardware_interface::return_type::ERROR);
  EXPECT_EQ(hardware.prepare_command_mode_switch({}, starts),
            hardware_interface::return_type::ERROR);
  EXPECT_EQ(hardware.perform_command_mode_switch({}, starts),
            hardware_interface::return_type::ERROR);
  EXPECT_EQ(harness.trace.size(), trace_size);
  EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST(FrankaMultiHardwareInterfaceGlobalFaultTest,
     ReadFailuresLatchFirstArmCauseMasksAndOnlyLocalFaultsClearAfterSafeActivation) {
  RclcppScope rclcpp_scope;
  struct Scenario {
    uint8_t arm_slot;
    GlobalFaultCause cause;
  };
  const std::array<Scenario, 6> scenarios{{
      {1, GlobalFaultCause::InvalidState},
      {2, GlobalFaultCause::InvalidState},
      {1, GlobalFaultCause::ReadFailure},
      {2, GlobalFaultCause::ReadFailure},
      {1, GlobalFaultCause::BackendFault},
      {2, GlobalFaultCause::BackendFault},
  }};
  for (const auto& scenario : scenarios) {
    SCOPED_TRACE(static_cast<int>(scenario.arm_slot));
    BackendHarness harness;
    const auto arm_name = "panda" + std::to_string(scenario.arm_slot);
    if (scenario.cause == GlobalFaultCause::InvalidState) {
      harness.configurations.at(arm_name).failure = {SyntheticFailurePoint::InvalidNanState, 3};
    } else if (scenario.cause == GlobalFaultCause::BackendFault) {
      harness.configurations.at(arm_name).failure = {SyntheticFailurePoint::ReadFault, 2};
    } else {
      harness.control(arm_name)->throw_read_call = 3;
    }
    FrankaMultiHardwareInterface hardware(harness.factory());
    initializeAndActivate(hardware, harness);
    EXPECT_EQ(hardware.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
              hardware_interface::return_type::ERROR);
    const uint8_t unsafe_mask = scenario.cause == GlobalFaultCause::BackendFault
                                    ? static_cast<uint8_t>(1U << (scenario.arm_slot - 1))
                                    : 0;
    expectDiagnostic(hardware, scenario.arm_slot, scenario.cause, unsafe_mask, unsafe_mask);
    const auto first_fault = hardware.globalFaultDiagnostic();
    EXPECT_EQ(hardware.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
              hardware_interface::return_type::ERROR);
    EXPECT_EQ(hardware.write(rclcpp::Time(0), rclcpp::Duration(0, 0)),
              hardware_interface::return_type::ERROR);
    EXPECT_EQ(hardware.globalFaultDiagnostic().origin_arm_slot, first_fault.origin_arm_slot);
    EXPECT_EQ(hardware.globalFaultDiagnostic().cause, first_fault.cause);
    EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    if (scenario.cause == GlobalFaultCause::BackendFault) {
      EXPECT_NE(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
      EXPECT_EQ(hardware.globalFaultDiagnostic().cause, GlobalFaultCause::BackendFault);
    } else {
      ASSERT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
      EXPECT_FALSE(hardware.globalFaultDiagnostic().latched());
      EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    }
  }
}

TEST(FrankaMultiHardwareInterfaceGlobalFaultTest,
     EveryInvalidCommandFamilyOnEitherArmRestoresBothSafeSnapshotsAndLatchesFirstFault) {
  RclcppScope rclcpp_scope;
  const std::array<std::string, 5> suffixes{"_joint1/effort", "_joint1/position",
                                            "_joint1/velocity", "_ee_cartesian_position/00",
                                            "_ee_cartesian_velocity/tx"};
  for (const uint8_t arm_slot : {1, 2}) {
    for (const auto& suffix : suffixes) {
      SCOPED_TRACE(std::to_string(arm_slot) + suffix);
      BackendHarness harness;
      FrankaMultiHardwareInterface hardware(harness.factory());
      initializeAndActivate(hardware, harness);
      auto commands = hardware.export_command_interfaces();
      setCommandInterfaceValue(commands, "panda" + std::to_string(arm_slot) + suffix,
                               std::numeric_limits<double>::quiet_NaN());
      EXPECT_EQ(hardware.write(rclcpp::Time(0), rclcpp::Duration(0, 0)),
                hardware_interface::return_type::ERROR);
      expectDiagnostic(hardware, arm_slot, GlobalFaultCause::InvalidCommand, 0, 0);
      ASSERT_EQ(harness.trace.size(), 4U);
      EXPECT_EQ(harness.trace.at(0).kind, TraceKind::Publish);
      EXPECT_EQ(harness.trace.at(0).arm_slot, 1);
      expectSafeCommand(harness.trace.at(0).command,
                        harness.configurations.at("panda1").initial_state);
      EXPECT_EQ(harness.trace.at(1).kind, TraceKind::Publish);
      EXPECT_EQ(harness.trace.at(1).arm_slot, 2);
      expectSafeCommand(harness.trace.at(1).command,
                        harness.configurations.at("panda2").initial_state);
      const auto trace_size = harness.trace.size();
      setCommandInterfaceValue(commands, "panda" + std::to_string(arm_slot) + suffix, 0.0);
      EXPECT_EQ(hardware.write(rclcpp::Time(0), rclcpp::Duration(0, 0)),
                hardware_interface::return_type::ERROR);
      EXPECT_EQ(hardware.prepare_command_mode_switch(jointModeInterfaces("panda1", "effort"), {}),
                hardware_interface::return_type::ERROR);
      EXPECT_EQ(harness.trace.size(), trace_size);
      EXPECT_EQ(hardware.globalFaultDiagnostic().origin_arm_slot, arm_slot);

      ASSERT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
      ASSERT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
      EXPECT_FALSE(hardware.globalFaultDiagnostic().latched());
      harness.trace.clear();
      EXPECT_EQ(hardware.write(rclcpp::Time(0), rclcpp::Duration(0, 0)),
                hardware_interface::return_type::OK);
      EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    }
  }
}

TEST(FrankaMultiHardwareInterfaceGlobalFaultTest,
     WriteBackendCapacityAndPublishFailuresLatchExactFirstArmAndBlockLaterWrites) {
  RclcppScope rclcpp_scope;
  enum class FailureKind : uint8_t { Backend, Capacity, Publish };
  for (const uint8_t arm_slot : {1, 2}) {
    for (const auto failure : {FailureKind::Backend, FailureKind::Capacity, FailureKind::Publish}) {
      SCOPED_TRACE(static_cast<int>(arm_slot));
      BackendHarness harness;
      const auto arm_name = "panda" + std::to_string(arm_slot);
      if (failure == FailureKind::Backend) {
        harness.configurations.at(arm_name).failure = {SyntheticFailurePoint::ReadFault, 2};
      }
      FrankaMultiHardwareInterface hardware(harness.factory());
      initializeAndActivate(hardware, harness);
      if (failure == FailureKind::Backend) {
        (void)harness.backend(arm_name)->readLatestState();
        ASSERT_TRUE(harness.backend(arm_name)->hasFault());
      } else if (failure == FailureKind::Capacity) {
        harness.control(arm_name)->no_command_capacity = true;
      } else {
        harness.control(arm_name)->fail_all_publishes = true;
      }

      EXPECT_EQ(hardware.write(rclcpp::Time(0), rclcpp::Duration(0, 0)),
                hardware_interface::return_type::ERROR);
      const auto cause = failure == FailureKind::Backend ? GlobalFaultCause::BackendFault
                                                         : (failure == FailureKind::Capacity
                                                                ? GlobalFaultCause::CommandCapacity
                                                                : GlobalFaultCause::CommandPublish);
      const auto unsafe_safe_mask = static_cast<uint8_t>(1U << (arm_slot - 1));
      const auto unsafe_none_mask =
          failure == FailureKind::Backend ? unsafe_safe_mask : static_cast<uint8_t>(0);
      expectDiagnostic(hardware, arm_slot, cause, unsafe_safe_mask, unsafe_none_mask);
      const auto trace_size = harness.trace.size();
      EXPECT_EQ(hardware.write(rclcpp::Time(0), rclcpp::Duration(0, 0)),
                hardware_interface::return_type::ERROR);
      EXPECT_EQ(harness.trace.size(), trace_size);
      EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    }
  }
}

}  // namespace
}  // namespace franka_hardware
