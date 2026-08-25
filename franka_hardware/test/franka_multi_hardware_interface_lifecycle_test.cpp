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
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rcutils/logging.h>
#include <yaml-cpp/yaml.h>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "franka_hardware/common/helper_functions.hpp"
#include "franka_hardware/real/franka_multi_hardware_interface.hpp"
#include "support/synthetic_franka_arm_backend.hpp"
#include "support/synthetic_model.hpp"

namespace franka_hardware {
namespace {

using test_support::SyntheticEventKind;
using test_support::SyntheticFailurePoint;
using test_support::SyntheticFrankaArmBackend;
using test_support::SyntheticFrankaArmBackendConfig;

hardware_interface::InterfaceInfo makeInterface(const std::string& name) {
  hardware_interface::InterfaceInfo interface{};
  interface.name = name;
  interface.data_type = "double";
  return interface;
}

hardware_interface::HardwareInfo makeHardwareInfo(size_t arm_count) {
  hardware_interface::HardwareInfo info{};
  info.name = "FrankaMultiHardwareInterface";
  info.type = "system";
  info.hardware_plugin_name = "franka_hardware/FrankaMultiHardwareInterface";
  info.hardware_parameters["robot_count"] = std::to_string(arm_count);

  for (size_t arm_index = 1; arm_index <= arm_count; ++arm_index) {
    const auto arm_name = "panda" + std::to_string(arm_index);
    info.hardware_parameters["ns_" + std::to_string(arm_index)] = arm_name;
    info.hardware_parameters["robot_ip_" + std::to_string(arm_index)] =
        "offline-placeholder-" + std::to_string(arm_index);
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

class DelegatingBackend final : public FrankaArmBackend {
 public:
  DelegatingBackend(std::shared_ptr<SyntheticFrankaArmBackend> backend,
                    std::string arm_name,
                    std::vector<std::string>* read_order,
                    bool null_model,
                    bool throw_on_stop,
                    size_t throw_on_start_call,
                    size_t throw_on_read_call,
                    size_t start_failures_remaining,
                    size_t tracked_command_capacity)
      : backend_(std::move(backend)),
        arm_name_(std::move(arm_name)),
        read_order_(read_order),
        null_model_(null_model),
        throw_on_stop_(throw_on_stop),
        throw_on_start_call_(throw_on_start_call),
        throw_on_read_call_(throw_on_read_call),
        start_failures_remaining_(start_failures_remaining),
        tracked_command_capacity_(tracked_command_capacity) {}

  bool startStateReading() override {
    ++start_call_count_;
    if (throw_on_start_call_ != 0 && start_call_count_ == throw_on_start_call_) {
      throw std::runtime_error("injected start exception");
    }
    if (start_failures_remaining_ != 0) {
      --start_failures_remaining_;
      return false;
    }
    const bool started = backend_->startStateReading();
    if (started) {
      tracked_in_flight_commands_ = 0;
    }
    return started;
  }
  bool stop() override {
    if (throw_on_stop_) {
      throw std::runtime_error("injected stop exception");
    }
    const bool stopped = backend_->stop();
    if (stopped) {
      tracked_in_flight_commands_ = 0;
    }
    return stopped;
  }
  franka::RobotState readLatestState() override {
    if (read_order_ != nullptr) {
      read_order_->push_back(arm_name_);
    }
    ++read_call_count_;
    if (throw_on_read_call_ != 0 && read_call_count_ == throw_on_read_call_) {
      throw std::runtime_error("injected read exception");
    }
    return backend_->readLatestState();
  }
  ModelBase* model() noexcept override { return null_model_ ? nullptr : backend_->model(); }
  bool canPublishCommand() const noexcept override {
    return (tracked_command_capacity_ == 0 ||
            tracked_in_flight_commands_ < tracked_command_capacity_) &&
           backend_->canPublishCommand();
  }
  bool publishCommand(const RobotCommand& command) noexcept override {
    if (!canPublishCommand() || !backend_->publishCommand(command)) {
      return false;
    }
    if (tracked_command_capacity_ != 0) {
      ++tracked_in_flight_commands_;
    }
    return true;
  }
  bool canRequestControlMode(ControlMode mode) const noexcept override {
    return backend_->canRequestControlMode(mode);
  }
  bool requestControlMode(ControlMode mode) noexcept override {
    return backend_->requestControlMode(mode);
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
  std::shared_ptr<SyntheticFrankaArmBackend> backend_;
  std::string arm_name_;
  std::vector<std::string>* read_order_;
  bool null_model_;
  bool throw_on_stop_;
  size_t throw_on_start_call_;
  size_t throw_on_read_call_;
  size_t start_failures_remaining_;
  size_t tracked_command_capacity_;
  size_t start_call_count_{0};
  size_t read_call_count_{0};
  size_t tracked_in_flight_commands_{0};
};

struct FactoryHarness {
  std::map<std::string, SyntheticFrankaArmBackendConfig> configurations;
  std::set<std::string> null_backends;
  std::set<std::string> null_models;
  std::set<std::string> throwing_stops;
  std::map<std::string, size_t> throwing_start_calls;
  std::map<std::string, size_t> throwing_read_calls;
  std::map<std::string, size_t> start_failures;
  std::map<std::string, size_t> tracked_command_capacities;
  bool trace_reads{false};
  std::vector<std::string> read_order;
  std::vector<std::pair<std::string, std::string>> calls;
  std::map<std::string, std::weak_ptr<SyntheticFrankaArmBackend>> synthetic_backends;
  std::map<std::string, std::weak_ptr<FrankaArmBackend>> returned_backends;

  explicit FactoryHarness(size_t arm_count) {
    for (size_t arm_index = 1; arm_index <= arm_count; ++arm_index) {
      configurations.emplace(
          "panda" + std::to_string(arm_index),
          SyntheticFrankaArmBackendConfig::forArm(static_cast<uint8_t>(arm_index)));
    }
  }

  BackendFactory factory() {
    return [this](const std::string& arm_name, const std::string& address,
                  const rclcpp::Logger&) -> std::shared_ptr<FrankaArmBackend> {
      calls.emplace_back(arm_name, address);
      if (null_backends.count(arm_name) != 0) {
        return nullptr;
      }
      auto synthetic = std::make_shared<SyntheticFrankaArmBackend>(configurations.at(arm_name));
      synthetic_backends[arm_name] = synthetic;
      std::shared_ptr<FrankaArmBackend> returned = synthetic;
      if (null_models.count(arm_name) != 0 || throwing_stops.count(arm_name) != 0 ||
          throwing_start_calls.count(arm_name) != 0 || throwing_read_calls.count(arm_name) != 0 ||
          start_failures.count(arm_name) != 0 || tracked_command_capacities.count(arm_name) != 0 ||
          trace_reads) {
        returned = std::make_shared<DelegatingBackend>(
            synthetic, arm_name, trace_reads ? &read_order : nullptr,
            null_models.count(arm_name) != 0, throwing_stops.count(arm_name) != 0,
            throwing_start_calls[arm_name], throwing_read_calls[arm_name], start_failures[arm_name],
            tracked_command_capacities[arm_name]);
      }
      returned_backends[arm_name] = returned;
      return returned;
    };
  }

  std::shared_ptr<SyntheticFrankaArmBackend> backend(const std::string& arm_name) const {
    return synthetic_backends.at(arm_name).lock();
  }
};

std::vector<SyntheticEventKind> eventKindsSince(const SyntheticFrankaArmBackend& backend,
                                                size_t first_event) {
  std::vector<SyntheticEventKind> kinds;
  for (size_t index = first_event; index < backend.capturedEventCount(); ++index) {
    kinds.push_back(backend.capturedEvent(index).kind);
  }
  return kinds;
}

double stateInterfaceValue(const std::vector<hardware_interface::StateInterface>& interfaces,
                           const std::string& full_name) {
  const auto found = std::find_if(
      interfaces.begin(), interfaces.end(),
      [&full_name](const auto& interface) { return interface.get_name() == full_name; });
  if (found == interfaces.end()) {
    throw std::runtime_error("state interface not found");
  }
  const auto value = found->template get_optional<double>();
  if (!value) {
    throw std::runtime_error("state interface value unavailable");
  }
  return *value;
}

void setCommandInterfaceValue(std::vector<hardware_interface::CommandInterface>& interfaces,
                              const std::string& full_name,
                              double value) {
  const auto found = std::find_if(
      interfaces.begin(), interfaces.end(),
      [&full_name](const auto& interface) { return interface.get_name() == full_name; });
  if (found == interfaces.end() || !found->template set_value<double>(value)) {
    throw std::runtime_error("command interface could not be set");
  }
}

franka::RobotState makeDistinctState(uint8_t arm_marker, uint64_t timestamp_ms, double base) {
  auto state = test_support::makeSyntheticRobotState(arm_marker, timestamp_ms);
  for (size_t index = 0; index < 7; ++index) {
    state.q.at(index) = base + static_cast<double>(index);
    state.dq.at(index) = base + 10.0 + static_cast<double>(index);
    state.tau_J.at(index) = base + 20.0 + static_cast<double>(index);
  }
  for (size_t index = 0; index < 16; ++index) {
    state.O_T_EE.at(index) = base + 30.0 + static_cast<double>(index);
    state.O_T_EE_d.at(index) = base + 50.0 + static_cast<double>(index);
  }
  return state;
}

RobotCommand makeDistinctCommand(double base) {
  RobotCommand command;
  for (size_t index = 0; index < 7; ++index) {
    command.efforts.at(index) = base + static_cast<double>(index);
    command.joint_positions.at(index) = base + 10.0 + static_cast<double>(index);
    command.joint_velocities.at(index) = base + 20.0 + static_cast<double>(index);
  }
  for (size_t index = 0; index < 16; ++index) {
    command.cartesian_positions.at(index) = base + 30.0 + static_cast<double>(index);
  }
  for (size_t index = 0; index < 6; ++index) {
    command.cartesian_velocities.at(index) = base + 50.0 + static_cast<double>(index);
  }
  return command;
}

void setArmCommand(std::vector<hardware_interface::CommandInterface>& interfaces,
                   const std::string& arm_name,
                   const RobotCommand& command) {
  static const std::array<std::string, 16> kCartesianMatrixNames{"00", "01", "02", "03", "04", "05",
                                                                 "06", "07", "08", "09", "10", "11",
                                                                 "12", "13", "14", "15"};
  static const std::array<std::string, 6> kCartesianVelocityNames{"tx",      "ty",      "tz",
                                                                  "omega_x", "omega_y", "omega_z"};
  for (size_t joint = 0; joint < FrankaMultiHardwareInterface::kNumberOfJoints; ++joint) {
    const auto prefix = arm_name + "_joint" + std::to_string(joint + 1) + "/";
    setCommandInterfaceValue(interfaces, prefix + "effort", command.efforts.at(joint));
    setCommandInterfaceValue(interfaces, prefix + "position", command.joint_positions.at(joint));
    setCommandInterfaceValue(interfaces, prefix + "velocity", command.joint_velocities.at(joint));
  }
  for (size_t index = 0; index < kCartesianMatrixNames.size(); ++index) {
    setCommandInterfaceValue(interfaces,
                             arm_name + "_ee_cartesian_position/" + kCartesianMatrixNames.at(index),
                             command.cartesian_positions.at(index));
  }
  for (size_t index = 0; index < kCartesianVelocityNames.size(); ++index) {
    setCommandInterfaceValue(
        interfaces, arm_name + "_ee_cartesian_velocity/" + kCartesianVelocityNames.at(index),
        command.cartesian_velocities.at(index));
  }
}

void expectArmState(const std::vector<hardware_interface::StateInterface>& interfaces,
                    const std::string& arm_name,
                    const franka::RobotState& expected) {
  static const std::array<std::string, 16> kCartesianMatrixNames{"00", "01", "02", "03", "04", "05",
                                                                 "06", "07", "08", "09", "10", "11",
                                                                 "12", "13", "14", "15"};
  for (size_t joint = 0; joint < FrankaMultiHardwareInterface::kNumberOfJoints; ++joint) {
    const auto prefix = arm_name + "_joint" + std::to_string(joint + 1) + "/";
    EXPECT_DOUBLE_EQ(stateInterfaceValue(interfaces, prefix + "position"), expected.q.at(joint));
    EXPECT_DOUBLE_EQ(stateInterfaceValue(interfaces, prefix + "velocity"), expected.dq.at(joint));
    EXPECT_DOUBLE_EQ(stateInterfaceValue(interfaces, prefix + "effort"), expected.tau_J.at(joint));
  }
  for (size_t index = 0; index < kCartesianMatrixNames.size(); ++index) {
    EXPECT_DOUBLE_EQ(stateInterfaceValue(interfaces, arm_name + "_ee_cartesian_position/" +
                                                         kCartesianMatrixNames.at(index)),
                     expected.O_T_EE.at(index));
    EXPECT_DOUBLE_EQ(stateInterfaceValue(interfaces, arm_name + "_ee_cartesian_velocity/" +
                                                         kCartesianMatrixNames.at(index)),
                     expected.O_T_EE_d.at(index));
  }
}

template <typename PointerType>
PointerType decodePointerInterface(
    const std::vector<hardware_interface::StateInterface>& interfaces,
    const std::string& full_name) {
  static_assert(sizeof(PointerType) == sizeof(double));
  const double encoded = stateInterfaceValue(interfaces, full_name);
  PointerType pointer = nullptr;
  std::memcpy(&pointer, &encoded, sizeof(pointer));
  return pointer;
}

std::vector<std::string> effortInterfaces(const std::string& arm_name) {
  std::vector<std::string> interfaces;
  for (size_t joint = 1; joint <= FrankaMultiHardwareInterface::kNumberOfJoints; ++joint) {
    interfaces.push_back(arm_name + "_joint" + std::to_string(joint) + "/effort");
  }
  return interfaces;
}

class ScopedLoggerLevel {
 public:
  ScopedLoggerLevel(const char* logger_name, int temporary_level)
      : logger_name_(logger_name),
        previous_level_(rcutils_logging_get_logger_effective_level(logger_name)) {
    if (rcutils_logging_set_logger_level(logger_name_, temporary_level) != RCUTILS_RET_OK) {
      throw std::runtime_error("failed to set test logger level");
    }
  }

  ScopedLoggerLevel(const ScopedLoggerLevel&) = delete;
  ScopedLoggerLevel& operator=(const ScopedLoggerLevel&) = delete;

  ~ScopedLoggerLevel() {
    [[maybe_unused]] const auto restored =
        rcutils_logging_set_logger_level(logger_name_, previous_level_);
  }

 private:
  const char* logger_name_;
  int previous_level_;
};

std::string applyGeneratedMetadataMutation(hardware_interface::HardwareInfo& info,
                                           std::size_t category,
                                           std::mt19937_64& engine) {
  const auto joint = static_cast<std::size_t>(engine() % info.joints.size());
  switch (category) {
    case 0:
      info.hardware_parameters.erase("robot_count");
      return "erase robot_count";
    case 1: {
      constexpr std::array<const char*, 8> kBadCounts{"", "0", "3", "-1", "01", "1x", " 2", "2 "};
      info.hardware_parameters["robot_count"] = kBadCounts[engine() % kBadCounts.size()];
      return "invalid robot_count=" + info.hardware_parameters.at("robot_count");
    }
    case 2:
      info.hardware_parameters.erase("ns_" + std::to_string(1U + engine() % 2U));
      return "erase namespace";
    case 3: {
      constexpr std::array<const char*, 7> kBadNames{
          "", "_panda", "1panda", "pan-da", "panda space", "panda/slash", "pand.a"};
      const auto slot = 1U + engine() % 2U;
      info.hardware_parameters["ns_" + std::to_string(slot)] =
          kBadNames[engine() % kBadNames.size()];
      return "invalid namespace slot=" + std::to_string(slot);
    }
    case 4:
      info.hardware_parameters["ns_1"] =
          "p" + std::string(FrankaMultiHardwareInterface::kMaximumArmIdentifierLength, 'a');
      return "namespace over maximum length";
    case 5:
      info.hardware_parameters["ns_2"] = info.hardware_parameters.at("ns_1");
      return "duplicate namespace";
    case 6:
      info.hardware_parameters.erase("robot_ip_" + std::to_string(1U + engine() % 2U));
      return "erase address";
    case 7:
      info.hardware_parameters["robot_ip_" + std::to_string(1U + engine() % 2U)].clear();
      return "empty address";
    case 8:
      info.joints.erase(info.joints.begin() + static_cast<std::ptrdiff_t>(joint));
      return "remove joint index=" + std::to_string(joint);
    case 9:
      info.joints.push_back(info.joints[joint]);
      return "add joint index=" + std::to_string(joint);
    case 10:
      info.joints[joint].name = "panda1_joint8";
      return "unknown joint index=" + std::to_string(joint);
    case 11:
      info.joints[joint].name = info.joints[(joint + 1U) % info.joints.size()].name;
      return "duplicate joint index=" + std::to_string(joint);
    case 12:
      info.joints[joint].type = "sensor";
      return "wrong component type index=" + std::to_string(joint);
    case 13:
      info.joints[joint].command_interfaces.pop_back();
      return "remove command interface index=" + std::to_string(joint);
    case 14:
      info.joints[joint].state_interfaces.pop_back();
      return "remove state interface index=" + std::to_string(joint);
    case 15:
      info.joints[joint].command_interfaces[2] = info.joints[joint].command_interfaces[1];
      return "duplicate command interface index=" + std::to_string(joint);
    case 16:
      info.joints[joint].state_interfaces[2] = info.joints[joint].state_interfaces[1];
      return "duplicate state interface index=" + std::to_string(joint);
    case 17:
      info.joints[joint].command_interfaces[engine() % 3U].name = "temperature";
      return "unknown command interface index=" + std::to_string(joint);
    case 18:
      info.joints[joint].state_interfaces[engine() % 3U].name = "temperature";
      return "unknown state interface index=" + std::to_string(joint);
    case 19:
      info.joints[joint].command_interfaces[engine() % 3U].data_type = "float";
      return "wrong command data type index=" + std::to_string(joint);
    case 20:
      info.joints[joint].state_interfaces[engine() % 3U].data_type = "float";
      return "wrong state data type index=" + std::to_string(joint);
    case 21:
      info.joints[joint].command_interfaces.push_back(makeInterface("temperature"));
      return "extra command interface index=" + std::to_string(joint);
    case 22:
      info.joints[joint].state_interfaces.push_back(makeInterface("temperature"));
      return "extra state interface index=" + std::to_string(joint);
    default:
      info.hardware_parameters["robot_count"] = "1";
      return "declared one arm with two-arm metadata";
  }
}

TEST(FrankaMultiHardwareInterfaceMetadataTest, RejectsEveryMalformedShapeBeforeFactoryCall) {
  std::vector<std::pair<std::string, hardware_interface::HardwareInfo>> invalid_cases;
  const auto add_case = [&invalid_cases](const std::string& name,
                                         hardware_interface::HardwareInfo info) {
    invalid_cases.emplace_back(name, std::move(info));
  };

  auto info = makeHardwareInfo(1);
  info.hardware_parameters.erase("robot_count");
  add_case("missing count", info);
  for (const auto& count : {"", "0", "3", "-1", "1x", " 1", "1 "}) {
    info = makeHardwareInfo(1);
    info.hardware_parameters["robot_count"] = count;
    add_case("invalid count " + std::string(count), info);
  }
  info = makeHardwareInfo(1);
  info.hardware_parameters.erase("ns_1");
  add_case("missing arm name", info);
  for (const auto& arm_name : {"", "_panda", "1panda", "pan-da", "panda space", "pand\xC3\xA4"}) {
    info = makeHardwareInfo(1);
    info.hardware_parameters["ns_1"] = arm_name;
    add_case("invalid arm name", info);
  }
  info = makeHardwareInfo(2);
  info.hardware_parameters["ns_2"] = info.hardware_parameters["ns_1"];
  add_case("duplicate arm name", info);
  info = makeHardwareInfo(1);
  info.hardware_parameters.erase("robot_ip_1");
  add_case("missing address", info);
  info = makeHardwareInfo(1);
  info.hardware_parameters["robot_ip_1"] = "";
  add_case("empty address", info);
  info = makeHardwareInfo(1);
  info.joints.pop_back();
  add_case("missing joint", info);
  info = makeHardwareInfo(1);
  info.joints[0].name = "panda1_joint8";
  add_case("unknown joint", info);
  info = makeHardwareInfo(1);
  info.joints[0].name = info.joints[1].name;
  add_case("duplicate joint", info);
  info = makeHardwareInfo(1);
  info.joints[0].type = "sensor";
  add_case("wrong component type", info);

  for (const bool command : {false, true}) {
    info = makeHardwareInfo(1);
    auto& interfaces =
        command ? info.joints[0].command_interfaces : info.joints[0].state_interfaces;
    interfaces.pop_back();
    add_case(command ? "missing command interface" : "missing state interface", info);
    info = makeHardwareInfo(1);
    auto& duplicate = command ? info.joints[0].command_interfaces : info.joints[0].state_interfaces;
    duplicate[2] = duplicate[1];
    add_case(command ? "duplicate command interface" : "duplicate state interface", info);
    info = makeHardwareInfo(1);
    auto& unknown = command ? info.joints[0].command_interfaces : info.joints[0].state_interfaces;
    unknown[2].name = "temperature";
    add_case(command ? "unknown command interface" : "unknown state interface", info);
    info = makeHardwareInfo(1);
    auto& wrong_type =
        command ? info.joints[0].command_interfaces : info.joints[0].state_interfaces;
    wrong_type[1].data_type = "float";
    add_case(command ? "wrong command data type" : "wrong state data type", info);
    info = makeHardwareInfo(1);
    auto& extra = command ? info.joints[0].command_interfaces : info.joints[0].state_interfaces;
    extra.push_back(makeInterface("temperature"));
    add_case(command ? "extra command interface" : "extra state interface", info);
  }

  for (const auto& invalid_case : invalid_cases) {
    SCOPED_TRACE(invalid_case.first);
    size_t factory_calls = 0;
    FrankaMultiHardwareInterface hardware(
        [&factory_calls](const std::string&, const std::string&,
                         const rclcpp::Logger&) -> std::shared_ptr<FrankaArmBackend> {
          ++factory_calls;
          return nullptr;
        });
    EXPECT_EQ(hardware.on_init(invalid_case.second), CallbackReturn::ERROR);
    EXPECT_EQ(factory_calls, 0U);
    EXPECT_EQ(hardware.robot_count_, 0U);
  }
}

TEST(FrankaMultiHardwareInterfaceMetadataTest,
     ArmIdentifierLengthBoundaryPassesValidationAndNextCharacterIsRejected) {
  EXPECT_EQ(FrankaMultiHardwareInterface::kMaximumArmIdentifierLength, 64U);
  EXPECT_EQ(FrankaMultiHardwareInterface::kMaximumJointModeInterfaceNameLength, 80U);
  EXPECT_EQ(FrankaMultiHardwareInterface::kMaximumDerivedInterfaceNameLength, 94U);

  const auto replace_arm_name = [](hardware_interface::HardwareInfo& info,
                                   const std::string& arm_name) {
    info.hardware_parameters["ns_1"] = arm_name;
    for (size_t joint_index = 0; joint_index < info.joints.size(); ++joint_index) {
      info.joints.at(joint_index).name = arm_name + "_joint" + std::to_string(joint_index + 1);
    }
  };
  const auto factory_call_count = [&](const std::string& arm_name) {
    auto info = makeHardwareInfo(1);
    replace_arm_name(info, arm_name);
    size_t calls = 0;
    FrankaMultiHardwareInterface hardware(
        [&calls](const std::string&, const std::string&,
                 const rclcpp::Logger&) -> std::shared_ptr<FrankaArmBackend> {
          ++calls;
          return nullptr;
        });
    EXPECT_EQ(hardware.on_init(info), CallbackReturn::ERROR);
    return calls;
  };

  EXPECT_EQ(factory_call_count(
                std::string(FrankaMultiHardwareInterface::kMaximumArmIdentifierLength, 'a')),
            1U);
  EXPECT_EQ(factory_call_count(
                std::string(FrankaMultiHardwareInterface::kMaximumArmIdentifierLength + 1, 'a')),
            0U);
}

TEST(FrankaMultiHardwareInterfaceMetadataTest,
     FixedSeedMalformedMetadataNeverReachesFactoryAndFullyRollsBack) {
  RclcppScope rclcpp_scope;
  ScopedLoggerLevel quiet_invalid_metadata{"FrankaMultiHardwareInterface",
                                           RCUTILS_LOG_SEVERITY_FATAL};
  constexpr std::array<std::uint64_t, 3> kSeeds{0x4d455441U, 0x8badf00dU, 0x726f6c6c6261636bULL};
  constexpr std::size_t kCasesPerSeed = 4000;

  FactoryHarness harness(2);
  FrankaMultiHardwareInterface hardware(harness.factory());
  for (const auto seed : kSeeds) {
    std::cout << "Franka metadata property seed=" << seed << " cases=" << kCasesPerSeed << '\n';
    std::mt19937_64 engine(seed);
    for (std::size_t case_index = 0; case_index < kCasesPerSeed; ++case_index) {
      auto info = makeHardwareInfo(2);
      const auto category = case_index % 24U;
      const auto operation = applyGeneratedMetadataMutation(info, category, engine);
      SCOPED_TRACE("seed=" + std::to_string(seed) + " case=" + std::to_string(case_index) +
                   " operation=" + operation);

      ASSERT_EQ(hardware.on_init(info), CallbackReturn::ERROR);
      ASSERT_TRUE(harness.calls.empty());
      ASSERT_EQ(hardware.robot_count_, 0U);
      EXPECT_TRUE(hardware.export_state_interfaces().empty());
      EXPECT_TRUE(hardware.export_command_interfaces().empty());
      EXPECT_FALSE(hardware.globalFaultDiagnostic().latched());
    }
  }

  ASSERT_EQ(hardware.on_init(makeHardwareInfo(2)), CallbackReturn::SUCCESS);
  ASSERT_EQ(harness.calls.size(), 2U);
  EXPECT_EQ(harness.calls[0].first, "panda1");
  EXPECT_EQ(harness.calls[1].first, "panda2");
  EXPECT_EQ(hardware.robot_count_, 2U);
  EXPECT_EQ(hardware.export_state_interfaces().size(), 110U);
  EXPECT_EQ(hardware.export_command_interfaces().size(), 86U);
}

TEST(FrankaMultiHardwareInterfaceInitializationTest, ExportsExactReviewedDualArmInterfaceContract) {
  RclcppScope rclcpp_scope;
  const auto contract = YAML::LoadFile(FRANKA_HARDWARE_INTERFACE_CONTRACT_PATH);
  ASSERT_TRUE(contract.IsMap());
  ASSERT_EQ(contract["schema_version"].as<int>(), 1);

  const auto sequence = [&contract](const char* key, size_t expected_size) {
    const auto values = contract[key];
    if (!values.IsSequence() || values.size() != expected_size) {
      throw std::runtime_error(std::string("invalid interface contract sequence: ") + key);
    }
    std::vector<std::string> result;
    result.reserve(values.size());
    for (const auto& value : values) {
      result.push_back(value.as<std::string>());
    }
    if (std::set<std::string>(result.begin(), result.end()).size() != result.size()) {
      throw std::runtime_error(std::string("duplicate interface contract value: ") + key);
    }
    return result;
  };

  const auto arm_ids = sequence("arm_ids", 2);
  const auto joint_commands = sequence("joint_command_interfaces", 3);
  const auto joint_states = sequence("joint_state_interfaces", 3);
  const auto matrix_names = sequence("cartesian_matrix_interfaces", 16);
  const auto cartesian_velocity_names = sequence("cartesian_velocity_command_interfaces", 6);
  const auto pointer_state_names = sequence("pointer_state_interfaces", 2);
  EXPECT_EQ(arm_ids, (std::vector<std::string>{"panda1", "panda2"}));

  std::set<std::string> expected_commands;
  std::set<std::string> expected_states;
  for (const auto& arm_id : arm_ids) {
    for (size_t joint = 1; joint <= FrankaMultiHardwareInterface::kNumberOfJoints; ++joint) {
      const auto joint_prefix = arm_id + "_joint" + std::to_string(joint) + "/";
      for (const auto& name : joint_commands) {
        expected_commands.insert(joint_prefix + name);
      }
      for (const auto& name : joint_states) {
        expected_states.insert(joint_prefix + name);
      }
    }
    for (const auto& name : matrix_names) {
      expected_commands.insert(arm_id + "_ee_cartesian_position/" + name);
      expected_states.insert(arm_id + "_ee_cartesian_position/" + name);
      expected_states.insert(arm_id + "_ee_cartesian_velocity/" + name);
    }
    for (const auto& name : cartesian_velocity_names) {
      expected_commands.insert(arm_id + "_ee_cartesian_velocity/" + name);
    }
    for (const auto& name : pointer_state_names) {
      expected_states.insert(arm_id + "/" + name);
    }
  }

  FactoryHarness harness(2);
  FrankaMultiHardwareInterface hardware(harness.factory());
  ASSERT_EQ(hardware.on_init(makeHardwareInfo(2)), CallbackReturn::SUCCESS);
  const auto exported_commands = hardware.export_command_interfaces();
  const auto exported_states = hardware.export_state_interfaces();
  std::set<std::string> actual_commands;
  std::set<std::string> actual_states;
  for (const auto& interface : exported_commands) {
    actual_commands.insert(interface.get_name());
  }
  for (const auto& interface : exported_states) {
    actual_states.insert(interface.get_name());
  }
  ASSERT_EQ(actual_commands.size(), exported_commands.size());
  ASSERT_EQ(actual_states.size(), exported_states.size());
  EXPECT_EQ(actual_commands, expected_commands);
  EXPECT_EQ(actual_states, expected_states);
}

TEST(FrankaMultiHardwareInterfaceInitializationTest,
     AcceptsReorderedInterfacesAndPreservesPerArmStateAndModelOwnership) {
  RclcppScope rclcpp_scope;
  for (const size_t arm_count : {1U, 2U}) {
    SCOPED_TRACE(arm_count);
    auto info = makeHardwareInfo(arm_count);
    std::reverse(info.joints.begin(), info.joints.end());
    for (auto& joint : info.joints) {
      std::reverse(joint.command_interfaces.begin(), joint.command_interfaces.end());
      std::rotate(joint.state_interfaces.begin(), joint.state_interfaces.begin() + 1,
                  joint.state_interfaces.end());
    }
    FactoryHarness harness(arm_count);
    FrankaMultiHardwareInterface hardware(harness.factory());
    ASSERT_EQ(hardware.on_init(info), CallbackReturn::SUCCESS);
    EXPECT_EQ(hardware.robot_count_, arm_count);
    ASSERT_EQ(harness.calls.size(), arm_count);

    const auto state_interfaces = hardware.export_state_interfaces();
    for (size_t arm_index = 1; arm_index <= arm_count; ++arm_index) {
      const auto arm_name = "panda" + std::to_string(arm_index);
      EXPECT_EQ(harness.calls[arm_index - 1].first, arm_name);
      EXPECT_EQ(harness.calls[arm_index - 1].second,
                "offline-placeholder-" + std::to_string(arm_index));
      const auto expected_state =
          test_support::makeSyntheticRobotState(static_cast<uint8_t>(arm_index), arm_index * 1000);
      EXPECT_DOUBLE_EQ(stateInterfaceValue(state_interfaces, arm_name + "_joint1/position"),
                       expected_state.q[0]);
      auto* state =
          decodePointerInterface<franka::RobotState*>(state_interfaces, arm_name + "/robot_state");
      ASSERT_NE(state, nullptr);
      EXPECT_DOUBLE_EQ(state->q[0], expected_state.q[0]);
      auto* model = decodePointerInterface<ModelBase*>(state_interfaces, arm_name + "/robot_model");
      auto* synthetic_model = dynamic_cast<test_support::SyntheticModel*>(model);
      ASSERT_NE(synthetic_model, nullptr);
      EXPECT_EQ(synthetic_model->armMarker(), arm_index);
    }
  }
}

TEST(FrankaMultiHardwareInterfaceInitializationTest,
     ConstructionReadAndModelFailuresReleaseEveryStagedBackend) {
  RclcppScope rclcpp_scope;
  struct Scenario {
    std::string name;
    size_t arm_count;
    std::string failing_arm;
    SyntheticFailurePoint failure_point;
    bool null_backend;
    bool null_model;
  };
  const std::vector<Scenario> scenarios{
      {"arm1 construction", 1, "panda1", SyntheticFailurePoint::Construction, false, false},
      {"arm2 construction", 2, "panda2", SyntheticFailurePoint::Construction, false, false},
      {"arm1 null backend", 1, "panda1", SyntheticFailurePoint::None, true, false},
      {"arm2 null backend", 2, "panda2", SyntheticFailurePoint::None, true, false},
      {"arm1 initial read", 1, "panda1", SyntheticFailurePoint::InitialRead, false, false},
      {"arm2 initial read", 2, "panda2", SyntheticFailurePoint::InitialRead, false, false},
      {"arm1 null model", 1, "panda1", SyntheticFailurePoint::None, false, true},
      {"arm2 null model", 2, "panda2", SyntheticFailurePoint::None, false, true},
      {"arm1 invalid initial state", 1, "panda1", SyntheticFailurePoint::InvalidNanState, false,
       false},
      {"arm2 invalid initial state", 2, "panda2", SyntheticFailurePoint::InvalidInfiniteState,
       false, false},
  };

  for (const auto& scenario : scenarios) {
    SCOPED_TRACE(scenario.name);
    FactoryHarness harness(scenario.arm_count);
    if (scenario.failure_point != SyntheticFailurePoint::None) {
      harness.configurations.at(scenario.failing_arm).failure = {scenario.failure_point, 1};
    }
    if (scenario.null_backend) {
      harness.null_backends.insert(scenario.failing_arm);
    }
    if (scenario.null_model) {
      harness.null_models.insert(scenario.failing_arm);
    }
    FrankaMultiHardwareInterface hardware(harness.factory());
    EXPECT_EQ(hardware.on_init(makeHardwareInfo(scenario.arm_count)), CallbackReturn::ERROR);
    EXPECT_EQ(hardware.robot_count_, 0U);
    for (const auto& backend : harness.returned_backends) {
      EXPECT_TRUE(backend.second.expired()) << backend.first;
    }
    for (const auto& backend : harness.synthetic_backends) {
      EXPECT_TRUE(backend.second.expired()) << backend.first;
    }
  }
}

TEST(FrankaMultiHardwareInterfaceInitializationTest,
     FailedInitializationIsRetryableAndSuccessfulInitializationIsNot) {
  RclcppScope rclcpp_scope;
  FactoryHarness harness(2);
  harness.configurations.at("panda2").failure = {SyntheticFailurePoint::InitialRead, 1};
  FrankaMultiHardwareInterface hardware(harness.factory());
  const auto info = makeHardwareInfo(2);

  EXPECT_EQ(hardware.on_init(info), CallbackReturn::ERROR);
  EXPECT_EQ(hardware.robot_count_, 0U);
  for (const auto& backend : harness.returned_backends) {
    EXPECT_TRUE(backend.second.expired());
  }

  harness.configurations.at("panda2").failure = {};
  EXPECT_EQ(hardware.on_init(info), CallbackReturn::SUCCESS);
  EXPECT_EQ(hardware.robot_count_, 2U);
  const auto calls_after_success = harness.calls.size();
  EXPECT_EQ(hardware.on_init(info), CallbackReturn::ERROR);
  EXPECT_EQ(harness.calls.size(), calls_after_success);
}

TEST(FrankaMultiHardwareInterfaceInitializationTest, DestructionReleasesCommittedBackends) {
  RclcppScope rclcpp_scope;
  FactoryHarness harness(2);
  {
    FrankaMultiHardwareInterface hardware(harness.factory());
    ASSERT_EQ(hardware.on_init(makeHardwareInfo(2)), CallbackReturn::SUCCESS);
    ASSERT_FALSE(harness.returned_backends.at("panda1").expired());
    ASSERT_FALSE(harness.returned_backends.at("panda2").expired());
  }
  EXPECT_TRUE(harness.returned_backends.at("panda1").expired());
  EXPECT_TRUE(harness.returned_backends.at("panda2").expired());
}

TEST(FrankaMultiHardwareInterfaceInitializationTest,
     EveryPostConstructionCheckpointCleansUpAndIsRetryable) {
  RclcppScope rclcpp_scope;
  struct Scenario {
    std::string name;
    InitializationStage stage;
    size_t arm_slot;
    size_t occurrence;
  };
  const std::vector<Scenario> scenarios{
      {"arm1 recovery service", InitializationStage::ErrorRecoveryServiceConstruction, 1, 0},
      {"arm2 recovery service", InitializationStage::ErrorRecoveryServiceConstruction, 2, 0},
      {"arm1 parameter service", InitializationStage::ParameterServiceConstruction, 1, 0},
      {"arm2 parameter service", InitializationStage::ParameterServiceConstruction, 2, 0},
      {"diagnostics", InitializationStage::DiagnosticsConstruction,
       InitializationCheckpoint::kNoArmSlot, 0},
      {"executor", InitializationStage::ExecutorConstruction, InitializationCheckpoint::kNoArmSlot,
       0},
      {"arm1 recovery registration", InitializationStage::ServiceRegistration, 1, 0},
      {"arm2 parameter registration", InitializationStage::ServiceRegistration, 2, 1},
      {"diagnostics registration", InitializationStage::DiagnosticsRegistration,
       InitializationCheckpoint::kNoArmSlot, 0},
      {"base initialization", InitializationStage::BaseInitialization,
       InitializationCheckpoint::kNoArmSlot, 0},
  };

  for (const auto& scenario : scenarios) {
    SCOPED_TRACE(scenario.name);
    FactoryHarness harness(2);
    bool inject_failure = true;
    FrankaMultiHardwareInterface hardware(
        harness.factory(),
        [&scenario, &inject_failure](const InitializationCheckpoint& checkpoint) {
          if (inject_failure && checkpoint.stage == scenario.stage &&
              checkpoint.arm_slot == scenario.arm_slot &&
              checkpoint.occurrence == scenario.occurrence) {
            throw std::runtime_error("injected initialization checkpoint failure");
          }
        });

    EXPECT_EQ(hardware.on_init(makeHardwareInfo(2)), CallbackReturn::ERROR);
    EXPECT_EQ(hardware.robot_count_, 0U);
    for (const auto& backend : harness.returned_backends) {
      EXPECT_TRUE(backend.second.expired()) << backend.first;
    }
    for (const auto& backend : harness.synthetic_backends) {
      EXPECT_TRUE(backend.second.expired()) << backend.first;
    }

    inject_failure = false;
    EXPECT_EQ(hardware.on_init(makeHardwareInfo(2)), CallbackReturn::SUCCESS);
    EXPECT_EQ(hardware.robot_count_, 2U);
  }
}

TEST(FrankaMultiHardwareInterfaceActivationTest,
     PublishesSafeCommandsThenStartsAndReadsEveryArmBeforeCommit) {
  RclcppScope rclcpp_scope;
  FactoryHarness harness(2);
  auto arm1_activation_state = test_support::makeSyntheticRobotState(1, 1001);
  arm1_activation_state.q[0] = 0.91;
  harness.configurations.at("panda1").replay_states = {arm1_activation_state};
  FrankaMultiHardwareInterface hardware(harness.factory());
  ASSERT_EQ(hardware.on_init(makeHardwareInfo(2)), CallbackReturn::SUCCESS);
  const auto arm1 = harness.backend("panda1");
  const auto arm2 = harness.backend("panda2");
  ASSERT_NE(arm1, nullptr);
  ASSERT_NE(arm2, nullptr);
  const auto arm1_event_start = arm1->capturedEventCount();
  const auto arm2_event_start = arm2->capturedEventCount();

  EXPECT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  EXPECT_EQ(eventKindsSince(*arm1, arm1_event_start),
            (std::vector<SyntheticEventKind>{
                SyntheticEventKind::CommandAccepted, SyntheticEventKind::StartAttempt,
                SyntheticEventKind::Started, SyntheticEventKind::ReadAttempt,
                SyntheticEventKind::StateReturned}));
  EXPECT_EQ(eventKindsSince(*arm2, arm2_event_start),
            (std::vector<SyntheticEventKind>{
                SyntheticEventKind::CommandAccepted, SyntheticEventKind::StartAttempt,
                SyntheticEventKind::Started, SyntheticEventKind::ReadAttempt,
                SyntheticEventKind::StateReturned}));
  ASSERT_EQ(arm1->capturedCommandCount(), 1U);
  const auto safe_command = makeSafeRobotCommand(harness.configurations.at("panda1").initial_state);
  EXPECT_EQ(arm1->capturedCommand(0).joint_positions, safe_command.joint_positions);
  const auto state_interfaces = hardware.export_state_interfaces();
  EXPECT_DOUBLE_EQ(stateInterfaceValue(state_interfaces, "panda1_joint1/position"), 0.91);
  EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST(FrankaMultiHardwareInterfaceActivationTest,
     Arm2InvalidStateRollsBackWithoutCommittingArm1Candidate) {
  RclcppScope rclcpp_scope;
  FactoryHarness harness(2);
  auto arm1_activation_state = test_support::makeSyntheticRobotState(1, 1001);
  arm1_activation_state.q[0] = 0.91;
  harness.configurations.at("panda1").replay_states = {arm1_activation_state};
  harness.configurations.at("panda2").failure = {SyntheticFailurePoint::InvalidNanState, 2};
  harness.configurations.at("panda2").replay_states = {
      test_support::makeSyntheticRobotState(2, 2001),
      test_support::makeSyntheticRobotState(2, 2002)};
  FrankaMultiHardwareInterface hardware(harness.factory());
  ASSERT_EQ(hardware.on_init(makeHardwareInfo(2)), CallbackReturn::SUCCESS);

  EXPECT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::FAILURE);
  const auto state_interfaces = hardware.export_state_interfaces();
  EXPECT_DOUBLE_EQ(stateInterfaceValue(state_interfaces, "panda1_joint1/position"),
                   harness.configurations.at("panda1").initial_state.q[0]);
  EXPECT_TRUE(harness.backend("panda1")->diagnostics().stopped);
  EXPECT_TRUE(harness.backend("panda2")->diagnostics().stopped);
  EXPECT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST(FrankaMultiHardwareInterfaceActivationTest,
     DeterministicFailuresStopEveryArmAndRecoveryGatesStartupRetry) {
  RclcppScope rclcpp_scope;
  struct Scenario {
    std::string name;
    std::string arm;
    SyntheticFailurePoint point;
    uint64_t fail_on_call;
    bool retryable;
  };
  std::vector<Scenario> scenarios;
  for (const auto& arm : {std::string("panda1"), std::string("panda2")}) {
    scenarios.push_back({arm + " publish", arm, SyntheticFailurePoint::CommandPublish, 1, true});
    scenarios.push_back({arm + " queue", arm, SyntheticFailurePoint::QueueSaturation, 1, true});
    scenarios.push_back({arm + " start", arm, SyntheticFailurePoint::StartStateReading, 1, true});
    scenarios.push_back({arm + " read fault", arm, SyntheticFailurePoint::ReadFault, 1, false});
    scenarios.push_back({arm + " nan", arm, SyntheticFailurePoint::InvalidNanState, 2, true});
    scenarios.push_back(
        {arm + " infinite", arm, SyntheticFailurePoint::InvalidInfiniteState, 2, true});
  }

  for (const auto& scenario : scenarios) {
    SCOPED_TRACE(scenario.name);
    FactoryHarness harness(2);
    harness.configurations.at(scenario.arm).failure = {scenario.point, scenario.fail_on_call};
    if (scenario.point == SyntheticFailurePoint::InvalidNanState ||
        scenario.point == SyntheticFailurePoint::InvalidInfiniteState) {
      const auto marker = static_cast<uint8_t>(scenario.arm == "panda1" ? 1 : 2);
      harness.configurations.at(scenario.arm).replay_states = {
          test_support::makeSyntheticRobotState(marker, static_cast<uint64_t>(marker) * 1000 + 1),
          test_support::makeSyntheticRobotState(marker, static_cast<uint64_t>(marker) * 1000 + 2)};
    }
    FrankaMultiHardwareInterface hardware(harness.factory());
    ASSERT_EQ(hardware.on_init(makeHardwareInfo(2)), CallbackReturn::SUCCESS);
    const auto arm1 = harness.backend("panda1");
    const auto arm2 = harness.backend("panda2");
    ASSERT_NE(arm1, nullptr);
    ASSERT_NE(arm2, nullptr);
    const auto arm1_start = arm1->capturedEventCount();
    const auto arm2_start = arm2->capturedEventCount();

    EXPECT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::FAILURE);
    EXPECT_TRUE(arm1->diagnostics().stopped);
    EXPECT_TRUE(arm2->diagnostics().stopped);
    const auto arm1_events = eventKindsSince(*arm1, arm1_start);
    const auto arm2_events = eventKindsSince(*arm2, arm2_start);
    EXPECT_NE(std::find(arm1_events.begin(), arm1_events.end(), SyntheticEventKind::StopAttempt),
              arm1_events.end());
    EXPECT_NE(std::find(arm2_events.begin(), arm2_events.end(), SyntheticEventKind::StopAttempt),
              arm2_events.end());
    if (scenario.retryable) {
      if (scenario.point == SyntheticFailurePoint::StartStateReading) {
        const auto failed_backend = harness.backend(scenario.arm);
        ASSERT_NE(failed_backend, nullptr);
        EXPECT_TRUE(failed_backend->hasFault());
        EXPECT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::FAILURE);
        EXPECT_TRUE(failed_backend->hasFault());
        EXPECT_EQ(failed_backend->diagnostics().failure_reason,
                  BackendFailureReason::WorkerStartupFailure);
        ASSERT_TRUE(failed_backend->recoverToReading());
        EXPECT_FALSE(failed_backend->hasFault());
        EXPECT_EQ(failed_backend->diagnostics().failure_reason, BackendFailureReason::None);
      }
      EXPECT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
      EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    }
  }
}

TEST(FrankaMultiHardwareInterfaceActivationTest,
     StartAndReadExceptionsOnEitherArmRollBackAndRemainRetryable) {
  RclcppScope rclcpp_scope;
  for (const bool start_exception : {false, true}) {
    for (const auto& arm_name : {std::string("panda1"), std::string("panda2")}) {
      SCOPED_TRACE(start_exception ? "start exception" : "read exception");
      SCOPED_TRACE(arm_name);
      FactoryHarness harness(2);
      if (start_exception) {
        harness.throwing_start_calls[arm_name] = 1;
      } else {
        harness.throwing_read_calls[arm_name] = 2;
      }
      FrankaMultiHardwareInterface hardware(harness.factory());
      ASSERT_EQ(hardware.on_init(makeHardwareInfo(2)), CallbackReturn::SUCCESS);

      EXPECT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::FAILURE);
      EXPECT_TRUE(harness.backend("panda1")->diagnostics().stopped);
      EXPECT_TRUE(harness.backend("panda2")->diagnostics().stopped);
      EXPECT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
      EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    }
  }
}

TEST(FrankaMultiHardwareInterfaceActivationTest,
     RepeatedPreStartFailuresDoNotAccumulateStoppedCommands) {
  RclcppScope rclcpp_scope;
  constexpr size_t kFailedActivationCount = 70;
  FactoryHarness harness(2);
  harness.start_failures["panda1"] = kFailedActivationCount;
  harness.tracked_command_capacities["panda1"] = 1;
  FrankaMultiHardwareInterface hardware(harness.factory());
  ASSERT_EQ(hardware.on_init(makeHardwareInfo(2)), CallbackReturn::SUCCESS);

  for (size_t attempt = 0; attempt < kFailedActivationCount; ++attempt) {
    SCOPED_TRACE(attempt);
    EXPECT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::FAILURE);
  }
  EXPECT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  EXPECT_EQ(harness.backend("panda1")->acceptedCommandCount(), kFailedActivationCount + 1);
  EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST(FrankaMultiHardwareInterfaceActivationTest, CapacityPreflightFailureRollsBackAndCanBeRetried) {
  RclcppScope rclcpp_scope;
  for (const auto& saturated_arm : {std::string("panda1"), std::string("panda2")}) {
    SCOPED_TRACE(saturated_arm);
    FactoryHarness harness(2);
    harness.configurations.at(saturated_arm).command_queue_capacity = 1;
    FrankaMultiHardwareInterface hardware(harness.factory());
    ASSERT_EQ(hardware.on_init(makeHardwareInfo(2)), CallbackReturn::SUCCESS);
    const auto backend = harness.backend(saturated_arm);
    ASSERT_NE(backend, nullptr);
    EXPECT_TRUE(backend->publishCommand(
        makeSafeRobotCommand(harness.configurations.at(saturated_arm).initial_state)));

    EXPECT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::FAILURE);
    EXPECT_TRUE(harness.backend("panda1")->diagnostics().stopped);
    EXPECT_TRUE(harness.backend("panda2")->diagnostics().stopped);
    EXPECT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  }
}

TEST(FrankaMultiHardwareInterfaceActivationTest,
     UnsafeRollbackReturnsErrorAndStillAttemptsEveryArm) {
  RclcppScope rclcpp_scope;
  FactoryHarness harness(2);
  harness.throwing_stops.insert("panda1");
  harness.configurations.at("panda2").failure = {SyntheticFailurePoint::StartStateReading, 1};
  FrankaMultiHardwareInterface hardware(harness.factory());
  ASSERT_EQ(hardware.on_init(makeHardwareInfo(2)), CallbackReturn::SUCCESS);
  const auto arm2 = harness.backend("panda2");
  const auto arm2_event_start = arm2->capturedEventCount();

  EXPECT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
  EXPECT_FALSE(harness.backend("panda1")->diagnostics().stopped);
  EXPECT_TRUE(arm2->diagnostics().stopped);
  const auto arm2_events = eventKindsSince(*arm2, arm2_event_start);
  EXPECT_NE(std::find(arm2_events.begin(), arm2_events.end(), SyntheticEventKind::StopAttempt),
            arm2_events.end());
}

TEST(FrankaMultiHardwareInterfaceActivationTest, RepeatedActivateDeactivateCyclesAreStable) {
  RclcppScope rclcpp_scope;
  FactoryHarness harness(2);
  FrankaMultiHardwareInterface hardware(harness.factory());
  ASSERT_EQ(hardware.on_init(makeHardwareInfo(2)), CallbackReturn::SUCCESS);
  for (size_t cycle = 0; cycle < 10; ++cycle) {
    SCOPED_TRACE(cycle);
    EXPECT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  }
  EXPECT_EQ(harness.backend("panda1")->successfulReadCount(), 11U);
  EXPECT_EQ(harness.backend("panda2")->successfulReadCount(), 11U);
}

TEST(FrankaMultiHardwareInterfaceReadWriteTest, ReadsInConfiguredSlotOrderNotMapOrder) {
  RclcppScope rclcpp_scope;
  auto info = makeHardwareInfo(2);
  info.hardware_parameters["ns_1"] = "zeta";
  info.hardware_parameters["ns_2"] = "alpha";
  for (auto& joint : info.joints) {
    if (joint.name.rfind("panda1_", 0) == 0) {
      joint.name.replace(0, std::string("panda1").size(), "zeta");
    } else if (joint.name.rfind("panda2_", 0) == 0) {
      joint.name.replace(0, std::string("panda2").size(), "alpha");
    }
  }
  FactoryHarness harness(0);
  harness.configurations.emplace("zeta", SyntheticFrankaArmBackendConfig::forArm(1));
  harness.configurations.emplace("alpha", SyntheticFrankaArmBackendConfig::forArm(2));
  harness.trace_reads = true;
  FrankaMultiHardwareInterface hardware(harness.factory());
  ASSERT_EQ(hardware.on_init(info), CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  harness.read_order.clear();

  EXPECT_EQ(hardware.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            hardware_interface::return_type::OK);
  EXPECT_EQ(harness.read_order, (std::vector<std::string>{"zeta", "alpha"}));
  EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST(FrankaMultiHardwareInterfaceReadWriteTest,
     CommitsExactTwoArmStateAndWritesExactCommandsWithoutCrossArmLeakage) {
  RclcppScope rclcpp_scope;
  FactoryHarness harness(2);
  const auto arm1_activation = makeDistinctState(1, 1001, 100.0);
  const auto arm2_activation = makeDistinctState(2, 2001, 200.0);
  const auto arm1_read = makeDistinctState(1, 1002, 300.0);
  const auto arm2_read = makeDistinctState(2, 2002, 400.0);
  harness.configurations.at("panda1").replay_states = {arm1_activation, arm1_read};
  harness.configurations.at("panda2").replay_states = {arm2_activation, arm2_read};
  FrankaMultiHardwareInterface hardware(harness.factory());
  ASSERT_EQ(hardware.on_init(makeHardwareInfo(2)), CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  auto state_interfaces = hardware.export_state_interfaces();
  expectArmState(state_interfaces, "panda1", arm1_activation);
  expectArmState(state_interfaces, "panda2", arm2_activation);

  EXPECT_EQ(hardware.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            hardware_interface::return_type::OK);
  expectArmState(state_interfaces, "panda1", arm1_read);
  expectArmState(state_interfaces, "panda2", arm2_read);
  auto* arm1_state =
      decodePointerInterface<franka::RobotState*>(state_interfaces, "panda1/robot_state");
  auto* arm2_state =
      decodePointerInterface<franka::RobotState*>(state_interfaces, "panda2/robot_state");
  ASSERT_NE(arm1_state, nullptr);
  ASSERT_NE(arm2_state, nullptr);
  EXPECT_EQ(arm1_state->q, arm1_read.q);
  EXPECT_EQ(arm2_state->q, arm2_read.q);

  auto command_interfaces = hardware.export_command_interfaces();
  const auto arm1_command = makeDistinctCommand(500.0);
  const auto arm2_command = makeDistinctCommand(600.0);
  setArmCommand(command_interfaces, "panda1", arm1_command);
  setArmCommand(command_interfaces, "panda2", arm2_command);
  EXPECT_EQ(hardware.write(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            hardware_interface::return_type::OK);
  const auto arm1_backend = harness.backend("panda1");
  const auto arm2_backend = harness.backend("panda2");
  ASSERT_NE(arm1_backend, nullptr);
  ASSERT_NE(arm2_backend, nullptr);
  EXPECT_EQ(arm1_backend->capturedCommand(arm1_backend->capturedCommandCount() - 1).efforts,
            arm1_command.efforts);
  EXPECT_EQ(arm1_backend->capturedCommand(arm1_backend->capturedCommandCount() - 1).joint_positions,
            arm1_command.joint_positions);
  EXPECT_EQ(
      arm1_backend->capturedCommand(arm1_backend->capturedCommandCount() - 1).joint_velocities,
      arm1_command.joint_velocities);
  EXPECT_EQ(
      arm1_backend->capturedCommand(arm1_backend->capturedCommandCount() - 1).cartesian_positions,
      arm1_command.cartesian_positions);
  EXPECT_EQ(
      arm1_backend->capturedCommand(arm1_backend->capturedCommandCount() - 1).cartesian_velocities,
      arm1_command.cartesian_velocities);
  EXPECT_EQ(arm2_backend->capturedCommand(arm2_backend->capturedCommandCount() - 1).efforts,
            arm2_command.efforts);
  EXPECT_EQ(arm2_backend->capturedCommand(arm2_backend->capturedCommandCount() - 1).joint_positions,
            arm2_command.joint_positions);
  EXPECT_EQ(
      arm2_backend->capturedCommand(arm2_backend->capturedCommandCount() - 1).joint_velocities,
      arm2_command.joint_velocities);
  EXPECT_EQ(
      arm2_backend->capturedCommand(arm2_backend->capturedCommandCount() - 1).cartesian_positions,
      arm2_command.cartesian_positions);
  EXPECT_EQ(
      arm2_backend->capturedCommand(arm2_backend->capturedCommandCount() - 1).cartesian_velocities,
      arm2_command.cartesian_velocities);
  EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST(FrankaMultiHardwareInterfaceReadWriteTest,
     ArmLocalInvalidExceptionAndFaultReadsNeverPartiallyCommit) {
  RclcppScope rclcpp_scope;
  struct Scenario {
    std::string name;
    std::string failing_arm;
    SyntheticFailurePoint failure_point;
    uint64_t failure_call;
    bool read_exception;
  };
  const std::vector<Scenario> scenarios{
      {"arm1 nan", "panda1", SyntheticFailurePoint::InvalidNanState, 3, false},
      {"arm2 nan", "panda2", SyntheticFailurePoint::InvalidNanState, 3, false},
      {"arm1 infinity", "panda1", SyntheticFailurePoint::InvalidInfiniteState, 3, false},
      {"arm2 infinity", "panda2", SyntheticFailurePoint::InvalidInfiniteState, 3, false},
      {"arm1 exception", "panda1", SyntheticFailurePoint::None, 0, true},
      {"arm2 exception", "panda2", SyntheticFailurePoint::None, 0, true},
      {"arm1 fault", "panda1", SyntheticFailurePoint::ReadFault, 2, false},
      {"arm2 fault", "panda2", SyntheticFailurePoint::ReadFault, 2, false},
  };

  for (const auto& scenario : scenarios) {
    SCOPED_TRACE(scenario.name);
    FactoryHarness harness(2);
    const auto arm1_activation = makeDistinctState(1, 1001, 100.0);
    const auto arm2_activation = makeDistinctState(2, 2001, 200.0);
    const auto arm1_candidate = makeDistinctState(1, 1002, 300.0);
    const auto arm2_candidate = makeDistinctState(2, 2002, 400.0);
    harness.configurations.at("panda1").replay_states = {arm1_activation, arm1_candidate};
    harness.configurations.at("panda2").replay_states = {arm2_activation, arm2_candidate};
    if (scenario.read_exception) {
      harness.throwing_read_calls[scenario.failing_arm] = 3;
    } else {
      harness.configurations.at(scenario.failing_arm).failure = {scenario.failure_point,
                                                                 scenario.failure_call};
    }
    FrankaMultiHardwareInterface hardware(harness.factory());
    ASSERT_EQ(hardware.on_init(makeHardwareInfo(2)), CallbackReturn::SUCCESS);
    ASSERT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    auto state_interfaces = hardware.export_state_interfaces();

    EXPECT_EQ(hardware.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
              hardware_interface::return_type::ERROR);
    expectArmState(state_interfaces, "panda1", arm1_activation);
    expectArmState(state_interfaces, "panda2", arm2_activation);
    const auto fault = hardware.globalFaultDiagnostic();
    EXPECT_EQ(fault.origin_arm_slot, scenario.failing_arm == "panda1" ? 1 : 2);
    EXPECT_EQ(fault.cause, scenario.failure_point == SyntheticFailurePoint::ReadFault
                               ? GlobalFaultCause::BackendFault
                               : (scenario.read_exception ? GlobalFaultCause::ReadFailure
                                                          : GlobalFaultCause::InvalidState));
    EXPECT_EQ(hardware.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
              hardware_interface::return_type::ERROR);
    EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  }
}

TEST(FrankaMultiHardwareInterfaceDeactivationTest,
     FalseAndThrowingStopsStillAttemptEveryLaterArmAndClearPreparedMode) {
  RclcppScope rclcpp_scope;
  for (const bool throw_on_stop : {false, true}) {
    SCOPED_TRACE(throw_on_stop);
    FactoryHarness harness(2);
    if (throw_on_stop) {
      harness.throwing_stops.insert("panda1");
    } else {
      harness.configurations.at("panda1").failure = {SyntheticFailurePoint::Shutdown, 1};
    }
    FrankaMultiHardwareInterface hardware(harness.factory());
    ASSERT_EQ(hardware.on_init(makeHardwareInfo(2)), CallbackReturn::SUCCESS);
    ASSERT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    const auto starts = effortInterfaces("panda1");
    ASSERT_EQ(hardware.prepare_command_mode_switch(starts, {}),
              hardware_interface::return_type::OK);
    const auto arm2 = harness.backend("panda2");
    const auto arm2_event_start = arm2->capturedEventCount();

    EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
    const auto arm2_events = eventKindsSince(*arm2, arm2_event_start);
    EXPECT_NE(std::find(arm2_events.begin(), arm2_events.end(), SyntheticEventKind::StopAttempt),
              arm2_events.end());
    EXPECT_TRUE(arm2->diagnostics().stopped);
    EXPECT_EQ(hardware.perform_command_mode_switch(starts, {}),
              hardware_interface::return_type::ERROR);
  }
}

TEST(FrankaMultiHardwareInterfaceActivationTest, FailedActivationClearsPreparedModeTransaction) {
  RclcppScope rclcpp_scope;
  FactoryHarness harness(1);
  harness.configurations.at("panda1").failure = {SyntheticFailurePoint::CommandPublish, 1};
  FrankaMultiHardwareInterface hardware(harness.factory());
  ASSERT_EQ(hardware.on_init(makeHardwareInfo(1)), CallbackReturn::SUCCESS);
  const auto starts = effortInterfaces("panda1");
  ASSERT_EQ(hardware.prepare_command_mode_switch(starts, {}), hardware_interface::return_type::OK);
  EXPECT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::FAILURE);
  EXPECT_EQ(hardware.perform_command_mode_switch(starts, {}),
            hardware_interface::return_type::ERROR);
}

}  // namespace
}  // namespace franka_hardware
