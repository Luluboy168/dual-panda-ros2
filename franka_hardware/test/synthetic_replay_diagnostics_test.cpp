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
#include <cstdint>
#include <fstream>
#include <iterator>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "franka_hardware/real/franka_hardware_diagnostics_node.hpp"
#include "support/synthetic_replay.hpp"

namespace franka_hardware {
namespace {

using test_support::kSyntheticReplayArmCount;
using test_support::kSyntheticReplayFieldCount;
using test_support::kSyntheticReplayMaximumFrames;
using test_support::parseSyntheticReplay;
using test_support::replayBackendConfig;
using test_support::SyntheticCondition;
using test_support::SyntheticFailurePoint;
using test_support::SyntheticFrankaArmBackend;
using test_support::SyntheticReplayError;

static_assert(kSyntheticReplayArmCount == 2);
static_assert(kSyntheticReplayFieldCount == 44);
static_assert(kSyntheticReplayMaximumFrames == SyntheticFrankaArmBackend::kMaximumReplayStates);

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

std::string loadFixture() {
  std::ifstream stream(SYNTHETIC_REPLAY_FIXTURE_PATH, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("failed to open synthetic replay fixture");
  }
  return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

std::string frameLine(uint64_t timestamp,
                      size_t numeric_field_count = 42,
                      const std::string& final_value = "0") {
  std::ostringstream stream;
  stream << "frame," << timestamp;
  for (size_t index = 0; index < numeric_field_count; ++index) {
    stream << ',' << (index + 1 == numeric_field_count ? final_value : "0");
  }
  return stream.str();
}

std::string replayText(size_t frame_count) {
  std::ostringstream stream;
  stream << "multipanda_synthetic_replay_v1\narms,panda1,panda2\n";
  for (size_t index = 0; index < frame_count; ++index) {
    stream << frameLine(1'000'000'000U + index) << '\n';
  }
  return stream.str();
}

void expectRejected(const std::string& content, SyntheticReplayError error) {
  const auto result = parseSyntheticReplay(content);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, error);
}

const std::string& valueFor(const diagnostic_msgs::msg::DiagnosticStatus& status,
                            const std::string& key) {
  const auto found = std::find_if(status.values.begin(), status.values.end(),
                                  [&key](const auto& value) { return value.key == key; });
  if (found == status.values.end()) {
    throw std::out_of_range("diagnostic key not found: " + key);
  }
  return found->value;
}

diagnostic_msgs::msg::DiagnosticStatus format(const FrankaArmDiagnosticSnapshot& snapshot,
                                              uint64_t now_ns) {
  diagnostic_updater::DiagnosticStatusWrapper wrapper;
  formatFrankaArmDiagnosticStatus(snapshot, now_ns, wrapper);
  return static_cast<const diagnostic_msgs::msg::DiagnosticStatus&>(wrapper);
}

FrankaArmDiagnosticSnapshot snapshotFor(const std::string& arm_id,
                                        const FrankaArmBackendDiagnostics& backend,
                                        HardwareLifecycleSnapshot lifecycle = {3, "active"}) {
  FrankaArmDiagnosticSnapshot snapshot;
  snapshot.arm_id = arm_id;
  snapshot.hardware_lifecycle = std::move(lifecycle);
  snapshot.backend = backend;
  snapshot.provenance = {
      "0123456789abcdef0123456789abcdef01234567", false, "jazzy", "28.1.21", "4.45.2", "0.9.2"};
  return snapshot;
}

const std::set<std::string>& expectedDiagnosticKeys() {
  static const std::set<std::string> keys{
      "accepted_state_samples",
      "active_mode",
      "arm_id",
      "command_queue_saturated",
      "dropped_state_samples",
      "failure_reason",
      "fault_category",
      "global_fault_cause",
      "global_fault_origin_arm_id",
      "global_fault_origin_arm_slot",
      "hardware_lifecycle_id",
      "hardware_lifecycle_label",
      "last_recovery_result",
      "libfranka_version",
      "rclcpp_version",
      "recovering",
      "recovery_attempts",
      "recovery_failures",
      "recovery_successes",
      "rejected_backend_commands",
      "requested_mode",
      "ros2_control_version",
      "ros_distro",
      "service_operation",
      "source_commit",
      "source_dirty",
      "state_age_ms",
      "state_queue_saturated",
      "stopped",
      "unsafe_none_request_mask",
      "unsafe_safe_publish_mask",
      "worker_state",
  };
  return keys;
}

void expectExactKeys(const diagnostic_msgs::msg::DiagnosticStatus& status) {
  std::set<std::string> actual;
  for (const auto& value : status.values) {
    EXPECT_TRUE(actual.insert(value.key).second) << value.key;
  }
  EXPECT_EQ(actual, expectedDiagnosticKeys());
}

hardware_interface::InterfaceInfo makeInterface(const std::string& name) {
  hardware_interface::InterfaceInfo interface;
  interface.name = name;
  interface.data_type = "double";
  return interface;
}

hardware_interface::HardwareInfo makeHardwareInfo() {
  hardware_interface::HardwareInfo info;
  info.name = "FrankaMultiHardwareInterface";
  info.type = "system";
  info.hardware_plugin_name = "franka_hardware/FrankaMultiHardwareInterface";
  info.hardware_parameters = {{"robot_count", "2"},
                              {"ns_1", "panda1"},
                              {"robot_ip_1", "offline-placeholder-1"},
                              {"ns_2", "panda2"},
                              {"robot_ip_2", "offline-placeholder-2"}};
  for (const auto& arm_id : {std::string{"panda1"}, std::string{"panda2"}}) {
    for (size_t joint = 1; joint <= 7; ++joint) {
      hardware_interface::ComponentInfo component;
      component.name = arm_id + "_joint" + std::to_string(joint);
      component.type = "joint";
      component.command_interfaces = {makeInterface("effort"), makeInterface("position"),
                                      makeInterface("velocity")};
      component.state_interfaces = {makeInterface("position"), makeInterface("velocity"),
                                    makeInterface("effort")};
      info.joints.push_back(std::move(component));
    }
  }
  return info;
}

TEST(SyntheticReplayParserTest, CheckedInFixtureIsExactBoundedAndPreservesBothArms) {
  const auto parsed = parseSyntheticReplay(loadFixture());
  ASSERT_TRUE(parsed.ok()) << static_cast<int>(parsed.error) << " at line " << parsed.line;
  ASSERT_EQ(parsed.replay.size, 4U);
  EXPECT_EQ(parsed.replay.frames[0].steady_ns, 1'000'000'000U);
  EXPECT_EQ(parsed.replay.frames[1].steady_ns, 1'100'000'000U);
  EXPECT_EQ(parsed.replay.frames[2].steady_ns, 2'100'000'001U);
  EXPECT_EQ(parsed.replay.frames[3].steady_ns, 3'200'000'002U);
  EXPECT_DOUBLE_EQ(parsed.replay.frames[0].states[0].q[0], 0.1);
  EXPECT_DOUBLE_EQ(parsed.replay.frames[3].states[0].tau_J[6], 2.0);
  EXPECT_DOUBLE_EQ(parsed.replay.frames[0].states[1].q[0], -0.1);
  EXPECT_DOUBLE_EQ(parsed.replay.frames[3].states[1].tau_J[6], 3.0);
  EXPECT_EQ(parsed.replay.frames[0].states[0].q_d, parsed.replay.frames[0].states[0].q);
  EXPECT_EQ(parsed.replay.frames[0].states[1].dq_d, parsed.replay.frames[0].states[1].dq);

  EXPECT_EQ(parseSyntheticReplay(replayText(1)).replay.size, 1U);
  EXPECT_EQ(parseSyntheticReplay(replayText(32)).replay.size, 32U);
}

TEST(SyntheticReplayParserTest, RejectsEveryNonCanonicalStructureAndUnsafeTokenClass) {
  expectRejected("", SyntheticReplayError::InvalidHeader);
  expectRejected("multipanda_synthetic_replay_v2\narms,panda1,panda2\n" + frameLine(1) + "\n",
                 SyntheticReplayError::InvalidHeader);
  expectRejected("multipanda_synthetic_replay_v1\narms,panda1,panda1\n" + frameLine(1) + "\n",
                 SyntheticReplayError::InvalidArms);
  expectRejected("multipanda_synthetic_replay_v1\narms,panda2,panda1\n" + frameLine(1) + "\n",
                 SyntheticReplayError::InvalidArms);
  expectRejected("multipanda_synthetic_replay_v1\narms,panda1,panda2\n\n" + frameLine(1) + "\n",
                 SyntheticReplayError::BlankLine);
  expectRejected(
      "multipanda_synthetic_replay_v1\narms,panda1,panda2\nrow," + frameLine(1).substr(6) + "\n",
      SyntheticReplayError::UnknownRow);
  expectRejected("multipanda_synthetic_replay_v1\narms,panda1,panda2\n" + frameLine(1, 41) + "\n",
                 SyntheticReplayError::FieldCount);
  expectRejected("multipanda_synthetic_replay_v1\narms,panda1,panda2\n" + frameLine(1, 43) + "\n",
                 SyntheticReplayError::FieldCount);
  expectRejected("multipanda_synthetic_replay_v1\narms,panda1,panda2\n",
                 SyntheticReplayError::FrameCount);
  expectRejected(replayText(33), SyntheticReplayError::FrameCount);

  for (const auto& token :
       {std::string{"#comment"}, std::string{"\"0\""}, std::string{"/tmp/replay"},
        std::string{"robot_address"}, std::string{"controller-hostname"}, std::string{"robot_ip"},
        std::string{"192.0.2.1"}, std::string{"fe80::1"}}) {
    SCOPED_TRACE(token);
    expectRejected(
        "multipanda_synthetic_replay_v1\narms,panda1,panda2\n" + frameLine(1, 42, token) + "\n",
        SyntheticReplayError::ForbiddenToken);
  }
}

TEST(SyntheticReplayParserTest, RejectsInvalidNonFiniteAndNonIncreasingData) {
  for (const auto& timestamp : {std::string{"0"}, std::string{"-1"}, std::string{"+1"},
                                std::string{"9223372036854775808"}, std::string{"1.0"}}) {
    SCOPED_TRACE(timestamp);
    auto line = frameLine(1);
    line.replace(6, 1, timestamp);
    expectRejected("multipanda_synthetic_replay_v1\narms,panda1,panda2\n" + line + "\n",
                   SyntheticReplayError::InvalidTimestamp);
  }
  for (const auto& value : {std::string{"NaN"}, std::string{"nan"}, std::string{"Inf"},
                            std::string{"-inf"}, std::string{"1e9999"}, std::string{"0x1p2"}}) {
    SCOPED_TRACE(value);
    expectRejected(
        "multipanda_synthetic_replay_v1\narms,panda1,panda2\n" + frameLine(1, 42, value) + "\n",
        SyntheticReplayError::NonFiniteValue);
  }
  expectRejected("multipanda_synthetic_replay_v1\narms,panda1,panda2\n" + frameLine(10) + "\n" +
                     frameLine(10) + "\n",
                 SyntheticReplayError::NonIncreasingTimestamp);
  expectRejected("multipanda_synthetic_replay_v1\narms,panda1,panda2\n" + frameLine(10) + "\n" +
                     frameLine(9) + "\n",
                 SyntheticReplayError::NonIncreasingTimestamp);
  std::string oversized(test_support::kSyntheticReplayMaximumBytes + 1, '0');
  expectRejected(oversized, SyntheticReplayError::InputTooLarge);
}

TEST(SyntheticReplayBackendTest, ReplaysSynchronouslyWithFixtureSteadyTimestampsAndModes) {
  const auto parsed = parseSyntheticReplay(loadFixture());
  ASSERT_TRUE(parsed.ok());
  for (size_t arm = 0; arm < kSyntheticReplayArmCount; ++arm) {
    SCOPED_TRACE(arm);
    SyntheticFrankaArmBackend backend(replayBackendConfig(parsed.replay, arm));
    EXPECT_EQ(backend.replaySize(), 3U);

    for (size_t frame = 0; frame < parsed.replay.size; ++frame) {
      if (frame == 1) {
        ASSERT_TRUE(backend.startStateReading());
        ASSERT_TRUE(backend.requestControlMode(arm == 0 ? ControlMode::JointTorque
                                                        : ControlMode::JointVelocity));
      }
      const auto state = backend.readLatestState();
      EXPECT_EQ(state.q, parsed.replay.frames[frame].states[arm].q);
      EXPECT_EQ(state.dq, parsed.replay.frames[frame].states[arm].dq);
      EXPECT_EQ(state.tau_J, parsed.replay.frames[frame].states[arm].tau_J);
      const auto diagnostics = backend.diagnostics();
      EXPECT_EQ(diagnostics.accepted_state_samples, frame + 1);
      EXPECT_EQ(diagnostics.last_accepted_state_steady_ns, parsed.replay.frames[frame].steady_ns);
    }
    EXPECT_EQ(backend.requestedControlMode(),
              arm == 0 ? ControlMode::JointTorque : ControlMode::JointVelocity);
    EXPECT_EQ(backend.activeControlMode(), backend.requestedControlMode());
    EXPECT_TRUE(backend.stop());
    EXPECT_TRUE(backend.diagnostics().stopped);
  }
}

TEST(SyntheticReplayBackendTest, ExplicitTimestampConfigurationIsDefensivelyValidated) {
  const auto parsed = parseSyntheticReplay(loadFixture());
  ASSERT_TRUE(parsed.ok());
  auto config = replayBackendConfig(parsed.replay, 0);
  config.replay_state_steady_ns.pop_back();
  EXPECT_THROW((void)SyntheticFrankaArmBackend(config), std::invalid_argument);

  config = replayBackendConfig(parsed.replay, 0);
  config.initial_state_steady_ns = 0;
  EXPECT_THROW((void)SyntheticFrankaArmBackend(config), std::invalid_argument);

  config = replayBackendConfig(parsed.replay, 0);
  config.replay_state_steady_ns[0] = config.initial_state_steady_ns;
  EXPECT_THROW((void)SyntheticFrankaArmBackend(config), std::invalid_argument);
  EXPECT_THROW((void)replayBackendConfig(parsed.replay, 2), std::invalid_argument);
}

TEST(SyntheticReplayDiagnosticsTest, ObjectiveLifecycleModeFaultAndAgeMatrixHasExactFields) {
  const auto parsed = parseSyntheticReplay(loadFixture());
  ASSERT_TRUE(parsed.ok());
  SyntheticFrankaArmBackend backend(replayBackendConfig(parsed.replay, 0));
  (void)backend.readLatestState();

  auto inactive = snapshotFor("panda1", backend.diagnostics(), {2, "inactive"});
  auto status = format(inactive, parsed.replay.frames[0].steady_ns + 9'000'000'000U);
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(status.hardware_id, "panda1");
  EXPECT_EQ(valueFor(status, "state_age_ms"), "not_applicable");
  expectExactKeys(status);

  for (const auto& lifecycle :
       {HardwareLifecycleSnapshot{1, "unconfigured"}, HardwareLifecycleSnapshot{4, "finalized"}}) {
    status = format(snapshotFor("panda1", backend.diagnostics(), lifecycle),
                    parsed.replay.frames[0].steady_ns + 9'000'000'000U);
    EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
    EXPECT_EQ(valueFor(status, "state_age_ms"), "not_applicable");
  }

  ASSERT_TRUE(backend.startStateReading());
  (void)backend.readLatestState();
  auto reading = snapshotFor("panda1", backend.diagnostics());
  const auto sample_time = parsed.replay.frames[1].steady_ns;
  status = format(reading, sample_time + 100'000'000U);
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(status.message, "backend state is healthy");
  EXPECT_EQ(valueFor(status, "worker_state"), "running");
  EXPECT_EQ(valueFor(status, "state_age_ms"), "100");

  status = format(reading, sample_time + 100'000'001U);
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_EQ(status.message, "accepted state sample is aging");
  status = format(reading, sample_time + 1'000'000'000U);
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  status = format(reading, sample_time + 1'000'000'001U);
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(status.message, "accepted state sample is stale");

  auto no_state = reading;
  no_state.backend.has_state_sample = false;
  no_state.backend.last_accepted_state_steady_ns = 0;
  status = format(no_state, sample_time);
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(valueFor(status, "state_age_ms"), "not_available");

  for (const auto mode : {ControlMode::JointTorque, ControlMode::JointVelocity}) {
    auto mode_snapshot = reading;
    mode_snapshot.backend.requested_mode = mode;
    mode_snapshot.backend.active_mode = mode;
    status = format(mode_snapshot, sample_time);
    EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
    EXPECT_EQ(valueFor(status, "requested_mode"),
              mode == ControlMode::JointTorque ? "joint_torque" : "joint_velocity");
  }
  auto mismatch = reading;
  mismatch.backend.requested_mode = ControlMode::JointTorque;
  mismatch.backend.active_mode = ControlMode::None;
  status = format(mismatch, sample_time);
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_EQ(status.message, "requested and active modes differ");

  backend.injectFaultForTest(SyntheticCondition::ControlFault);
  auto fault = snapshotFor("panda1", backend.diagnostics());
  status = format(fault, sample_time);
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(valueFor(status, "fault_category"), "worker");
  EXPECT_EQ(valueFor(status, "failure_reason"), "franka_control_exception");
}

TEST(SyntheticReplayDiagnosticsTest, RecoveryQueueAndGlobalStopAllStatesAreObjective) {
  const auto parsed = parseSyntheticReplay(loadFixture());
  ASSERT_TRUE(parsed.ok());
  auto config = replayBackendConfig(parsed.replay, 0);
  config.command_queue_capacity = 1;
  SyntheticFrankaArmBackend backend(config);
  (void)backend.readLatestState();
  ASSERT_TRUE(backend.startStateReading());
  (void)backend.readLatestState();
  const auto now = parsed.replay.frames[1].steady_ns;

  RobotCommand command{};
  ASSERT_TRUE(backend.publishCommand(command));
  EXPECT_FALSE(backend.publishCommand(command));
  auto status = format(snapshotFor("panda1", backend.diagnostics()), now);
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_EQ(valueFor(status, "rejected_backend_commands"), "1");
  EXPECT_EQ(valueFor(status, "command_queue_saturated"), "true");

  auto saturation_config = replayBackendConfig(parsed.replay, 1);
  saturation_config.failure = {SyntheticFailurePoint::StateQueueSaturation, 1};
  SyntheticFrankaArmBackend saturation_backend(saturation_config);
  (void)saturation_backend.readLatestState();
  ASSERT_TRUE(saturation_backend.startStateReading());
  (void)saturation_backend.readLatestState();
  status = format(snapshotFor("panda2", saturation_backend.diagnostics()), now);
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_EQ(valueFor(status, "dropped_state_samples"), "1");
  EXPECT_EQ(valueFor(status, "state_queue_saturated"), "true");

  ASSERT_TRUE(backend.holdServiceOperationForTest(BackendServiceOperation::Recovery));
  status = format(snapshotFor("panda1", backend.diagnostics()), now);
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_EQ(valueFor(status, "service_operation"), "recovery");
  EXPECT_EQ(valueFor(status, "recovering"), "true");
  backend.releaseServiceOperationForTest();

  backend.injectFaultForTest();
  ASSERT_TRUE(backend.recoverToReading());
  auto recovered = backend.diagnostics();
  status = format(snapshotFor("panda1", recovered), now);
  EXPECT_EQ(valueFor(status, "recovery_attempts"), "1");
  EXPECT_EQ(valueFor(status, "recovery_successes"), "1");
  EXPECT_EQ(valueFor(status, "last_recovery_result"), "succeeded");

  auto recovery_failure_config = replayBackendConfig(parsed.replay, 1);
  recovery_failure_config.failure = {SyntheticFailurePoint::Recovery, 1};
  SyntheticFrankaArmBackend recovery_failure_backend(recovery_failure_config);
  (void)recovery_failure_backend.readLatestState();
  ASSERT_TRUE(recovery_failure_backend.startStateReading());
  recovery_failure_backend.injectFaultForTest();
  EXPECT_FALSE(recovery_failure_backend.recoverToReading());
  status = format(snapshotFor("panda2", recovery_failure_backend.diagnostics()), now);
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(valueFor(status, "recovery_attempts"), "1");
  EXPECT_EQ(valueFor(status, "recovery_failures"), "1");
  EXPECT_EQ(valueFor(status, "last_recovery_result"), "failed");

  auto global = snapshotFor("panda2", recovered);
  global.global_fault = {1, GlobalFaultCause::BackendFault, 1, 1};
  global.global_fault_origin_arm_id = "panda1";
  status = format(global, now);
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(status.message, "global fault latched");
  EXPECT_EQ(valueFor(status, "global_fault_origin_arm_slot"), "1");
  EXPECT_EQ(valueFor(status, "global_fault_origin_arm_id"), "panda1");
  EXPECT_EQ(valueFor(status, "global_fault_cause"), "backend_fault");
  EXPECT_EQ(valueFor(status, "unsafe_safe_publish_mask"), "1");
  EXPECT_EQ(valueFor(status, "unsafe_none_request_mask"), "1");
}

TEST(SyntheticReplayIntegrationTest,
     GlobalFaultForcesBothArmsSafeAndCleanDeactivateAndDestructionReleaseThem) {
  RclcppScope rclcpp_scope;
  const auto parsed = parseSyntheticReplay(loadFixture());
  ASSERT_TRUE(parsed.ok());
  std::map<std::string, std::weak_ptr<SyntheticFrankaArmBackend>> weak_backends;
  BackendFactory factory = [&parsed, &weak_backends](
                               const std::string& arm_id, const std::string&,
                               const rclcpp::Logger&) -> std::shared_ptr<FrankaArmBackend> {
    const size_t arm = arm_id == "panda1" ? 0 : 1;
    auto config = replayBackendConfig(parsed.replay, arm);
    if (arm == 0) {
      config.failure = {SyntheticFailurePoint::ReadFault, 2};
    }
    auto backend = std::make_shared<SyntheticFrankaArmBackend>(config);
    weak_backends[arm_id] = backend;
    return backend;
  };

  auto hardware = std::make_unique<FrankaMultiHardwareInterface>(std::move(factory));
  ASSERT_EQ(hardware->on_init(makeHardwareInfo()), CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  EXPECT_EQ(hardware->read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            hardware_interface::return_type::ERROR);
  const auto global = hardware->globalFaultDiagnostic();
  EXPECT_EQ(global.origin_arm_slot, 1U);
  EXPECT_EQ(global.cause, GlobalFaultCause::BackendFault);
  EXPECT_EQ(global.unsafe_safe_publish_mask, 1U);
  EXPECT_EQ(global.unsafe_none_request_mask, 1U);

  auto backend1 = weak_backends.at("panda1").lock();
  auto backend2 = weak_backends.at("panda2").lock();
  ASSERT_TRUE(backend1);
  ASSERT_TRUE(backend2);
  EXPECT_EQ(backend1->requestedControlMode(), ControlMode::None);
  EXPECT_EQ(backend2->requestedControlMode(), ControlMode::None);
  EXPECT_EQ(hardware->on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  EXPECT_TRUE(backend1->diagnostics().stopped);
  EXPECT_TRUE(backend2->diagnostics().stopped);
  backend1.reset();
  backend2.reset();
  hardware.reset();
  EXPECT_TRUE(weak_backends.at("panda1").expired());
  EXPECT_TRUE(weak_backends.at("panda2").expired());
}

}  // namespace
}  // namespace franka_hardware
