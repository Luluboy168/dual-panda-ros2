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

#include "franka_hardware/real/franka_hardware_diagnostics_node.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <rclcpp/rclcpp.hpp>

#include "support/synthetic_franka_arm_backend.hpp"

namespace franka_hardware {
namespace {

using namespace std::chrono_literals;
using test_support::SyntheticCondition;
using test_support::SyntheticFailurePoint;
using test_support::SyntheticFrankaArmBackend;
using test_support::SyntheticFrankaArmBackendConfig;

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

hardware_interface::InterfaceInfo makeInterfaceInfo(const std::string& name) {
  hardware_interface::InterfaceInfo interface{};
  interface.name = name;
  return interface;
}

hardware_interface::HardwareInfo makeHardwareInfo() {
  hardware_interface::HardwareInfo info{};
  info.name = "FrankaMultiHardwareInterface";
  info.type = "system";
  info.hardware_plugin_name = "franka_hardware/FrankaMultiHardwareInterface";
  info.hardware_parameters = {{"robot_count", "2"},
                              {"ns_1", "panda1"},
                              {"robot_ip_1", "offline-placeholder-1"},
                              {"ns_2", "panda2"},
                              {"robot_ip_2", "offline-placeholder-2"}};
  for (const auto& arm_id : {std::string("panda1"), std::string("panda2")}) {
    for (size_t joint_index = 1; joint_index <= 7; ++joint_index) {
      hardware_interface::ComponentInfo joint{};
      joint.name = arm_id + "_joint" + std::to_string(joint_index);
      joint.type = "joint";
      joint.command_interfaces = {makeInterfaceInfo("effort"), makeInterfaceInfo("position"),
                                  makeInterfaceInfo("velocity")};
      joint.state_interfaces = {makeInterfaceInfo("position"), makeInterfaceInfo("velocity"),
                                makeInterfaceInfo("effort")};
      info.joints.push_back(std::move(joint));
    }
  }
  return info;
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

FrankaArmDiagnosticSnapshot baseSnapshot() {
  FrankaArmDiagnosticSnapshot snapshot;
  snapshot.arm_id = "panda1";
  snapshot.hardware_lifecycle = {2, "inactive"};
  snapshot.provenance = {
      "0123456789abcdef0123456789abcdef01234567", true, "jazzy", "28.1.21", "4.45.2", "0.9.2"};
  return snapshot;
}

diagnostic_msgs::msg::DiagnosticStatus format(const FrankaArmDiagnosticSnapshot& snapshot,
                                              uint64_t now_ns) {
  diagnostic_updater::DiagnosticStatusWrapper status;
  formatFrankaArmDiagnosticStatus(snapshot, now_ns, status);
  return static_cast<const diagnostic_msgs::msg::DiagnosticStatus&>(status);
}

TEST(FrankaHardwareDiagnosticsFormatterTest,
     InactiveAgeIsNotApplicableAndEveryRequiredFieldIsStable) {
  auto status = format(baseSnapshot(), 10'000'000'000U);
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(status.hardware_id, "panda1");
  EXPECT_EQ(valueFor(status, "arm_id"), "panda1");
  EXPECT_EQ(valueFor(status, "hardware_lifecycle_id"), "2");
  EXPECT_EQ(valueFor(status, "hardware_lifecycle_label"), "inactive");
  EXPECT_EQ(valueFor(status, "requested_mode"), "none");
  EXPECT_EQ(valueFor(status, "active_mode"), "none");
  EXPECT_EQ(valueFor(status, "worker_state"), "stopped");
  EXPECT_EQ(valueFor(status, "fault_category"), "none");
  EXPECT_EQ(valueFor(status, "failure_reason"), "none");
  EXPECT_EQ(valueFor(status, "service_operation"), "idle");
  EXPECT_EQ(valueFor(status, "stopped"), "true");
  EXPECT_EQ(valueFor(status, "recovering"), "false");
  EXPECT_EQ(valueFor(status, "state_age_ms"), "not_applicable");
  EXPECT_EQ(valueFor(status, "accepted_state_samples"), "0");
  EXPECT_EQ(valueFor(status, "dropped_state_samples"), "0");
  EXPECT_EQ(valueFor(status, "rejected_backend_commands"), "0");
  EXPECT_EQ(valueFor(status, "state_queue_saturated"), "false");
  EXPECT_EQ(valueFor(status, "command_queue_saturated"), "false");
  EXPECT_EQ(valueFor(status, "recovery_attempts"), "0");
  EXPECT_EQ(valueFor(status, "recovery_successes"), "0");
  EXPECT_EQ(valueFor(status, "recovery_failures"), "0");
  EXPECT_EQ(valueFor(status, "last_recovery_result"), "never_attempted");
  EXPECT_EQ(valueFor(status, "global_fault_origin_arm_slot"), "0");
  EXPECT_EQ(valueFor(status, "global_fault_origin_arm_id"), "none");
  EXPECT_EQ(valueFor(status, "global_fault_cause"), "none");
  EXPECT_EQ(valueFor(status, "unsafe_safe_publish_mask"), "0");
  EXPECT_EQ(valueFor(status, "unsafe_none_request_mask"), "0");
  EXPECT_EQ(valueFor(status, "source_commit"), "0123456789abcdef0123456789abcdef01234567");
  EXPECT_EQ(valueFor(status, "source_dirty"), "true");
  EXPECT_EQ(valueFor(status, "ros_distro"), "jazzy");
  EXPECT_EQ(valueFor(status, "rclcpp_version"), "28.1.21");
  EXPECT_EQ(valueFor(status, "ros2_control_version"), "4.45.2");
  EXPECT_EQ(valueFor(status, "libfranka_version"), "0.9.2");

  std::set<std::string> keys;
  for (const auto& value : status.values) {
    keys.insert(value.key);
    EXPECT_TRUE(std::all_of(value.key.begin(), value.key.end(), [](unsigned char character) {
      return (character >= static_cast<unsigned char>('a') &&
              character <= static_cast<unsigned char>('z')) ||
             (character >= static_cast<unsigned char>('0') &&
              character <= static_cast<unsigned char>('9')) ||
             character == static_cast<unsigned char>('_');
    }));
  }
  EXPECT_EQ(keys.size(), status.values.size());
  EXPECT_EQ(keys, (std::set<std::string>{
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
                  }));
}

TEST(FrankaHardwareDiagnosticsFormatterTest, UnconfiguredAndFinalizedAgeAreNotApplicable) {
  for (const auto& lifecycle :
       {HardwareLifecycleSnapshot{1, "unconfigured"}, HardwareLifecycleSnapshot{4, "finalized"}}) {
    auto snapshot = baseSnapshot();
    snapshot.hardware_lifecycle = lifecycle;
    snapshot.backend.has_state_sample = true;
    snapshot.backend.last_accepted_state_steady_ns = 1;
    const auto status = format(snapshot, 10'000'000'000U);
    EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
    EXPECT_EQ(valueFor(status, "state_age_ms"), "not_applicable");
  }
}

TEST(FrankaHardwareDiagnosticsFormatterTest, LifecycleGateHasStableDiagnosticLabel) {
  auto snapshot = baseSnapshot();
  snapshot.backend.service_operation = BackendServiceOperation::Lifecycle;
  const auto status = format(snapshot, 0);
  EXPECT_EQ(valueFor(status, "service_operation"), "lifecycle");
}

TEST(FrankaHardwareDiagnosticsFormatterTest, ExplicitSteadyNowControlsAgeAndThresholds) {
  auto snapshot = baseSnapshot();
  snapshot.hardware_lifecycle = {3, "active"};
  snapshot.backend.worker_state = BackendWorkerState::Running;
  snapshot.backend.stopped = false;

  auto unavailable = format(snapshot, 5'000'000'000U);
  EXPECT_EQ(unavailable.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(valueFor(unavailable, "state_age_ms"), "not_available");

  snapshot.backend.has_state_sample = true;
  snapshot.backend.accepted_state_samples = 4;
  snapshot.backend.last_accepted_state_steady_ns = 4'950'000'000U;
  auto healthy = format(snapshot, 5'000'000'000U);
  EXPECT_EQ(healthy.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(valueFor(healthy, "state_age_ms"), "50");

  auto warning_boundary = format(snapshot, 5'050'000'000U);
  EXPECT_EQ(warning_boundary.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  auto warning = format(snapshot, 5'050'000'001U);
  EXPECT_EQ(warning.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_EQ(valueFor(warning, "state_age_ms"), "100");

  auto error_boundary = format(snapshot, 5'950'000'000U);
  EXPECT_EQ(error_boundary.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  auto error = format(snapshot, 5'950'000'001U);
  EXPECT_EQ(error.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(valueFor(error, "state_age_ms"), "1000");

  auto later = format(snapshot, 6'951'000'000U);
  EXPECT_GT(std::stoull(valueFor(later, "state_age_ms")),
            std::stoull(valueFor(error, "state_age_ms")));
}

TEST(FrankaHardwareDiagnosticsFormatterTest, FaultRecoveryQueueAndCounterLevelsHavePrecedence) {
  auto snapshot = baseSnapshot();
  snapshot.backend.dropped_state_samples = 1;
  snapshot.backend.rejected_command_samples = 2;
  auto warning = format(snapshot, 0);
  EXPECT_EQ(warning.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);

  snapshot.backend.command_queue_saturated = true;
  warning = format(snapshot, 0);
  EXPECT_EQ(warning.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);

  snapshot.backend.last_recovery_result = BackendRecoveryResult::Failed;
  auto recovery_error = format(snapshot, 0);
  EXPECT_EQ(recovery_error.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);

  snapshot.global_fault = {2, GlobalFaultCause::ModeRequest, 1, 2};
  snapshot.global_fault_origin_arm_id = "panda2";
  auto global_error = format(snapshot, 0);
  EXPECT_EQ(global_error.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(global_error.message, "global fault latched");
  EXPECT_EQ(valueFor(global_error, "global_fault_origin_arm_id"), "panda2");
  EXPECT_EQ(valueFor(global_error, "global_fault_cause"), "mode_request");
  EXPECT_EQ(valueFor(global_error, "unsafe_safe_publish_mask"), "1");
  EXPECT_EQ(valueFor(global_error, "unsafe_none_request_mask"), "2");
}

TEST(FrankaHardwareDiagnosticsFormatterTest, BackendAndTransitionWarningLevelsAreComplete) {
  auto snapshot = baseSnapshot();

  snapshot.backend.worker_state = BackendWorkerState::Faulted;
  EXPECT_EQ(format(snapshot, 0).level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);

  snapshot.backend.worker_state = BackendWorkerState::Stopped;
  snapshot.backend.fault_category = BackendFaultCategory::Worker;
  EXPECT_EQ(format(snapshot, 0).level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);

  snapshot.backend.fault_category = BackendFaultCategory::None;
  snapshot.backend.failure_reason = BackendFailureReason::FrankaNetworkException;
  EXPECT_EQ(format(snapshot, 0).level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);

  snapshot.backend.failure_reason = BackendFailureReason::None;
  snapshot.backend.worker_state = BackendWorkerState::Starting;
  EXPECT_EQ(format(snapshot, 0).level, diagnostic_msgs::msg::DiagnosticStatus::WARN);

  snapshot.backend.worker_state = BackendWorkerState::Stopped;
  snapshot.backend.recovering = true;
  EXPECT_EQ(format(snapshot, 0).level, diagnostic_msgs::msg::DiagnosticStatus::WARN);

  snapshot.backend.recovering = false;
  snapshot.backend.requested_mode = ControlMode::JointVelocity;
  EXPECT_EQ(format(snapshot, 0).level, diagnostic_msgs::msg::DiagnosticStatus::WARN);

  snapshot.backend.requested_mode = ControlMode::None;
  snapshot.backend.state_queue_saturated = true;
  EXPECT_EQ(format(snapshot, 0).level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
}

TEST(FrankaHardwareProvenanceTest, RejectsPathsAddressesWhitespaceAndRawOutput) {
  const std::string documentation_address = std::string{"192.0.2"} + ".42";
  const auto sanitized =
      sanitizeBuildProvenance({"/home/private/repository", false, "jazzy\nraw", "/opt/ros/jazzy",
                               documentation_address, "0.9.2 command output"});
  EXPECT_EQ(sanitized.source_commit, "unknown");
  EXPECT_EQ(sanitized.ros_distro, "unknown");
  EXPECT_EQ(sanitized.rclcpp_version, "unknown");
  EXPECT_EQ(sanitized.ros2_control_version, "unknown");
  EXPECT_EQ(sanitized.libfranka_version, "unknown");
  EXPECT_TRUE(isSanitizedBuildProvenance(sanitized));

  const auto current = currentBuildProvenance();
  EXPECT_TRUE(isSanitizedBuildProvenance(current));
  EXPECT_TRUE(current.source_commit == "unknown" || current.source_commit.size() == 40);
}

TEST(FrankaArmBackendDiagnosticsTest,
     AcceptedSampleTimestampQueueCountersAndRecoveryAccountingAreMonotonic) {
  auto config = SyntheticFrankaArmBackendConfig::forArm(1);
  config.failure = {SyntheticFailurePoint::StateQueueSaturation, 1};
  SyntheticFrankaArmBackend backend(config);

  const auto before = std::chrono::steady_clock::now().time_since_epoch();
  (void)backend.readLatestState();
  const auto after = std::chrono::steady_clock::now().time_since_epoch();
  auto diagnostics = backend.diagnostics();
  EXPECT_TRUE(diagnostics.has_state_sample);
  EXPECT_EQ(diagnostics.accepted_state_samples, 1U);
  EXPECT_GE(
      diagnostics.last_accepted_state_steady_ns,
      static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(before).count()));
  EXPECT_LE(
      diagnostics.last_accepted_state_steady_ns,
      static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(after).count()));
  const auto first_timestamp = diagnostics.last_accepted_state_steady_ns;

  ASSERT_TRUE(backend.startStateReading());
  (void)backend.readLatestState();
  diagnostics = backend.diagnostics();
  EXPECT_EQ(diagnostics.accepted_state_samples, 1U);
  EXPECT_EQ(diagnostics.dropped_state_samples, 1U);
  EXPECT_TRUE(diagnostics.state_queue_saturated);
  EXPECT_EQ(diagnostics.last_accepted_state_steady_ns, first_timestamp);

  (void)backend.readLatestState();
  diagnostics = backend.diagnostics();
  EXPECT_EQ(diagnostics.accepted_state_samples, 2U);
  EXPECT_FALSE(diagnostics.state_queue_saturated);
  EXPECT_GE(diagnostics.last_accepted_state_steady_ns, first_timestamp);

  auto recovery_config = SyntheticFrankaArmBackendConfig::forArm(2);
  recovery_config.failure = {SyntheticFailurePoint::Recovery, 1};
  SyntheticFrankaArmBackend recovery_backend(recovery_config);
  recovery_backend.injectFaultForTest(SyntheticCondition::ControlFault);
  EXPECT_EQ(recovery_backend.diagnostics().failure_reason,
            BackendFailureReason::FrankaControlException);
  EXPECT_FALSE(recovery_backend.recoverToReading());
  diagnostics = recovery_backend.diagnostics();
  EXPECT_EQ(diagnostics.recovery_attempts, 1U);
  EXPECT_EQ(diagnostics.recovery_failures, 1U);
  EXPECT_EQ(diagnostics.last_recovery_result, BackendRecoveryResult::Failed);
  EXPECT_EQ(diagnostics.failure_reason, BackendFailureReason::FrankaControlException);

  EXPECT_TRUE(recovery_backend.recoverToReading());
  diagnostics = recovery_backend.diagnostics();
  EXPECT_EQ(diagnostics.recovery_attempts, 2U);
  EXPECT_EQ(diagnostics.recovery_successes, 1U);
  EXPECT_EQ(diagnostics.recovery_failures, 1U);
  EXPECT_EQ(diagnostics.last_recovery_result, BackendRecoveryResult::Succeeded);
  EXPECT_EQ(diagnostics.failure_reason, BackendFailureReason::None);
}

TEST(FrankaHardwareDiagnosticsIntegrationTest,
     StandardUpdaterPublishesTwoExactPerArmStatusesAndDestructionReleasesComponents) {
  RclcppScope rclcpp_scope;
  std::map<std::string, std::weak_ptr<SyntheticFrankaArmBackend>> backends;
  BackendFactory factory = [&backends](const std::string& arm_id, const std::string&,
                                       const rclcpp::Logger&) -> std::shared_ptr<FrankaArmBackend> {
    const uint8_t marker = arm_id == "panda1" ? 1 : 2;
    auto backend = std::make_shared<SyntheticFrankaArmBackend>(
        SyntheticFrankaArmBackendConfig::forArm(marker));
    backends[arm_id] = backend;
    return backend;
  };

  auto observer = std::make_shared<rclcpp::Node>("franka_hardware_diagnostics_test_observer");
  std::mutex messages_mutex;
  std::vector<diagnostic_msgs::msg::DiagnosticStatus> matching_statuses;
  std::set<std::string> observed_status_names;
  auto subscription = observer->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10),
      [&](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr message) {
        std::vector<diagnostic_msgs::msg::DiagnosticStatus> candidate;
        for (const auto& status : message->status) {
          {
            std::lock_guard<std::mutex> lock(messages_mutex);
            observed_status_names.insert(status.name);
          }
          if (status.name == "franka_hardware_diagnostics: franka_hardware/panda1" ||
              status.name == "franka_hardware_diagnostics: franka_hardware/panda2") {
            candidate.push_back(status);
          }
        }
        if (candidate.size() == 2) {
          std::lock_guard<std::mutex> lock(messages_mutex);
          matching_statuses = std::move(candidate);
        }
      });

  auto hardware = std::make_unique<FrankaMultiHardwareInterface>(std::move(factory));
  ASSERT_EQ(hardware->on_init(makeHardwareInfo()), CallbackReturn::SUCCESS);

  const auto publication_deadline = std::chrono::steady_clock::now() + 3s;
  while (std::chrono::steady_clock::now() < publication_deadline) {
    rclcpp::spin_some(observer);
    {
      std::lock_guard<std::mutex> lock(messages_mutex);
      if (matching_statuses.size() == 2) {
        break;
      }
    }
    std::this_thread::yield();
  }

  std::vector<diagnostic_msgs::msg::DiagnosticStatus> received;
  {
    std::lock_guard<std::mutex> lock(messages_mutex);
    received = matching_statuses;
  }
  std::string observed_names;
  {
    std::lock_guard<std::mutex> lock(messages_mutex);
    for (const auto& name : observed_status_names) {
      observed_names += "[" + name + "]";
    }
  }
  ASSERT_EQ(received.size(), 2U) << observed_names;
  std::set<std::string> names;
  for (const auto& status : received) {
    names.insert(status.name);
    EXPECT_TRUE(status.hardware_id == "panda1" || status.hardware_id == "panda2");
    EXPECT_EQ(valueFor(status, "arm_id"), status.hardware_id);
    EXPECT_NO_THROW((void)valueFor(status, "failure_reason"));
    EXPECT_NO_THROW((void)valueFor(status, "source_commit"));
  }
  EXPECT_EQ(names, (std::set<std::string>{"franka_hardware_diagnostics: franka_hardware/panda1",
                                          "franka_hardware_diagnostics: franka_hardware/panda2"}));

  hardware.reset();
  EXPECT_TRUE(backends.at("panda1").expired());
  EXPECT_TRUE(backends.at("panda2").expired());
  const auto teardown_deadline = std::chrono::steady_clock::now() + 2s;
  while (observer->count_publishers("/diagnostics") != 0 &&
         std::chrono::steady_clock::now() < teardown_deadline) {
    rclcpp::spin_some(observer);
    std::this_thread::yield();
  }
  EXPECT_EQ(observer->count_publishers("/diagnostics"), 0U);
  (void)subscription;
}

// ---------------------------------------------------------------------------------------------
// F-10j regressions (2026-08-29): the diagnostics node's read ORDER.
//
// FrankaHardwareDiagnosticsNode::diagnoseArm() used to pass the backend snapshot and
// steadyNowNanoseconds() as two arguments of one call, where C++ leaves the evaluation order
// unspecified; the shipped build read the clock FIRST. A state sample accepted by the 1 kHz
// control worker between the two reads therefore produced last_accepted > now, and the
// !timestamp_valid branch reported "active hardware has no accepted state sample" for an arm
// whose state stream was healthy -- ~0.24 % of ticks on both arms across phases 9 and 10
// (test_logs/offline_hardening_2026-08-28/f10j_forensics/).
//
// ProbeBackend below makes that interleave deterministic instead of probabilistic: its
// diagnostics() stamps last_accepted with the clock AT THE MOMENT OF THAT CALL, i.e. it models a
// sample accepted exactly between the node's two reads -- the worst case of the race. With the
// clock read first, last_accepted is unconditionally in the future and the false alarm fires on
// every tick. With the snapshot read first, the age is valid on every tick.
// ---------------------------------------------------------------------------------------------

class ProbeBackend final : public FrankaArmBackend {
 public:
  enum class Sampling { AcceptsBetweenTheNodesReads, HasNeverAcceptedASample };

  explicit ProbeBackend(Sampling sampling) : sampling_(sampling) {}

  FrankaArmBackendDiagnostics diagnostics() const noexcept override {
    FrankaArmBackendDiagnostics diagnostics;
    diagnostics.worker_state = BackendWorkerState::Running;
    diagnostics.stopped = false;
    if (sampling_ == Sampling::HasNeverAcceptedASample) {
      // The GENUINE case the ERROR exists for: an active arm whose backend has never accepted a
      // state sample. Nothing about the fix may silence this.
      diagnostics.has_state_sample = false;
      return diagnostics;
    }
    diagnostics.has_state_sample = true;
    diagnostics.accepted_state_samples = ++accepted_samples_;
    // The interleaved accept: stamped now, while the caller is between its two reads.
    diagnostics.last_accepted_state_steady_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
    return diagnostics;
  }

  bool startStateReading() override { return true; }
  bool stop() override { return true; }
  franka::RobotState readLatestState() override { return franka::RobotState{}; }
  ModelBase* model() noexcept override { return nullptr; }
  bool canPublishCommand() const noexcept override { return true; }
  bool publishCommand(const RobotCommand&) noexcept override { return true; }
  bool canRequestControlMode(ControlMode) const noexcept override { return true; }
  bool requestControlMode(ControlMode) noexcept override { return true; }
  ControlMode requestedControlMode() const noexcept override { return ControlMode::None; }
  ControlMode activeControlMode() const noexcept override { return ControlMode::None; }
  bool modeEntryInFlight() const noexcept override { return false; }
  bool hasFault() const noexcept override { return false; }
  bool recoverToReading() override { return true; }
  void setJointStiffness(
      const franka_msgs::srv::SetJointStiffness::Request::SharedPtr&) override {}
  void setCartesianStiffness(
      const franka_msgs::srv::SetCartesianStiffness::Request::SharedPtr&) override {}
  void setLoad(const franka_msgs::srv::SetLoad::Request::SharedPtr&) override {}
  void setTCPFrame(const franka_msgs::srv::SetTCPFrame::Request::SharedPtr&) override {}
  void setStiffnessFrame(
      const franka_msgs::srv::SetStiffnessFrame::Request::SharedPtr&) override {}
  void setForceTorqueCollisionBehavior(
      const franka_msgs::srv::SetForceTorqueCollisionBehavior::Request::SharedPtr&) override {}
  void setFullCollisionBehavior(
      const franka_msgs::srv::SetFullCollisionBehavior::Request::SharedPtr&) override {}

 private:
  Sampling sampling_;
  mutable uint64_t accepted_samples_{0};
};

// Runs a real FrankaHardwareDiagnosticsNode over a ProbeBackend and returns the per-arm statuses
// it actually published, so the assertions below are on the node's own read order and not on a
// re-implementation of it.
std::vector<diagnostic_msgs::msg::DiagnosticStatus> collectProbeStatuses(
    const std::string& arm_id,
    ProbeBackend::Sampling sampling,
    size_t wanted) {
  const std::string status_name = "franka_hardware_diagnostics: franka_hardware/" + arm_id;
  auto observer = std::make_shared<rclcpp::Node>("f10j_probe_observer_" + arm_id);
  std::mutex collected_mutex;
  std::vector<diagnostic_msgs::msg::DiagnosticStatus> collected;
  auto subscription = observer->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(50),
      [&](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr message) {
        std::lock_guard<std::mutex> lock(collected_mutex);
        for (const auto& status : message->status) {
          // diagnostic_updater emits one valueless "Node starting up" status per task before the
          // task itself has ever run; only statuses the node's own diagnoseArm() produced (they
          // carry arm_id) say anything about the read order.
          const bool from_diagnose_arm =
              std::any_of(status.values.begin(), status.values.end(),
                          [](const auto& value) { return value.key == "arm_id"; });
          if (status.name == status_name && from_diagnose_arm) {
            collected.push_back(status);
          }
        }
      });

  std::vector<FrankaArmDiagnosticSource> sources;
  sources.push_back({arm_id, std::make_shared<ProbeBackend>(sampling)});
  auto node = std::make_shared<FrankaHardwareDiagnosticsNode>(
      rclcpp::NodeOptions(), std::move(sources),
      []() { return GlobalFaultDiagnostic{}; },
      []() { return HardwareLifecycleSnapshot{3, "active"}; }, 0.05);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(observer);
  const auto deadline = std::chrono::steady_clock::now() + 20s;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some(10ms);
    std::lock_guard<std::mutex> lock(collected_mutex);
    if (collected.size() >= wanted) {
      break;
    }
  }
  executor.remove_node(observer);
  executor.remove_node(node);
  std::lock_guard<std::mutex> lock(collected_mutex);
  (void)subscription;
  return collected;
}

TEST(FrankaHardwareDiagnosticsRaceTest,
     AStateSampleAcceptedBetweenTheNodesTwoReadsIsReportedWithAValidAgeAndNoFalseAlarm) {
  RclcppScope rclcpp_scope;
  const auto statuses =
      collectProbeStatuses("f10jrace", ProbeBackend::Sampling::AcceptsBetweenTheNodesReads, 4);
  ASSERT_GE(statuses.size(), size_t{4});
  for (size_t index = 0; index < statuses.size(); ++index) {
    const auto& status = statuses.at(index);
    EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::OK)
        << "tick " << index << " summary=" << status.message
        << " -- an accept interleaved between the node's two reads was reported as a fault;"
        << " the clock is being read before the backend snapshot again (F-10j)";
    EXPECT_NE(status.message, "active hardware has no accepted state sample") << "tick " << index;
    const std::string age = valueFor(status, "state_age_ms");
    EXPECT_NE(age, "not_available")
        << "tick " << index << " -- the age must be a number, not the negative-age sentinel";
    EXPECT_NE(age, "not_applicable") << "tick " << index;
    EXPECT_NE(valueFor(status, "accepted_state_samples"), "0") << "tick " << index;
    EXPECT_EQ(valueFor(status, "dropped_state_samples"), "0") << "tick " << index;
  }
}

// NEGATIVE CONTROL. The fix reorders two reads; it must not weaken the branch those reads feed.
// An active arm whose backend genuinely has no accepted state sample must still alarm, with the
// same level and the same message text operators and the Phase 11 runbook key on.
TEST(FrankaHardwareDiagnosticsRaceTest,
     AnActiveArmThatTrulyHasNoAcceptedStateSampleStillAlarmsWithTheSameMessage) {
  RclcppScope rclcpp_scope;
  const auto statuses =
      collectProbeStatuses("f10jmissing", ProbeBackend::Sampling::HasNeverAcceptedASample, 2);
  ASSERT_GE(statuses.size(), size_t{2});
  for (size_t index = 0; index < statuses.size(); ++index) {
    const auto& status = statuses.at(index);
    EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR) << "tick " << index;
    EXPECT_EQ(status.message, "active hardware has no accepted state sample") << "tick " << index;
    EXPECT_EQ(valueFor(status, "state_age_ms"), "not_available") << "tick " << index;
  }
}

}  // namespace
}  // namespace franka_hardware
