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

#include <algorithm>
#include <cctype>
#include <chrono>
#include <stdexcept>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include "franka_hardware/build_provenance.hpp"

namespace franka_hardware {
namespace {

constexpr uint8_t kLifecycleUnknown = 0;
constexpr uint8_t kLifecycleUnconfigured = 1;
constexpr uint8_t kLifecycleInactive = 2;
constexpr uint8_t kLifecycleActive = 3;
constexpr uint8_t kLifecycleFinalized = 4;
constexpr uint8_t kFirstTransitionState = 10;
constexpr uint64_t kWarningStateAgeNanoseconds = 100'000'000U;
constexpr uint64_t kErrorStateAgeNanoseconds = 1'000'000'000U;

bool isLowerHexCommit(const std::string& value) noexcept {
  return value == "unknown" ||
         (value.size() == 40 &&
          std::all_of(value.begin(), value.end(), [](unsigned char character) {
            return std::isdigit(character) != 0 || (character >= static_cast<unsigned char>('a') &&
                                                    character <= static_cast<unsigned char>('f'));
          }));
}

bool isRosDistroToken(const std::string& value) noexcept {
  if (value == "unknown") {
    return true;
  }
  if (value.empty() || value.size() > 32 || value.front() < 'a' || value.front() > 'z') {
    return false;
  }
  return std::all_of(value.begin() + 1, value.end(), [](unsigned char character) {
    return (character >= static_cast<unsigned char>('a') &&
            character <= static_cast<unsigned char>('z')) ||
           (character >= static_cast<unsigned char>('0') &&
            character <= static_cast<unsigned char>('9')) ||
           character == static_cast<unsigned char>('_');
  });
}

bool isVersionToken(const std::string& value) noexcept {
  if (value == "unknown") {
    return true;
  }
  if (value.empty() || value.size() > 64) {
    return false;
  }
  size_t component = 0;
  size_t digits_in_component = 0;
  for (const unsigned char character : value) {
    if (std::isdigit(character) != 0) {
      ++digits_in_component;
      continue;
    }
    if (character == static_cast<unsigned char>('.') && component < 2 && digits_in_component != 0) {
      ++component;
      digits_in_component = 0;
      continue;
    }
    return false;
  }
  return component == 2 && digits_in_component != 0;
}

const char* controlModeLabel(ControlMode mode) noexcept {
  switch (mode) {
    case ControlMode::None:
      return "none";
    case ControlMode::JointTorque:
      return "joint_torque";
    case ControlMode::JointPosition:
      return "joint_position";
    case ControlMode::JointVelocity:
      return "joint_velocity";
    case ControlMode::CartesianVelocity:
      return "cartesian_velocity";
    case ControlMode::CartesianPose:
      return "cartesian_pose";
  }
  return "invalid";
}

const char* workerStateLabel(BackendWorkerState state) noexcept {
  switch (state) {
    case BackendWorkerState::Stopped:
      return "stopped";
    case BackendWorkerState::Starting:
      return "starting";
    case BackendWorkerState::Running:
      return "running";
    case BackendWorkerState::StopRequested:
      return "stop_requested";
    case BackendWorkerState::Faulted:
      return "faulted";
  }
  return "invalid";
}

const char* faultCategoryLabel(BackendFaultCategory category) noexcept {
  switch (category) {
    case BackendFaultCategory::None:
      return "none";
    case BackendFaultCategory::Worker:
      return "worker";
  }
  return "invalid";
}

const char* serviceOperationLabel(BackendServiceOperation operation) noexcept {
  switch (operation) {
    case BackendServiceOperation::Idle:
      return "idle";
    case BackendServiceOperation::Parameter:
      return "parameter";
    case BackendServiceOperation::Recovery:
      return "recovery";
    case BackendServiceOperation::ModeRequest:
      return "mode_request";
    case BackendServiceOperation::Lifecycle:
      return "lifecycle";
  }
  return "invalid";
}

const char* globalFaultCauseLabel(GlobalFaultCause cause) noexcept {
  switch (cause) {
    case GlobalFaultCause::None:
      return "none";
    case GlobalFaultCause::BackendFault:
      return "backend_fault";
    case GlobalFaultCause::ReadFailure:
      return "read_failure";
    case GlobalFaultCause::InvalidState:
      return "invalid_state";
    case GlobalFaultCause::InvalidCommand:
      return "invalid_command";
    case GlobalFaultCause::CommandCapacity:
      return "command_capacity";
    case GlobalFaultCause::CommandPublish:
      return "command_publish";
    case GlobalFaultCause::ModeRequest:
      return "mode_request";
  }
  return "invalid";
}

const char* boolLabel(bool value) noexcept {
  return value ? "true" : "false";
}

bool stateAgeNotApplicable(uint8_t lifecycle_id) noexcept {
  return lifecycle_id == kLifecycleUnconfigured || lifecycle_id == kLifecycleInactive ||
         lifecycle_id == kLifecycleFinalized;
}

uint64_t steadyNowNanoseconds() noexcept {
  const auto duration = std::chrono::steady_clock::now().time_since_epoch();
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count());
}

}  // namespace

BuildProvenance sanitizeBuildProvenance(const BuildProvenance& provenance) {
  BuildProvenance sanitized;
  sanitized.source_commit =
      isLowerHexCommit(provenance.source_commit) ? provenance.source_commit : "unknown";
  sanitized.source_dirty = provenance.source_dirty;
  sanitized.ros_distro =
      isRosDistroToken(provenance.ros_distro) ? provenance.ros_distro : "unknown";
  sanitized.rclcpp_version =
      isVersionToken(provenance.rclcpp_version) ? provenance.rclcpp_version : "unknown";
  sanitized.ros2_control_version =
      isVersionToken(provenance.ros2_control_version) ? provenance.ros2_control_version : "unknown";
  sanitized.libfranka_version =
      isVersionToken(provenance.libfranka_version) ? provenance.libfranka_version : "unknown";
  return sanitized;
}

bool isSanitizedBuildProvenance(const BuildProvenance& provenance) noexcept {
  return isLowerHexCommit(provenance.source_commit) && isRosDistroToken(provenance.ros_distro) &&
         isVersionToken(provenance.rclcpp_version) &&
         isVersionToken(provenance.ros2_control_version) &&
         isVersionToken(provenance.libfranka_version);
}

BuildProvenance currentBuildProvenance() {
  return sanitizeBuildProvenance(BuildProvenance{
      build_provenance::kSourceCommit,
      build_provenance::kSourceDirty,
      build_provenance::kRosDistro,
      build_provenance::kRclcppVersion,
      build_provenance::kRos2ControlVersion,
      build_provenance::kLibfrankaVersion,
  });
}

const char* backendFailureReasonLabel(BackendFailureReason reason) noexcept {
  switch (reason) {
    case BackendFailureReason::None:
      return "none";
    case BackendFailureReason::WorkerStartupFailure:
      return "worker_startup_failure";
    case BackendFailureReason::WorkerStateTransitionFailure:
      return "worker_state_transition_failure";
    case BackendFailureReason::UnexpectedLoopReturn:
      return "unexpected_loop_return";
    case BackendFailureReason::RobotReflex:
      return "robot_reflex";
    case BackendFailureReason::FrankaControlException:
      return "franka_control_exception";
    case BackendFailureReason::FrankaCommandException:
      return "franka_command_exception";
    case BackendFailureReason::FrankaNetworkException:
      return "franka_network_exception";
    case BackendFailureReason::FrankaProtocolException:
      return "franka_protocol_exception";
    case BackendFailureReason::FrankaIncompatibleVersionException:
      return "franka_incompatible_version_exception";
    case BackendFailureReason::FrankaRealtimeException:
      return "franka_realtime_exception";
    case BackendFailureReason::FrankaModelException:
      return "franka_model_exception";
    case BackendFailureReason::FrankaInvalidOperationException:
      return "franka_invalid_operation_exception";
    case BackendFailureReason::FrankaException:
      return "franka_exception";
    case BackendFailureReason::StandardException:
      return "standard_exception";
    case BackendFailureReason::UnknownException:
      return "unknown_exception";
  }
  return "invalid";
}

const char* backendRecoveryResultLabel(BackendRecoveryResult result) noexcept {
  switch (result) {
    case BackendRecoveryResult::NeverAttempted:
      return "never_attempted";
    case BackendRecoveryResult::Succeeded:
      return "succeeded";
    case BackendRecoveryResult::Failed:
      return "failed";
  }
  return "invalid";
}

void formatFrankaArmDiagnosticStatus(const FrankaArmDiagnosticSnapshot& snapshot,
                                     uint64_t now_steady_ns,
                                     diagnostic_updater::DiagnosticStatusWrapper& status) {
  const auto& diagnostics = snapshot.backend;
  const uint8_t lifecycle_id = snapshot.hardware_lifecycle.id;
  const bool active_hardware = lifecycle_id == kLifecycleActive;
  const bool transition_state = lifecycle_id >= kFirstTransitionState;
  const bool age_not_applicable = stateAgeNotApplicable(lifecycle_id);
  const bool timestamp_valid =
      diagnostics.has_state_sample && diagnostics.last_accepted_state_steady_ns <= now_steady_ns;
  const uint64_t age_ns =
      timestamp_valid ? now_steady_ns - diagnostics.last_accepted_state_steady_ns : 0;
  const uint64_t age_ms = age_ns / 1000000U;

  status.hardware_id = snapshot.arm_id;
  status.add("arm_id", snapshot.arm_id);
  status.add("hardware_lifecycle_id", static_cast<uint32_t>(lifecycle_id));
  status.add("hardware_lifecycle_label", snapshot.hardware_lifecycle.label);
  status.add("requested_mode", controlModeLabel(diagnostics.requested_mode));
  status.add("active_mode", controlModeLabel(diagnostics.active_mode));
  status.add("worker_state", workerStateLabel(diagnostics.worker_state));
  status.add("fault_category", faultCategoryLabel(diagnostics.fault_category));
  status.add("failure_reason", backendFailureReasonLabel(diagnostics.failure_reason));
  status.add("service_operation", serviceOperationLabel(diagnostics.service_operation));
  status.add("stopped", boolLabel(diagnostics.stopped));
  status.add("recovering", boolLabel(diagnostics.recovering));
  if (age_not_applicable) {
    status.add("state_age_ms", "not_applicable");
  } else if (!timestamp_valid) {
    status.add("state_age_ms", "not_available");
  } else {
    status.add("state_age_ms", age_ms);
  }
  status.add("accepted_state_samples", diagnostics.accepted_state_samples);
  status.add("dropped_state_samples", diagnostics.dropped_state_samples);
  status.add("rejected_backend_commands", diagnostics.rejected_command_samples);
  status.add("state_queue_saturated", boolLabel(diagnostics.state_queue_saturated));
  status.add("command_queue_saturated", boolLabel(diagnostics.command_queue_saturated));
  status.add("recovery_attempts", diagnostics.recovery_attempts);
  status.add("recovery_successes", diagnostics.recovery_successes);
  status.add("recovery_failures", diagnostics.recovery_failures);
  status.add("last_recovery_result", backendRecoveryResultLabel(diagnostics.last_recovery_result));
  status.add("global_fault_origin_arm_slot",
             static_cast<uint32_t>(snapshot.global_fault.origin_arm_slot));
  status.add("global_fault_origin_arm_id", snapshot.global_fault_origin_arm_id);
  status.add("global_fault_cause", globalFaultCauseLabel(snapshot.global_fault.cause));
  status.add("unsafe_safe_publish_mask",
             static_cast<uint32_t>(snapshot.global_fault.unsafe_safe_publish_mask));
  status.add("unsafe_none_request_mask",
             static_cast<uint32_t>(snapshot.global_fault.unsafe_none_request_mask));
  status.add("source_commit", snapshot.provenance.source_commit);
  status.add("source_dirty", boolLabel(snapshot.provenance.source_dirty));
  status.add("ros_distro", snapshot.provenance.ros_distro);
  status.add("rclcpp_version", snapshot.provenance.rclcpp_version);
  status.add("ros2_control_version", snapshot.provenance.ros2_control_version);
  status.add("libfranka_version", snapshot.provenance.libfranka_version);

  using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
  if (snapshot.global_fault.latched()) {
    status.summary(DiagnosticStatus::ERROR, "global fault latched");
  } else if (diagnostics.fault_category != BackendFaultCategory::None ||
             diagnostics.worker_state == BackendWorkerState::Faulted ||
             diagnostics.failure_reason != BackendFailureReason::None) {
    status.summary(DiagnosticStatus::ERROR, "backend worker fault");
  } else if (diagnostics.last_recovery_result == BackendRecoveryResult::Failed) {
    status.summary(DiagnosticStatus::ERROR, "recovery failed");
  } else if (active_hardware && !timestamp_valid) {
    status.summary(DiagnosticStatus::ERROR, "active hardware has no accepted state sample");
  } else if (active_hardware && age_ns > kErrorStateAgeNanoseconds) {
    status.summary(DiagnosticStatus::ERROR, "accepted state sample is stale");
  } else if (diagnostics.worker_state == BackendWorkerState::Starting ||
             diagnostics.worker_state == BackendWorkerState::StopRequested ||
             diagnostics.recovering || transition_state) {
    status.summary(DiagnosticStatus::WARN, "backend transition in progress");
  } else if (diagnostics.requested_mode != diagnostics.active_mode) {
    status.summary(DiagnosticStatus::WARN, "requested and active modes differ");
  } else if (!age_not_applicable && timestamp_valid && age_ns > kWarningStateAgeNanoseconds) {
    status.summary(DiagnosticStatus::WARN, "accepted state sample is aging");
  } else if (diagnostics.state_queue_saturated || diagnostics.command_queue_saturated) {
    status.summary(DiagnosticStatus::WARN, "backend queue is saturated");
  } else if (diagnostics.dropped_state_samples != 0 || diagnostics.rejected_command_samples != 0) {
    status.summary(DiagnosticStatus::WARN, "backend has dropped or rejected data");
  } else if (lifecycle_id == kLifecycleUnknown) {
    status.summary(DiagnosticStatus::WARN, "hardware lifecycle is unknown");
  } else {
    status.summary(DiagnosticStatus::OK,
                   active_hardware ? "backend state is healthy" : "hardware is safely inactive");
  }
}

FrankaHardwareDiagnosticsNode::FrankaHardwareDiagnosticsNode(
    const rclcpp::NodeOptions& options,
    std::vector<FrankaArmDiagnosticSource> arm_sources,
    GlobalFaultDiagnosticProvider global_fault_provider,
    HardwareLifecycleProvider hardware_lifecycle_provider,
    double update_period_seconds)
    : rclcpp::Node("franka_hardware_diagnostics", options),
      arm_sources_(std::move(arm_sources)),
      global_fault_provider_(std::move(global_fault_provider)),
      hardware_lifecycle_provider_(std::move(hardware_lifecycle_provider)),
      provenance_(currentBuildProvenance()),
      updater_(this, update_period_seconds) {
  if (arm_sources_.empty() || arm_sources_.size() > 2 || !global_fault_provider_ ||
      !hardware_lifecycle_provider_ || update_period_seconds <= 0.0) {
    throw std::invalid_argument("invalid Franka hardware diagnostics configuration");
  }
  RCLCPP_INFO(get_logger(),
              "Franka hardware build: commit=%s dirty=%s ros=%s rclcpp=%s ros2_control=%s "
              "libfranka=%s",
              provenance_.source_commit.c_str(), boolLabel(provenance_.source_dirty),
              provenance_.ros_distro.c_str(), provenance_.rclcpp_version.c_str(),
              provenance_.ros2_control_version.c_str(), provenance_.libfranka_version.c_str());
  updater_.setHardwareID("franka_multi_hardware");
  for (size_t arm_index = 0; arm_index < arm_sources_.size(); ++arm_index) {
    if (arm_sources_[arm_index].arm_id.empty() || !arm_sources_[arm_index].backend) {
      throw std::invalid_argument("Franka hardware diagnostics arm source is invalid");
    }
    updater_.add("franka_hardware/" + arm_sources_[arm_index].arm_id,
                 [this, arm_index](diagnostic_updater::DiagnosticStatusWrapper& status) {
                   diagnoseArm(arm_index, status);
                 });
  }
}

void FrankaHardwareDiagnosticsNode::diagnoseArm(
    size_t arm_index,
    diagnostic_updater::DiagnosticStatusWrapper& status) {
  const auto global_fault = global_fault_provider_();
  std::string origin_arm_id{"none"};
  if (global_fault.origin_arm_slot != 0 && global_fault.origin_arm_slot <= arm_sources_.size()) {
    origin_arm_id = arm_sources_[global_fault.origin_arm_slot - 1].arm_id;
  }
  const auto& source = arm_sources_.at(arm_index);
  formatFrankaArmDiagnosticStatus(
      FrankaArmDiagnosticSnapshot{source.arm_id, hardware_lifecycle_provider_(),
                                  source.backend->diagnostics(), global_fault,
                                  std::move(origin_arm_id), provenance_},
      steadyNowNanoseconds(), status);
}

}  // namespace franka_hardware
