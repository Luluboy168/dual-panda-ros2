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

#include "support/synthetic_franka_arm_backend.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace franka_hardware::test_support
{
namespace
{

std::array<double, 16> makeTransform(double x, double y, double z)
{
  std::array<double, 16> transform{};
  transform[0] = 1.0;
  transform[5] = 1.0;
  transform[10] = 1.0;
  transform[12] = x;
  transform[13] = y;
  transform[14] = z;
  transform[15] = 1.0;
  return transform;
}

bool isValidControlMode(ControlMode mode) noexcept
{
  switch (mode) {
    case ControlMode::None:
    case ControlMode::JointTorque:
    case ControlMode::JointPosition:
    case ControlMode::JointVelocity:
    case ControlMode::CartesianVelocity:
    case ControlMode::CartesianPose:
      return true;
  }
  return false;
}

template <size_t Size>
bool allFinite(const std::array<double, Size> & values) noexcept
{
  return std::all_of(
    values.begin(), values.end(), [](double value) { return std::isfinite(value); });
}

bool isSafeSnapshot(const RobotCommand & command, const franka::RobotState & state) noexcept
{
  return std::all_of(
           command.efforts.begin(), command.efforts.end(),
           [](double value) { return value == 0.0; }) &&
         std::all_of(
           command.joint_velocities.begin(), command.joint_velocities.end(),
           [](double value) { return value == 0.0; }) &&
         std::all_of(
           command.cartesian_velocities.begin(), command.cartesian_velocities.end(),
           [](double value) { return value == 0.0; }) &&
         command.joint_positions == state.q && command.cartesian_positions == state.O_T_EE;
}

class ServiceOperationReset
{
public:
  explicit ServiceOperationReset(std::atomic<BackendServiceOperation> & operation) noexcept
  : operation_(operation)
  {
  }
  ServiceOperationReset(const ServiceOperationReset &) = delete;
  ServiceOperationReset & operator=(const ServiceOperationReset &) = delete;
  ~ServiceOperationReset() { operation_.store(BackendServiceOperation::Idle); }

private:
  std::atomic<BackendServiceOperation> & operation_;
};

}  // namespace

const char * SyntheticBackendException::what() const noexcept
{
  switch (code_) {
    case SyntheticErrorCode::InjectedConstructionFailure:
      return "injected synthetic backend construction failure";
    case SyntheticErrorCode::InjectedInitialReadFailure:
      return "injected synthetic backend initial read failure";
    case SyntheticErrorCode::UnsafeParameterOperation:
      return "synthetic parameter operation is not allowed in the current state";
    case SyntheticErrorCode::InjectedParameterFailure:
      return "injected synthetic parameter operation failure";
    case SyntheticErrorCode::NullParameterRequest:
      return "synthetic parameter request is null";
  }
  return "unknown synthetic backend failure";
}

franka::RobotState makeSyntheticRobotState(uint8_t arm_marker, uint64_t timestamp_ms)
{
  if (arm_marker == 0) {
    throw std::invalid_argument("synthetic state arm marker must be nonzero");
  }

  franka::RobotState state{};
  if (arm_marker % 2 == 1) {
    state.q = {0.11, -0.21, 0.31, -1.41, 0.51, 1.61, 0.71};
    state.dq = {0.011, -0.012, 0.013, -0.014, 0.015, -0.016, 0.017};
    state.tau_J = {1.11, 1.12, 1.13, 1.14, 1.15, 1.16, 1.17};
  } else {
    state.q = {-0.12, 0.22, -0.32, -1.52, -0.42, 1.72, -0.62};
    state.dq = {-0.021, 0.022, -0.023, 0.024, -0.025, 0.026, -0.027};
    state.tau_J = {2.21, 2.22, 2.23, 2.24, 2.25, 2.26, 2.27};
  }
  state.q_d = state.q;
  state.dq_d = state.dq;
  state.theta = state.q;
  state.dtheta = state.dq;

  const double marker = static_cast<double>(arm_marker);
  state.O_T_EE = makeTransform(
    0.40 + marker * 0.01, (arm_marker % 2 == 1 ? 1.0 : -1.0) * marker * 0.01, 0.50 + marker * 0.01);
  state.O_T_EE_d = state.O_T_EE;
  state.O_T_EE_c = state.O_T_EE;
  state.F_T_EE = makeTransform(0.0, 0.0, 0.10 + marker * 0.001);
  state.F_T_NE = makeTransform(0.0, 0.0, 0.10);
  state.NE_T_EE = makeTransform(0.0, 0.0, marker * 0.001);
  state.EE_T_K = makeTransform(0.0, 0.0, 0.0);
  state.m_ee = 0.50 + marker * 0.01;
  state.m_load = 0.10 + marker * 0.01;
  state.m_total = state.m_ee + state.m_load;
  state.control_command_success_rate = 1.0;
  state.robot_mode = franka::RobotMode::kIdle;
  state.time = franka::Duration(timestamp_ms);
  return state;
}

SyntheticFrankaArmBackendConfig SyntheticFrankaArmBackendConfig::forArm(uint8_t arm_marker)
{
  SyntheticFrankaArmBackendConfig config;
  config.arm_marker = arm_marker;
  config.initial_state =
    makeSyntheticRobotState(arm_marker, static_cast<uint64_t>(arm_marker) * 1000);
  config.initial_sequence = static_cast<uint64_t>(arm_marker) * 100;
  return config;
}

SyntheticFrankaArmBackend::SyntheticFrankaArmBackend(const SyntheticFrankaArmBackendConfig & config)
: arm_marker_(config.arm_marker),
  failure_(config.failure),
  model_(config.arm_marker, config.model_coriolis_scale),
  initial_state_(config.initial_state),
  last_state_(config.initial_state),
  initial_state_steady_ns_(config.initial_state_steady_ns),
  initial_sequence_(config.initial_sequence),
  timestamp_step_ms_(config.timestamp_step_ms),
  command_queue_capacity_(config.command_queue_capacity)
{
  if (arm_marker_ == 0) {
    throw std::invalid_argument("synthetic backend arm marker must be nonzero");
  }
  if (initial_sequence_ == 0) {
    throw std::invalid_argument("synthetic backend initial sequence must be nonzero");
  }
  if (command_queue_capacity_ == 0 || command_queue_capacity_ > kCommandCaptureCapacity) {
    throw std::invalid_argument("synthetic backend command queue capacity is invalid");
  }
  if (config.replay_states.size() > kMaximumReplayStates) {
    throw std::invalid_argument("synthetic backend replay exceeds its fixed capacity");
  }
  if (
    !config.replay_state_steady_ns.empty() &&
    config.replay_state_steady_ns.size() != config.replay_states.size()) {
    throw std::invalid_argument("synthetic backend replay timestamps do not match replay states");
  }
  if (!config.replay_state_steady_ns.empty()) {
    if (initial_state_steady_ns_ == 0) {
      throw std::invalid_argument("synthetic backend initial replay timestamp must be positive");
    }
    uint64_t previous = initial_state_steady_ns_;
    for (const auto timestamp : config.replay_state_steady_ns) {
      if (timestamp <= previous) {
        throw std::invalid_argument("synthetic backend replay timestamps must increase");
      }
      previous = timestamp;
    }
  }
  if (failure_.point != SyntheticFailurePoint::None && failure_.fail_on_call == 0) {
    throw std::invalid_argument("synthetic backend failure call must be nonzero");
  }
  if (failure_.point == SyntheticFailurePoint::Construction && failure_.fail_on_call == 1) {
    throw SyntheticBackendException(SyntheticErrorCode::InjectedConstructionFailure);
  }

  replay_size_ = config.replay_states.size();
  std::copy(config.replay_states.begin(), config.replay_states.end(), replay_states_.begin());
  std::copy(
    config.replay_state_steady_ns.begin(), config.replay_state_steady_ns.end(),
    replay_state_steady_ns_.begin());
}

bool SyntheticFrankaArmBackend::startStateReading()
{
  auto expected = BackendServiceOperation::Idle;
  if (!service_operation_.compare_exchange_strong(expected, BackendServiceOperation::Lifecycle)) {
    return false;
  }
  ServiceOperationReset reset(service_operation_);
  recordEvent(SyntheticEventKind::StartAttempt, SyntheticFailurePoint::StartStateReading);
  if (faulted_ || !stopped_ || worker_state_ != BackendWorkerState::Stopped) {
    return false;
  }
  clearBackendFailureReasons(failure_reason_mask_);
  if (shouldFail(SyntheticFailurePoint::StartStateReading)) {
    latchFault(SyntheticCondition::StartFailure, SyntheticFailurePoint::StartStateReading);
    return false;
  }
  requested_mode_ = ControlMode::None;
  active_mode_ = ControlMode::None;
  worker_state_ = BackendWorkerState::Running;
  stopped_ = false;
  lifecycle_active_ = true;
  recordEvent(SyntheticEventKind::Started);
  return true;
}

bool SyntheticFrankaArmBackend::stop()
{
  auto expected = BackendServiceOperation::Idle;
  if (!service_operation_.compare_exchange_strong(expected, BackendServiceOperation::Lifecycle)) {
    return false;
  }
  ServiceOperationReset reset(service_operation_);
  recordEvent(SyntheticEventKind::StopAttempt, SyntheticFailurePoint::Shutdown);
  lifecycle_active_ = false;
  if (shouldFail(SyntheticFailurePoint::Shutdown)) {
    latchFault(SyntheticCondition::ShutdownFailure, SyntheticFailurePoint::Shutdown);
    return false;
  }
  requested_mode_ = ControlMode::None;
  active_mode_ = ControlMode::None;
  worker_state_ = faulted_ ? BackendWorkerState::Faulted : BackendWorkerState::Stopped;
  stopped_ = true;
  in_flight_commands_ = 0;
  recordEvent(SyntheticEventKind::Stopped);
  return true;
}

franka::RobotState SyntheticFrankaArmBackend::readLatestState()
{
  recordEvent(SyntheticEventKind::ReadAttempt);
  if (faulted_) {
    return last_state_;
  }
  if (!initial_state_returned_ && shouldFail(SyntheticFailurePoint::InitialRead)) {
    latchFault(SyntheticCondition::InitialReadFailure, SyntheticFailurePoint::InitialRead);
    throw SyntheticBackendException(SyntheticErrorCode::InjectedInitialReadFailure);
  }

  if (initial_state_returned_ && worker_state_ == BackendWorkerState::Running) {
    if (shouldFail(SyntheticFailurePoint::UnexpectedLoopReturn)) {
      ++dropped_state_samples_;
      latchFault(
        SyntheticCondition::UnexpectedLoopReturn, SyntheticFailurePoint::UnexpectedLoopReturn);
      return last_state_;
    }
    if (shouldFail(SyntheticFailurePoint::ReadFault)) {
      ++dropped_state_samples_;
      latchFault(SyntheticCondition::ReadFault, SyntheticFailurePoint::ReadFault);
      return last_state_;
    }
    if (active_mode_ != ControlMode::None && shouldFail(SyntheticFailurePoint::ControlFault)) {
      ++dropped_state_samples_;
      latchFault(SyntheticCondition::ControlFault, SyntheticFailurePoint::ControlFault);
      return last_state_;
    }
    if (shouldFail(SyntheticFailurePoint::StateQueueSaturation)) {
      ++dropped_state_samples_;
      state_queue_saturated_.store(true, std::memory_order_release);
      condition_ = SyntheticCondition::StateQueueSaturation;
      return last_state_;
    }
    if (shouldFail(SyntheticFailurePoint::ExplicitStaleState)) {
      explicitly_stale_ = true;
      ++dropped_state_samples_;
      latchFault(SyntheticCondition::ExplicitStaleState, SyntheticFailurePoint::ExplicitStaleState);
      return last_state_;
    }
  }

  auto candidate = nextCandidateState();
  bool invalid_candidate_injected = false;
  if (shouldFail(SyntheticFailurePoint::InvalidNanState)) {
    candidate.q[0] = std::numeric_limits<double>::quiet_NaN();
    invalid_candidate_injected = true;
  }
  if (shouldFail(SyntheticFailurePoint::InvalidInfiniteState)) {
    candidate.O_T_EE[12] = std::numeric_limits<double>::infinity();
    invalid_candidate_injected = true;
  }
  if (!validateTimestamp(candidate)) {
    ++dropped_state_samples_;
    latchFault(SyntheticCondition::TimestampRegression, SyntheticFailurePoint::None);
    return last_state_;
  }
  if (invalid_candidate_injected) {
    initial_state_returned_ = true;
    last_sequence_ = initial_sequence_ + successful_read_count_;
    ++successful_read_count_;
    recordEvent(
      SyntheticEventKind::StateReturned, SyntheticFailurePoint::None, ControlMode::None,
      last_sequence_);
    return candidate;
  }

  initial_state_returned_ = true;
  last_state_ = candidate;
  last_sequence_ = initial_sequence_ + successful_read_count_;
  ++successful_read_count_;
  if (worker_state_ == BackendWorkerState::Running) {
    in_flight_commands_ = 0;
  }
  recordAcceptedStateSample(next_accepted_state_steady_ns_);
  recordEvent(
    SyntheticEventKind::StateReturned, SyntheticFailurePoint::None, ControlMode::None,
    last_sequence_);
  return last_state_;
}

ModelBase * SyntheticFrankaArmBackend::model() noexcept { return &model_; }

bool SyntheticFrankaArmBackend::canPublishCommand() const noexcept
{
  return !faulted_ && in_flight_commands_ < command_queue_capacity_;
}

bool SyntheticFrankaArmBackend::publishCommand(const RobotCommand & command) noexcept
{
  if (!isCommandFinite(command)) {
    ++rejected_command_samples_;
    command_queue_saturated_.store(false, std::memory_order_release);
    condition_ = SyntheticCondition::InvalidCommand;
    recordEvent(SyntheticEventKind::CommandRejected);
    return false;
  }
  if (shouldFail(SyntheticFailurePoint::CommandPublish)) {
    ++rejected_command_samples_;
    command_queue_saturated_.store(false, std::memory_order_release);
    condition_ = SyntheticCondition::CommandPublishFailure;
    recordEvent(SyntheticEventKind::CommandRejected, SyntheticFailurePoint::CommandPublish);
    return false;
  }
  if (shouldFail(SyntheticFailurePoint::QueueSaturation) || !canPublishCommand()) {
    ++rejected_command_samples_;
    command_queue_saturated_.store(true, std::memory_order_release);
    condition_ = SyntheticCondition::QueueSaturation;
    recordEvent(SyntheticEventKind::CommandRejected, SyntheticFailurePoint::QueueSaturation);
    return false;
  }

  const size_t slot = static_cast<size_t>(accepted_command_count_ % kCommandCaptureCapacity);
  command_capture_[slot] = command;
  ++accepted_command_count_;
  command_capture_size_ =
    static_cast<size_t>(std::min<uint64_t>(accepted_command_count_, kCommandCaptureCapacity));
  if (isSafeSnapshot(command, last_state_)) {
    ++accepted_safe_snapshot_count_;
  } else {
    ++accepted_unsafe_snapshot_count_;
  }
  ++in_flight_commands_;
  command_queue_saturated_.store(false, std::memory_order_release);
  recordEvent(SyntheticEventKind::CommandAccepted);
  return true;
}

bool SyntheticFrankaArmBackend::canRequestControlMode(ControlMode control_mode) const noexcept
{
  return isValidControlMode(control_mode) && !faulted_ &&
         service_operation_ == BackendServiceOperation::Idle &&
         worker_state_ == BackendWorkerState::Running;
}

bool SyntheticFrankaArmBackend::requestControlMode(ControlMode control_mode) noexcept
{
  auto expected = BackendServiceOperation::Idle;
  if (!service_operation_.compare_exchange_strong(expected, BackendServiceOperation::ModeRequest)) {
    rejected_mode_request_count_.fetch_add(1, std::memory_order_relaxed);
    if (control_mode != ControlMode::None) {
      rejected_non_none_mode_request_count_.fetch_add(1, std::memory_order_relaxed);
    }
    return false;
  }
  ServiceOperationReset reset(service_operation_);
  if (
    !isValidControlMode(control_mode) || faulted_ || worker_state_ != BackendWorkerState::Running) {
    rejected_mode_request_count_.fetch_add(1, std::memory_order_relaxed);
    if (control_mode != ControlMode::None) {
      rejected_non_none_mode_request_count_.fetch_add(1, std::memory_order_relaxed);
    }
    recordEvent(SyntheticEventKind::ModeRejected, SyntheticFailurePoint::ModeRequest, control_mode);
    return false;
  }
  if (shouldFail(SyntheticFailurePoint::ModeRequest)) {
    condition_ = SyntheticCondition::ModeRequestFailure;
    rejected_mode_request_count_.fetch_add(1, std::memory_order_relaxed);
    if (control_mode != ControlMode::None) {
      rejected_non_none_mode_request_count_.fetch_add(1, std::memory_order_relaxed);
    }
    recordEvent(SyntheticEventKind::ModeRejected, SyntheticFailurePoint::ModeRequest, control_mode);
    return false;
  }
  requested_mode_ = control_mode;
  active_mode_ = control_mode;
  accepted_mode_request_count_.fetch_add(1, std::memory_order_relaxed);
  if (control_mode != ControlMode::None) {
    accepted_non_none_mode_request_count_.fetch_add(1, std::memory_order_relaxed);
  }
  recordEvent(SyntheticEventKind::ModeRequested, SyntheticFailurePoint::None, control_mode);
  return true;
}

ControlMode SyntheticFrankaArmBackend::requestedControlMode() const noexcept
{
  return requested_mode_;
}

ControlMode SyntheticFrankaArmBackend::activeControlMode() const noexcept { return active_mode_; }

bool SyntheticFrankaArmBackend::hasFault() const noexcept { return faulted_; }

bool SyntheticFrankaArmBackend::recoverToReading()
{
  auto expected = BackendServiceOperation::Idle;
  if (!service_operation_.compare_exchange_strong(expected, BackendServiceOperation::Recovery)) {
    return false;
  }
  ServiceOperationReset reset(service_operation_);
  recovery_attempts_.fetch_add(1, std::memory_order_relaxed);
  recordEvent(SyntheticEventKind::RecoveryAttempt, SyntheticFailurePoint::Recovery);
  if (!faulted_) {
    recordEvent(SyntheticEventKind::RecoveryRejected, SyntheticFailurePoint::Recovery);
    finishRecoveryAttempt(false);
    return false;
  }
  requested_mode_ = ControlMode::None;
  active_mode_ = ControlMode::None;
  if (shouldFail(SyntheticFailurePoint::Recovery)) {
    condition_ = SyntheticCondition::RecoveryFailure;
    worker_state_ = BackendWorkerState::Faulted;
    stopped_ = true;
    recordEvent(SyntheticEventKind::RecoveryRejected, SyntheticFailurePoint::Recovery);
    finishRecoveryAttempt(false);
    return false;
  }

  faulted_ = false;
  explicitly_stale_ = false;
  condition_ = SyntheticCondition::None;
  worker_state_ = lifecycle_active_ ? BackendWorkerState::Running : BackendWorkerState::Stopped;
  stopped_ = !lifecycle_active_;
  in_flight_commands_ = 0;
  clearBackendFailureReasons(failure_reason_mask_);
  finishRecoveryAttempt(true);
  recordEvent(SyntheticEventKind::Recovered);
  return true;
}

FrankaArmBackendDiagnostics SyntheticFrankaArmBackend::diagnostics() const noexcept
{
  FrankaArmBackendDiagnostics result;
  result.requested_mode = requested_mode_;
  result.active_mode = active_mode_;
  result.worker_state = worker_state_;
  result.fault_category = faulted_ ? BackendFaultCategory::Worker : BackendFaultCategory::None;
  result.failure_reason =
    backendFailureReasonFromMask(failure_reason_mask_.load(std::memory_order_acquire));
  result.service_operation = service_operation_;
  result.stopped = stopped_;
  result.recovering = service_operation_ == BackendServiceOperation::Recovery;
  result.has_state_sample = has_state_sample_.load(std::memory_order_acquire);
  result.accepted_state_samples = accepted_state_samples_.load(std::memory_order_acquire);
  result.last_accepted_state_steady_ns =
    last_accepted_state_steady_ns_.load(std::memory_order_acquire);
  result.dropped_state_samples = dropped_state_samples_;
  result.rejected_command_samples = rejected_command_samples_;
  result.state_queue_saturated = state_queue_saturated_.load(std::memory_order_acquire);
  result.command_queue_saturated =
    command_queue_saturated_.load(std::memory_order_acquire) ||
    in_flight_commands_.load(std::memory_order_acquire) >= command_queue_capacity_;
  result.recovery_attempts = recovery_attempts_.load(std::memory_order_acquire);
  result.recovery_successes = recovery_successes_.load(std::memory_order_acquire);
  result.recovery_failures = recovery_failures_.load(std::memory_order_acquire);
  result.last_recovery_result = last_recovery_result_.load(std::memory_order_acquire);
  return result;
}

void SyntheticFrankaArmBackend::setJointStiffness(
  const franka_msgs::srv::SetJointStiffness::Request::SharedPtr & request)
{
  if (!request) {
    throw SyntheticBackendException(SyntheticErrorCode::NullParameterRequest);
  }
  beginParameterOperation(SyntheticFailurePoint::JointStiffness);
  parameter_snapshot_.joint_stiffness = request->joint_stiffness;
  acceptParameterOperation(SyntheticFailurePoint::JointStiffness);
}

void SyntheticFrankaArmBackend::setCartesianStiffness(
  const franka_msgs::srv::SetCartesianStiffness::Request::SharedPtr & request)
{
  if (!request) {
    throw SyntheticBackendException(SyntheticErrorCode::NullParameterRequest);
  }
  beginParameterOperation(SyntheticFailurePoint::CartesianStiffness);
  parameter_snapshot_.cartesian_stiffness = request->cartesian_stiffness;
  acceptParameterOperation(SyntheticFailurePoint::CartesianStiffness);
}

void SyntheticFrankaArmBackend::setLoad(
  const franka_msgs::srv::SetLoad::Request::SharedPtr & request)
{
  if (!request) {
    throw SyntheticBackendException(SyntheticErrorCode::NullParameterRequest);
  }
  beginParameterOperation(SyntheticFailurePoint::Load);
  parameter_snapshot_.load_mass = request->mass;
  parameter_snapshot_.load_center_of_mass = request->center_of_mass;
  parameter_snapshot_.load_inertia = request->load_inertia;
  acceptParameterOperation(SyntheticFailurePoint::Load);
}

void SyntheticFrankaArmBackend::setTCPFrame(
  const franka_msgs::srv::SetTCPFrame::Request::SharedPtr & request)
{
  if (!request) {
    throw SyntheticBackendException(SyntheticErrorCode::NullParameterRequest);
  }
  beginParameterOperation(SyntheticFailurePoint::TCPFrame);
  parameter_snapshot_.tcp_frame = request->transformation;
  acceptParameterOperation(SyntheticFailurePoint::TCPFrame);
}

void SyntheticFrankaArmBackend::setStiffnessFrame(
  const franka_msgs::srv::SetStiffnessFrame::Request::SharedPtr & request)
{
  if (!request) {
    throw SyntheticBackendException(SyntheticErrorCode::NullParameterRequest);
  }
  beginParameterOperation(SyntheticFailurePoint::StiffnessFrame);
  parameter_snapshot_.stiffness_frame = request->transformation;
  acceptParameterOperation(SyntheticFailurePoint::StiffnessFrame);
}

void SyntheticFrankaArmBackend::setForceTorqueCollisionBehavior(
  const franka_msgs::srv::SetForceTorqueCollisionBehavior::Request::SharedPtr & request)
{
  if (!request) {
    throw SyntheticBackendException(SyntheticErrorCode::NullParameterRequest);
  }
  beginParameterOperation(SyntheticFailurePoint::ForceTorqueCollisionBehavior);
  parameter_snapshot_.lower_torque_thresholds_nominal = request->lower_torque_thresholds_nominal;
  parameter_snapshot_.upper_torque_thresholds_nominal = request->upper_torque_thresholds_nominal;
  parameter_snapshot_.lower_force_thresholds_nominal = request->lower_force_thresholds_nominal;
  parameter_snapshot_.upper_force_thresholds_nominal = request->upper_force_thresholds_nominal;
  acceptParameterOperation(SyntheticFailurePoint::ForceTorqueCollisionBehavior);
}

void SyntheticFrankaArmBackend::setFullCollisionBehavior(
  const franka_msgs::srv::SetFullCollisionBehavior::Request::SharedPtr & request)
{
  if (!request) {
    throw SyntheticBackendException(SyntheticErrorCode::NullParameterRequest);
  }
  beginParameterOperation(SyntheticFailurePoint::FullCollisionBehavior);
  parameter_snapshot_.lower_torque_thresholds_acceleration =
    request->lower_torque_thresholds_acceleration;
  parameter_snapshot_.upper_torque_thresholds_acceleration =
    request->upper_torque_thresholds_acceleration;
  parameter_snapshot_.lower_torque_thresholds_nominal = request->lower_torque_thresholds_nominal;
  parameter_snapshot_.upper_torque_thresholds_nominal = request->upper_torque_thresholds_nominal;
  parameter_snapshot_.lower_force_thresholds_acceleration =
    request->lower_force_thresholds_acceleration;
  parameter_snapshot_.upper_force_thresholds_acceleration =
    request->upper_force_thresholds_acceleration;
  parameter_snapshot_.lower_force_thresholds_nominal = request->lower_force_thresholds_nominal;
  parameter_snapshot_.upper_force_thresholds_nominal = request->upper_force_thresholds_nominal;
  acceptParameterOperation(SyntheticFailurePoint::FullCollisionBehavior);
}

uint64_t SyntheticFrankaArmBackend::failureCallCount(SyntheticFailurePoint point) const noexcept
{
  const auto index = static_cast<size_t>(point);
  return index < failure_call_counts_.size() ? failure_call_counts_[index] : 0;
}

RobotCommand SyntheticFrankaArmBackend::capturedCommand(size_t index) const
{
  if (index >= command_capture_size_) {
    throw std::out_of_range("synthetic command capture index is out of range");
  }
  const uint64_t oldest = accepted_command_count_ - command_capture_size_;
  const size_t slot = static_cast<size_t>((oldest + index) % kCommandCaptureCapacity);
  return command_capture_[slot];
}

SyntheticEvent SyntheticFrankaArmBackend::capturedEvent(size_t index) const
{
  if (index >= event_capture_size_) {
    throw std::out_of_range("synthetic event capture index is out of range");
  }
  const uint64_t oldest = total_event_count_ - event_capture_size_;
  const size_t slot = static_cast<size_t>((oldest + index) % kEventCaptureCapacity);
  return event_capture_[slot];
}

bool SyntheticFrankaArmBackend::holdServiceOperationForTest(
  BackendServiceOperation operation) noexcept
{
  if (operation == BackendServiceOperation::Idle) {
    return false;
  }
  auto expected = BackendServiceOperation::Idle;
  return service_operation_.compare_exchange_strong(expected, operation);
}

void SyntheticFrankaArmBackend::releaseServiceOperationForTest() noexcept
{
  service_operation_ = BackendServiceOperation::Idle;
}

void SyntheticFrankaArmBackend::injectFaultForTest(SyntheticCondition condition) noexcept
{
  if (condition == SyntheticCondition::None) {
    condition = SyntheticCondition::ControlFault;
  }
  latchFault(condition, SyntheticFailurePoint::ControlFault);
}

bool SyntheticFrankaArmBackend::shouldFail(SyntheticFailurePoint point) noexcept
{
  const auto index = static_cast<size_t>(point);
  if (index >= failure_call_counts_.size() || point == SyntheticFailurePoint::None) {
    return false;
  }
  const uint64_t call = ++failure_call_counts_[index];
  return failure_.point == point && call == failure_.fail_on_call;
}

bool SyntheticFrankaArmBackend::parameterOperationAllowed() const noexcept
{
  return !faulted_ &&
         (stopped_ || (requested_mode_ == ControlMode::None && active_mode_ == ControlMode::None));
}

void SyntheticFrankaArmBackend::beginParameterOperation(SyntheticFailurePoint point)
{
  auto expected = BackendServiceOperation::Idle;
  if (!service_operation_.compare_exchange_strong(expected, BackendServiceOperation::Parameter)) {
    throw SyntheticBackendException(SyntheticErrorCode::UnsafeParameterOperation);
  }
  if (!parameterOperationAllowed()) {
    recordEvent(SyntheticEventKind::ParameterRejected, point);
    endParameterOperation();
    throw SyntheticBackendException(SyntheticErrorCode::UnsafeParameterOperation);
  }
  if (shouldFail(point)) {
    condition_ = SyntheticCondition::ParameterFailure;
    recordEvent(SyntheticEventKind::ParameterRejected, point);
    endParameterOperation();
    throw SyntheticBackendException(SyntheticErrorCode::InjectedParameterFailure);
  }
}

void SyntheticFrankaArmBackend::endParameterOperation() noexcept
{
  service_operation_ = BackendServiceOperation::Idle;
}

void SyntheticFrankaArmBackend::latchFault(
  SyntheticCondition condition, SyntheticFailurePoint point) noexcept
{
  condition_ = condition;
  faulted_ = true;
  requested_mode_ = ControlMode::None;
  active_mode_ = ControlMode::None;
  worker_state_ = BackendWorkerState::Faulted;
  stopped_ = true;
  switch (condition) {
    case SyntheticCondition::UnexpectedLoopReturn:
      recordFailure(BackendFailureReason::UnexpectedLoopReturn);
      break;
    case SyntheticCondition::ControlFault:
      recordFailure(BackendFailureReason::FrankaControlException);
      break;
    case SyntheticCondition::ReadFault:
      recordFailure(BackendFailureReason::FrankaNetworkException);
      break;
    case SyntheticCondition::StartFailure:
      recordFailure(BackendFailureReason::WorkerStartupFailure);
      break;
    case SyntheticCondition::ShutdownFailure:
      recordFailure(BackendFailureReason::WorkerStateTransitionFailure);
      break;
    default:
      recordFailure(BackendFailureReason::StandardException);
      break;
  }
  recordEvent(SyntheticEventKind::FaultLatched, point);
}

void SyntheticFrankaArmBackend::recordFailure(BackendFailureReason reason) noexcept
{
  recordBackendFailureReason(failure_reason_mask_, reason);
}

void SyntheticFrankaArmBackend::recordAcceptedStateSample(uint64_t explicit_steady_ns) noexcept
{
  uint64_t accepted_steady_ns = explicit_steady_ns;
  if (accepted_steady_ns == 0) {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    accepted_steady_ns =
      static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
  }
  last_accepted_state_steady_ns_.store(accepted_steady_ns, std::memory_order_relaxed);
  accepted_state_samples_.fetch_add(1, std::memory_order_release);
  has_state_sample_.store(true, std::memory_order_release);
  state_queue_saturated_.store(false, std::memory_order_release);
}

void SyntheticFrankaArmBackend::finishRecoveryAttempt(bool succeeded) noexcept
{
  if (succeeded) {
    recovery_successes_.fetch_add(1, std::memory_order_relaxed);
    last_recovery_result_.store(BackendRecoveryResult::Succeeded, std::memory_order_release);
  } else {
    recovery_failures_.fetch_add(1, std::memory_order_relaxed);
    last_recovery_result_.store(BackendRecoveryResult::Failed, std::memory_order_release);
  }
}

void SyntheticFrankaArmBackend::recordEvent(
  SyntheticEventKind kind, SyntheticFailurePoint point, ControlMode mode,
  uint64_t sequence) noexcept
{
  const size_t slot = static_cast<size_t>(total_event_count_ % kEventCaptureCapacity);
  event_capture_[slot] = SyntheticEvent{kind, point, mode, sequence};
  ++total_event_count_;
  event_capture_size_ =
    static_cast<size_t>(std::min<uint64_t>(total_event_count_, kEventCaptureCapacity));
}

franka::RobotState SyntheticFrankaArmBackend::nextCandidateState() noexcept
{
  if (!initial_state_returned_) {
    next_accepted_state_steady_ns_ = initial_state_steady_ns_;
    return initial_state_;
  }
  if (replay_index_ < replay_size_) {
    const size_t index = replay_index_++;
    next_accepted_state_steady_ns_ = replay_state_steady_ns_[index];
    return replay_states_[index];
  }
  next_accepted_state_steady_ns_ = 0;
  auto candidate = last_state_;
  const uint64_t last_timestamp = last_state_.time.toMSec();
  if (timestamp_step_ms_ <= std::numeric_limits<uint64_t>::max() - last_timestamp) {
    candidate.time = franka::Duration(last_timestamp + timestamp_step_ms_);
  } else {
    candidate.time = franka::Duration(0);
  }
  return candidate;
}

bool SyntheticFrankaArmBackend::validateTimestamp(const franka::RobotState & candidate) noexcept
{
  return !initial_state_returned_ || candidate.time.toMSec() >= last_state_.time.toMSec();
}

bool SyntheticFrankaArmBackend::isCommandFinite(const RobotCommand & command) const noexcept
{
  return allFinite(command.efforts) && allFinite(command.joint_positions) &&
         allFinite(command.joint_velocities) && allFinite(command.cartesian_positions) &&
         allFinite(command.cartesian_velocities);
}

void SyntheticFrankaArmBackend::acceptParameterOperation(SyntheticFailurePoint point)
{
  recordEvent(SyntheticEventKind::ParameterAccepted, point);
  endParameterOperation();
}

}  // namespace franka_hardware::test_support
