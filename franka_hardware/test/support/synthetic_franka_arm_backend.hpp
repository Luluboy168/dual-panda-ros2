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
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <vector>

#include "franka_hardware/real/franka_arm_backend.hpp"
#include "support/synthetic_model.hpp"

namespace franka_hardware::test_support
{

enum class SyntheticFailurePoint : uint8_t
{
  None,
  Construction,
  InitialRead,
  StartStateReading,
  UnexpectedLoopReturn,
  ReadFault,
  ControlFault,
  StateQueueSaturation,
  CommandPublish,
  QueueSaturation,
  ModeRequest,
  ExplicitStaleState,
  InvalidNanState,
  InvalidInfiniteState,
  Recovery,
  Shutdown,
  JointStiffness,
  CartesianStiffness,
  Load,
  TCPFrame,
  StiffnessFrame,
  ForceTorqueCollisionBehavior,
  FullCollisionBehavior,
  Count,
};

enum class SyntheticCondition : uint8_t
{
  None,
  ConstructionFailure,
  InitialReadFailure,
  StartFailure,
  UnexpectedLoopReturn,
  ReadFault,
  ControlFault,
  StateQueueSaturation,
  CommandPublishFailure,
  QueueSaturation,
  ModeRequestFailure,
  ExplicitStaleState,
  TimestampRegression,
  RecoveryFailure,
  ShutdownFailure,
  InvalidCommand,
  ParameterFailure,
};

enum class SyntheticEventKind : uint8_t
{
  StartAttempt,
  Started,
  StopAttempt,
  Stopped,
  ReadAttempt,
  StateReturned,
  CommandAccepted,
  CommandRejected,
  ModeRequested,
  ModeRejected,
  FaultLatched,
  RecoveryAttempt,
  Recovered,
  RecoveryRejected,
  ParameterAccepted,
  ParameterRejected,
};

enum class SyntheticErrorCode : uint8_t
{
  InjectedConstructionFailure,
  InjectedInitialReadFailure,
  UnsafeParameterOperation,
  InjectedParameterFailure,
  NullParameterRequest,
};

class SyntheticBackendException final : public std::exception
{
public:
  explicit SyntheticBackendException(SyntheticErrorCode code) noexcept : code_(code) {}

  [[nodiscard]] SyntheticErrorCode code() const noexcept { return code_; }
  [[nodiscard]] const char * what() const noexcept override;

private:
  SyntheticErrorCode code_;
};

struct SyntheticFailureScript
{
  SyntheticFailurePoint point{SyntheticFailurePoint::None};
  uint64_t fail_on_call{1};
};

struct SyntheticEvent
{
  SyntheticEventKind kind{SyntheticEventKind::ReadAttempt};
  SyntheticFailurePoint failure_point{SyntheticFailurePoint::None};
  ControlMode mode{ControlMode::None};
  uint64_t sequence{0};
};

struct SyntheticParameterSnapshot
{
  std::array<double, 7> joint_stiffness{};
  std::array<double, 6> cartesian_stiffness{};
  double load_mass{0.0};
  std::array<double, 3> load_center_of_mass{};
  std::array<double, 9> load_inertia{};
  std::array<double, 16> tcp_frame{};
  std::array<double, 16> stiffness_frame{};
  std::array<double, 7> lower_torque_thresholds_nominal{};
  std::array<double, 7> upper_torque_thresholds_nominal{};
  std::array<double, 6> lower_force_thresholds_nominal{};
  std::array<double, 6> upper_force_thresholds_nominal{};
  std::array<double, 7> lower_torque_thresholds_acceleration{};
  std::array<double, 7> upper_torque_thresholds_acceleration{};
  std::array<double, 6> lower_force_thresholds_acceleration{};
  std::array<double, 6> upper_force_thresholds_acceleration{};
};

struct SyntheticFrankaArmBackendConfig
{
  uint8_t arm_marker{0};
  franka::RobotState initial_state{};
  uint64_t initial_state_steady_ns{0};
  uint64_t initial_sequence{1};
  uint64_t timestamp_step_ms{1};
  // F-10d (2026-08-28): matches the real command channel's capacity exactly --
  // Robot::kRealtimeBufferCapacity == 64 (franka_hardware/include/franka_hardware/real/robot.hpp:390),
  // the capacity of SpscRingBuffer<RobotCommand, 64> command_buffer_ (robot.hpp:411). The live
  // F-10d failure took exactly 64 unconsumed write() cycles to latch CommandCapacity; an emulated
  // channel of any other depth cannot reproduce that count.
  size_t command_queue_capacity{64};
  // F-10g (2026-08-28), amendment C.4: length in read cycles of the modelled mode-ENTRY window --
  // the stretch after a control-mode request during which the new control loop's first callback
  // has not yet run, so NOTHING consumes the command channel. The real window spans libfranka's
  // finishMotion() exit handshake plus its startMotion() entry handshake
  // (ControlLoopWorker::run(), control_loop_worker.hpp:186-257; Robot::runLoop(),
  // robot.cpp:286-351), measured at up to ~64 ms on hardware and never below zero.
  //
  // The default is deliberately NON-ZERO: entry is asynchronous on the real robot, so a model that
  // switches instantly is the same class of divergence that made the whole offline battery blind
  // to F-10d. 8 is a modelling choice, not a measurement -- large enough that every test crosses a
  // real window, far enough below the 64-slot channel that no test which is not about capacity can
  // saturate it. Set 0 for a deliberate legacy-instant test; set >62 to reproduce F-10g.
  size_t mode_entry_window_cycles{8};
  double model_coriolis_scale{1.0};
  std::vector<franka::RobotState> replay_states;
  std::vector<uint64_t> replay_state_steady_ns;
  SyntheticFailureScript failure{};

  [[nodiscard]] static SyntheticFrankaArmBackendConfig forArm(uint8_t arm_marker);
};

[[nodiscard]] franka::RobotState makeSyntheticRobotState(uint8_t arm_marker, uint64_t timestamp_ms);

// This backend is a deterministic test double. Accepted operations mutate it on one test thread;
// concurrent service-gate losers are safe because they return immediately after the atomic CAS.
// It owns no worker, performs no network I/O, and is compiled only into test targets.
class SyntheticFrankaArmBackend final : public FrankaArmBackend
{
public:
  static constexpr size_t kMaximumReplayStates = 32;
  static constexpr size_t kCommandCaptureCapacity = 64;
  static constexpr size_t kEventCaptureCapacity = 128;

  explicit SyntheticFrankaArmBackend(const SyntheticFrankaArmBackendConfig & config);

  bool startStateReading() override;
  bool stop() override;
  franka::RobotState readLatestState() override;
  ModelBase * model() noexcept override;

  bool canPublishCommand() const noexcept override;
  bool publishCommand(const RobotCommand & command) noexcept override;
  bool canRequestControlMode(ControlMode control_mode) const noexcept override;
  bool requestControlMode(ControlMode control_mode) noexcept override;
  ControlMode requestedControlMode() const noexcept override;
  ControlMode activeControlMode() const noexcept override;
  bool modeEntryInFlight() const noexcept override;

  bool hasFault() const noexcept override;
  bool recoverToReading() override;
  FrankaArmBackendDiagnostics diagnostics() const noexcept override;

  void setJointStiffness(
    const franka_msgs::srv::SetJointStiffness::Request::SharedPtr & request) override;
  void setCartesianStiffness(
    const franka_msgs::srv::SetCartesianStiffness::Request::SharedPtr & request) override;
  void setLoad(const franka_msgs::srv::SetLoad::Request::SharedPtr & request) override;
  void setTCPFrame(const franka_msgs::srv::SetTCPFrame::Request::SharedPtr & request) override;
  void setStiffnessFrame(
    const franka_msgs::srv::SetStiffnessFrame::Request::SharedPtr & request) override;
  void setForceTorqueCollisionBehavior(
    const franka_msgs::srv::SetForceTorqueCollisionBehavior::Request::SharedPtr & request) override;
  void setFullCollisionBehavior(
    const franka_msgs::srv::SetFullCollisionBehavior::Request::SharedPtr & request) override;

  [[nodiscard]] uint8_t armMarker() const noexcept { return arm_marker_; }
  [[nodiscard]] uint64_t successfulReadCount() const noexcept { return successful_read_count_; }
  [[nodiscard]] uint64_t lastSequence() const noexcept { return last_sequence_; }
  [[nodiscard]] SyntheticCondition condition() const noexcept { return condition_; }
  [[nodiscard]] bool explicitlyStale() const noexcept { return explicitly_stale_; }
  [[nodiscard]] size_t replaySize() const noexcept { return replay_size_; }
  [[nodiscard]] size_t replayIndex() const noexcept { return replay_index_; }
  [[nodiscard]] uint64_t failureCallCount(SyntheticFailurePoint point) const noexcept;
  [[nodiscard]] size_t capturedCommandCount() const noexcept { return command_capture_size_; }
  [[nodiscard]] uint64_t acceptedCommandCount() const noexcept { return accepted_command_count_; }
  [[nodiscard]] size_t commandQueueDepth() const noexcept { return in_flight_commands_.load(); }
  [[nodiscard]] size_t commandQueueCapacity() const noexcept { return command_queue_capacity_; }
  [[nodiscard]] size_t modeEntryWindowCycles() const noexcept { return mode_entry_window_cycles_; }
  [[nodiscard]] size_t modeEntryCyclesRemaining() const noexcept
  {
    return mode_entry_remaining_.load(std::memory_order_acquire);
  }
  [[nodiscard]] uint64_t acceptedModeRequestCount() const noexcept
  {
    return accepted_mode_request_count_.load(std::memory_order_relaxed);
  }
  [[nodiscard]] uint64_t rejectedModeRequestCount() const noexcept
  {
    return rejected_mode_request_count_.load(std::memory_order_relaxed);
  }
  [[nodiscard]] uint64_t acceptedNonNoneModeRequestCount() const noexcept
  {
    return accepted_non_none_mode_request_count_.load(std::memory_order_relaxed);
  }
  [[nodiscard]] uint64_t rejectedNonNoneModeRequestCount() const noexcept
  {
    return rejected_non_none_mode_request_count_.load(std::memory_order_relaxed);
  }
  [[nodiscard]] uint64_t acceptedSafeSnapshotCount() const noexcept
  {
    return accepted_safe_snapshot_count_;
  }
  [[nodiscard]] uint64_t acceptedUnsafeSnapshotCount() const noexcept
  {
    return accepted_unsafe_snapshot_count_;
  }
  [[nodiscard]] RobotCommand capturedCommand(size_t index) const;
  [[nodiscard]] size_t capturedEventCount() const noexcept { return event_capture_size_; }
  [[nodiscard]] uint64_t totalEventCount() const noexcept { return total_event_count_; }
  [[nodiscard]] SyntheticEvent capturedEvent(size_t index) const;
  [[nodiscard]] const SyntheticParameterSnapshot & parameterSnapshot() const noexcept
  {
    return parameter_snapshot_;
  }

  bool holdServiceOperationForTest(BackendServiceOperation operation) noexcept;
  void releaseServiceOperationForTest() noexcept;
  void injectFaultForTest(SyntheticCondition condition = SyntheticCondition::ControlFault) noexcept;

private:
  static constexpr size_t kFailurePointCount = static_cast<size_t>(SyntheticFailurePoint::Count);

  [[nodiscard]] bool shouldFail(SyntheticFailurePoint point) noexcept;
  [[nodiscard]] bool parameterOperationAllowed() const noexcept;
  void beginParameterOperation(SyntheticFailurePoint point);
  void endParameterOperation() noexcept;
  void latchFault(SyntheticCondition condition, SyntheticFailurePoint point) noexcept;
  void recordFailure(BackendFailureReason reason) noexcept;
  void recordAcceptedStateSample(uint64_t explicit_steady_ns) noexcept;
  void finishRecoveryAttempt(bool succeeded) noexcept;
  void recordEvent(
    SyntheticEventKind kind, SyntheticFailurePoint point = SyntheticFailurePoint::None,
    ControlMode mode = ControlMode::None, uint64_t sequence = 0) noexcept;
  [[nodiscard]] franka::RobotState nextCandidateState() noexcept;
  [[nodiscard]] bool validateTimestamp(const franka::RobotState & candidate) noexcept;
  [[nodiscard]] bool isCommandFinite(const RobotCommand & command) const noexcept;
  void armModeEntryWindow() noexcept;
  void disarmModeEntryWindow() noexcept;
  void acceptParameterOperation(SyntheticFailurePoint point);

  uint8_t arm_marker_;
  SyntheticFailureScript failure_;
  std::array<uint64_t, kFailurePointCount> failure_call_counts_{};
  SyntheticModel model_;
  franka::RobotState initial_state_;
  franka::RobotState last_state_;
  std::array<franka::RobotState, kMaximumReplayStates> replay_states_{};
  std::array<uint64_t, kMaximumReplayStates> replay_state_steady_ns_{};
  size_t replay_size_{0};
  size_t replay_index_{0};
  uint64_t initial_state_steady_ns_{0};
  uint64_t next_accepted_state_steady_ns_{0};
  uint64_t initial_sequence_{1};
  uint64_t timestamp_step_ms_{1};
  uint64_t successful_read_count_{0};
  uint64_t last_sequence_{0};
  bool initial_state_returned_{false};

  std::atomic<ControlMode> requested_mode_{ControlMode::None};
  std::atomic<ControlMode> active_mode_{ControlMode::None};
  std::atomic<BackendWorkerState> worker_state_{BackendWorkerState::Stopped};
  std::atomic<BackendServiceOperation> service_operation_{BackendServiceOperation::Idle};
  std::atomic<BackendFailureReasonMask> failure_reason_mask_{0};
  SyntheticCondition condition_{SyntheticCondition::None};
  std::atomic_bool stopped_{true};
  std::atomic_bool lifecycle_active_{false};
  std::atomic_bool faulted_{false};
  std::atomic_bool explicitly_stale_{false};
  std::atomic_bool has_state_sample_{false};
  std::atomic_bool state_queue_saturated_{false};
  std::atomic_bool command_queue_saturated_{false};
  std::atomic_uint64_t accepted_state_samples_{0};
  std::atomic_uint64_t last_accepted_state_steady_ns_{0};
  std::atomic_uint64_t dropped_state_samples_{0};
  std::atomic_uint64_t rejected_command_samples_{0};
  std::atomic_uint64_t recovery_attempts_{0};
  std::atomic_uint64_t recovery_successes_{0};
  std::atomic_uint64_t recovery_failures_{0};
  std::atomic<BackendRecoveryResult> last_recovery_result_{BackendRecoveryResult::NeverAttempted};

  size_t command_queue_capacity_{64};
  // F-10g amendment C.4: the modelled mode-ENTRY window. `mode_entry_remaining_` is armed to
  // `mode_entry_window_cycles_` by every accepted mode request and by every worker start, and
  // decremented once per readLatestState() while the worker is Running. Consumption -- the drain
  // in readLatestState() -- begins only once it reaches zero, which is what makes entry
  // asynchronous. Atomic because perform_command_mode_switch()'s preflight runs on whichever
  // thread called it while the control-cycle owner thread is reading the same backend.
  size_t mode_entry_window_cycles_{8};
  std::atomic_size_t mode_entry_remaining_{0};
  // "The first callback of the new control loop has run." The real flag is cleared by
  // Robot::updateCommandSnapshot() -- the consumer itself -- so `not in flight` provably means a
  // callback has executed. The countdown alone cannot say that: the read that takes it to zero is
  // still a read with no consumer behind it, and reporting "settled" there would hand write() a
  // full channel with the gate already shut, one cycle early. This flag is what the emulator
  // clears in the same place the real one is cleared: the draining read.
  std::atomic_bool mode_entry_settled_{true};
  std::atomic_uint64_t accepted_mode_request_count_{0};
  std::atomic_uint64_t rejected_mode_request_count_{0};
  std::atomic_uint64_t accepted_non_none_mode_request_count_{0};
  std::atomic_uint64_t rejected_non_none_mode_request_count_{0};
  uint64_t accepted_safe_snapshot_count_{0};
  uint64_t accepted_unsafe_snapshot_count_{0};
  std::atomic_size_t in_flight_commands_{0};
  std::array<RobotCommand, kCommandCaptureCapacity> command_capture_{};
  size_t command_capture_size_{0};
  uint64_t accepted_command_count_{0};

  std::array<SyntheticEvent, kEventCaptureCapacity> event_capture_{};
  size_t event_capture_size_{0};
  uint64_t total_event_count_{0};

  SyntheticParameterSnapshot parameter_snapshot_{};
};

}  // namespace franka_hardware::test_support
