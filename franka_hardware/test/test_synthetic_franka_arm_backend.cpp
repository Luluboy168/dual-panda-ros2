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
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include "support/synthetic_franka_arm_backend.hpp"

namespace franka_hardware::test_support
{
namespace
{

static_assert(std::is_final_v<SyntheticFrankaArmBackend>);
static_assert(std::is_final_v<SyntheticModel>);
static_assert(!std::is_copy_constructible_v<SyntheticFrankaArmBackend>);

template <typename Callable>
void expectSyntheticError(Callable && callable, SyntheticErrorCode expected_code)
{
  try {
    std::forward<Callable>(callable)();
    FAIL() << "expected SyntheticBackendException";
  } catch (const SyntheticBackendException & exception) {
    EXPECT_EQ(exception.code(), expected_code);
  }
}

template <size_t Size>
std::array<double, Size> filledArray(double first)
{
  std::array<double, Size> result{};
  for (size_t index = 0; index < result.size(); ++index) {
    result[index] = first + static_cast<double>(index);
  }
  return result;
}

RobotCommand makeCommand(double marker)
{
  RobotCommand command;
  command.efforts = filledArray<7>(marker + 10.0);
  command.joint_positions = filledArray<7>(marker + 20.0);
  command.joint_velocities = filledArray<7>(marker + 30.0);
  command.cartesian_positions = filledArray<16>(marker + 40.0);
  command.cartesian_velocities = filledArray<6>(marker + 50.0);
  return command;
}

void invokeParameter(SyntheticFrankaArmBackend & backend, SyntheticFailurePoint point)
{
  switch (point) {
    case SyntheticFailurePoint::JointStiffness:
      backend.setJointStiffness(std::make_shared<franka_msgs::srv::SetJointStiffness::Request>());
      return;
    case SyntheticFailurePoint::CartesianStiffness:
      backend.setCartesianStiffness(
        std::make_shared<franka_msgs::srv::SetCartesianStiffness::Request>());
      return;
    case SyntheticFailurePoint::Load:
      backend.setLoad(std::make_shared<franka_msgs::srv::SetLoad::Request>());
      return;
    case SyntheticFailurePoint::TCPFrame:
      backend.setTCPFrame(std::make_shared<franka_msgs::srv::SetTCPFrame::Request>());
      return;
    case SyntheticFailurePoint::StiffnessFrame:
      backend.setStiffnessFrame(std::make_shared<franka_msgs::srv::SetStiffnessFrame::Request>());
      return;
    case SyntheticFailurePoint::ForceTorqueCollisionBehavior:
      backend.setForceTorqueCollisionBehavior(
        std::make_shared<franka_msgs::srv::SetForceTorqueCollisionBehavior::Request>());
      return;
    case SyntheticFailurePoint::FullCollisionBehavior:
      backend.setFullCollisionBehavior(
        std::make_shared<franka_msgs::srv::SetFullCollisionBehavior::Request>());
      return;
    default:
      FAIL() << "failure point is not a parameter operation";
  }
}

TEST(SyntheticFrankaArmBackendTest, ConfigurationAndConstructionFailureAreDeterministic)
{
  auto invalid_marker = SyntheticFrankaArmBackendConfig::forArm(1);
  invalid_marker.arm_marker = 0;
  EXPECT_THROW(SyntheticFrankaArmBackend backend(invalid_marker), std::invalid_argument);

  auto invalid_sequence = SyntheticFrankaArmBackendConfig::forArm(1);
  invalid_sequence.initial_sequence = 0;
  EXPECT_THROW(SyntheticFrankaArmBackend backend(invalid_sequence), std::invalid_argument);

  auto excessive_replay = SyntheticFrankaArmBackendConfig::forArm(1);
  excessive_replay.replay_states.resize(SyntheticFrankaArmBackend::kMaximumReplayStates + 1);
  EXPECT_THROW(SyntheticFrankaArmBackend backend(excessive_replay), std::invalid_argument);

  auto construction_failure = SyntheticFrankaArmBackendConfig::forArm(1);
  construction_failure.failure = {SyntheticFailurePoint::Construction, 1};
  expectSyntheticError(
    [&construction_failure]() { SyntheticFrankaArmBackend backend(construction_failure); },
    SyntheticErrorCode::InjectedConstructionFailure);

  construction_failure.failure.fail_on_call = 2;
  EXPECT_NO_THROW(SyntheticFrankaArmBackend backend(construction_failure));
}

TEST(SyntheticFrankaArmBackendTest, InitialStatesSequencesAndTimestampsStayArmLocal)
{
  SyntheticFrankaArmBackend panda1(SyntheticFrankaArmBackendConfig::forArm(1));
  SyntheticFrankaArmBackend panda2(SyntheticFrankaArmBackendConfig::forArm(2));

  const auto panda1_initial = panda1.readLatestState();
  const auto panda2_initial = panda2.readLatestState();
  EXPECT_EQ(panda1_initial.q, (std::array<double, 7>{0.11, -0.21, 0.31, -1.41, 0.51, 1.61, 0.71}));
  EXPECT_EQ(
    panda2_initial.q, (std::array<double, 7>{-0.12, 0.22, -0.32, -1.52, -0.42, 1.72, -0.62}));
  EXPECT_NE(panda1_initial.O_T_EE, panda2_initial.O_T_EE);
  EXPECT_EQ(panda1_initial.time.toMSec(), 1000U);
  EXPECT_EQ(panda2_initial.time.toMSec(), 2000U);
  EXPECT_EQ(panda1.lastSequence(), 100U);
  EXPECT_EQ(panda2.lastSequence(), 200U);

  ASSERT_TRUE(panda1.startStateReading());
  ASSERT_TRUE(panda2.startStateReading());
  EXPECT_EQ(panda1.readLatestState().time.toMSec(), 1001U);
  EXPECT_EQ(panda2.readLatestState().time.toMSec(), 2001U);
  EXPECT_EQ(panda1.lastSequence(), 101U);
  EXPECT_EQ(panda2.lastSequence(), 201U);
  EXPECT_EQ(panda1.successfulReadCount(), 2U);
  EXPECT_EQ(panda2.successfulReadCount(), 2U);
}

TEST(SyntheticFrankaArmBackendTest, ModelPointerAndResponsesAreStableAndArmSpecific)
{
  SyntheticFrankaArmBackend panda1(SyntheticFrankaArmBackendConfig::forArm(1));
  SyntheticFrankaArmBackend panda2(SyntheticFrankaArmBackendConfig::forArm(2));
  const auto state1 = panda1.readLatestState();
  const auto state2 = panda2.readLatestState();

  ASSERT_EQ(panda1.model(), panda1.model());
  ASSERT_NE(panda1.model(), panda2.model());
  const auto * model1 = dynamic_cast<SyntheticModel *>(panda1.model());
  const auto * model2 = dynamic_cast<SyntheticModel *>(panda2.model());
  ASSERT_NE(model1, nullptr);
  ASSERT_NE(model2, nullptr);
  EXPECT_EQ(model1->armMarker(), 1U);
  EXPECT_EQ(model2->armMarker(), 2U);

  const auto mass1 = model1->mass(state1);
  const auto mass2 = model2->mass(state2);
  EXPECT_DOUBLE_EQ(mass1[0], 101.0);
  EXPECT_DOUBLE_EQ(mass1[48], 107.0);
  EXPECT_DOUBLE_EQ(mass2[0], 201.0);
  EXPECT_DOUBLE_EQ(mass2[48], 207.0);
  EXPECT_DOUBLE_EQ(model1->coriolis(state1)[0], 11.0);
  EXPECT_DOUBLE_EQ(model2->coriolis(state2)[6], 27.0);
  EXPECT_DOUBLE_EQ(model1->gravity(state1)[0], 21.0);
  EXPECT_DOUBLE_EQ(model2->gravity(state2)[6], 47.0);

  const auto pose1 = model1->pose(franka::Frame::kEndEffector, state1);
  const auto pose2 = model2->pose(franka::Frame::kEndEffector, state2);
  EXPECT_DOUBLE_EQ(pose1[0], 1.0);
  EXPECT_DOUBLE_EQ(pose1[15], 1.0);
  EXPECT_NE(pose1[12], pose2[12]);
  EXPECT_DOUBLE_EQ(model1->bodyJacobian(franka::Frame::kEndEffector, state1)[0], 1801.0);
  EXPECT_DOUBLE_EQ(model2->zeroJacobian(franka::Frame::kEndEffector, state2)[41], 4842.0);
}

TEST(SyntheticFrankaArmBackendTest, ReplayAllowsEqualTimestampsAndRejectsRegressionImmediately)
{
  auto config = SyntheticFrankaArmBackendConfig::forArm(1);
  auto equal_timestamp = makeSyntheticRobotState(1, 1000);
  equal_timestamp.q[0] = 0.25;
  auto regressed_timestamp = makeSyntheticRobotState(1, 999);
  regressed_timestamp.q[0] = 0.35;
  auto post_fault_state = makeSyntheticRobotState(1, 1002);
  post_fault_state.q[0] = 0.45;
  config.replay_states = {equal_timestamp, regressed_timestamp, post_fault_state};
  SyntheticFrankaArmBackend backend(config);

  EXPECT_DOUBLE_EQ(backend.readLatestState().q[0], 0.11);
  ASSERT_TRUE(backend.startStateReading());
  EXPECT_DOUBLE_EQ(backend.readLatestState().q[0], 0.25);
  EXPECT_FALSE(backend.hasFault());
  EXPECT_EQ(backend.lastSequence(), 101U);

  const auto rejected = backend.readLatestState();
  EXPECT_DOUBLE_EQ(rejected.q[0], 0.25);
  EXPECT_TRUE(backend.hasFault());
  EXPECT_EQ(backend.condition(), SyntheticCondition::TimestampRegression);
  EXPECT_EQ(backend.lastSequence(), 101U);
  EXPECT_EQ(backend.replayIndex(), 2U);
  EXPECT_EQ(backend.diagnostics().dropped_state_samples, 1U);

  const auto frozen_state = rejected;
  const auto frozen_replay_index = backend.replayIndex();
  const auto frozen_sequence = backend.lastSequence();
  const auto frozen_successful_reads = backend.successfulReadCount();
  for (size_t read = 0; read < 5; ++read) {
    const auto post_fault_read = backend.readLatestState();
    EXPECT_EQ(post_fault_read.q, frozen_state.q);
    EXPECT_EQ(post_fault_read.time.toMSec(), frozen_state.time.toMSec());
    EXPECT_EQ(backend.replayIndex(), frozen_replay_index);
    EXPECT_EQ(backend.lastSequence(), frozen_sequence);
    EXPECT_EQ(backend.successfulReadCount(), frozen_successful_reads);
  }
}

TEST(SyntheticFrankaArmBackendTest, ExplicitStaleSignalDoesNotInferFromEqualTimestamp)
{
  auto equal_config = SyntheticFrankaArmBackendConfig::forArm(1);
  equal_config.timestamp_step_ms = 0;
  SyntheticFrankaArmBackend equal_backend(equal_config);
  (void)equal_backend.readLatestState();
  ASSERT_TRUE(equal_backend.startStateReading());
  EXPECT_EQ(equal_backend.readLatestState().time.toMSec(), 1000U);
  EXPECT_FALSE(equal_backend.explicitlyStale());
  EXPECT_FALSE(equal_backend.hasFault());

  auto stale_config = SyntheticFrankaArmBackendConfig::forArm(1);
  stale_config.failure = {SyntheticFailurePoint::ExplicitStaleState, 1};
  SyntheticFrankaArmBackend stale_backend(stale_config);
  (void)stale_backend.readLatestState();
  ASSERT_TRUE(stale_backend.startStateReading());
  (void)stale_backend.readLatestState();
  EXPECT_TRUE(stale_backend.explicitlyStale());
  EXPECT_TRUE(stale_backend.hasFault());
  EXPECT_EQ(stale_backend.condition(), SyntheticCondition::ExplicitStaleState);
}

class InvalidStateInjectionTest : public testing::TestWithParam<SyntheticFailurePoint>
{
};

TEST_P(InvalidStateInjectionTest, ReturnsTheRequestedNonFiniteCandidateWithoutHiddenSubstitution)
{
  auto config = SyntheticFrankaArmBackendConfig::forArm(1);
  config.failure = {GetParam(), 1};
  SyntheticFrankaArmBackend backend(config);
  const auto state = backend.readLatestState();

  if (GetParam() == SyntheticFailurePoint::InvalidNanState) {
    EXPECT_TRUE(std::isnan(state.q[0]));
  } else {
    EXPECT_TRUE(std::isinf(state.O_T_EE[12]));
  }
  EXPECT_FALSE(backend.hasFault());
  EXPECT_EQ(backend.successfulReadCount(), 1U);

  const auto next_state = backend.readLatestState();
  EXPECT_TRUE(std::isfinite(next_state.q[0]));
  EXPECT_TRUE(std::isfinite(next_state.O_T_EE[12]));
  EXPECT_DOUBLE_EQ(next_state.q[0], config.initial_state.q[0]);
  EXPECT_DOUBLE_EQ(next_state.O_T_EE[12], config.initial_state.O_T_EE[12]);
  EXPECT_EQ(backend.successfulReadCount(), 2U);
}

INSTANTIATE_TEST_SUITE_P(
  NanAndInfinity, InvalidStateInjectionTest,
  testing::Values(
    SyntheticFailurePoint::InvalidNanState, SyntheticFailurePoint::InvalidInfiniteState));

TEST(SyntheticFrankaArmBackendTest, CommandCaptureIsBoundedExactAndHasNoCrossArmLeakage)
{
  auto config1 = SyntheticFrankaArmBackendConfig::forArm(1);
  auto config2 = SyntheticFrankaArmBackendConfig::forArm(2);
  config1.command_queue_capacity = 2;
  config2.command_queue_capacity = 2;
  SyntheticFrankaArmBackend panda1(config1);
  SyntheticFrankaArmBackend panda2(config2);
  const auto command1 = makeCommand(1.0);
  const auto command2 = makeCommand(2.0);

  EXPECT_TRUE(panda1.publishCommand(command1));
  EXPECT_TRUE(panda2.publishCommand(command2));
  EXPECT_EQ(panda1.capturedCommandCount(), 1U);
  EXPECT_EQ(panda2.capturedCommandCount(), 1U);
  EXPECT_EQ(panda1.capturedCommand(0).efforts, command1.efforts);
  EXPECT_EQ(panda2.capturedCommand(0).efforts, command2.efforts);
  EXPECT_NE(panda1.capturedCommand(0).efforts, panda2.capturedCommand(0).efforts);

  EXPECT_TRUE(panda1.publishCommand(makeCommand(3.0)));
  EXPECT_FALSE(panda1.canPublishCommand());
  EXPECT_FALSE(panda1.publishCommand(makeCommand(4.0)));
  EXPECT_TRUE(panda1.diagnostics().command_queue_saturated);
  EXPECT_EQ(panda1.diagnostics().rejected_command_samples, 1U);
  EXPECT_TRUE(panda2.canPublishCommand());

  (void)panda1.readLatestState();
  ASSERT_TRUE(panda1.startStateReading());
  (void)panda1.readLatestState();
  EXPECT_TRUE(panda1.canPublishCommand());
  EXPECT_TRUE(panda1.publishCommand(makeCommand(5.0)));
  EXPECT_EQ(panda1.acceptedCommandCount(), 3U);
}

TEST(SyntheticFrankaArmBackendTest, NanAndInfiniteCommandsAreRejectedAndNeverCaptured)
{
  SyntheticFrankaArmBackend backend(SyntheticFrankaArmBackendConfig::forArm(1));
  auto nan_command = makeCommand(1.0);
  nan_command.efforts[3] = std::numeric_limits<double>::quiet_NaN();
  auto infinite_command = makeCommand(2.0);
  infinite_command.cartesian_velocities[4] = std::numeric_limits<double>::infinity();

  EXPECT_FALSE(backend.publishCommand(nan_command));
  EXPECT_FALSE(backend.publishCommand(infinite_command));
  EXPECT_EQ(backend.capturedCommandCount(), 0U);
  EXPECT_EQ(backend.diagnostics().rejected_command_samples, 2U);
  EXPECT_EQ(backend.condition(), SyntheticCondition::InvalidCommand);
  EXPECT_TRUE(backend.publishCommand(makeCommand(3.0)));
  EXPECT_EQ(backend.capturedCommandCount(), 1U);
  EXPECT_EQ(backend.diagnostics().rejected_command_samples, 2U);
}

TEST(SyntheticFrankaArmBackendTest, LifecycleModesAndDiagnosticsAreDeterministic)
{
  SyntheticFrankaArmBackend backend(SyntheticFrankaArmBackendConfig::forArm(1));
  EXPECT_TRUE(backend.diagnostics().stopped);
  EXPECT_FALSE(backend.canRequestControlMode(ControlMode::JointTorque));
  ASSERT_TRUE(backend.startStateReading());
  EXPECT_EQ(backend.diagnostics().worker_state, BackendWorkerState::Running);

  const std::array<ControlMode, 6> modes{ControlMode::JointTorque,   ControlMode::JointPosition,
                                         ControlMode::JointVelocity, ControlMode::CartesianVelocity,
                                         ControlMode::CartesianPose, ControlMode::None};
  for (const auto mode : modes) {
    ASSERT_TRUE(backend.canRequestControlMode(mode));
    ASSERT_TRUE(backend.requestControlMode(mode));
    EXPECT_EQ(backend.requestedControlMode(), mode);
    EXPECT_EQ(backend.activeControlMode(), mode);
  }

  ASSERT_TRUE(backend.stop());
  EXPECT_TRUE(backend.diagnostics().stopped);
  EXPECT_EQ(backend.requestedControlMode(), ControlMode::None);
  EXPECT_EQ(backend.activeControlMode(), ControlMode::None);
  EXPECT_FALSE(backend.canRequestControlMode(ControlMode::JointVelocity));
  EXPECT_EQ(backend.acceptedModeRequestCount(), 6U);
  EXPECT_EQ(backend.rejectedModeRequestCount(), 0U);
  EXPECT_EQ(backend.acceptedNonNoneModeRequestCount(), 5U);
  EXPECT_EQ(backend.rejectedNonNoneModeRequestCount(), 0U);
}

TEST(
  SyntheticFrankaArmBackendTest, LifecycleGateRejectsStartAndStopWithoutEffectThenAllowsExactRetry)
{
  SyntheticFrankaArmBackend backend(SyntheticFrankaArmBackendConfig::forArm(1));

  ASSERT_TRUE(backend.holdServiceOperationForTest(BackendServiceOperation::Parameter));
  const auto stopped_snapshot = backend.diagnostics();
  EXPECT_FALSE(backend.startStateReading());
  auto diagnostics = backend.diagnostics();
  EXPECT_EQ(diagnostics.service_operation, BackendServiceOperation::Parameter);
  EXPECT_EQ(diagnostics.worker_state, stopped_snapshot.worker_state);
  EXPECT_EQ(diagnostics.stopped, stopped_snapshot.stopped);
  EXPECT_EQ(diagnostics.failure_reason, stopped_snapshot.failure_reason);
  backend.releaseServiceOperationForTest();

  ASSERT_TRUE(backend.startStateReading());
  EXPECT_EQ(backend.diagnostics().service_operation, BackendServiceOperation::Idle);
  ASSERT_TRUE(backend.holdServiceOperationForTest(BackendServiceOperation::Parameter));
  const auto running_snapshot = backend.diagnostics();
  EXPECT_FALSE(backend.stop());
  diagnostics = backend.diagnostics();
  EXPECT_EQ(diagnostics.service_operation, BackendServiceOperation::Parameter);
  EXPECT_EQ(diagnostics.worker_state, running_snapshot.worker_state);
  EXPECT_EQ(diagnostics.stopped, running_snapshot.stopped);
  EXPECT_EQ(diagnostics.failure_reason, running_snapshot.failure_reason);
  backend.releaseServiceOperationForTest();

  EXPECT_TRUE(backend.stop());
  diagnostics = backend.diagnostics();
  EXPECT_EQ(diagnostics.service_operation, BackendServiceOperation::Idle);
  EXPECT_EQ(diagnostics.worker_state, BackendWorkerState::Stopped);
  EXPECT_TRUE(diagnostics.stopped);
}

TEST(SyntheticFrankaArmBackendTest, FailOnNthCallIsExactForCommandModeAndQueueFailures)
{
  auto command_config = SyntheticFrankaArmBackendConfig::forArm(1);
  command_config.failure = {SyntheticFailurePoint::CommandPublish, 2};
  SyntheticFrankaArmBackend command_backend(command_config);
  EXPECT_TRUE(command_backend.publishCommand(makeCommand(1.0)));
  EXPECT_FALSE(command_backend.publishCommand(makeCommand(2.0)));
  EXPECT_TRUE(command_backend.publishCommand(makeCommand(3.0)));
  EXPECT_EQ(command_backend.failureCallCount(SyntheticFailurePoint::CommandPublish), 3U);
  EXPECT_EQ(command_backend.acceptedCommandCount(), 2U);

  auto mode_config = SyntheticFrankaArmBackendConfig::forArm(1);
  mode_config.failure = {SyntheticFailurePoint::ModeRequest, 2};
  SyntheticFrankaArmBackend mode_backend(mode_config);
  ASSERT_TRUE(mode_backend.startStateReading());
  EXPECT_TRUE(mode_backend.requestControlMode(ControlMode::JointTorque));
  EXPECT_FALSE(mode_backend.requestControlMode(ControlMode::JointVelocity));
  EXPECT_EQ(mode_backend.activeControlMode(), ControlMode::JointTorque);
  EXPECT_EQ(mode_backend.condition(), SyntheticCondition::ModeRequestFailure);
  EXPECT_EQ(mode_backend.diagnostics().service_operation, BackendServiceOperation::Idle);
  EXPECT_TRUE(mode_backend.requestControlMode(ControlMode::JointVelocity));
  EXPECT_EQ(mode_backend.acceptedModeRequestCount(), 2U);
  EXPECT_EQ(mode_backend.rejectedModeRequestCount(), 1U);
  EXPECT_EQ(mode_backend.acceptedNonNoneModeRequestCount(), 2U);
  EXPECT_EQ(mode_backend.rejectedNonNoneModeRequestCount(), 1U);

  auto queue_config = SyntheticFrankaArmBackendConfig::forArm(1);
  queue_config.failure = {SyntheticFailurePoint::QueueSaturation, 2};
  SyntheticFrankaArmBackend queue_backend(queue_config);
  EXPECT_TRUE(queue_backend.publishCommand(makeCommand(1.0)));
  EXPECT_FALSE(queue_backend.publishCommand(makeCommand(2.0)));
  EXPECT_TRUE(queue_backend.diagnostics().command_queue_saturated);
  EXPECT_TRUE(queue_backend.publishCommand(makeCommand(3.0)));
  EXPECT_FALSE(queue_backend.diagnostics().command_queue_saturated);
}

TEST(SyntheticFrankaArmBackendTest, InitialReadStartupAndShutdownFailuresRemainObservable)
{
  auto initial_config = SyntheticFrankaArmBackendConfig::forArm(1);
  initial_config.failure = {SyntheticFailurePoint::InitialRead, 1};
  SyntheticFrankaArmBackend initial_backend(initial_config);
  expectSyntheticError(
    [&initial_backend]() { (void)initial_backend.readLatestState(); },
    SyntheticErrorCode::InjectedInitialReadFailure);
  EXPECT_TRUE(initial_backend.hasFault());
  EXPECT_EQ(initial_backend.condition(), SyntheticCondition::InitialReadFailure);

  auto start_config = SyntheticFrankaArmBackendConfig::forArm(1);
  start_config.failure = {SyntheticFailurePoint::StartStateReading, 1};
  SyntheticFrankaArmBackend start_backend(start_config);
  EXPECT_FALSE(start_backend.startStateReading());
  EXPECT_TRUE(start_backend.hasFault());
  EXPECT_TRUE(start_backend.diagnostics().stopped);
  EXPECT_EQ(start_backend.condition(), SyntheticCondition::StartFailure);
  EXPECT_EQ(start_backend.diagnostics().worker_state, BackendWorkerState::Faulted);
  EXPECT_EQ(start_backend.diagnostics().failure_reason, BackendFailureReason::WorkerStartupFailure);
  EXPECT_EQ(start_backend.diagnostics().service_operation, BackendServiceOperation::Idle);
  EXPECT_FALSE(start_backend.startStateReading());
  EXPECT_TRUE(start_backend.hasFault());
  EXPECT_EQ(start_backend.diagnostics().failure_reason, BackendFailureReason::WorkerStartupFailure);
  EXPECT_TRUE(start_backend.recoverToReading());
  EXPECT_FALSE(start_backend.hasFault());
  EXPECT_EQ(start_backend.diagnostics().failure_reason, BackendFailureReason::None);
  EXPECT_TRUE(start_backend.startStateReading());
  EXPECT_EQ(start_backend.diagnostics().failure_reason, BackendFailureReason::None);

  auto shutdown_config = SyntheticFrankaArmBackendConfig::forArm(1);
  shutdown_config.failure = {SyntheticFailurePoint::Shutdown, 1};
  SyntheticFrankaArmBackend shutdown_backend(shutdown_config);
  ASSERT_TRUE(shutdown_backend.startStateReading());
  EXPECT_FALSE(shutdown_backend.stop());
  EXPECT_TRUE(shutdown_backend.hasFault());
  EXPECT_EQ(shutdown_backend.condition(), SyntheticCondition::ShutdownFailure);
  EXPECT_EQ(shutdown_backend.diagnostics().worker_state, BackendWorkerState::Faulted);
  EXPECT_EQ(shutdown_backend.diagnostics().service_operation, BackendServiceOperation::Idle);
}

TEST(SyntheticFrankaArmBackendTest, SuccessfulStopPreservesAnExistingFaultedWorkerState)
{
  auto config = SyntheticFrankaArmBackendConfig::forArm(1);
  config.failure = {SyntheticFailurePoint::ReadFault, 1};
  SyntheticFrankaArmBackend backend(config);
  (void)backend.readLatestState();
  ASSERT_TRUE(backend.startStateReading());
  (void)backend.readLatestState();
  ASSERT_TRUE(backend.hasFault());
  ASSERT_EQ(backend.diagnostics().worker_state, BackendWorkerState::Faulted);

  EXPECT_TRUE(backend.stop());
  EXPECT_TRUE(backend.hasFault());
  EXPECT_TRUE(backend.diagnostics().stopped);
  EXPECT_EQ(backend.diagnostics().worker_state, BackendWorkerState::Faulted);
  EXPECT_EQ(backend.requestedControlMode(), ControlMode::None);
  EXPECT_EQ(backend.activeControlMode(), ControlMode::None);
}

class AsynchronousFaultTest : public testing::TestWithParam<SyntheticFailurePoint>
{
};

TEST_P(AsynchronousFaultTest, FaultIsObservedOnTheNextReadWithoutAThread)
{
  auto config = SyntheticFrankaArmBackendConfig::forArm(1);
  config.failure = {GetParam(), 1};
  SyntheticFrankaArmBackend backend(config);
  const auto initial = backend.readLatestState();
  ASSERT_TRUE(backend.startStateReading());
  if (GetParam() == SyntheticFailurePoint::ControlFault) {
    ASSERT_TRUE(backend.requestControlMode(ControlMode::JointTorque));
  }

  const auto observed = backend.readLatestState();
  EXPECT_EQ(observed.q, initial.q);
  EXPECT_TRUE(backend.hasFault());
  EXPECT_EQ(backend.diagnostics().worker_state, BackendWorkerState::Faulted);
  EXPECT_EQ(backend.requestedControlMode(), ControlMode::None);
  EXPECT_EQ(backend.activeControlMode(), ControlMode::None);
  EXPECT_EQ(backend.diagnostics().dropped_state_samples, 1U);

  const auto expected_condition =
    GetParam() == SyntheticFailurePoint::UnexpectedLoopReturn
      ? SyntheticCondition::UnexpectedLoopReturn
      : (GetParam() == SyntheticFailurePoint::ReadFault ? SyntheticCondition::ReadFault
                                                        : SyntheticCondition::ControlFault);
  EXPECT_EQ(backend.condition(), expected_condition);
}

INSTANTIATE_TEST_SUITE_P(
  UnexpectedReadAndControl, AsynchronousFaultTest,
  testing::Values(
    SyntheticFailurePoint::UnexpectedLoopReturn, SyntheticFailurePoint::ReadFault,
    SyntheticFailurePoint::ControlFault));

TEST(SyntheticFrankaArmBackendTest, RecoveryReturnsToReadingAndNeverResumesMotion)
{
  auto config = SyntheticFrankaArmBackendConfig::forArm(1);
  config.replay_states = {makeSyntheticRobotState(1, 999)};
  SyntheticFrankaArmBackend backend(config);
  (void)backend.readLatestState();
  ASSERT_TRUE(backend.startStateReading());
  ASSERT_TRUE(backend.requestControlMode(ControlMode::JointTorque));
  (void)backend.readLatestState();
  ASSERT_TRUE(backend.hasFault());

  EXPECT_TRUE(backend.recoverToReading());
  EXPECT_FALSE(backend.hasFault());
  EXPECT_EQ(backend.requestedControlMode(), ControlMode::None);
  EXPECT_EQ(backend.activeControlMode(), ControlMode::None);
  EXPECT_EQ(backend.diagnostics().worker_state, BackendWorkerState::Running);
  EXPECT_FALSE(backend.diagnostics().stopped);
  EXPECT_EQ(backend.condition(), SyntheticCondition::None);
}

TEST(SyntheticFrankaArmBackendTest, FailedRecoveryStaysFaultedAndNeverResumesMotion)
{
  auto config = SyntheticFrankaArmBackendConfig::forArm(1);
  config.replay_states = {makeSyntheticRobotState(1, 999)};
  config.failure = {SyntheticFailurePoint::Recovery, 1};
  SyntheticFrankaArmBackend backend(config);
  (void)backend.readLatestState();
  ASSERT_TRUE(backend.startStateReading());
  ASSERT_TRUE(backend.requestControlMode(ControlMode::JointVelocity));
  (void)backend.readLatestState();
  ASSERT_TRUE(backend.hasFault());

  EXPECT_FALSE(backend.recoverToReading());
  EXPECT_TRUE(backend.hasFault());
  EXPECT_EQ(backend.condition(), SyntheticCondition::RecoveryFailure);
  EXPECT_EQ(backend.requestedControlMode(), ControlMode::None);
  EXPECT_EQ(backend.activeControlMode(), ControlMode::None);
  EXPECT_EQ(backend.diagnostics().worker_state, BackendWorkerState::Faulted);
}

TEST(SyntheticFrankaArmBackendTest, BusyLifecycleGateRejectsRecoveryWithoutAccountingOrStateEffect)
{
  SyntheticFrankaArmBackend backend(SyntheticFrankaArmBackendConfig::forArm(1));
  backend.injectFaultForTest();
  const auto before = backend.diagnostics();
  ASSERT_TRUE(backend.holdServiceOperationForTest(BackendServiceOperation::Lifecycle));

  EXPECT_FALSE(backend.recoverToReading());
  const auto held = backend.diagnostics();
  EXPECT_EQ(held.service_operation, BackendServiceOperation::Lifecycle);
  EXPECT_EQ(held.worker_state, before.worker_state);
  EXPECT_EQ(held.failure_reason, before.failure_reason);
  EXPECT_EQ(held.recovery_attempts, before.recovery_attempts);
  EXPECT_EQ(held.recovery_failures, before.recovery_failures);

  backend.releaseServiceOperationForTest();
  EXPECT_TRUE(backend.recoverToReading());
  EXPECT_EQ(backend.diagnostics().service_operation, BackendServiceOperation::Idle);
}

TEST(
  SyntheticFrankaArmBackendTest,
  HeldLifecycleGateHasNoConcurrentParameterRecoveryOrModeLoserEffects)
{
  SyntheticFrankaArmBackend backend(SyntheticFrankaArmBackendConfig::forArm(1));
  auto request = std::make_shared<franka_msgs::srv::SetJointStiffness::Request>();
  const auto before = backend.diagnostics();
  const auto events_before = backend.totalEventCount();
  ASSERT_TRUE(backend.holdServiceOperationForTest(BackendServiceOperation::Lifecycle));

  for (size_t cycle = 0; cycle < 100; ++cycle) {
    std::atomic_size_t ready{0};
    std::atomic_bool release{false};
    std::atomic_bool parameter_rejected{false};
    std::atomic_bool recovery_result{true};
    std::atomic_bool mode_result{true};
    const auto wait_for_release = [&ready, &release]() {
      ready.fetch_add(1, std::memory_order_release);
      while (!release.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
    };

    std::thread parameter_loser([&]() {
      wait_for_release();
      try {
        backend.setJointStiffness(request);
      } catch (const SyntheticBackendException & exception) {
        parameter_rejected.store(
          exception.code() == SyntheticErrorCode::UnsafeParameterOperation,
          std::memory_order_release);
      }
    });
    std::thread recovery_loser([&]() {
      wait_for_release();
      recovery_result.store(backend.recoverToReading(), std::memory_order_release);
    });
    std::thread mode_loser([&]() {
      wait_for_release();
      mode_result.store(
        backend.requestControlMode(ControlMode::JointVelocity), std::memory_order_release);
    });
    while (ready.load(std::memory_order_acquire) != 3U) {
      std::this_thread::yield();
    }
    release.store(true, std::memory_order_release);
    parameter_loser.join();
    recovery_loser.join();
    mode_loser.join();

    ASSERT_TRUE(parameter_rejected.load(std::memory_order_acquire)) << cycle;
    EXPECT_FALSE(recovery_result.load(std::memory_order_acquire)) << cycle;
    EXPECT_FALSE(mode_result.load(std::memory_order_acquire)) << cycle;
  }

  const auto held = backend.diagnostics();
  EXPECT_EQ(held.service_operation, BackendServiceOperation::Lifecycle);
  EXPECT_EQ(held.worker_state, before.worker_state);
  EXPECT_EQ(held.failure_reason, before.failure_reason);
  EXPECT_EQ(held.recovery_attempts, before.recovery_attempts);
  EXPECT_EQ(held.recovery_successes, before.recovery_successes);
  EXPECT_EQ(held.recovery_failures, before.recovery_failures);
  EXPECT_EQ(held.last_recovery_result, before.last_recovery_result);
  EXPECT_EQ(backend.requestedControlMode(), ControlMode::None);
  EXPECT_EQ(backend.activeControlMode(), ControlMode::None);
  EXPECT_EQ(backend.condition(), SyntheticCondition::None);
  EXPECT_EQ(backend.failureCallCount(SyntheticFailurePoint::JointStiffness), 0U);
  EXPECT_EQ(backend.failureCallCount(SyntheticFailurePoint::Recovery), 0U);
  EXPECT_EQ(backend.failureCallCount(SyntheticFailurePoint::ModeRequest), 0U);
  EXPECT_EQ(backend.totalEventCount(), events_before);
  EXPECT_EQ(backend.acceptedModeRequestCount(), 0U);
  EXPECT_EQ(backend.rejectedModeRequestCount(), 100U);
  EXPECT_EQ(backend.acceptedNonNoneModeRequestCount(), 0U);
  EXPECT_EQ(backend.rejectedNonNoneModeRequestCount(), 100U);

  backend.releaseServiceOperationForTest();
  EXPECT_EQ(backend.diagnostics().service_operation, BackendServiceOperation::Idle);
  EXPECT_NO_THROW(backend.setJointStiffness(request));
  ASSERT_TRUE(backend.startStateReading());
  EXPECT_TRUE(backend.requestControlMode(ControlMode::JointVelocity));
  EXPECT_EQ(backend.acceptedModeRequestCount(), 1U);
  EXPECT_EQ(backend.rejectedModeRequestCount(), 100U);
  EXPECT_EQ(backend.acceptedNonNoneModeRequestCount(), 1U);
  EXPECT_EQ(backend.rejectedNonNoneModeRequestCount(), 100U);
}

TEST(SyntheticFrankaArmBackendTest, InactiveRecoveryRemainsStoppedAndNeverResumesMotion)
{
  SyntheticFrankaArmBackend backend(SyntheticFrankaArmBackendConfig::forArm(1));
  (void)backend.readLatestState();
  ASSERT_TRUE(backend.startStateReading());
  ASSERT_TRUE(backend.requestControlMode(ControlMode::JointVelocity));
  backend.injectFaultForTest();
  ASSERT_TRUE(backend.hasFault());
  ASSERT_TRUE(backend.stop());

  ASSERT_TRUE(backend.recoverToReading());
  EXPECT_FALSE(backend.hasFault());
  EXPECT_EQ(backend.requestedControlMode(), ControlMode::None);
  EXPECT_EQ(backend.activeControlMode(), ControlMode::None);
  EXPECT_EQ(backend.diagnostics().worker_state, BackendWorkerState::Stopped);
  EXPECT_TRUE(backend.diagnostics().stopped);
}

TEST(SyntheticFrankaArmBackendTest, EveryParameterOperationCapturesExactRequestInSafeStates)
{
  SyntheticFrankaArmBackend backend(SyntheticFrankaArmBackendConfig::forArm(1));

  auto joint = std::make_shared<franka_msgs::srv::SetJointStiffness::Request>();
  joint->joint_stiffness = filledArray<7>(11.0);
  backend.setJointStiffness(joint);
  EXPECT_EQ(backend.parameterSnapshot().joint_stiffness, joint->joint_stiffness);

  auto cartesian = std::make_shared<franka_msgs::srv::SetCartesianStiffness::Request>();
  cartesian->cartesian_stiffness = filledArray<6>(21.0);
  backend.setCartesianStiffness(cartesian);
  EXPECT_EQ(backend.parameterSnapshot().cartesian_stiffness, cartesian->cartesian_stiffness);

  auto load = std::make_shared<franka_msgs::srv::SetLoad::Request>();
  load->mass = 3.25;
  load->center_of_mass = filledArray<3>(31.0);
  load->load_inertia = filledArray<9>(41.0);
  backend.setLoad(load);
  EXPECT_DOUBLE_EQ(backend.parameterSnapshot().load_mass, 3.25);
  EXPECT_EQ(backend.parameterSnapshot().load_center_of_mass, load->center_of_mass);
  EXPECT_EQ(backend.parameterSnapshot().load_inertia, load->load_inertia);

  auto tcp = std::make_shared<franka_msgs::srv::SetTCPFrame::Request>();
  tcp->transformation = filledArray<16>(51.0);
  backend.setTCPFrame(tcp);
  EXPECT_EQ(backend.parameterSnapshot().tcp_frame, tcp->transformation);

  auto stiffness = std::make_shared<franka_msgs::srv::SetStiffnessFrame::Request>();
  stiffness->transformation = filledArray<16>(61.0);
  backend.setStiffnessFrame(stiffness);
  EXPECT_EQ(backend.parameterSnapshot().stiffness_frame, stiffness->transformation);

  auto nominal = std::make_shared<franka_msgs::srv::SetForceTorqueCollisionBehavior::Request>();
  nominal->lower_torque_thresholds_nominal = filledArray<7>(71.0);
  nominal->upper_torque_thresholds_nominal = filledArray<7>(81.0);
  nominal->lower_force_thresholds_nominal = filledArray<6>(91.0);
  nominal->upper_force_thresholds_nominal = filledArray<6>(101.0);
  backend.setForceTorqueCollisionBehavior(nominal);
  EXPECT_EQ(
    backend.parameterSnapshot().lower_torque_thresholds_nominal,
    nominal->lower_torque_thresholds_nominal);
  EXPECT_EQ(
    backend.parameterSnapshot().upper_torque_thresholds_nominal,
    nominal->upper_torque_thresholds_nominal);
  EXPECT_EQ(
    backend.parameterSnapshot().lower_force_thresholds_nominal,
    nominal->lower_force_thresholds_nominal);
  EXPECT_EQ(
    backend.parameterSnapshot().upper_force_thresholds_nominal,
    nominal->upper_force_thresholds_nominal);

  ASSERT_TRUE(backend.startStateReading());
  auto full = std::make_shared<franka_msgs::srv::SetFullCollisionBehavior::Request>();
  full->lower_torque_thresholds_acceleration = filledArray<7>(111.0);
  full->upper_torque_thresholds_acceleration = filledArray<7>(121.0);
  full->lower_torque_thresholds_nominal = filledArray<7>(131.0);
  full->upper_torque_thresholds_nominal = filledArray<7>(141.0);
  full->lower_force_thresholds_acceleration = filledArray<6>(151.0);
  full->upper_force_thresholds_acceleration = filledArray<6>(161.0);
  full->lower_force_thresholds_nominal = filledArray<6>(171.0);
  full->upper_force_thresholds_nominal = filledArray<6>(181.0);
  backend.setFullCollisionBehavior(full);
  EXPECT_EQ(
    backend.parameterSnapshot().lower_torque_thresholds_acceleration,
    full->lower_torque_thresholds_acceleration);
  EXPECT_EQ(
    backend.parameterSnapshot().upper_torque_thresholds_acceleration,
    full->upper_torque_thresholds_acceleration);
  EXPECT_EQ(
    backend.parameterSnapshot().lower_torque_thresholds_nominal,
    full->lower_torque_thresholds_nominal);
  EXPECT_EQ(
    backend.parameterSnapshot().upper_torque_thresholds_nominal,
    full->upper_torque_thresholds_nominal);
  EXPECT_EQ(
    backend.parameterSnapshot().lower_force_thresholds_acceleration,
    full->lower_force_thresholds_acceleration);
  EXPECT_EQ(
    backend.parameterSnapshot().upper_force_thresholds_acceleration,
    full->upper_force_thresholds_acceleration);
  EXPECT_EQ(
    backend.parameterSnapshot().lower_force_thresholds_nominal,
    full->lower_force_thresholds_nominal);
  EXPECT_EQ(
    backend.parameterSnapshot().upper_force_thresholds_nominal,
    full->upper_force_thresholds_nominal);
  EXPECT_EQ(backend.diagnostics().service_operation, BackendServiceOperation::Idle);
}

TEST(SyntheticFrankaArmBackendTest, ParameterGateRejectsControllingFaultedAndRecoveringStates)
{
  auto request = std::make_shared<franka_msgs::srv::SetJointStiffness::Request>();

  SyntheticFrankaArmBackend controlling(SyntheticFrankaArmBackendConfig::forArm(1));
  ASSERT_TRUE(controlling.startStateReading());
  ASSERT_TRUE(controlling.requestControlMode(ControlMode::JointTorque));
  expectSyntheticError(
    [&controlling, &request]() { controlling.setJointStiffness(request); },
    SyntheticErrorCode::UnsafeParameterOperation);

  auto fault_config = SyntheticFrankaArmBackendConfig::forArm(1);
  fault_config.replay_states = {makeSyntheticRobotState(1, 999)};
  SyntheticFrankaArmBackend faulted(fault_config);
  (void)faulted.readLatestState();
  ASSERT_TRUE(faulted.startStateReading());
  (void)faulted.readLatestState();
  ASSERT_TRUE(faulted.hasFault());
  expectSyntheticError(
    [&faulted, &request]() { faulted.setJointStiffness(request); },
    SyntheticErrorCode::UnsafeParameterOperation);

  SyntheticFrankaArmBackend recovering(SyntheticFrankaArmBackendConfig::forArm(1));
  ASSERT_TRUE(recovering.startStateReading());
  ASSERT_TRUE(recovering.holdServiceOperationForTest(BackendServiceOperation::Recovery));
  EXPECT_TRUE(recovering.diagnostics().recovering);
  EXPECT_FALSE(recovering.canRequestControlMode(ControlMode::JointVelocity));
  expectSyntheticError(
    [&recovering, &request]() { recovering.setJointStiffness(request); },
    SyntheticErrorCode::UnsafeParameterOperation);
  recovering.releaseServiceOperationForTest();
  EXPECT_FALSE(recovering.diagnostics().recovering);
  EXPECT_NO_THROW(recovering.setJointStiffness(request));
}

class ParameterFailureTest : public testing::TestWithParam<SyntheticFailurePoint>
{
};

TEST_P(ParameterFailureTest, InjectedFailureThrowsAndAlwaysReleasesTheServiceGate)
{
  auto config = SyntheticFrankaArmBackendConfig::forArm(1);
  config.failure = {GetParam(), 1};
  SyntheticFrankaArmBackend backend(config);

  expectSyntheticError(
    [&backend, this]() { invokeParameter(backend, GetParam()); },
    SyntheticErrorCode::InjectedParameterFailure);
  EXPECT_EQ(backend.condition(), SyntheticCondition::ParameterFailure);
  EXPECT_EQ(backend.failureCallCount(GetParam()), 1U);
  EXPECT_EQ(backend.diagnostics().service_operation, BackendServiceOperation::Idle);
  EXPECT_FALSE(backend.hasFault());
}

INSTANTIATE_TEST_SUITE_P(
  EveryOperation, ParameterFailureTest,
  testing::Values(
    SyntheticFailurePoint::JointStiffness, SyntheticFailurePoint::CartesianStiffness,
    SyntheticFailurePoint::Load, SyntheticFailurePoint::TCPFrame,
    SyntheticFailurePoint::StiffnessFrame, SyntheticFailurePoint::ForceTorqueCollisionBehavior,
    SyntheticFailurePoint::FullCollisionBehavior));

TEST(SyntheticFrankaArmBackendTest, NullParameterRequestsUseAStableExceptionAndDoNotOpenGate)
{
  SyntheticFrankaArmBackend backend(SyntheticFrankaArmBackendConfig::forArm(1));
  franka_msgs::srv::SetLoad::Request::SharedPtr request;
  expectSyntheticError(
    [&backend, &request]() { backend.setLoad(request); }, SyntheticErrorCode::NullParameterRequest);
  EXPECT_EQ(backend.diagnostics().service_operation, BackendServiceOperation::Idle);
}

TEST(SyntheticFrankaArmBackendTest, RepeatedLifecycleHasNoHiddenThreadOrUnboundedCapture)
{
  SyntheticFrankaArmBackend backend(SyntheticFrankaArmBackendConfig::forArm(1));
  for (size_t cycle = 0; cycle < 100; ++cycle) {
    ASSERT_TRUE(backend.startStateReading()) << cycle;
    ASSERT_TRUE(backend.requestControlMode(ControlMode::JointVelocity)) << cycle;
    ASSERT_TRUE(backend.requestControlMode(ControlMode::None)) << cycle;
    ASSERT_TRUE(backend.stop()) << cycle;
  }
  EXPECT_FALSE(backend.hasFault());
  EXPECT_TRUE(backend.diagnostics().stopped);
  EXPECT_GT(backend.totalEventCount(), SyntheticFrankaArmBackend::kEventCaptureCapacity);
  EXPECT_EQ(backend.capturedEventCount(), SyntheticFrankaArmBackend::kEventCaptureCapacity);
  EXPECT_EQ(
    backend.capturedEvent(backend.capturedEventCount() - 1).kind, SyntheticEventKind::Stopped);
}

}  // namespace
}  // namespace franka_hardware::test_support
