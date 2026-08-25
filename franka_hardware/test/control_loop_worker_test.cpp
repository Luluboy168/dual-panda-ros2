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

#include "franka_hardware/real/control_loop_worker.hpp"
#include "franka_hardware/real/robot.hpp"

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <chrono>
#include <fstream>
#include <future>
#include <mutex>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <vector>

namespace franka_hardware {
namespace {

using namespace std::chrono_literals;

template <typename Predicate>
bool waitUntil(Predicate predicate, std::chrono::milliseconds timeout = 1s) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::yield();
  }
  return predicate();
}

TEST(ControlLoopWorkerTest, IsInitiallyStopped) {
  ControlLoopWorker worker;

  EXPECT_EQ(worker.state(), ControlLoopWorker::State::Stopped);
  EXPECT_EQ(worker.requestedMode(), ControlMode::None);
  EXPECT_EQ(worker.runningMode(), ControlMode::None);
}

TEST(RobotCommandBufferStopTest, ClearsOnlyAfterStoppedRunningAndFaultedConsumersAreQuiescent) {
  const auto fill_command_buffer = [](auto& command_buffer) {
    const RobotCommand command{};
    ASSERT_TRUE(command_buffer.tryPush(command));
    ASSERT_TRUE(command_buffer.tryPush(command));
    ASSERT_FALSE(command_buffer.canPush());
  };

  {
    ControlLoopWorker worker;
    SpscRingBuffer<RobotCommand, 2> command_buffer;
    fill_command_buffer(command_buffer);
    EXPECT_TRUE(detail::stopWorkerAndClearCommandBuffer(worker, command_buffer));
    EXPECT_EQ(worker.state(), ControlLoopWorker::State::Stopped);
    EXPECT_TRUE(command_buffer.canPush());
  }

  {
    ControlLoopWorker worker;
    SpscRingBuffer<RobotCommand, 2> command_buffer;
    ASSERT_TRUE(worker.start([&worker](ControlMode mode) {
      while (!worker.shouldExitMode(mode)) {
        std::this_thread::yield();
      }
    }));
    ASSERT_TRUE(
        waitUntil([&worker]() { return worker.state() == ControlLoopWorker::State::Running; }));
    fill_command_buffer(command_buffer);
    EXPECT_TRUE(detail::stopWorkerAndClearCommandBuffer(worker, command_buffer));
    EXPECT_EQ(worker.state(), ControlLoopWorker::State::Stopped);
    EXPECT_TRUE(command_buffer.canPush());
  }

  {
    ControlLoopWorker worker;
    SpscRingBuffer<RobotCommand, 2> command_buffer;
    ASSERT_TRUE(worker.start([](ControlMode) { throw std::runtime_error("loop failure"); }));
    ASSERT_TRUE(
        waitUntil([&worker]() { return worker.state() == ControlLoopWorker::State::Faulted; }));
    fill_command_buffer(command_buffer);
    EXPECT_TRUE(detail::stopWorkerAndClearCommandBuffer(worker, command_buffer));
    EXPECT_EQ(worker.state(), ControlLoopWorker::State::Faulted);
    EXPECT_TRUE(command_buffer.canPush());
    fill_command_buffer(command_buffer);
    EXPECT_TRUE(detail::stopWorkerAndClearCommandBuffer(worker, command_buffer));
    EXPECT_EQ(worker.state(), ControlLoopWorker::State::Faulted);
    EXPECT_TRUE(command_buffer.canPush());
    EXPECT_TRUE(worker.clearFault());
  }
}

TEST(ControlLoopWorkerTest, RejectsDuplicateStart) {
  ControlLoopWorker worker;
  ASSERT_TRUE(worker.start([&worker](ControlMode mode) {
    while (!worker.shouldExitMode(mode)) {
      std::this_thread::yield();
    }
  }));
  ASSERT_TRUE(
      waitUntil([&worker]() { return worker.state() == ControlLoopWorker::State::Running; }));

  EXPECT_FALSE(worker.start([](ControlMode) {}));
  EXPECT_TRUE(worker.shutdown());
}

TEST(ControlLoopWorkerTest, ReusesOneThreadAcrossModeChanges) {
  ControlLoopWorker worker;
  std::mutex observations_mutex;
  std::vector<ControlMode> observed_modes;
  std::vector<std::thread::id> observed_thread_ids;

  ASSERT_TRUE(worker.start([&](ControlMode mode) {
    {
      std::lock_guard<std::mutex> lock(observations_mutex);
      observed_modes.push_back(mode);
      observed_thread_ids.push_back(std::this_thread::get_id());
    }
    while (!worker.shouldExitMode(mode)) {
      std::this_thread::yield();
    }
  }));

  const auto observed_count_is = [&](size_t count) {
    return waitUntil([&]() {
      std::lock_guard<std::mutex> lock(observations_mutex);
      return observed_modes.size() == count;
    });
  };

  ASSERT_TRUE(observed_count_is(1));
  ASSERT_TRUE(worker.requestMode(ControlMode::JointTorque));
  ASSERT_TRUE(observed_count_is(2));
  ASSERT_TRUE(worker.requestMode(ControlMode::JointVelocity));
  ASSERT_TRUE(observed_count_is(3));
  ASSERT_TRUE(worker.requestMode(ControlMode::None));
  ASSERT_TRUE(observed_count_is(4));
  ASSERT_TRUE(worker.shutdown());

  std::lock_guard<std::mutex> lock(observations_mutex);
  EXPECT_EQ(observed_modes,
            (std::vector<ControlMode>{ControlMode::None, ControlMode::JointTorque,
                                      ControlMode::JointVelocity, ControlMode::None}));
  ASSERT_EQ(observed_thread_ids.size(), observed_modes.size());
  for (const auto thread_id : observed_thread_ids) {
    EXPECT_EQ(thread_id, observed_thread_ids.front());
  }
}

TEST(ControlLoopWorkerTest, CoalescesRequestsBeforeCurrentLoopReturns) {
  ControlLoopWorker worker;
  std::atomic_bool release_initial_mode{false};
  std::mutex observations_mutex;
  std::vector<ControlMode> observed_modes;

  ASSERT_TRUE(worker.start([&](ControlMode mode) {
    {
      std::lock_guard<std::mutex> lock(observations_mutex);
      observed_modes.push_back(mode);
    }
    if (mode == ControlMode::None) {
      while (!release_initial_mode.load()) {
        std::this_thread::yield();
      }
    }
    while (!worker.shouldExitMode(mode)) {
      std::this_thread::yield();
    }
  }));

  ASSERT_TRUE(waitUntil([&]() {
    std::lock_guard<std::mutex> lock(observations_mutex);
    return observed_modes.size() == 1;
  }));
  ASSERT_TRUE(worker.requestMode(ControlMode::JointTorque));
  ASSERT_TRUE(worker.requestMode(ControlMode::JointVelocity));
  release_initial_mode.store(true);
  ASSERT_TRUE(waitUntil([&]() {
    std::lock_guard<std::mutex> lock(observations_mutex);
    return observed_modes.size() == 2;
  }));
  ASSERT_TRUE(worker.shutdown());

  std::lock_guard<std::mutex> lock(observations_mutex);
  EXPECT_EQ(observed_modes,
            (std::vector<ControlMode>{ControlMode::None, ControlMode::JointVelocity}));
}

TEST(ControlLoopWorkerTest, SerializesConcurrentShutdown) {
  ControlLoopWorker worker;
  ASSERT_TRUE(worker.start([&worker](ControlMode mode) {
    while (!worker.shouldExitMode(mode)) {
      std::this_thread::yield();
    }
  }));
  ASSERT_TRUE(
      waitUntil([&worker]() { return worker.state() == ControlLoopWorker::State::Running; }));

  auto first = std::async(std::launch::async, [&worker]() { return worker.shutdown(); });
  auto second = std::async(std::launch::async, [&worker]() { return worker.shutdown(); });

  EXPECT_NE(first.get(), second.get());
  EXPECT_EQ(worker.state(), ControlLoopWorker::State::Stopped);
  EXPECT_FALSE(worker.shutdown());
}

TEST(ControlLoopWorkerTest, RestartsAfterCleanShutdown) {
  ControlLoopWorker worker;
  const auto loop = [&worker](ControlMode mode) {
    while (!worker.shouldExitMode(mode)) {
      std::this_thread::yield();
    }
  };

  ASSERT_TRUE(worker.start(loop));
  ASSERT_TRUE(
      waitUntil([&worker]() { return worker.state() == ControlLoopWorker::State::Running; }));
  EXPECT_TRUE(worker.shutdown());

  ASSERT_TRUE(worker.start(loop, ControlMode::JointTorque));
  ASSERT_TRUE(waitUntil([&worker]() {
    return worker.state() == ControlLoopWorker::State::Running &&
           worker.runningMode() == ControlMode::JointTorque;
  }));
  EXPECT_TRUE(worker.shutdown());
}

TEST(ControlLoopWorkerTest, RecordsFaultAndCanRestart) {
  ControlLoopWorker worker;
  ASSERT_TRUE(worker.start([](ControlMode) { throw std::runtime_error("loop failure"); }));
  ASSERT_TRUE(
      waitUntil([&worker]() { return worker.state() == ControlLoopWorker::State::Faulted; }));

  EXPECT_FALSE(worker.requestMode(ControlMode::JointTorque));
  EXPECT_FALSE(worker.start([](ControlMode) {}));
  ASSERT_TRUE(worker.clearFault());
  EXPECT_EQ(worker.state(), ControlLoopWorker::State::Stopped);

  ASSERT_TRUE(worker.start([&worker](ControlMode mode) {
    while (!worker.shouldExitMode(mode)) {
      std::this_thread::yield();
    }
  }));
  ASSERT_TRUE(
      waitUntil([&worker]() { return worker.state() == ControlLoopWorker::State::Running; }));
  EXPECT_TRUE(worker.shutdown());
  EXPECT_EQ(worker.state(), ControlLoopWorker::State::Stopped);
}

TEST(ControlLoopWorkerTest, PreservesMostSpecificFailureUntilAConfirmedFreshStart) {
  ControlLoopWorker worker;
  worker.recordFailure(BackendFailureReason::UnexpectedLoopReturn);
  worker.recordFailure(BackendFailureReason::FrankaNetworkException);
  worker.recordFailure(BackendFailureReason::StandardException);
  worker.recordFailure(BackendFailureReason::UnexpectedLoopReturn);
  EXPECT_EQ(worker.failureReason(), BackendFailureReason::FrankaNetworkException);

  worker.recordFailure(BackendFailureReason::RobotReflex);
  worker.recordFailure(BackendFailureReason::FrankaControlException);
  EXPECT_EQ(worker.failureReason(), BackendFailureReason::RobotReflex);

  std::atomic_bool release_loop{false};
  ASSERT_TRUE(worker.start([&](ControlMode) {
    while (!release_loop.load()) {
      std::this_thread::yield();
    }
  }));
  ASSERT_TRUE(waitUntil([&worker]() {
    return worker.state() == ControlLoopWorker::State::Running &&
           worker.failureReason() == BackendFailureReason::None;
  }));
  release_loop.store(true);
  ASSERT_TRUE(
      waitUntil([&worker]() { return worker.state() == ControlLoopWorker::State::Faulted; }));
  EXPECT_EQ(worker.failureReason(), BackendFailureReason::UnexpectedLoopReturn);
  EXPECT_TRUE(worker.clearFault());
}

TEST(ControlLoopWorkerTest, LoopBodyTypedFailureCannotBeClearedAfterRunningTransition) {
  ControlLoopWorker worker;
  std::atomic_bool loop_entered{false};
  std::atomic_bool release_loop{false};
  std::atomic_bool has_error{false};

  worker.recordFailure(BackendFailureReason::StandardException);
  ASSERT_TRUE(worker.start([&](ControlMode mode) {
    detail::recordBackendFault(worker, has_error, BackendFailureReason::FrankaNetworkException);
    loop_entered.store(true, std::memory_order_release);
    while (!release_loop.load(std::memory_order_acquire)) {
      std::this_thread::yield();
    }
    while (!worker.shouldExitMode(mode)) {
      std::this_thread::yield();
    }
  }));
  ASSERT_TRUE(
      waitUntil([&loop_entered]() { return loop_entered.load(std::memory_order_acquire); }));
  EXPECT_TRUE(has_error.load(std::memory_order_acquire));
  EXPECT_EQ(worker.failureReason(), BackendFailureReason::FrankaNetworkException);
  release_loop.store(true, std::memory_order_release);
  EXPECT_TRUE(worker.shutdown());
}

TEST(BackendFailureReasonTest, FixedBitmaskPreservesPriorityUnderContentionAndClearsExactly) {
  constexpr std::array<BackendFailureReason, 15> reasons{
      BackendFailureReason::WorkerStartupFailure,
      BackendFailureReason::WorkerStateTransitionFailure,
      BackendFailureReason::UnexpectedLoopReturn,
      BackendFailureReason::RobotReflex,
      BackendFailureReason::FrankaControlException,
      BackendFailureReason::FrankaCommandException,
      BackendFailureReason::FrankaNetworkException,
      BackendFailureReason::FrankaProtocolException,
      BackendFailureReason::FrankaIncompatibleVersionException,
      BackendFailureReason::FrankaRealtimeException,
      BackendFailureReason::FrankaModelException,
      BackendFailureReason::FrankaInvalidOperationException,
      BackendFailureReason::FrankaException,
      BackendFailureReason::StandardException,
      BackendFailureReason::UnknownException,
  };
  std::atomic<BackendFailureReasonMask> mask{0};
  std::atomic_size_t ready{0};
  std::atomic_bool release{false};
  std::vector<std::thread> writers;
  writers.reserve(reasons.size());
  for (const auto reason : reasons) {
    writers.emplace_back([&mask, &ready, &release, reason]() {
      ready.fetch_add(1, std::memory_order_release);
      while (!release.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      recordBackendFailureReason(mask, reason);
    });
  }
  ASSERT_TRUE(waitUntil([&ready]() { return ready.load(std::memory_order_acquire) == 15; }));
  release.store(true, std::memory_order_release);
  for (auto& writer : writers) {
    writer.join();
  }

  BackendFailureReasonMask expected_mask = 0;
  for (const auto reason : reasons) {
    expected_mask |= backendFailureReasonBit(reason);
  }
  EXPECT_EQ(mask.load(std::memory_order_acquire), expected_mask);
  EXPECT_EQ(backendFailureReasonFromMask(expected_mask), BackendFailureReason::RobotReflex);
  EXPECT_EQ(backendFailureReasonFromMask(
                expected_mask & ~backendFailureReasonBit(BackendFailureReason::RobotReflex)),
            BackendFailureReason::FrankaControlException);
  EXPECT_EQ(kBackendFailureRecordMaximumAtomicOperations, 1U);
  clearBackendFailureReasons(mask);
  EXPECT_EQ(backendFailureReasonFromMask(mask.load(std::memory_order_acquire)),
            BackendFailureReason::None);
}

TEST(BackendFailureReasonTest, RecordingImplementationHasOneFetchOrAndNoRetryLoop) {
  const std::string test_source_path = __FILE__;
  const auto test_directory = test_source_path.rfind("/test/");
  ASSERT_NE(test_directory, std::string::npos);
  std::ifstream backend_header(test_source_path.substr(0, test_directory) +
                               "/include/franka_hardware/real/franka_arm_backend.hpp");
  ASSERT_TRUE(backend_header.is_open());
  const std::string source((std::istreambuf_iterator<char>(backend_header)),
                           std::istreambuf_iterator<char>());
  const auto record_start = source.find("inline void recordBackendFailureReason");
  const auto record_end = source.find("inline void clearBackendFailureReasons", record_start);
  ASSERT_NE(record_start, std::string::npos);
  ASSERT_NE(record_end, std::string::npos);
  const auto implementation = source.substr(record_start, record_end - record_start);
  EXPECT_NE(implementation.find("fetch_or"), std::string::npos);
  const auto nonzero_guard = implementation.find("if (bit != 0)");
  const auto fetch_or = implementation.find("fetch_or");
  ASSERT_NE(nonzero_guard, std::string::npos);
  ASSERT_NE(fetch_or, std::string::npos);
  EXPECT_LT(nonzero_guard, fetch_or);
  EXPECT_EQ(implementation.find("compare_exchange"), std::string::npos);
  EXPECT_EQ(implementation.find("while"), std::string::npos);
}

TEST(BackendFailureReasonTest, InvalidValuesAndUnknownMaskBitsNeverMutateOrDecode) {
  constexpr std::array<uint8_t, 6> invalid_values{0, 16, 31, 32, 127, 255};
  constexpr BackendFailureReasonMask unknown_high_bits = 0xFFFF8000U;
  static_assert(backendFailureReasonBit(static_cast<BackendFailureReason>(16)) == 0);
  static_assert(backendFailureReasonBit(static_cast<BackendFailureReason>(31)) == 0);
  static_assert(backendFailureReasonBit(static_cast<BackendFailureReason>(255)) == 0);
  static_assert(backendFailureReasonFromMask(unknown_high_bits) == BackendFailureReason::None);

  const auto network_bit = backendFailureReasonBit(BackendFailureReason::FrankaNetworkException);
  std::atomic<BackendFailureReasonMask> mask{network_bit};
  for (const auto value : invalid_values) {
    const auto reason = static_cast<BackendFailureReason>(value);
    EXPECT_EQ(backendFailureReasonBit(reason), 0U) << static_cast<unsigned int>(value);
    recordBackendFailureReason(mask, reason);
    EXPECT_EQ(mask.load(std::memory_order_acquire), network_bit)
        << static_cast<unsigned int>(value);
  }
  EXPECT_EQ(backendFailureReasonFromMask(unknown_high_bits), BackendFailureReason::None);
  EXPECT_EQ(backendFailureReasonFromMask(unknown_high_bits | network_bit),
            BackendFailureReason::FrankaNetworkException);
}

TEST(ControlLoopWorkerTest, SourceGuardKeepsClearPreLaunchAndRerecordsStartupFailures) {
  const std::string test_source_path = __FILE__;
  const auto test_directory = test_source_path.rfind("/test/");
  ASSERT_NE(test_directory, std::string::npos);
  std::ifstream worker_header(test_source_path.substr(0, test_directory) +
                              "/include/franka_hardware/real/control_loop_worker.hpp");
  ASSERT_TRUE(worker_header.is_open());
  const std::string source((std::istreambuf_iterator<char>(worker_header)),
                           std::istreambuf_iterator<char>());
  const auto start = source.find("bool start(Loop loop");
  const auto run = source.find("void run(Loop loop) noexcept", start);
  ASSERT_NE(start, std::string::npos);
  ASSERT_NE(run, std::string::npos);
  const auto start_implementation = source.substr(start, run - start);
  const auto clear = start_implementation.find("clearFailureReason();");
  const auto starting = start_implementation.find("state_.store(State::Starting);");
  const auto launch = start_implementation.find("worker_ = std::thread");
  ASSERT_NE(clear, std::string::npos);
  ASSERT_NE(starting, std::string::npos);
  ASSERT_NE(launch, std::string::npos);
  EXPECT_LT(clear, starting);
  EXPECT_LT(clear, launch);
  EXPECT_NE(start_implementation.find("recordFailure(BackendFailureReason::WorkerStartupFailure)",
                                      launch),
            std::string::npos);

  const auto run_implementation = source.substr(run);
  EXPECT_EQ(run_implementation.find("clearFailureReason();"), std::string::npos);
  EXPECT_EQ(source.find("RunningTransitionHook"), std::string::npos);
  EXPECT_EQ(source.find("running_transition_hook"), std::string::npos);
  EXPECT_NE(
      run_implementation.find("recordFailure(BackendFailureReason::WorkerStateTransitionFailure)"),
      std::string::npos);
}

TEST(ControlLoopWorkerTest, ClassifiesTypedAndGenericLoopExceptionsWithoutFormatting) {
  {
    ControlLoopWorker worker;
    ASSERT_TRUE(worker.start(
        [](ControlMode) { throw franka::NetworkException("synthetic network failure"); }));
    ASSERT_TRUE(
        waitUntil([&worker]() { return worker.state() == ControlLoopWorker::State::Faulted; }));
    EXPECT_EQ(worker.failureReason(), BackendFailureReason::FrankaNetworkException);
    EXPECT_TRUE(worker.clearFault());
  }
  {
    ControlLoopWorker worker;
    ASSERT_TRUE(worker.start([](ControlMode) { throw std::runtime_error("synthetic failure"); }));
    ASSERT_TRUE(
        waitUntil([&worker]() { return worker.state() == ControlLoopWorker::State::Faulted; }));
    EXPECT_EQ(worker.failureReason(), BackendFailureReason::StandardException);
    EXPECT_TRUE(worker.clearFault());
  }
  {
    ControlLoopWorker worker;
    EXPECT_FALSE(worker.start(ControlLoopWorker::Loop{}));
    EXPECT_EQ(worker.failureReason(), BackendFailureReason::WorkerStartupFailure);
  }
}

TEST(RobotBackendFailureRecordingTest,
     ContinuedConstructorFailuresAreTypedAndPreserveMoreSpecificReasons) {
  ControlLoopWorker worker;
  std::atomic_bool has_error{false};

  detail::recordBackendFault(worker, has_error, BackendFailureReason::FrankaControlException);
  EXPECT_TRUE(has_error.load(std::memory_order_acquire));
  EXPECT_EQ(worker.failureReason(), BackendFailureReason::FrankaControlException);

  has_error.store(false, std::memory_order_release);
  worker.recordFailure(BackendFailureReason::RobotReflex);
  detail::recordBackendFault(worker, has_error, BackendFailureReason::FrankaCommandException);
  EXPECT_TRUE(has_error.load(std::memory_order_acquire));
  EXPECT_EQ(worker.failureReason(), BackendFailureReason::RobotReflex);
}

TEST(RobotBackendFailureRecordingTest,
     ParameterOperationRecordsCurrentTypedFailureBeforeRethrowing) {
  ControlLoopWorker worker;
  std::atomic_bool has_error{false};

  EXPECT_THROW(detail::runBackendOperationWithFailureRecording(
                   worker, has_error,
                   []() { throw franka::NetworkException("synthetic parameter network failure"); }),
               franka::NetworkException);
  EXPECT_TRUE(has_error.load(std::memory_order_acquire));
  EXPECT_EQ(worker.failureReason(), BackendFailureReason::FrankaNetworkException);
}

TEST(RobotFaultBoundaryTest,
     ConstructorAndEveryParameterFaultRejectLifecycleAndModeUntilRecoveryClears) {
  struct FaultOrigin {
    const char* name;
    BackendFailureReason reason;
  };
  constexpr std::array<FaultOrigin, 9> origins{{
      {"constructor_control", BackendFailureReason::FrankaControlException},
      {"constructor_command", BackendFailureReason::FrankaCommandException},
      {"joint_stiffness", BackendFailureReason::FrankaCommandException},
      {"cartesian_stiffness", BackendFailureReason::FrankaNetworkException},
      {"load", BackendFailureReason::FrankaCommandException},
      {"tcp_frame", BackendFailureReason::FrankaNetworkException},
      {"stiffness_frame", BackendFailureReason::FrankaCommandException},
      {"force_torque_collision", BackendFailureReason::FrankaNetworkException},
      {"full_collision", BackendFailureReason::FrankaCommandException},
  }};

  for (const auto& origin : origins) {
    SCOPED_TRACE(origin.name);
    ControlLoopWorker lifecycle_worker;
    std::atomic_bool lifecycle_error{false};
    std::atomic_bool loop_called{false};
    detail::recordBackendFault(lifecycle_worker, lifecycle_error, origin.reason);

    EXPECT_FALSE(detail::startOrRequestWorkerUnlessFaulted(
        lifecycle_worker, lifecycle_error,
        [&loop_called](ControlMode) { loop_called.store(true, std::memory_order_release); },
        ControlMode::None));
    EXPECT_FALSE(detail::requestWorkerModeUnlessFaulted(lifecycle_worker, lifecycle_error,
                                                        ControlMode::JointVelocity));
    EXPECT_FALSE(loop_called.load(std::memory_order_acquire));
    EXPECT_EQ(lifecycle_worker.state(), ControlLoopWorker::State::Stopped);
    EXPECT_EQ(lifecycle_worker.requestedMode(), ControlMode::None);
    EXPECT_EQ(lifecycle_worker.failureReason(), origin.reason);

    // Model a successful, non-restarting recovery: it is the only path that lowers has_error and
    // clears the stopped worker's prior reason. A fresh lifecycle start is accepted afterwards.
    lifecycle_error.store(false, std::memory_order_release);
    detail::finalizeSuccessfulRecoveryFailureReason(lifecycle_worker, false);
    EXPECT_EQ(lifecycle_worker.failureReason(), BackendFailureReason::None);
    ASSERT_TRUE(detail::startOrRequestWorkerUnlessFaulted(
        lifecycle_worker, lifecycle_error,
        [&lifecycle_worker](ControlMode mode) {
          while (!lifecycle_worker.shouldExitMode(mode)) {
            std::this_thread::yield();
          }
        },
        ControlMode::None));
    ASSERT_TRUE(waitUntil([&lifecycle_worker]() {
      return lifecycle_worker.state() == ControlLoopWorker::State::Running;
    }));
    EXPECT_TRUE(lifecycle_worker.shutdown());

    ControlLoopWorker mode_worker;
    std::atomic_bool mode_error{false};
    ASSERT_TRUE(mode_worker.start([&mode_worker](ControlMode mode) {
      while (!mode_worker.shouldExitMode(mode)) {
        std::this_thread::yield();
      }
    }));
    ASSERT_TRUE(waitUntil(
        [&mode_worker]() { return mode_worker.state() == ControlLoopWorker::State::Running; }));
    detail::recordBackendFault(mode_worker, mode_error, origin.reason);
    EXPECT_FALSE(detail::requestWorkerModeUnlessFaulted(mode_worker, mode_error,
                                                        ControlMode::JointVelocity));
    EXPECT_EQ(mode_worker.requestedMode(), ControlMode::None);
    EXPECT_EQ(mode_worker.failureReason(), origin.reason);
    EXPECT_TRUE(mode_worker.shutdown());
  }
}

TEST(RobotFaultBoundaryTest, FailedStartLatchesErrorAndRetryCannotClearBeforeRecovery) {
  ControlLoopWorker worker;
  std::atomic_bool has_error{false};
  const auto invalid_mode = ControlMode::JointTorque | ControlMode::JointVelocity;

  EXPECT_FALSE(detail::startOrRequestWorkerUnlessFaulted(
      worker, has_error, [](ControlMode) {}, invalid_mode));
  EXPECT_EQ(worker.failureReason(), BackendFailureReason::WorkerStartupFailure);
  detail::recordBackendFault(worker, has_error, BackendFailureReason::WorkerStateTransitionFailure);
  EXPECT_TRUE(has_error.load(std::memory_order_acquire));
  EXPECT_EQ(worker.failureReason(), BackendFailureReason::WorkerStartupFailure);

  EXPECT_FALSE(detail::startOrRequestWorkerUnlessFaulted(
      worker, has_error, [](ControlMode) {}, ControlMode::None));
  EXPECT_EQ(worker.state(), ControlLoopWorker::State::Stopped);
  EXPECT_EQ(worker.failureReason(), BackendFailureReason::WorkerStartupFailure);

  has_error.store(false, std::memory_order_release);
  detail::finalizeSuccessfulRecoveryFailureReason(worker, false);
  EXPECT_EQ(worker.failureReason(), BackendFailureReason::None);
  ASSERT_TRUE(detail::startOrRequestWorkerUnlessFaulted(
      worker, has_error,
      [&worker](ControlMode mode) {
        while (!worker.shouldExitMode(mode)) {
          std::this_thread::yield();
        }
      },
      ControlMode::None));
  ASSERT_TRUE(
      waitUntil([&worker]() { return worker.state() == ControlLoopWorker::State::Running; }));
  EXPECT_TRUE(worker.shutdown());
}

TEST(RobotBackendFailureRecordingTest, ConstructorCatchBlocksUseExactTypedRecordingHelper) {
  const std::string test_source_path = __FILE__;
  const auto test_directory = test_source_path.rfind("/test/");
  ASSERT_NE(test_directory, std::string::npos);
  std::ifstream robot_source(test_source_path.substr(0, test_directory) + "/src/real/robot.cpp");
  ASSERT_TRUE(robot_source.is_open());
  const std::string source((std::istreambuf_iterator<char>(robot_source)),
                           std::istreambuf_iterator<char>());

  const auto control_catch = source.find("catch (const franka::ControlException& exception)");
  const auto command_catch =
      source.find("catch (const franka::CommandException& exception)", control_catch);
  const auto constructor_read = source.find("current_state_ = robot_->readOnce()", command_catch);
  ASSERT_NE(control_catch, std::string::npos);
  ASSERT_NE(command_catch, std::string::npos);
  ASSERT_NE(constructor_read, std::string::npos);

  const auto control_block = source.substr(control_catch, command_catch - control_catch);
  const auto command_block = source.substr(command_catch, constructor_read - command_catch);
  EXPECT_NE(control_block.find("detail::recordBackendFault"), std::string::npos);
  EXPECT_NE(control_block.find("BackendFailureReason::FrankaControlException"), std::string::npos);
  EXPECT_NE(command_block.find("detail::recordBackendFault"), std::string::npos);
  EXPECT_NE(command_block.find("BackendFailureReason::FrankaCommandException"), std::string::npos);
}

TEST(RobotFaultBoundaryTest, SourceGuardBindsAllRealEntryPointsToFaultChecksAndLatching) {
  const std::string test_source_path = __FILE__;
  const auto test_directory = test_source_path.rfind("/test/");
  ASSERT_NE(test_directory, std::string::npos);
  std::ifstream robot_source(test_source_path.substr(0, test_directory) + "/src/real/robot.cpp");
  ASSERT_TRUE(robot_source.is_open());
  const std::string source((std::istreambuf_iterator<char>(robot_source)),
                           std::istreambuf_iterator<char>());

  const auto start = source.find("bool Robot::startLoop(ControlMode initial_mode)");
  const auto start_end = source.find("bool Robot::initializeTorqueControl()", start);
  ASSERT_NE(start, std::string::npos);
  ASSERT_NE(start_end, std::string::npos);
  const auto start_block = source.substr(start, start_end - start);
  const auto fault_guard = start_block.find("detail::hasBackendFault");
  const auto worker_operation = start_block.find("detail::startOrRequestWorkerUnlessFaulted");
  const auto transition_latch = start_block.find("detail::recordBackendFault", worker_operation);
  const auto transition_reason =
      start_block.find("BackendFailureReason::WorkerStateTransitionFailure", transition_latch);
  ASSERT_NE(fault_guard, std::string::npos);
  ASSERT_NE(worker_operation, std::string::npos);
  ASSERT_NE(transition_latch, std::string::npos);
  ASSERT_NE(transition_reason, std::string::npos);
  EXPECT_LT(fault_guard, worker_operation);
  EXPECT_LT(transition_latch, transition_reason);

  const auto request = source.find("bool Robot::requestControlMode(ControlMode control_mode)");
  const auto can_request = source.find("bool Robot::canRequestControlMode", request);
  const auto get_mode = source.find("ControlMode Robot::getControlMode", can_request);
  ASSERT_NE(request, std::string::npos);
  ASSERT_NE(can_request, std::string::npos);
  ASSERT_NE(get_mode, std::string::npos);
  EXPECT_NE(
      source.substr(request, can_request - request).find("detail::requestWorkerModeUnlessFaulted"),
      std::string::npos);
  EXPECT_NE(source.substr(can_request, get_mode - can_request).find("detail::hasBackendFault"),
            std::string::npos);

  constexpr std::string_view parameter_helper{"detail::runBackendOperationWithFailureRecording("};
  size_t parameter_helper_count = 0;
  for (size_t position = source.find(parameter_helper); position != std::string::npos;
       position = source.find(parameter_helper, position + parameter_helper.size())) {
    ++parameter_helper_count;
  }
  EXPECT_EQ(parameter_helper_count, 7U);
}

TEST(RobotRecoveryFailureReasonTest,
     ImmediateRestartFaultIsPreservedAndNonRestartingSuccessClearsPriorReason) {
  ControlLoopWorker worker;
  worker.recordFailure(BackendFailureReason::FrankaControlException);

  std::promise<void> loop_entered;
  std::promise<void> release_fault;
  auto release_future = release_fault.get_future();
  ASSERT_TRUE(worker.start([&](ControlMode) {
    loop_entered.set_value();
    release_future.wait();
    throw franka::NetworkException("synthetic immediate recovery restart failure");
  }));
  const auto loop_entry_status = loop_entered.get_future().wait_for(1s);

  // Deterministically place the new typed worker fault before the recovery caller finalizes and
  // returns. A restarted recovery must never clear this new, more specific reason.
  release_fault.set_value();
  ASSERT_EQ(loop_entry_status, std::future_status::ready);
  ASSERT_TRUE(
      waitUntil([&worker]() { return worker.state() == ControlLoopWorker::State::Faulted; }));
  detail::finalizeSuccessfulRecoveryFailureReason(worker, true);
  EXPECT_EQ(worker.failureReason(), BackendFailureReason::FrankaNetworkException);

  ASSERT_TRUE(worker.clearFault());
  worker.recordFailure(BackendFailureReason::FrankaControlException);
  detail::finalizeSuccessfulRecoveryFailureReason(worker, false);
  EXPECT_EQ(worker.failureReason(), BackendFailureReason::None);
}

TEST(ControlLoopWorkerTest, RejectsCombinedControlModes) {
  ControlLoopWorker worker;
  const auto combined_mode = ControlMode::JointTorque | ControlMode::JointVelocity;

  EXPECT_FALSE(worker.start([](ControlMode) {}, combined_mode));
  ASSERT_TRUE(worker.start([&worker](ControlMode mode) {
    while (!worker.shouldExitMode(mode)) {
      std::this_thread::yield();
    }
  }));
  ASSERT_TRUE(
      waitUntil([&worker]() { return worker.state() == ControlLoopWorker::State::Running; }));
  EXPECT_FALSE(worker.requestMode(combined_mode));
  EXPECT_TRUE(worker.shutdown());
}

TEST(ControlLoopWorkerTest, DestructorRequestsStopAndJoins) {
  std::atomic_bool loop_entered{false};
  std::atomic_bool loop_exited{false};
  {
    ControlLoopWorker worker;
    ASSERT_TRUE(worker.start([&](ControlMode mode) {
      loop_entered.store(true);
      while (!worker.shouldExitMode(mode)) {
        std::this_thread::yield();
      }
      loop_exited.store(true);
    }));
    ASSERT_TRUE(waitUntil([&loop_entered]() { return loop_entered.load(); }));
  }

  EXPECT_TRUE(loop_exited.load());
}

TEST(ControlLoopWorkerTest, ImmediateShutdownCannotBeOverwrittenByWorkerStartup) {
  for (size_t iteration = 0; iteration < 1000; ++iteration) {
    ControlLoopWorker worker;
    ASSERT_TRUE(worker.start([&worker](ControlMode mode) {
      while (!worker.shouldExitMode(mode)) {
        std::this_thread::yield();
      }
    }));
    EXPECT_TRUE(worker.shutdown());
    EXPECT_EQ(worker.state(), ControlLoopWorker::State::Stopped);
    EXPECT_FALSE(worker.requestMode(ControlMode::JointTorque));
  }
}

}  // namespace
}  // namespace franka_hardware
