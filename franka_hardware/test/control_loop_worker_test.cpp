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

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <future>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

namespace franka_hardware
{
namespace
{

using namespace std::chrono_literals;

template <typename Predicate>
bool waitUntil(Predicate predicate, std::chrono::milliseconds timeout = 1s)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::yield();
  }
  return predicate();
}

TEST(ControlLoopWorkerTest, IsInitiallyStopped)
{
  ControlLoopWorker worker;

  EXPECT_EQ(worker.state(), ControlLoopWorker::State::Stopped);
  EXPECT_EQ(worker.requestedMode(), ControlMode::None);
  EXPECT_EQ(worker.runningMode(), ControlMode::None);
}

TEST(ControlLoopWorkerTest, RejectsDuplicateStart)
{
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

TEST(ControlLoopWorkerTest, ReusesOneThreadAcrossModeChanges)
{
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
  EXPECT_EQ(
    observed_modes,
    (std::vector<ControlMode>{
      ControlMode::None, ControlMode::JointTorque, ControlMode::JointVelocity, ControlMode::None}));
  ASSERT_EQ(observed_thread_ids.size(), observed_modes.size());
  for (const auto thread_id : observed_thread_ids) {
    EXPECT_EQ(thread_id, observed_thread_ids.front());
  }
}

TEST(ControlLoopWorkerTest, CoalescesRequestsBeforeCurrentLoopReturns)
{
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
  EXPECT_EQ(
    observed_modes, (std::vector<ControlMode>{ControlMode::None, ControlMode::JointVelocity}));
}

TEST(ControlLoopWorkerTest, SerializesConcurrentShutdown)
{
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

TEST(ControlLoopWorkerTest, RestartsAfterCleanShutdown)
{
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

TEST(ControlLoopWorkerTest, RecordsFaultAndCanRestart)
{
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

TEST(ControlLoopWorkerTest, RejectsCombinedControlModes)
{
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

TEST(ControlLoopWorkerTest, DestructorRequestsStopAndJoins)
{
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

TEST(ControlLoopWorkerTest, ImmediateShutdownCannotBeOverwrittenByWorkerStartup)
{
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
