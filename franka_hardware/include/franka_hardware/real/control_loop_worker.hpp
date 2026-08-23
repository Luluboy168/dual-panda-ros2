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

#include <atomic>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>

#include "franka_hardware/common/control_mode.h"

namespace franka_hardware
{

class ControlLoopWorker
{
public:
  enum class State
  {
    Stopped,
    Starting,
    Running,
    StopRequested,
    Faulted
  };
  using Loop = std::function<void(ControlMode)>;

  ControlLoopWorker() = default;
  ControlLoopWorker(const ControlLoopWorker &) = delete;
  ControlLoopWorker & operator=(const ControlLoopWorker &) = delete;
  ControlLoopWorker(ControlLoopWorker &&) = delete;
  ControlLoopWorker & operator=(ControlLoopWorker &&) = delete;

  ~ControlLoopWorker() { shutdown(); }

  bool start(Loop loop, ControlMode initial_mode = ControlMode::None)
  {
    if (!loop || !isValidMode(initial_mode)) {
      return false;
    }

    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    const auto current_state = state_.load();
    if (current_state != State::Stopped) {
      return false;
    }

    if (worker_.joinable()) {
      worker_.join();
    }

    shutdown_requested_.store(false);
    requested_mode_.store(initial_mode);
    running_mode_.store(ControlMode::None);
    state_.store(State::Starting);

    try {
      worker_ = std::thread([this, loop = std::move(loop)]() mutable { run(std::move(loop)); });
    } catch (...) {
      shutdown_requested_.store(false);
      requested_mode_.store(ControlMode::None);
      running_mode_.store(ControlMode::None);
      state_.store(State::Stopped);
      throw;
    }
    return true;
  }

  bool requestMode(ControlMode mode)
  {
    if (!canRequestMode(mode)) {
      return false;
    }
    requested_mode_.store(mode);
    return canRequestMode(mode);
  }

  bool canRequestMode(ControlMode mode) const
  {
    if (!isValidMode(mode) || shutdown_requested_.load()) {
      return false;
    }
    const auto current_state = state_.load();
    return current_state == State::Starting || current_state == State::Running;
  }

  bool shouldExitMode(ControlMode running_mode) const
  {
    return shutdown_requested_.load() || requested_mode_.load() != running_mode;
  }

  bool shutdown()
  {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (!worker_.joinable()) {
      if (state_.load() != State::Faulted) {
        state_.store(State::Stopped);
      }
      return false;
    }

    shutdown_requested_.store(true);
    if (state_.load() != State::Faulted) {
      state_.store(State::StopRequested);
    }
    if (worker_.get_id() == std::this_thread::get_id()) {
      return false;
    }
    worker_.join();
    running_mode_.store(ControlMode::None);
    if (state_.load() != State::Faulted) {
      state_.store(State::Stopped);
    }
    return true;
  }

  bool clearFault()
  {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (state_.load() != State::Faulted) {
      return false;
    }
    if (worker_.joinable()) {
      if (worker_.get_id() == std::this_thread::get_id()) {
        return false;
      }
      worker_.join();
    }
    shutdown_requested_.store(false);
    requested_mode_.store(ControlMode::None);
    running_mode_.store(ControlMode::None);
    state_.store(State::Stopped);
    return true;
  }

  State state() const { return state_.load(); }

  ControlMode requestedMode() const { return requested_mode_.load(); }

  ControlMode runningMode() const { return running_mode_.load(); }

private:
  static bool isValidMode(ControlMode mode)
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

  void run(Loop loop) noexcept
  {
    auto expected_state = State::Starting;
    if (!state_.compare_exchange_strong(expected_state, State::Running)) {
      running_mode_.store(ControlMode::None);
      if (shutdown_requested_.load()) {
        state_.store(State::Stopped);
      } else {
        state_.store(State::Faulted);
      }
      return;
    }
    while (!shutdown_requested_.load()) {
      const auto mode = requested_mode_.load();
      running_mode_.store(mode);
      try {
        loop(mode);
      } catch (...) {
        running_mode_.store(ControlMode::None);
        state_.store(State::Faulted);
        return;
      }
      running_mode_.store(ControlMode::None);

      if (shutdown_requested_.load()) {
        break;
      }
      if (requested_mode_.load() == mode) {
        state_.store(State::Faulted);
        return;
      }
    }
    state_.store(State::Stopped);
  }

  mutable std::mutex lifecycle_mutex_;
  std::thread worker_;
  std::atomic<State> state_{State::Stopped};
  std::atomic<ControlMode> requested_mode_{ControlMode::None};
  std::atomic<ControlMode> running_mode_{ControlMode::None};
  std::atomic_bool shutdown_requested_{false};
};

}  // namespace franka_hardware
