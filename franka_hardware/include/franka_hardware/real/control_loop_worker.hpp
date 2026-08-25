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

#include <franka/exception.h>

#include <atomic>
#include <functional>
#include <mutex>
#include <thread>
#include <type_traits>
#include <utility>

#include "franka_hardware/common/control_mode.h"
#include "franka_hardware/real/franka_arm_backend.hpp"

namespace franka_hardware {

class ControlLoopWorker {
 public:
  enum class State { Stopped, Starting, Running, StopRequested, Faulted };
  using Loop = std::function<void(ControlMode)>;

  ControlLoopWorker() = default;
  ControlLoopWorker(const ControlLoopWorker&) = delete;
  ControlLoopWorker& operator=(const ControlLoopWorker&) = delete;
  ControlLoopWorker(ControlLoopWorker&&) = delete;
  ControlLoopWorker& operator=(ControlLoopWorker&&) = delete;

  ~ControlLoopWorker() { shutdown(); }

  bool start(Loop loop, ControlMode initial_mode = ControlMode::None) {
    if (!loop || !isValidMode(initial_mode)) {
      recordFailure(BackendFailureReason::WorkerStartupFailure);
      return false;
    }

    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    const auto current_state = state_.load();
    if (current_state != State::Stopped) {
      recordFailure(BackendFailureReason::WorkerStateTransitionFailure);
      return false;
    }

    if (worker_.joinable()) {
      worker_.join();
    }

    // The prior run is quiescent and the real backend Lifecycle/Recovery gate excludes service
    // failure writers here. Clear before publishing Starting or launching the new worker.
    clearFailureReason();
    shutdown_requested_.store(false);
    requested_mode_.store(initial_mode);
    running_mode_.store(ControlMode::None);
    state_.store(State::Starting);

    try {
      worker_ = std::thread([this, loop = std::move(loop)]() mutable { run(std::move(loop)); });
    } catch (...) {
      recordFailure(BackendFailureReason::WorkerStartupFailure);
      shutdown_requested_.store(false);
      requested_mode_.store(ControlMode::None);
      running_mode_.store(ControlMode::None);
      state_.store(State::Stopped);
      throw;
    }
    return true;
  }

  bool requestMode(ControlMode mode) {
    if (!canRequestMode(mode)) {
      return false;
    }
    requested_mode_.store(mode);
    return canRequestMode(mode);
  }

  bool canRequestMode(ControlMode mode) const {
    if (!isValidMode(mode) || shutdown_requested_.load()) {
      return false;
    }
    const auto current_state = state_.load();
    return current_state == State::Starting || current_state == State::Running;
  }

  bool shouldExitMode(ControlMode running_mode) const {
    return shutdown_requested_.load() || requested_mode_.load() != running_mode;
  }

  bool shutdown() {
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

  bool clearFault() {
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

  BackendFailureReason failureReason() const noexcept {
    return backendFailureReasonFromMask(failure_reason_mask_.load(std::memory_order_acquire));
  }

  void recordFailure(BackendFailureReason reason) noexcept {
    recordBackendFailureReason(failure_reason_mask_, reason);
  }

  void clearFailureReason() noexcept { clearBackendFailureReasons(failure_reason_mask_); }

 private:
  static bool isValidMode(ControlMode mode) {
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

  void run(Loop loop) noexcept {
    auto expected_state = State::Starting;
    if (!state_.compare_exchange_strong(expected_state, State::Running)) {
      running_mode_.store(ControlMode::None);
      if (shutdown_requested_.load()) {
        state_.store(State::Stopped);
      } else {
        recordFailure(BackendFailureReason::WorkerStateTransitionFailure);
        state_.store(State::Faulted);
      }
      return;
    }
    while (!shutdown_requested_.load()) {
      const auto mode = requested_mode_.load();
      running_mode_.store(mode);
      try {
        loop(mode);
      } catch (const franka::ControlException&) {
        recordFailure(BackendFailureReason::FrankaControlException);
        running_mode_.store(ControlMode::None);
        state_.store(State::Faulted);
        return;
      } catch (const franka::CommandException&) {
        recordFailure(BackendFailureReason::FrankaCommandException);
        running_mode_.store(ControlMode::None);
        state_.store(State::Faulted);
        return;
      } catch (const franka::NetworkException&) {
        recordFailure(BackendFailureReason::FrankaNetworkException);
        running_mode_.store(ControlMode::None);
        state_.store(State::Faulted);
        return;
      } catch (const franka::ProtocolException&) {
        recordFailure(BackendFailureReason::FrankaProtocolException);
        running_mode_.store(ControlMode::None);
        state_.store(State::Faulted);
        return;
      } catch (const franka::IncompatibleVersionException&) {
        recordFailure(BackendFailureReason::FrankaIncompatibleVersionException);
        running_mode_.store(ControlMode::None);
        state_.store(State::Faulted);
        return;
      } catch (const franka::RealtimeException&) {
        recordFailure(BackendFailureReason::FrankaRealtimeException);
        running_mode_.store(ControlMode::None);
        state_.store(State::Faulted);
        return;
      } catch (const franka::ModelException&) {
        recordFailure(BackendFailureReason::FrankaModelException);
        running_mode_.store(ControlMode::None);
        state_.store(State::Faulted);
        return;
      } catch (const franka::InvalidOperationException&) {
        recordFailure(BackendFailureReason::FrankaInvalidOperationException);
        running_mode_.store(ControlMode::None);
        state_.store(State::Faulted);
        return;
      } catch (const franka::Exception&) {
        recordFailure(BackendFailureReason::FrankaException);
        running_mode_.store(ControlMode::None);
        state_.store(State::Faulted);
        return;
      } catch (const std::exception&) {
        recordFailure(BackendFailureReason::StandardException);
        running_mode_.store(ControlMode::None);
        state_.store(State::Faulted);
        return;
      } catch (...) {
        recordFailure(BackendFailureReason::UnknownException);
        running_mode_.store(ControlMode::None);
        state_.store(State::Faulted);
        return;
      }
      running_mode_.store(ControlMode::None);

      if (shutdown_requested_.load()) {
        break;
      }
      if (requested_mode_.load() == mode) {
        recordFailure(BackendFailureReason::UnexpectedLoopReturn);
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
  std::atomic<BackendFailureReasonMask> failure_reason_mask_{0};
};

namespace detail {

inline void recordBackendFault(ControlLoopWorker& worker,
                               std::atomic_bool& has_error,
                               BackendFailureReason reason) noexcept {
  worker.recordFailure(reason);
  has_error.store(true, std::memory_order_release);
}

inline bool hasBackendFault(const ControlLoopWorker& worker,
                            const std::atomic_bool& has_error) noexcept {
  return has_error.load(std::memory_order_acquire) ||
         worker.state() == ControlLoopWorker::State::Faulted;
}

inline bool requestWorkerModeUnlessFaulted(ControlLoopWorker& worker,
                                           const std::atomic_bool& has_error,
                                           ControlMode mode) noexcept {
  if (hasBackendFault(worker, has_error) || !worker.requestMode(mode)) {
    return false;
  }
  // A libfranka loop fault can race the request. Report rejection if it won; the faulted worker
  // will not consume the newly requested mode.
  return !hasBackendFault(worker, has_error);
}

inline bool startOrRequestWorkerUnlessFaulted(ControlLoopWorker& worker,
                                              const std::atomic_bool& has_error,
                                              ControlLoopWorker::Loop loop,
                                              ControlMode mode) {
  if (hasBackendFault(worker, has_error)) {
    return false;
  }
  if (worker.state() == ControlLoopWorker::State::Stopped) {
    return worker.start(std::move(loop), mode);
  }
  return requestWorkerModeUnlessFaulted(worker, has_error, mode);
}

inline void recordCurrentBackendFault(ControlLoopWorker& worker,
                                      std::atomic_bool& has_error) noexcept {
  try {
    throw;
  } catch (const franka::ControlException&) {
    recordBackendFault(worker, has_error, BackendFailureReason::FrankaControlException);
  } catch (const franka::CommandException&) {
    recordBackendFault(worker, has_error, BackendFailureReason::FrankaCommandException);
  } catch (const franka::NetworkException&) {
    recordBackendFault(worker, has_error, BackendFailureReason::FrankaNetworkException);
  } catch (const franka::ProtocolException&) {
    recordBackendFault(worker, has_error, BackendFailureReason::FrankaProtocolException);
  } catch (const franka::IncompatibleVersionException&) {
    recordBackendFault(worker, has_error, BackendFailureReason::FrankaIncompatibleVersionException);
  } catch (const franka::RealtimeException&) {
    recordBackendFault(worker, has_error, BackendFailureReason::FrankaRealtimeException);
  } catch (const franka::ModelException&) {
    recordBackendFault(worker, has_error, BackendFailureReason::FrankaModelException);
  } catch (const franka::InvalidOperationException&) {
    recordBackendFault(worker, has_error, BackendFailureReason::FrankaInvalidOperationException);
  } catch (const franka::Exception&) {
    recordBackendFault(worker, has_error, BackendFailureReason::FrankaException);
  } catch (const std::exception&) {
    recordBackendFault(worker, has_error, BackendFailureReason::StandardException);
  } catch (...) {
    recordBackendFault(worker, has_error, BackendFailureReason::UnknownException);
  }
}

template <typename Operation>
void runBackendOperationWithFailureRecording(ControlLoopWorker& worker,
                                             std::atomic_bool& has_error,
                                             Operation&& operation) {
  static_assert(std::is_invocable_v<Operation>);
  try {
    std::forward<Operation>(operation)();
  } catch (...) {
    recordCurrentBackendFault(worker, has_error);
    throw;
  }
}

}  // namespace detail

static_assert(std::atomic<ControlLoopWorker::State>::is_always_lock_free,
              "The RT-written worker state must be lock-free");

}  // namespace franka_hardware
