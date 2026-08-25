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

#include <franka/robot_state.h>

#include <atomic>
#include <cstdint>

#include "franka_hardware/common/control_mode.h"
#include "franka_hardware/common/model_base.hpp"
#include "franka_hardware/real/robot_command.hpp"
#include "franka_msgs/srv/set_cartesian_stiffness.hpp"
#include "franka_msgs/srv/set_force_torque_collision_behavior.hpp"
#include "franka_msgs/srv/set_full_collision_behavior.hpp"
#include "franka_msgs/srv/set_joint_stiffness.hpp"
#include "franka_msgs/srv/set_load.hpp"
#include "franka_msgs/srv/set_stiffness_frame.hpp"
#include "franka_msgs/srv/set_tcp_frame.hpp"

namespace franka_hardware {

enum class BackendWorkerState : uint8_t { Stopped, Starting, Running, StopRequested, Faulted };
enum class BackendFaultCategory : uint8_t { None, Worker };
enum class BackendServiceOperation : uint8_t { Idle, Parameter, Recovery, ModeRequest, Lifecycle };
enum class BackendFailureReason : uint8_t {
  None,
  WorkerStartupFailure,
  WorkerStateTransitionFailure,
  UnexpectedLoopReturn,
  RobotReflex,
  FrankaControlException,
  FrankaCommandException,
  FrankaNetworkException,
  FrankaProtocolException,
  FrankaIncompatibleVersionException,
  FrankaRealtimeException,
  FrankaModelException,
  FrankaInvalidOperationException,
  FrankaException,
  StandardException,
  UnknownException,
};
enum class BackendRecoveryResult : uint8_t { NeverAttempted, Succeeded, Failed };

constexpr uint8_t backendFailureReasonPriority(BackendFailureReason reason) noexcept {
  switch (reason) {
    case BackendFailureReason::None:
      return 0;
    case BackendFailureReason::UnexpectedLoopReturn:
      return 1;
    case BackendFailureReason::WorkerStartupFailure:
    case BackendFailureReason::WorkerStateTransitionFailure:
      return 2;
    case BackendFailureReason::StandardException:
    case BackendFailureReason::UnknownException:
      return 3;
    case BackendFailureReason::FrankaException:
      return 4;
    case BackendFailureReason::FrankaControlException:
    case BackendFailureReason::FrankaCommandException:
    case BackendFailureReason::FrankaNetworkException:
    case BackendFailureReason::FrankaProtocolException:
    case BackendFailureReason::FrankaIncompatibleVersionException:
    case BackendFailureReason::FrankaRealtimeException:
    case BackendFailureReason::FrankaModelException:
    case BackendFailureReason::FrankaInvalidOperationException:
      return 5;
    case BackendFailureReason::RobotReflex:
      return 6;
  }
  return 0;
}

using BackendFailureReasonMask = uint32_t;

constexpr size_t kBackendFailureRecordMaximumAtomicOperations = 1;

constexpr BackendFailureReasonMask backendFailureReasonBit(BackendFailureReason reason) noexcept {
  const auto index = static_cast<uint8_t>(reason);
  constexpr auto first = static_cast<uint8_t>(BackendFailureReason::WorkerStartupFailure);
  constexpr auto last = static_cast<uint8_t>(BackendFailureReason::UnknownException);
  if (index < first || index > last) {
    return BackendFailureReasonMask{0};
  }
  return static_cast<BackendFailureReasonMask>(BackendFailureReasonMask{1} << (index - first));
}

constexpr BackendFailureReason backendFailureReasonFromMask(
    BackendFailureReasonMask mask) noexcept {
  auto selected = BackendFailureReason::None;
  for (uint8_t index = static_cast<uint8_t>(BackendFailureReason::WorkerStartupFailure);
       index <= static_cast<uint8_t>(BackendFailureReason::UnknownException); ++index) {
    const auto candidate = static_cast<BackendFailureReason>(index);
    if ((mask & backendFailureReasonBit(candidate)) != 0 &&
        backendFailureReasonPriority(candidate) > backendFailureReasonPriority(selected)) {
      // Equal-priority reasons have a deterministic taxonomy-order tie break.
      selected = candidate;
    }
  }
  return selected;
}

inline void recordBackendFailureReason(std::atomic<BackendFailureReasonMask>& destination,
                                       BackendFailureReason candidate) noexcept {
  const auto bit = backendFailureReasonBit(candidate);
  if (bit != 0) {
    // One lock-free RMW and no retry loop. This is callable from a libfranka callback.
    (void)destination.fetch_or(bit, std::memory_order_release);
  }
}

inline void clearBackendFailureReasons(
    std::atomic<BackendFailureReasonMask>& destination) noexcept {
  destination.store(0, std::memory_order_release);
}

static_assert(static_cast<uint8_t>(BackendFailureReason::UnknownException) <=
                  sizeof(BackendFailureReasonMask) * 8,
              "Every backend failure reason must fit in the fixed failure mask");
static_assert(backendFailureReasonBit(BackendFailureReason::None) == 0);
static_assert(backendFailureReasonBit(static_cast<BackendFailureReason>(16)) == 0);
static_assert(backendFailureReasonBit(static_cast<BackendFailureReason>(255)) == 0);
static_assert(backendFailureReasonFromMask(BackendFailureReasonMask{1} << 31U) ==
              BackendFailureReason::None);
static_assert(std::atomic<BackendFailureReasonMask>::is_always_lock_free,
              "The RT-written backend failure mask must be lock-free");
static_assert(std::atomic<BackendRecoveryResult>::is_always_lock_free,
              "The backend recovery result must be lock-free");
static_assert(std::atomic<ControlMode>::is_always_lock_free,
              "The RT-written backend control modes must be lock-free");
static_assert(std::atomic<uint64_t>::is_always_lock_free,
              "The RT-written backend counters and timestamp must be lock-free");
static_assert(std::atomic_bool::is_always_lock_free,
              "The RT-written backend flags must be lock-free");

struct FrankaArmBackendDiagnostics {
  ControlMode requested_mode{ControlMode::None};
  ControlMode active_mode{ControlMode::None};
  BackendWorkerState worker_state{BackendWorkerState::Stopped};
  BackendFaultCategory fault_category{BackendFaultCategory::None};
  BackendFailureReason failure_reason{BackendFailureReason::None};
  BackendServiceOperation service_operation{BackendServiceOperation::Idle};
  bool stopped{true};
  bool recovering{false};
  bool has_state_sample{false};
  uint64_t accepted_state_samples{0};
  uint64_t last_accepted_state_steady_ns{0};
  uint64_t dropped_state_samples{0};
  uint64_t rejected_command_samples{0};
  bool state_queue_saturated{false};
  bool command_queue_saturated{false};
  uint64_t recovery_attempts{0};
  uint64_t recovery_successes{0};
  uint64_t recovery_failures{0};
  BackendRecoveryResult last_recovery_result{BackendRecoveryResult::NeverAttempted};
};

class FrankaArmBackend {
 public:
  FrankaArmBackend() = default;
  FrankaArmBackend(const FrankaArmBackend&) = delete;
  FrankaArmBackend& operator=(const FrankaArmBackend&) = delete;
  FrankaArmBackend(FrankaArmBackend&&) = delete;
  FrankaArmBackend& operator=(FrankaArmBackend&&) = delete;
  virtual ~FrankaArmBackend() = default;

  virtual bool startStateReading() = 0;
  virtual bool stop() = 0;
  virtual franka::RobotState readLatestState() = 0;
  virtual ModelBase* model() noexcept = 0;

  virtual bool canPublishCommand() const noexcept = 0;
  virtual bool publishCommand(const RobotCommand& command) noexcept = 0;
  virtual bool canRequestControlMode(ControlMode control_mode) const noexcept = 0;
  virtual bool requestControlMode(ControlMode control_mode) noexcept = 0;
  virtual ControlMode requestedControlMode() const noexcept = 0;
  virtual ControlMode activeControlMode() const noexcept = 0;

  virtual bool hasFault() const noexcept = 0;
  virtual bool recoverToReading() = 0;
  virtual FrankaArmBackendDiagnostics diagnostics() const noexcept = 0;

  virtual void setJointStiffness(
      const franka_msgs::srv::SetJointStiffness::Request::SharedPtr& request) = 0;
  virtual void setCartesianStiffness(
      const franka_msgs::srv::SetCartesianStiffness::Request::SharedPtr& request) = 0;
  virtual void setLoad(const franka_msgs::srv::SetLoad::Request::SharedPtr& request) = 0;
  virtual void setTCPFrame(const franka_msgs::srv::SetTCPFrame::Request::SharedPtr& request) = 0;
  virtual void setStiffnessFrame(
      const franka_msgs::srv::SetStiffnessFrame::Request::SharedPtr& request) = 0;
  virtual void setForceTorqueCollisionBehavior(
      const franka_msgs::srv::SetForceTorqueCollisionBehavior::Request::SharedPtr& request) = 0;
  virtual void setFullCollisionBehavior(
      const franka_msgs::srv::SetFullCollisionBehavior::Request::SharedPtr& request) = 0;
};

}  // namespace franka_hardware
