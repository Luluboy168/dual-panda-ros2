// Copyright (c) 2021 Franka Emika GmbH
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

#include <pthread.h>
#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <map>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <string>
#include <type_traits>
#include <vector>

#include "franka_hardware/common/control_mode.h"
#include "franka_hardware/common/franka_executor.hpp"
#include "franka_hardware/common/helper_functions.hpp"
#include "franka_hardware/real/command_mode_switch_planner.hpp"
#include "franka_hardware/real/franka_arm_backend.hpp"
#include "franka_hardware/real/franka_error_recovery_service_server.hpp"
#include "franka_hardware/real/franka_param_service_server.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_hardware {

using BackendFactory =
    std::function<std::shared_ptr<FrankaArmBackend>(const std::string& arm_name,
                                                    const std::string& robot_address,
                                                    const rclcpp::Logger& logger)>;

enum class InitializationStage {
  ErrorRecoveryServiceConstruction,
  ParameterServiceConstruction,
  DiagnosticsConstruction,
  ExecutorConstruction,
  ServiceRegistration,
  DiagnosticsRegistration,
  BaseInitialization,
};

struct InitializationCheckpoint {
  static constexpr size_t kNoArmSlot = 0;

  InitializationStage stage;
  size_t arm_slot{kNoArmSlot};
  size_t occurrence{0};
};

using InitializationCheckpointHook = std::function<void(const InitializationCheckpoint&)>;

enum class ModeSwitchCheckpoint : uint8_t { PreparedPayloadWritten };
using ModeSwitchCheckpointHook = std::function<void(ModeSwitchCheckpoint)>;

enum class GlobalFaultCause : uint8_t {
  None,
  BackendFault,
  ReadFailure,
  InvalidState,
  InvalidCommand,
  CommandCapacity,
  CommandPublish,
  ModeRequest,
};

struct GlobalFaultDiagnostic {
  uint8_t origin_arm_slot{0};
  GlobalFaultCause cause{GlobalFaultCause::None};
  uint8_t unsafe_safe_publish_mask{0};
  uint8_t unsafe_none_request_mask{0};

  [[nodiscard]] bool latched() const noexcept { return cause != GlobalFaultCause::None; }
};

struct ModeSwitchInterfaceSignature {
  uint32_t start_mask{0};
  uint32_t stop_mask{0};

  [[nodiscard]] bool operator==(const ModeSwitchInterfaceSignature& other) const noexcept {
    return start_mask == other.start_mask && stop_mask == other.stop_mask;
  }
};

struct PreparedArmModeRequest {
  ControlMode requested_mode{ControlMode::None};
  CommandInitialization command_initialization{CommandInitialization::None};
  bool has_request{false};
};

struct PreparedModeTransaction {
  uint64_t generation{0};
  ModeSwitchInterfaceSignature signature{};
  std::array<PreparedArmModeRequest, 2> arms{};
};

inline constexpr uint64_t kPreparedTransactionGenerationMask = (uint64_t{1} << 61U) - uint64_t{1};

[[nodiscard]] inline constexpr uint64_t preparedTransactionGenerationCandidate(
    uint64_t sequence) noexcept {
  return sequence & kPreparedTransactionGenerationMask;
}

static_assert(sizeof(GlobalFaultDiagnostic) == 4,
              "The global fault diagnostic must remain a fixed-size value");
static_assert(std::atomic<ControlMode>::is_always_lock_free,
              "The RT-visible control mode must be lock-free");

struct ArmContainer {
  size_t arm_slot_{InitializationCheckpoint::kNoArmSlot};
  std::string robot_ip_;
  std::string robot_name_;
  std::shared_ptr<FrankaArmBackend> backend_;
  std::shared_ptr<FrankaErrorRecoveryServiceServer> error_recovery_service_node_;
  std::shared_ptr<FrankaParamServiceServer> param_service_node_;

  std::array<double, 7> hw_commands_joint_effort_{0, 0, 0, 0, 0, 0, 0};
  std::array<double, 7> hw_commands_joint_position_{0, 0, 0, 0, 0, 0, 0};
  std::array<double, 7> hw_commands_joint_velocity_{0, 0, 0, 0, 0, 0, 0};
  std::array<double, 16> hw_commands_cartesian_position_;
  std::array<double, 6> hw_commands_cartesian_velocity_;

  // States
  std::atomic<ControlMode> control_mode_{ControlMode::None};
  std::array<double, 7> hw_positions_{0, 0, 0, 0, 0, 0, 0};
  std::array<double, 7> hw_velocities_{0, 0, 0, 0, 0, 0, 0};
  std::array<double, 7> hw_efforts_{0, 0, 0, 0, 0, 0, 0};
  std::array<double, 16> hw_cartesian_positions_;
  std::array<double, 16> hw_cartesian_velocities_;

  franka::RobotState hw_franka_robot_state_;
};

class FrankaHardwareDiagnosticsNode;

class FrankaMultiHardwareInterface : public hardware_interface::SystemInterface {
 public:
  FrankaMultiHardwareInterface();
  explicit FrankaMultiHardwareInterface(BackendFactory factory);
  // Direct-construction seam for deterministic offline lifecycle tests. The plugin/default path
  // never installs a checkpoint hook and always uses the real backend factory.
  FrankaMultiHardwareInterface(BackendFactory factory,
                               InitializationCheckpointHook initialization_checkpoint_hook);
  FrankaMultiHardwareInterface(BackendFactory factory,
                               InitializationCheckpointHook initialization_checkpoint_hook,
                               ModeSwitchCheckpointHook mode_switch_checkpoint_hook);

  hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  // Every one of these edges (including the ones the base class would otherwise leave as a
  // no-op SUCCESS) is wired to the same fail-safe stop as on_deactivate. See
  // driveAllArmsToFailSafeStop() for why: TRANSITION_ACTIVE_SHUTDOWN and ErrorProcessing can
  // reach on_shutdown/on_error directly from ACTIVE without on_deactivate ever running, and
  // on_cleanup/on_configure must not assume a caller respected the documented state graph.
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  [[nodiscard]] GlobalFaultDiagnostic globalFaultDiagnostic() const noexcept;
  static const size_t kNumberOfJoints = 7;
  // Hardware metadata is validated against this bound before any derived resource or interface
  // names are constructed. The longest exported full interface name is therefore also fixed.
  static constexpr size_t kMaximumArmIdentifierLength = 64;
  static constexpr size_t kMaximumJointModeInterfaceNameLength =
      kMaximumArmIdentifierLength + sizeof("_joint7/velocity") - 1;
  static constexpr size_t kMaximumDerivedInterfaceNameLength =
      kMaximumArmIdentifierLength + sizeof("_ee_cartesian_velocity/omega_x") - 1;
  static constexpr size_t kMaximumModeSwitchInterfaceCount = 2 * 2 * kNumberOfJoints;
  static_assert(kMaximumJointModeInterfaceNameLength <= kMaximumDerivedInterfaceNameLength);
  size_t robot_count_{0};

 private:
  BackendFactory backend_factory_;
  InitializationCheckpointHook initialization_checkpoint_hook_;
  ModeSwitchCheckpointHook mode_switch_checkpoint_hook_;
  std::array<std::string, 16> cartesian_matrix_names{"00", "01", "02", "03", "04", "05",
                                                     "06", "07", "08", "09", "10", "11",
                                                     "12", "13", "14", "15"};
  std::array<std::string, 6> cartesian_velocity_command_names{"tx",      "ty",      "tz",
                                                              "omega_x", "omega_y", "omega_z"};

  std::map<std::string, ArmContainer> arms_;
  std::array<ArmContainer*, 2> arm_slots_{};
  std::map<std::string, franka::RobotState*> state_pointers_;
  std::map<std::string, ModelBase*> model_pointers_;
  PreparedModeTransaction prepared_transaction_{};

  // Commands

  static rclcpp::Logger getLogger();
  static RobotCommand safeCommandForArm(const ArmContainer& arm,
                                        CommandInitialization initialization) noexcept;
  static void assignExportedCommands(ArmContainer& arm, const RobotCommand& command) noexcept;
  static bool publishCommands(ArmContainer& arm) noexcept;
  static bool publishCommand(ArmContainer& arm, const RobotCommand& command) noexcept;
  static bool commandsAreFinite(const ArmContainer& arm) noexcept;
  [[nodiscard]] bool makeModeSwitchSignature(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces,
      ModeSwitchInterfaceSignature& signature) const noexcept;
  [[nodiscard]] uint64_t nextPreparedTransactionGeneration() noexcept;
  [[nodiscard]] bool beginPreparedTransactionPublication(uint64_t generation) noexcept;
  [[nodiscard]] bool finishPreparedTransactionPublication(uint64_t generation) noexcept;
  [[nodiscard]] bool consumePreparedTransaction(PreparedModeTransaction& transaction) noexcept;
  void finishPreparedTransactionConsumption(uint64_t generation) noexcept;
  void invalidatePreparedTransaction() noexcept;
  [[nodiscard]] bool discardReadyPreparedTransaction() noexcept;
  [[nodiscard]] bool bindControlCycleOwner() noexcept;
  [[nodiscard]] bool isControlCycleOwner() const noexcept;
  // Applies a consumed, already-validated transaction's effects (safe-command publish, mode
  // request, control-mode store) against the backends. Bounded, allocation-free, lock-free --
  // see the .cpp for the full contract. Must only ever be called on the control-cycle owner
  // thread: either directly, when perform_command_mode_switch() itself runs on that thread, or
  // from serviceOwnerHandoffIfPending(), which runs it on that thread on behalf of a waiting
  // off-owner caller.
  [[nodiscard]] hardware_interface::return_type applyPreparedTransactionEffects(
      const PreparedModeTransaction& transaction) noexcept;
  // Hands an already-validated transaction to the control-cycle owner thread and blocks (bounded,
  // off the RT path) until that thread has applied it via serviceOwnerHandoffIfPending() and
  // published a result. Called only when the calling thread is not the owner.
  [[nodiscard]] hardware_interface::return_type requestOwnerExecutedEffects(
      const PreparedModeTransaction& transaction) noexcept;
  // Called once per write() on the control-cycle owner thread. A cheap atomic load in the common
  // (no pending handoff) case; applies the pending transaction's effects and publishes the result
  // only when an off-owner perform_command_mode_switch() call is waiting. Never blocks, allocates
  // or locks -- see the .cpp for the full contract.
  void serviceOwnerHandoffIfPending() noexcept;
  void resetCurrentModeState() noexcept;
  void enterGlobalFault(uint8_t origin_arm_slot, GlobalFaultCause cause) noexcept;
  [[nodiscard]] bool globalFaultLatched() const noexcept;
  [[nodiscard]] bool tryClearRecoveredBackendFault() noexcept;
  bool stopAllBackendsForRollback() noexcept;
  CallbackReturn rollbackActivation(const char* reason) noexcept;
  // Single fail-safe convergence point for every lifecycle edge that must leave the
  // hardware in a known-safe state: clears the logical control-mode/prepared-transaction
  // state, then stops every configured arm's backend and confirms it reports stopped.
  // Idempotent and safe to call from any state (including repeatedly, and including with
  // robot_count_ == 0 or already-stopped arms) -- see the .cpp for the full contract.
  // Never blocks on the RT path: it is never called from read()/write()/update().
  [[nodiscard]] bool driveAllArmsToFailSafeStop() noexcept;

  const std::string k_robot_state_interface_name{"robot_state"};
  const std::string k_robot_model_interface_name{"robot_model"};

  static_assert(std::atomic<uint64_t>::is_always_lock_free,
                "The packed global fault latch must be lock-free");
  std::atomic<uint64_t> global_fault_latch_{0};

  enum class PreparedTransactionStage : uint8_t {
    Empty,
    Publishing,
    Ready,
    Consuming,
    PublishingInvalidated,
    ConsumingInvalidated,
  };

  // Bounded cross-thread handoff for an already-validated transaction whose caller is not the
  // control-cycle owner thread (e.g. controller_manager's switch_controller() service handler
  // invoking perform_command_mode_switch() with activate_asap=false, which Jazzy executes on its
  // own service thread while the control cycle keeps running -- see perform_command_mode_switch()
  // in the .cpp). Idle -> Requested is published by the off-owner caller; the owner thread
  // observes Requested from inside write(), applies the transaction itself, and publishes
  // CompletedOk/CompletedError. Only one handoff may be in flight at a time, enforced by the same
  // CAS-gated single-slot pattern as prepared_transaction_state_ above; the requester's own
  // transaction generation tags the slot so a stale completion can never be mistaken for a fresh
  // one.
  enum class OwnerHandoffStage : uint8_t {
    Idle,
    Requested,
    CompletedOk,
    CompletedError,
  };

  static_assert(std::atomic<uintptr_t>::is_always_lock_free,
                "The control-cycle owner token must be lock-free");
  static_assert(std::atomic<uint64_t>::is_always_lock_free,
                "The prepared transaction state must be lock-free");
  static_assert(std::is_integral_v<pthread_t> && sizeof(pthread_t) <= sizeof(uintptr_t),
                "The supported Linux pthread token must fit in uintptr_t");
  std::atomic<uintptr_t> control_cycle_owner_{0};
  std::atomic<uint64_t> prepared_transaction_state_{0};
  // (generation << 3U) | OwnerHandoffStage, mirroring prepared_transaction_state_'s packing.
  // owner_handoff_transaction_ is a fixed-size POD guarded by this atomic exactly like
  // prepared_transaction_ is guarded by prepared_transaction_state_: written by the requester
  // before the release-CAS to Requested, read by the owner only after an acquire-load observes
  // Requested (and matching generation), written by the owner before the release-CAS to
  // Completed{Ok,Error}, read by the requester only after an acquire-load observes it.
  std::atomic<uint64_t> owner_handoff_state_{0};
  PreparedModeTransaction owner_handoff_transaction_{};
  std::atomic<uint64_t> next_prepared_generation_{1};

  std::shared_ptr<FrankaHardwareDiagnosticsNode> diagnostics_node_;

  // Declared last so its spinning thread is cancelled and joined before service nodes and
  // diagnostics node, services and backends are destroyed.
  std::shared_ptr<FrankaExecutor> executor_;
};
}  // namespace franka_hardware
