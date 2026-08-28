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

#include <franka/exception.h>

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <exception>
#include <franka_hardware/real/franka_hardware_diagnostics_node.hpp>
#include <franka_hardware/real/franka_multi_hardware_interface.hpp>
#include <franka_hardware/real/real_franka_arm_backend.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <stdexcept>
#include <system_error>
#include <thread>
#include <utility>

namespace franka_hardware {

using StateInterface = hardware_interface::StateInterface;
using CommandInterface = hardware_interface::CommandInterface;

namespace {

struct ArmConfiguration {
  std::string name;
  std::string address;
};

struct ValidatedConfiguration {
  size_t robot_count{0};
  std::vector<ArmConfiguration> arms;
};

bool isAsciiIdentifier(const std::string& value) {
  if (value.empty() || !((value.front() >= 'A' && value.front() <= 'Z') ||
                         (value.front() >= 'a' && value.front() <= 'z'))) {
    return false;
  }
  return std::all_of(value.begin() + 1, value.end(), [](char character) {
    return (character >= 'A' && character <= 'Z') || (character >= 'a' && character <= 'z') ||
           (character >= '0' && character <= '9') || character == '_';
  });
}

bool validateInterfaces(const std::vector<hardware_interface::InterfaceInfo>& interfaces,
                        const std::string& joint_name,
                        const std::string& interface_kind,
                        std::string& error) {
  static const std::array<std::string, 3> kExpectedInterfaces{hardware_interface::HW_IF_EFFORT,
                                                              hardware_interface::HW_IF_POSITION,
                                                              hardware_interface::HW_IF_VELOCITY};
  if (interfaces.size() != kExpectedInterfaces.size()) {
    error =
        "Joint '" + joint_name + "' must provide exactly three " + interface_kind + " interfaces";
    return false;
  }

  std::set<std::string> seen;
  for (const auto& interface : interfaces) {
    if (interface.data_type != "double") {
      error = "Joint '" + joint_name + "' " + interface_kind + " interface '" + interface.name +
              "' must have data_type 'double'";
      return false;
    }
    if (std::find(kExpectedInterfaces.begin(), kExpectedInterfaces.end(), interface.name) ==
        kExpectedInterfaces.end()) {
      error = "Joint '" + joint_name + "' has unknown " + interface_kind + " interface '" +
              interface.name + "'";
      return false;
    }
    if (!seen.insert(interface.name).second) {
      error = "Joint '" + joint_name + "' has duplicate " + interface_kind + " interface '" +
              interface.name + "'";
      return false;
    }
  }
  return true;
}

bool validateHardwareInfo(const hardware_interface::HardwareInfo& info,
                          ValidatedConfiguration& configuration,
                          std::string& error) {
  const auto count_parameter = info.hardware_parameters.find("robot_count");
  if (count_parameter == info.hardware_parameters.end()) {
    error = "Required hardware parameter 'robot_count' is missing";
    return false;
  }

  size_t robot_count = 0;
  const auto* count_begin = count_parameter->second.data();
  const auto* count_end = count_begin + count_parameter->second.size();
  const auto parse_result = std::from_chars(count_begin, count_end, robot_count);
  if (count_begin == count_end || parse_result.ec != std::errc{} || parse_result.ptr != count_end ||
      (robot_count != 1 && robot_count != 2)) {
    error = "Hardware parameter 'robot_count' must be exactly '1' or '2'";
    return false;
  }

  configuration.robot_count = robot_count;
  configuration.arms.clear();
  configuration.arms.reserve(robot_count);
  std::set<std::string> arm_names;
  for (size_t index = 1; index <= robot_count; ++index) {
    const auto suffix = "_" + std::to_string(index);
    const auto name_parameter = info.hardware_parameters.find("ns" + suffix);
    const auto address_parameter = info.hardware_parameters.find("robot_ip" + suffix);
    if (name_parameter == info.hardware_parameters.end() || name_parameter->second.empty()) {
      error = "Required hardware parameter 'ns" + suffix + "' is missing or empty";
      return false;
    }
    if (name_parameter->second.size() > FrankaMultiHardwareInterface::kMaximumArmIdentifierLength) {
      error = "Hardware parameter 'ns" + suffix + "' must contain at most " +
              std::to_string(FrankaMultiHardwareInterface::kMaximumArmIdentifierLength) +
              " characters";
      return false;
    }
    if (!isAsciiIdentifier(name_parameter->second)) {
      error = "Hardware parameter 'ns" + suffix + "' must be an ASCII identifier";
      return false;
    }
    if (!arm_names.insert(name_parameter->second).second) {
      error = "Arm identifiers must be unique";
      return false;
    }
    if (address_parameter == info.hardware_parameters.end() || address_parameter->second.empty()) {
      error = "Required hardware parameter 'robot_ip" + suffix + "' is missing or empty";
      return false;
    }
    configuration.arms.push_back({name_parameter->second, address_parameter->second});
  }

  if (info.joints.size() != FrankaMultiHardwareInterface::kNumberOfJoints * robot_count) {
    error = "Joint count does not match robot_count";
    return false;
  }

  std::set<std::string> expected_joint_names;
  for (const auto& arm : configuration.arms) {
    for (size_t joint_index = 1; joint_index <= FrankaMultiHardwareInterface::kNumberOfJoints;
         ++joint_index) {
      expected_joint_names.insert(arm.name + "_joint" + std::to_string(joint_index));
    }
  }

  std::set<std::string> seen_joint_names;
  for (const auto& joint : info.joints) {
    if (joint.type != "joint") {
      error = "Component '" + joint.name + "' must have type 'joint'";
      return false;
    }
    if (expected_joint_names.count(joint.name) == 0) {
      error = "Unexpected joint '" + joint.name + "'";
      return false;
    }
    if (!seen_joint_names.insert(joint.name).second) {
      error = "Joint '" + joint.name + "' must appear exactly once";
      return false;
    }
    if (!validateInterfaces(joint.command_interfaces, joint.name, "command", error) ||
        !validateInterfaces(joint.state_interfaces, joint.name, "state", error)) {
      return false;
    }
  }
  if (seen_joint_names != expected_joint_names) {
    error = "Every configured arm must provide joints 1 through 7 exactly once";
    return false;
  }
  return true;
}

template <size_t Size>
bool allFinite(const std::array<double, Size>& values) noexcept {
  return std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); });
}

bool isFiniteState(const franka::RobotState& state) noexcept {
  return allFinite(state.O_T_EE) && allFinite(state.O_T_EE_d) && allFinite(state.F_T_EE) &&
         allFinite(state.F_T_NE) && allFinite(state.NE_T_EE) && allFinite(state.EE_T_K) &&
         std::isfinite(state.m_ee) && allFinite(state.I_ee) && allFinite(state.F_x_Cee) &&
         std::isfinite(state.m_load) && allFinite(state.I_load) && allFinite(state.F_x_Cload) &&
         std::isfinite(state.m_total) && allFinite(state.I_total) && allFinite(state.F_x_Ctotal) &&
         allFinite(state.elbow) && allFinite(state.elbow_d) && allFinite(state.elbow_c) &&
         allFinite(state.delbow_c) && allFinite(state.ddelbow_c) && allFinite(state.tau_J) &&
         allFinite(state.tau_J_d) && allFinite(state.dtau_J) && allFinite(state.q) &&
         allFinite(state.q_d) && allFinite(state.dq) && allFinite(state.dq_d) &&
         allFinite(state.ddq_d) && allFinite(state.joint_contact) &&
         allFinite(state.cartesian_contact) && allFinite(state.joint_collision) &&
         allFinite(state.cartesian_collision) && allFinite(state.tau_ext_hat_filtered) &&
         allFinite(state.O_F_ext_hat_K) && allFinite(state.K_F_ext_hat_K) &&
         allFinite(state.O_dP_EE_d) && allFinite(state.O_ddP_O) && allFinite(state.O_T_EE_c) &&
         allFinite(state.O_dP_EE_c) && allFinite(state.O_ddP_EE_c) && allFinite(state.theta) &&
         allFinite(state.dtheta) && std::isfinite(state.control_command_success_rate);
}

void assignState(ArmContainer& arm, const franka::RobotState& state) {
  arm.hw_franka_robot_state_ = state;
  arm.hw_positions_ = state.q;
  arm.hw_velocities_ = state.dq;
  arm.hw_efforts_ = state.tau_J;
  arm.hw_cartesian_positions_ = state.O_T_EE;
  arm.hw_cartesian_velocities_ = state.O_T_EE_d;
}

constexpr uint64_t packGlobalFault(uint8_t origin_arm_slot,
                                   GlobalFaultCause cause,
                                   uint8_t unsafe_safe_publish_mask,
                                   uint8_t unsafe_none_request_mask) noexcept {
  return static_cast<uint64_t>(origin_arm_slot) | (static_cast<uint64_t>(cause) << 8U) |
         (static_cast<uint64_t>(unsafe_safe_publish_mask) << 16U) |
         (static_cast<uint64_t>(unsafe_none_request_mask) << 24U);
}

constexpr uint8_t armMask(size_t arm_index) noexcept {
  return static_cast<uint8_t>(1U << arm_index);
}

}  // namespace

FrankaMultiHardwareInterface::FrankaMultiHardwareInterface()
    : FrankaMultiHardwareInterface(
          [](const std::string& arm_name,
             const std::string& robot_address,
             const rclcpp::Logger& logger) -> std::shared_ptr<FrankaArmBackend> {
            return std::make_shared<RealFrankaArmBackend>(arm_name, robot_address, logger);
          },
          {},
          {}) {}

FrankaMultiHardwareInterface::FrankaMultiHardwareInterface(BackendFactory factory)
    : FrankaMultiHardwareInterface(std::move(factory), {}, {}) {}

FrankaMultiHardwareInterface::FrankaMultiHardwareInterface(
    BackendFactory factory,
    InitializationCheckpointHook initialization_checkpoint_hook)
    : FrankaMultiHardwareInterface(std::move(factory),
                                   std::move(initialization_checkpoint_hook),
                                   {}) {}

FrankaMultiHardwareInterface::FrankaMultiHardwareInterface(
    BackendFactory factory,
    InitializationCheckpointHook initialization_checkpoint_hook,
    ModeSwitchCheckpointHook mode_switch_checkpoint_hook)
    : backend_factory_(std::move(factory)),
      initialization_checkpoint_hook_(std::move(initialization_checkpoint_hook)),
      mode_switch_checkpoint_hook_(std::move(mode_switch_checkpoint_hook)) {
  if (!backend_factory_) {
    throw std::invalid_argument("Franka backend factory must be callable");
  }
}

CallbackReturn FrankaMultiHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
  if (robot_count_ != 0 || !arms_.empty() || diagnostics_node_ || executor_) {
    RCLCPP_ERROR(getLogger(),
                 "A successfully initialized hardware instance cannot be reinitialized");
    return CallbackReturn::ERROR;
  }

  ValidatedConfiguration configuration;
  std::string validation_error;
  if (!validateHardwareInfo(info, configuration, validation_error)) {
    RCLCPP_ERROR(getLogger(), "Invalid hardware metadata: %s", validation_error.c_str());
    return CallbackReturn::ERROR;
  }

  std::map<std::string, ArmContainer> staged_arms;
  std::map<std::string, franka::RobotState*> staged_state_pointers;
  std::map<std::string, ModelBase*> staged_model_pointers;
  std::array<ArmContainer*, 2> staged_arm_slots{};
  std::string construction_stage{"backend construction"};
  std::shared_ptr<FrankaHardwareDiagnosticsNode> staged_diagnostics_node;
  std::shared_ptr<FrankaExecutor> staged_executor;

  try {
    for (size_t arm_index = 0; arm_index < configuration.arms.size(); ++arm_index) {
      const auto& arm_configuration = configuration.arms.at(arm_index);
      const size_t arm_slot = arm_index + 1;
      construction_stage = "backend construction for arm '" + arm_configuration.name + "'";
      auto insertion = staged_arms.try_emplace(arm_configuration.name);
      auto& arm = insertion.first->second;
      arm.arm_slot_ = arm_slot;
      arm.robot_name_ = arm_configuration.name;
      arm.robot_ip_ = arm_configuration.address;
      arm.backend_ = backend_factory_(arm.robot_name_, arm.robot_ip_, getLogger());
      if (!arm.backend_) {
        throw std::runtime_error("Franka backend factory returned null");
      }

      construction_stage = "initial state read for arm '" + arm.robot_name_ + "'";
      const auto initial_state = arm.backend_->readLatestState();
      if (!isFiniteState(initial_state)) {
        throw std::runtime_error("Franka backend returned a non-finite initial state");
      }
      assignState(arm, initial_state);

      construction_stage = "model acquisition for arm '" + arm.robot_name_ + "'";
      auto* model = arm.backend_->model();
      if (model == nullptr) {
        throw std::runtime_error("Franka backend returned a null model");
      }
      staged_state_pointers.emplace(arm.robot_name_, &arm.hw_franka_robot_state_);
      staged_model_pointers.emplace(arm.robot_name_, model);
      assignExportedCommands(arm, safeCommandForArm(arm, CommandInitialization::None));

      construction_stage = "service construction for arm '" + arm.robot_name_ + "'";
      arm.error_recovery_service_node_ = std::make_shared<FrankaErrorRecoveryServiceServer>(
          rclcpp::NodeOptions(), arm.backend_, arm.robot_name_ + "_");
      if (initialization_checkpoint_hook_) {
        initialization_checkpoint_hook_(
            {InitializationStage::ErrorRecoveryServiceConstruction, arm_slot, 0});
      }
      arm.param_service_node_ = std::make_shared<FrankaParamServiceServer>(
          rclcpp::NodeOptions(), arm.backend_, arm.robot_name_ + "_");
      if (initialization_checkpoint_hook_) {
        initialization_checkpoint_hook_(
            {InitializationStage::ParameterServiceConstruction, arm_slot, 0});
      }
      staged_arm_slots.at(arm_index) = &arm;
    }

    construction_stage = "diagnostics construction";
    std::vector<FrankaArmDiagnosticSource> diagnostic_sources;
    diagnostic_sources.reserve(configuration.robot_count);
    for (size_t arm_index = 0; arm_index < configuration.robot_count; ++arm_index) {
      const auto& arm = *staged_arm_slots.at(arm_index);
      diagnostic_sources.push_back({arm.robot_name_, arm.backend_});
    }
    staged_diagnostics_node = std::make_shared<FrankaHardwareDiagnosticsNode>(
        rclcpp::NodeOptions(), std::move(diagnostic_sources),
        [this]() { return globalFaultDiagnostic(); },
        [this]() {
          const auto& lifecycle_state = get_lifecycle_state();
          return HardwareLifecycleSnapshot{lifecycle_state.id(), lifecycle_state.label()};
        });
    if (initialization_checkpoint_hook_) {
      initialization_checkpoint_hook_(
          {InitializationStage::DiagnosticsConstruction, InitializationCheckpoint::kNoArmSlot, 0});
    }

    construction_stage = "executor construction";
    staged_executor = std::make_shared<FrankaExecutor>();
    if (initialization_checkpoint_hook_) {
      initialization_checkpoint_hook_(
          {InitializationStage::ExecutorConstruction, InitializationCheckpoint::kNoArmSlot, 0});
    }
    construction_stage = "service registration";
    for (auto& arm_entry : staged_arms) {
      auto& arm = arm_entry.second;
      staged_executor->add_node(arm.error_recovery_service_node_);
      if (initialization_checkpoint_hook_) {
        initialization_checkpoint_hook_(
            {InitializationStage::ServiceRegistration, arm.arm_slot_, 0});
      }
      staged_executor->add_node(arm.param_service_node_);
      if (initialization_checkpoint_hook_) {
        initialization_checkpoint_hook_(
            {InitializationStage::ServiceRegistration, arm.arm_slot_, 1});
      }
    }
    staged_executor->add_node(staged_diagnostics_node);
    if (initialization_checkpoint_hook_) {
      initialization_checkpoint_hook_(
          {InitializationStage::DiagnosticsRegistration, InitializationCheckpoint::kNoArmSlot, 0});
    }

    construction_stage = "ros2_control base initialization";
    if (initialization_checkpoint_hook_) {
      initialization_checkpoint_hook_(
          {InitializationStage::BaseInitialization, InitializationCheckpoint::kNoArmSlot, 0});
    }
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
      RCLCPP_ERROR(getLogger(), "ros2_control rejected the validated hardware metadata");
      return CallbackReturn::ERROR;
    }

    arms_.swap(staged_arms);
    state_pointers_.swap(staged_state_pointers);
    model_pointers_.swap(staged_model_pointers);
    diagnostics_node_.swap(staged_diagnostics_node);
    executor_.swap(staged_executor);
    // std::map::swap preserves pointers to its elements, so the staged slot pointers now refer to
    // the committed arm containers.
    arm_slots_ = staged_arm_slots;
    robot_count_ = configuration.robot_count;
    resetCurrentModeState();
    control_cycle_owner_.store(0, std::memory_order_release);
    owner_handoff_state_.store(0, std::memory_order_release);
    global_fault_latch_.store(0, std::memory_order_release);
  } catch (const franka::Exception& exception) {
    RCLCPP_ERROR(getLogger(), "Initialization failed during %s: %s", construction_stage.c_str(),
                 exception.what());
    return CallbackReturn::ERROR;
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(getLogger(), "Initialization failed during %s: %s", construction_stage.c_str(),
                 exception.what());
    return CallbackReturn::ERROR;
  } catch (...) {
    RCLCPP_ERROR(getLogger(), "Initialization failed during %s with an unknown exception",
                 construction_stage.c_str());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(getLogger(), "All %zu robots have been initialized", robot_count_);
  return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> FrankaMultiHardwareInterface::export_state_interfaces() {
  std::vector<StateInterface> state_interfaces;
  for (auto i = 0U; i < info_.joints.size(); i++) {
    // std::cout << get_ns(info_.joints[i].name) << std::endl;
    state_interfaces.emplace_back(
        StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION,
                       &arms_.at(get_ns(info_.joints[i].name))
                            .hw_positions_.at(get_joint_no(info_.joints[i].name))));
    state_interfaces.emplace_back(
        StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
                       &arms_.at(get_ns(info_.joints[i].name))
                            .hw_velocities_.at(get_joint_no(info_.joints[i].name))));
    state_interfaces.emplace_back(
        StateInterface(info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
                       &arms_.at(get_ns(info_.joints[i].name))
                            .hw_efforts_.at(get_joint_no(info_.joints[i].name))));
  }

  for (auto& arm_container_pair : arms_) {
    auto& arm = arm_container_pair.second;

    std::string cartesian_position_prefix = arm.robot_name_ + "_ee_cartesian_position";
    std::string cartesian_velocity_prefix = arm.robot_name_ + "_ee_cartesian_velocity";

    for (auto i = 0; i < 16; i++) {
      state_interfaces.emplace_back(StateInterface(
          cartesian_position_prefix, cartesian_matrix_names[i], &arm.hw_cartesian_positions_[i]));
      state_interfaces.emplace_back(StateInterface(
          cartesian_velocity_prefix, cartesian_matrix_names[i], &arm.hw_cartesian_velocities_[i]));
    }

    state_interfaces.emplace_back(StateInterface(
        arm.robot_name_, k_robot_state_interface_name,
        reinterpret_cast<double*>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
            &state_pointers_.at(arm_container_pair.first))));
    state_interfaces.emplace_back(StateInterface(
        arm.robot_name_, k_robot_model_interface_name,
        reinterpret_cast<double*>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
            &model_pointers_.at(arm_container_pair.first))));
  }

  return state_interfaces;
}

std::vector<CommandInterface> FrankaMultiHardwareInterface::export_command_interfaces() {
  std::vector<CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());
  // RCLCPP_INFO(getLogger(), "%ld", info_.joints.size());

  for (auto i = 0U; i < info_.joints.size(); i++) {
    // RCLCPP_INFO(getLogger(), "%s", info_.joints[i].name.c_str());

    command_interfaces.emplace_back(CommandInterface(  // JOINT EFFORT
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
        &arms_.at(get_ns(info_.joints[i].name))
             .hw_commands_joint_effort_.at(get_joint_no(info_.joints[i].name))));
    command_interfaces.emplace_back(CommandInterface(  // JOINT POSITION
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &arms_.at(get_ns(info_.joints[i].name))
             .hw_commands_joint_position_.at(get_joint_no(info_.joints[i].name))));
    command_interfaces.emplace_back(CommandInterface(  // JOINT VELOCITY
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &arms_.at(get_ns(info_.joints[i].name))
             .hw_commands_joint_velocity_.at(get_joint_no(info_.joints[i].name))));
  }

  for (auto& arm_container_pair : arms_) {
    auto& arm = arm_container_pair.second;
    std::string cartesian_position_prefix = arm.robot_name_ + "_ee_cartesian_position";
    std::string cartesian_velocity_prefix = arm.robot_name_ + "_ee_cartesian_velocity";

    for (auto i = 0; i < 16; i++) {
      command_interfaces.emplace_back(CommandInterface(cartesian_position_prefix,
                                                       cartesian_matrix_names[i],
                                                       &arm.hw_commands_cartesian_position_[i]));
    }
    for (auto i = 0; i < 6; i++) {
      command_interfaces.emplace_back(CommandInterface(cartesian_velocity_prefix,
                                                       cartesian_velocity_command_names[i],
                                                       &arm.hw_commands_cartesian_velocity_[i]));
    }
  }

  return command_interfaces;
}

CallbackReturn FrankaMultiHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  resetCurrentModeState();
  if (arms_.empty()) {
    return CallbackReturn::ERROR;
  }
  std::array<ArmContainer*, 2> staged_arms{};
  std::array<franka::RobotState, 2> staged_states{};
  size_t staged_state_count = 0;

  for (size_t arm_index = 0; arm_index < robot_count_; ++arm_index) {
    auto& arm = *arm_slots_.at(arm_index);
    assignExportedCommands(arm, safeCommandForArm(arm, CommandInitialization::None));
    const auto diagnostics = arm.backend_->diagnostics();
    if (arm.backend_->hasFault() || !diagnostics.stopped || !arm.backend_->canPublishCommand()) {
      RCLCPP_ERROR(getLogger(), "Safe-command preflight failed for arm '%s'",
                   arm.robot_name_.c_str());
      return rollbackActivation("safe-command preflight failed");
    }
  }

  for (size_t arm_index = 0; arm_index < robot_count_; ++arm_index) {
    auto& arm = *arm_slots_.at(arm_index);
    if (!publishCommands(arm)) {
      RCLCPP_ERROR(getLogger(), "Safe-command publication failed for arm '%s'",
                   arm.robot_name_.c_str());
      return rollbackActivation("safe-command publication failed");
    }
  }

  for (size_t arm_index = 0; arm_index < robot_count_; ++arm_index) {
    auto& arm = *arm_slots_.at(arm_index);
    try {
      if (!arm.backend_->startStateReading()) {
        RCLCPP_ERROR(getLogger(), "State reader start failed for arm '%s'",
                     arm.robot_name_.c_str());
        return rollbackActivation("state reader start failed");
      }
    } catch (const std::exception& exception) {
      RCLCPP_ERROR(getLogger(), "State reader start threw for arm '%s': %s",
                   arm.robot_name_.c_str(), exception.what());
      return rollbackActivation("state reader start exception");
    } catch (...) {
      RCLCPP_ERROR(getLogger(), "State reader start threw for arm '%s'", arm.robot_name_.c_str());
      return rollbackActivation("state reader start exception");
    }
  }

  for (size_t arm_index = 0; arm_index < robot_count_; ++arm_index) {
    auto& arm = *arm_slots_.at(arm_index);
    try {
      auto state = arm.backend_->readLatestState();
      if (arm.backend_->hasFault()) {
        RCLCPP_ERROR(getLogger(), "Backend faulted during activation read for arm '%s'",
                     arm.robot_name_.c_str());
        return rollbackActivation("backend faulted during activation read");
      }
      if (!isFiniteState(state)) {
        RCLCPP_ERROR(getLogger(), "Backend returned a non-finite activation state for arm '%s'",
                     arm.robot_name_.c_str());
        return rollbackActivation("backend returned a non-finite activation state");
      }
      staged_arms.at(staged_state_count) = &arm;
      staged_states.at(staged_state_count) = std::move(state);
      ++staged_state_count;
    } catch (const std::exception& exception) {
      RCLCPP_ERROR(getLogger(), "Activation read threw for arm '%s': %s", arm.robot_name_.c_str(),
                   exception.what());
      return rollbackActivation("activation state read exception");
    } catch (...) {
      RCLCPP_ERROR(getLogger(), "Activation read threw for arm '%s'", arm.robot_name_.c_str());
      return rollbackActivation("activation state read exception");
    }
  }

  for (size_t index = 0; index < staged_state_count; ++index) {
    assignState(*staged_arms.at(index), staged_states.at(index));
  }
  if (globalFaultDiagnostic().cause == GlobalFaultCause::BackendFault) {
    (void)tryClearRecoveredBackendFault();
  } else {
    global_fault_latch_.store(0, std::memory_order_release);
  }
  control_cycle_owner_.store(0, std::memory_order_release);
  owner_handoff_state_.store(0, std::memory_order_release);
  RCLCPP_INFO(getLogger(), "Started");
  return CallbackReturn::SUCCESS;
}

CallbackReturn FrankaMultiHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(getLogger(), "trying to Stop...");
  const bool all_stops_succeeded = driveAllArmsToFailSafeStop();
  RCLCPP_INFO(getLogger(), "Stopped");
  return all_stops_succeeded ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

CallbackReturn FrankaMultiHardwareInterface::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // TRANSITION_CONFIGURE only ever connects UNCONFIGURED -> INACTIVE in the installed
  // lifecycle_msgs graph (unlike TRANSITION_ACTIVE_SHUTDOWN, there is no edge that reaches
  // on_configure directly from ACTIVE). We still run the guard unconditionally: this is
  // pluginlib-loaded by a generic resource manager, so nothing stops a caller from invoking
  // on_configure() directly regardless of the documented graph -- the probe that found this
  // P0 proved every one of these callbacks can be invoked that way. And "on_configure can
  // only be reached from an already-inactive state" would not even make backend
  // communication safe to assume idle: on_init() already constructs the backend (for the
  // real backend, that is a TCP connect to the arm plus setDefaultParams/readOnce/loadModel,
  // see RealFrankaArmBackend/Robot::Robot), so backend communication may already be live
  // before on_configure is ever called, direct invocation or not. The guard is idempotent and
  // off the RT path, so paying for it here is free insurance, not busywork: on the real
  // UNCONFIGURED->INACTIVE path every arm already reports stopped/None and this returns true
  // immediately.
  const bool all_confirmed_safe = driveAllArmsToFailSafeStop();
  if (!all_confirmed_safe) {
    RCLCPP_ERROR(getLogger(),
                 "on_configure: could not confirm every arm is in the fail-safe "
                 "stopped state");
  }
  return all_confirmed_safe ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

CallbackReturn FrankaMultiHardwareInterface::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // TRANSITION_CLEANUP is documented as INACTIVE -> UNCONFIGURED, so by the time it legally
  // runs on_deactivate has already stopped everything -- but this callback releases
  // resources back to the UNCONFIGURED contract, and (like on_configure) nothing prevents a
  // caller from invoking it out of the documented order. Re-running the same idempotent
  // guard here is the cheapest way to guarantee UNCONFIGURED always means "actually safe".
  RCLCPP_INFO(getLogger(), "on_cleanup: confirming fail-safe stop before releasing resources");
  const bool all_confirmed_safe = driveAllArmsToFailSafeStop();
  if (!all_confirmed_safe) {
    RCLCPP_ERROR(getLogger(),
                 "on_cleanup: could not confirm every arm is in the fail-safe "
                 "stopped state");
  }
  return all_confirmed_safe ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

CallbackReturn FrankaMultiHardwareInterface::on_shutdown(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // TRANSITION_ACTIVE_SHUTDOWN is a direct ACTIVE -> (on_shutdown) -> FINALIZED edge that
  // never calls on_deactivate first, so this is the primary edge this P0 exists for: without
  // this override nothing stopped the worker or cleared a live mode on shutdown from ACTIVE.
  RCLCPP_INFO(getLogger(), "on_shutdown: trying to Stop...");
  const bool all_confirmed_safe = driveAllArmsToFailSafeStop();
  RCLCPP_INFO(getLogger(), all_confirmed_safe ? "on_shutdown: Stopped"
                                              : "on_shutdown: Stop NOT confirmed for all arms");
  // FINALIZED is reached either way once shutdown is invoked; ERROR (rather than a quiet
  // SUCCESS) ensures an unconfirmed stop is never silently reported as safe and still routes
  // through on_error for one more attempt + one more loud log before finalizing.
  return all_confirmed_safe ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

CallbackReturn FrankaMultiHardwareInterface::on_error(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // ErrorProcessing can be entered from ACTIVE (e.g. a thrown exception from another
  // callback) without on_deactivate ever running, so this is the other primary edge this P0
  // exists for. Per LifecycleNodeInterface, returning SUCCESS here means "handled, node
  // resets to UNCONFIGURED"; returning FAILURE/ERROR means "unrecoverable, node finalizes".
  // We must not mask a fault: SUCCESS is only honest if every arm is actually confirmed
  // stopped. If it isn't, we report ERROR rather than pretending recovery succeeded, so the
  // node is finalized (and the failure loudly logged) instead of being reused unsafely.
  RCLCPP_ERROR(getLogger(), "on_error: attempting fail-safe stop of all arms");
  const bool all_confirmed_safe = driveAllArmsToFailSafeStop();
  if (!all_confirmed_safe) {
    RCLCPP_ERROR(getLogger(),
                 "on_error: could not confirm every arm is in the fail-safe stopped state; "
                 "reporting unrecoverable so the hardware interface is finalized rather than "
                 "reused");
  }
  return all_confirmed_safe ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

hardware_interface::return_type FrankaMultiHardwareInterface::read(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  if (!bindControlCycleOwner()) {
    return hardware_interface::return_type::ERROR;
  }
  if (globalFaultLatched() && !tryClearRecoveredBackendFault()) {
    return hardware_interface::return_type::ERROR;
  }

  std::array<franka::RobotState, 2> staged_states{};
  bool all_states_valid = robot_count_ != 0;
  uint8_t fault_origin = 0;
  GlobalFaultCause fault_cause = GlobalFaultCause::None;

  for (size_t arm_index = 0; arm_index < robot_count_; ++arm_index) {
    auto* arm = arm_slots_.at(arm_index);
    if (arm == nullptr) {
      all_states_valid = false;
      if (fault_cause == GlobalFaultCause::None) {
        fault_origin = static_cast<uint8_t>(arm_index + 1);
        fault_cause = GlobalFaultCause::ReadFailure;
      }
      continue;
    }
    try {
      auto candidate = arm->backend_->readLatestState();
      const bool backend_faulted = arm->backend_->hasFault();
      const bool candidate_is_finite = isFiniteState(candidate);
      if (backend_faulted || !candidate_is_finite) {
        all_states_valid = false;
        if (fault_cause == GlobalFaultCause::None) {
          fault_origin = static_cast<uint8_t>(arm_index + 1);
          fault_cause =
              backend_faulted ? GlobalFaultCause::BackendFault : GlobalFaultCause::InvalidState;
        }
        continue;
      }
      staged_states.at(arm_index) = std::move(candidate);
    } catch (...) {
      all_states_valid = false;
      if (fault_cause == GlobalFaultCause::None) {
        fault_origin = static_cast<uint8_t>(arm_index + 1);
        fault_cause = GlobalFaultCause::ReadFailure;
      }
    }
  }

  if (!all_states_valid) {
    enterGlobalFault(fault_origin, fault_cause);
    return hardware_interface::return_type::ERROR;
  }
  for (size_t arm_index = 0; arm_index < robot_count_; ++arm_index) {
    assignState(*arm_slots_.at(arm_index), staged_states.at(arm_index));
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FrankaMultiHardwareInterface::write(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  if (!bindControlCycleOwner()) {
    return hardware_interface::return_type::ERROR;
  }
  // Must run even under a latched fault (checked next) so a waiting off-owner
  // perform_command_mode_switch() call fails fast through applyPreparedTransactionEffects()'s own
  // fault check instead of blocking its caller for the full handoff timeout.
  serviceOwnerHandoffIfPending();
  if (globalFaultLatched()) {
    return hardware_interface::return_type::ERROR;
  }

  uint8_t fault_origin = 0;
  GlobalFaultCause fault_cause = GlobalFaultCause::None;
  for (size_t arm_index = 0; arm_index < robot_count_; ++arm_index) {
    auto& arm = *arm_slots_.at(arm_index);
    const bool backend_faulted = arm.backend_->hasFault();
    const bool command_is_finite = commandsAreFinite(arm);
    if (fault_cause == GlobalFaultCause::None && (backend_faulted || !command_is_finite)) {
      fault_origin = static_cast<uint8_t>(arm_index + 1);
      fault_cause =
          backend_faulted ? GlobalFaultCause::BackendFault : GlobalFaultCause::InvalidCommand;
    }
  }
  if (fault_cause != GlobalFaultCause::None) {
    enterGlobalFault(fault_origin, fault_cause);
    return hardware_interface::return_type::ERROR;
  }

  for (size_t arm_index = 0; arm_index < robot_count_; ++arm_index) {
    if (!arm_slots_.at(arm_index)->backend_->canPublishCommand()) {
      enterGlobalFault(static_cast<uint8_t>(arm_index + 1), GlobalFaultCause::CommandCapacity);
      return hardware_interface::return_type::ERROR;
    }
  }

  for (size_t arm_index = 0; arm_index < robot_count_; ++arm_index) {
    auto& arm = *arm_slots_.at(arm_index);
    // F-10c amendment A (2026-08-28): with the controller-side lifecycle zero deleted, a
    // deactivated controller's last real command stays in arm.hw_commands_* -- nothing overwrites
    // it until some controller claims those interfaces again. An arm in ControlMode::None has no
    // motion generator consuming published commands, so republishing that stale motion command
    // cannot actuate anything; but handing a backend a stale non-zero motion command every cycle
    // is still the wrong thing to publish, and it is what a controller's own zero used to hide.
    // Publish the state-derived safe command instead -- zero efforts, zero joint and cartesian
    // velocities, position held at the measured pose -- so "no live mode" and "only safe commands
    // leave this component" are the same fact.
    //
    // This is the hardware-side, owner-thread replacement for that deleted controller write: it
    // runs on the control-cycle owner thread like every other backend call here, is allocation-
    // free and lock-free (one fixed-size RobotCommand in automatic storage), and derives from
    // arm.hw_franka_robot_state_ rather than from arm.hw_commands_*, so no controller can corrupt
    // it. Note it deliberately does not *write* arm.hw_commands_*: the finiteness check above
    // still sees exactly what the last controller left there.
    const bool published =
        arm.control_mode_.load(std::memory_order_acquire) == ControlMode::None
            ? publishCommand(arm, safeCommandForArm(arm, CommandInitialization::None))
            : publishCommands(arm);
    if (!published) {
      enterGlobalFault(static_cast<uint8_t>(arm_index + 1), GlobalFaultCause::CommandPublish);
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FrankaMultiHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  if (!discardReadyPreparedTransaction() || globalFaultLatched()) {
    return hardware_interface::return_type::ERROR;
  }

  std::vector<ArmCommandModeState> arm_states;
  arm_states.reserve(robot_count_);
  for (size_t arm_index = 0; arm_index < robot_count_; ++arm_index) {
    const auto& arm = *arm_slots_.at(arm_index);
    arm_states.push_back({arm.robot_name_, arm.control_mode_.load(std::memory_order_acquire)});
  }

  const auto result =
      CommandModeSwitchPlanner::makePlan(arm_states, start_interfaces, stop_interfaces);
  if (!result) {
    RCLCPP_ERROR(getLogger(), "Cannot prepare command mode switch: %s", result.message.c_str());
    return hardware_interface::return_type::ERROR;
  }

  PreparedModeTransaction transaction;
  transaction.generation = nextPreparedTransactionGeneration();
  if (!makeModeSwitchSignature(start_interfaces, stop_interfaces, transaction.signature)) {
    return hardware_interface::return_type::ERROR;
  }
  for (size_t arm_index = 0; arm_index < robot_count_; ++arm_index) {
    const auto& request = result.plan->arms.at(arm_index);
    transaction.arms.at(arm_index) = {request.requested_mode, request.command_initialization,
                                      request.has_request};
  }

  if (!beginPreparedTransactionPublication(transaction.generation)) {
    return hardware_interface::return_type::ERROR;
  }
  prepared_transaction_ = transaction;
  if (mode_switch_checkpoint_hook_) {
    try {
      mode_switch_checkpoint_hook_(ModeSwitchCheckpoint::PreparedPayloadWritten);
    } catch (...) {
      invalidatePreparedTransaction();
      (void)finishPreparedTransactionPublication(transaction.generation);
      return hardware_interface::return_type::ERROR;
    }
  }
  if (globalFaultLatched()) {
    invalidatePreparedTransaction();
    (void)finishPreparedTransactionPublication(transaction.generation);
    return hardware_interface::return_type::ERROR;
  }
  if (!finishPreparedTransactionPublication(transaction.generation)) {
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FrankaMultiHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  // With Jazzy activate_asap=false this callback executes on controller_manager's service thread
  // while the control cycle keeps running on its own thread; with activate_asap=true it is
  // deferred by controller_manager into the same update-thread call sequence as read()/write().
  // Both are legitimate production paths -- e.g. a plain `ros2 control switch_controllers
  // --deactivate` (no --switch-asap) resolves to the former. Neither is rejected here: every
  // preflight check below runs on whichever thread called us (they only ever read atomics or
  // immutable post-activation state, never backend/exported command state), and only the final
  // effects step is routed to the control-cycle owner thread -- directly, if we are already on
  // it, or via a bounded cross-thread handoff (requestOwnerExecutedEffects()) if we are not. That
  // keeps "the RT thread is the sole producer of backend-mutating command/mode-request calls"
  // true regardless of which thread this function runs on, without gating perform on thread
  // identity the way the original owner-only check did.
  if (globalFaultLatched()) {
    return hardware_interface::return_type::ERROR;
  }

  PreparedModeTransaction transaction;
  if (!consumePreparedTransaction(transaction)) {
    return hardware_interface::return_type::ERROR;
  }
  const auto finish_transaction = [this, generation = transaction.generation]() noexcept {
    finishPreparedTransactionConsumption(generation);
  };

  ModeSwitchInterfaceSignature signature;
  if (!makeModeSwitchSignature(start_interfaces, stop_interfaces, signature) ||
      !(signature == transaction.signature) || globalFaultLatched()) {
    finish_transaction();
    return hardware_interface::return_type::ERROR;
  }

  for (size_t arm_index = 0; arm_index < robot_count_; ++arm_index) {
    auto& arm = *arm_slots_.at(arm_index);
    if (arm.backend_->hasFault()) {
      enterGlobalFault(static_cast<uint8_t>(arm_index + 1), GlobalFaultCause::BackendFault);
      finish_transaction();
      return hardware_interface::return_type::ERROR;
    }
  }

  // A temporarily-held service-operation gate is not a robot fault. Reject this prepared
  // transaction without publishing a snapshot or latching the global fault.
  for (size_t arm_index = 0; arm_index < robot_count_; ++arm_index) {
    const auto& arm = *arm_slots_.at(arm_index);
    const auto& request = transaction.arms.at(arm_index);
    if (request.has_request && !arm.backend_->canRequestControlMode(request.requested_mode)) {
      if (arm.backend_->hasFault()) {
        enterGlobalFault(static_cast<uint8_t>(arm_index + 1), GlobalFaultCause::BackendFault);
      }
      finish_transaction();
      return hardware_interface::return_type::ERROR;
    }
  }

  for (size_t arm_index = 0; arm_index < robot_count_; ++arm_index) {
    if (globalFaultLatched() || arm_slots_.at(arm_index)->backend_->hasFault()) {
      enterGlobalFault(static_cast<uint8_t>(arm_index + 1), GlobalFaultCause::BackendFault);
      finish_transaction();
      return hardware_interface::return_type::ERROR;
    }
  }

  for (size_t arm_index = 0; arm_index < robot_count_; ++arm_index) {
    const auto& arm = *arm_slots_.at(arm_index);
    if (transaction.arms.at(arm_index).has_request && !arm.backend_->canPublishCommand()) {
      enterGlobalFault(static_cast<uint8_t>(arm_index + 1), GlobalFaultCause::CommandCapacity);
      finish_transaction();
      return hardware_interface::return_type::ERROR;
    }
  }

  // Recheck after every non-mutating preflight. Test backends use this boundary to inject a fault
  // deterministically; no safe command or motion request may have occurred yet.
  for (size_t arm_index = 0; arm_index < robot_count_; ++arm_index) {
    if (globalFaultLatched() || arm_slots_.at(arm_index)->backend_->hasFault()) {
      enterGlobalFault(static_cast<uint8_t>(arm_index + 1), GlobalFaultCause::BackendFault);
      finish_transaction();
      return hardware_interface::return_type::ERROR;
    }
  }

  const auto result = isControlCycleOwner() ? applyPreparedTransactionEffects(transaction)
                                            : requestOwnerExecutedEffects(transaction);
  finish_transaction();
  return result;
}

/*
 * The planner allocates only in prepare_command_mode_switch(). The accepted perform path consumes
 * one atomically published fixed-size value and performs bounded parsing, preflight and backend
 * calls. Everything through the preflight above does not allocate, lock, log, mutate exported
 * command storage, or touch a backend. The effects step below is the one part of perform that does
 * touch a backend (safe-command publish, mode request) and the one part that must run on the
 * control-cycle owner thread -- see applyPreparedTransactionEffects(),
 * requestOwnerExecutedEffects() and serviceOwnerHandoffIfPending() immediately below.
 */

hardware_interface::return_type FrankaMultiHardwareInterface::applyPreparedTransactionEffects(
    const PreparedModeTransaction& transaction) noexcept {
  // Only ever called on the control-cycle owner thread (directly from perform_command_mode_switch()
  // when it is that thread, or from serviceOwnerHandoffIfPending() on behalf of a waiting off-owner
  // caller), so this never races write()'s own backend calls on the same arm -- they are the same
  // call sequence on the same thread, never concurrent with each other. Bounded, allocation-free,
  // lock-free: fixed-size local storage only, no heap, no mutex, no syscall other than the backend
  // calls a normal write() cycle already makes.
  if (globalFaultLatched()) {
    return hardware_interface::return_type::ERROR;
  }

  std::array<RobotCommand, 2> safe_commands{};
  for (size_t arm_index = 0; arm_index < robot_count_; ++arm_index) {
    const auto& request = transaction.arms.at(arm_index);
    if (request.has_request) {
      safe_commands.at(arm_index) =
          safeCommandForArm(*arm_slots_.at(arm_index), request.command_initialization);
    }
  }
  for (size_t arm_index = 0; arm_index < robot_count_; ++arm_index) {
    auto& arm = *arm_slots_.at(arm_index);
    if (transaction.arms.at(arm_index).has_request &&
        !publishCommand(arm, safe_commands.at(arm_index))) {
      enterGlobalFault(static_cast<uint8_t>(arm_index + 1), GlobalFaultCause::CommandPublish);
      return hardware_interface::return_type::ERROR;
    }
  }

  for (size_t arm_index = 0; arm_index < robot_count_; ++arm_index) {
    auto& arm = *arm_slots_.at(arm_index);
    const auto& request = transaction.arms.at(arm_index);
    if (request.has_request && !arm.backend_->requestControlMode(request.requested_mode)) {
      enterGlobalFault(static_cast<uint8_t>(arm_index + 1), GlobalFaultCause::ModeRequest);
      return hardware_interface::return_type::ERROR;
    }
  }

  for (size_t arm_index = 0; arm_index < robot_count_; ++arm_index) {
    auto& arm = *arm_slots_.at(arm_index);
    const auto& request = transaction.arms.at(arm_index);
    if (request.has_request) {
      arm.control_mode_.store(request.requested_mode, std::memory_order_release);
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FrankaMultiHardwareInterface::requestOwnerExecutedEffects(
    const PreparedModeTransaction& transaction) noexcept {
  // Single in-flight slot, exactly like the prepared-transaction protocol above: publish the
  // fixed-size payload, then CAS Idle -> Requested tagged with this transaction's own generation.
  // In production this CAS can only ever contend with another off-owner perform() call, which the
  // prepared-transaction single-slot protocol and controller_manager's own switch serialization
  // already prevent from running concurrently; failing safe here is defense in depth, not a path
  // production is expected to take.
  owner_handoff_transaction_ = transaction;
  const uint64_t requested =
      (transaction.generation << 3U) | static_cast<uint64_t>(OwnerHandoffStage::Requested);
  uint64_t expected = 0;
  if (!owner_handoff_state_.compare_exchange_strong(expected, requested, std::memory_order_release,
                                                    std::memory_order_acquire)) {
    return hardware_interface::return_type::ERROR;
  }

  // Bounded wait, off the RT path: this is the calling (non-owner) thread, e.g.
  // controller_manager's service thread, never the control-cycle owner thread. Nothing here is
  // reached from read()/write()/update(). The ceiling is well inside controller_manager's own
  // switch_controller() timeout (callers of switch_controller() already pass one, and this handoff
  // is only ever awaited from inside a single perform_command_mode_switch() call already bounded
  // by that timeout).
  constexpr auto kPollInterval = std::chrono::microseconds(200);
  constexpr int kMaxPolls = 25000;  // ~5 s ceiling
  for (int attempt = 0; attempt < kMaxPolls; ++attempt) {
    const auto observed = owner_handoff_state_.load(std::memory_order_acquire);
    // Our own generation is always >= 1 (see nextPreparedTransactionGeneration()), so a raw-zero
    // observation while we are still waiting can only mean the owner thread went away and
    // driveAllArmsToFailSafeStop() reset the slot out from under us: nothing will ever service
    // this request now, so fail immediately instead of spinning out the full ceiling.
    if (observed == 0) {
      return hardware_interface::return_type::ERROR;
    }
    if ((observed >> 3U) == transaction.generation) {
      const auto stage = static_cast<OwnerHandoffStage>(observed & 0x7U);
      if (stage == OwnerHandoffStage::CompletedOk || stage == OwnerHandoffStage::CompletedError) {
        owner_handoff_state_.store(0, std::memory_order_release);
        return stage == OwnerHandoffStage::CompletedOk ? hardware_interface::return_type::OK
                                                       : hardware_interface::return_type::ERROR;
      }
    }
    std::this_thread::sleep_for(kPollInterval);
  }
  // The owner thread never observed the request (e.g. the RT loop stopped concurrently with this
  // call). Reset the slot so a future handoff is not left permanently blocked, and fail safe.
  uint64_t stuck = requested;
  (void)owner_handoff_state_.compare_exchange_strong(stuck, 0, std::memory_order_release,
                                                     std::memory_order_relaxed);
  return hardware_interface::return_type::ERROR;
}

void FrankaMultiHardwareInterface::serviceOwnerHandoffIfPending() noexcept {
  // Common case: no handoff pending. One atomic load, no branch taken -- the same cost profile as
  // the other per-cycle atomic checks already in write().
  const auto observed = owner_handoff_state_.load(std::memory_order_acquire);
  if (static_cast<OwnerHandoffStage>(observed & 0x7U) != OwnerHandoffStage::Requested) {
    return;
  }
  const auto generation = observed >> 3U;
  // Safe to read without further synchronization: the acquire-load above observed Requested,
  // which synchronizes-with the requester's release-CAS that published owner_handoff_transaction_
  // beforehand (see requestOwnerExecutedEffects()).
  const PreparedModeTransaction transaction = owner_handoff_transaction_;
  if (transaction.generation != generation) {
    // Cannot happen given the CAS-gated single-slot protocol; never apply a mismatched payload.
    return;
  }

  const auto result = applyPreparedTransactionEffects(transaction);
  const auto completed =
      (generation << 3U) | static_cast<uint64_t>(result == hardware_interface::return_type::OK
                                                     ? OwnerHandoffStage::CompletedOk
                                                     : OwnerHandoffStage::CompletedError);
  uint64_t expected = observed;
  // If this CAS ever fails, the requester already timed out and reset the slot to Idle (or,
  // vanishingly unlikely, a second request landed) -- either way the waiting caller (if any) is
  // not relying on this write, so there is nothing to retry.
  (void)owner_handoff_state_.compare_exchange_strong(expected, completed, std::memory_order_release,
                                                     std::memory_order_relaxed);
}

RobotCommand FrankaMultiHardwareInterface::safeCommandForArm(
    const ArmContainer& arm,
    CommandInitialization /*initialization*/) noexcept {
  return makeSafeRobotCommand(arm.hw_franka_robot_state_);
}

void FrankaMultiHardwareInterface::assignExportedCommands(ArmContainer& arm,
                                                          const RobotCommand& command) noexcept {
  arm.hw_commands_joint_effort_ = command.efforts;
  arm.hw_commands_joint_position_ = command.joint_positions;
  arm.hw_commands_joint_velocity_ = command.joint_velocities;
  arm.hw_commands_cartesian_position_ = command.cartesian_positions;
  arm.hw_commands_cartesian_velocity_ = command.cartesian_velocities;
}

bool FrankaMultiHardwareInterface::publishCommands(ArmContainer& arm) noexcept {
  RobotCommand command;
  command.efforts = arm.hw_commands_joint_effort_;
  command.joint_positions = arm.hw_commands_joint_position_;
  command.joint_velocities = arm.hw_commands_joint_velocity_;
  command.cartesian_positions = arm.hw_commands_cartesian_position_;
  command.cartesian_velocities = arm.hw_commands_cartesian_velocity_;
  return publishCommand(arm, command);
}

bool FrankaMultiHardwareInterface::publishCommand(ArmContainer& arm,
                                                  const RobotCommand& command) noexcept {
  return arm.backend_->publishCommand(command);
}

bool FrankaMultiHardwareInterface::commandsAreFinite(const ArmContainer& arm) noexcept {
  return allFinite(arm.hw_commands_joint_effort_) && allFinite(arm.hw_commands_joint_position_) &&
         allFinite(arm.hw_commands_joint_velocity_) &&
         allFinite(arm.hw_commands_cartesian_position_) &&
         allFinite(arm.hw_commands_cartesian_velocity_);
}

bool FrankaMultiHardwareInterface::makeModeSwitchSignature(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces,
    ModeSwitchInterfaceSignature& signature) const noexcept {
  if (start_interfaces.size() > kMaximumModeSwitchInterfaceCount ||
      stop_interfaces.size() > kMaximumModeSwitchInterfaceCount) {
    return false;
  }
  const auto encode = [this](const std::vector<std::string>& interfaces, uint32_t& mask) noexcept {
    mask = 0;
    for (const auto& interface_name : interfaces) {
      bool claims_configured_arm = false;
      bool matched = false;
      uint32_t interface_bit = 0;
      const bool within_supported_name_bound =
          interface_name.size() <= kMaximumJointModeInterfaceNameLength;
      for (size_t arm_index = 0; arm_index < robot_count_; ++arm_index) {
        const auto& arm_name = arm_slots_.at(arm_index)->robot_name_;
        if (interface_name.size() > arm_name.size() &&
            interface_name.compare(0, arm_name.size(), arm_name) == 0 &&
            interface_name[arm_name.size()] == '_') {
          claims_configured_arm = true;
        }

        // The configured arm prefix comparison above is bounded by the validated 64-character arm
        // identifier. Any supported joint-mode name is at most 80 characters; longer foreign
        // names remain ignored, while longer names claiming a configured arm are rejected below.
        if (!within_supported_name_bound) {
          continue;
        }
        const size_t joint_digit_index = arm_name.size() + 6;
        const size_t mode_index = joint_digit_index + 2;
        if (interface_name.size() <= mode_index ||
            interface_name.compare(0, arm_name.size(), arm_name) != 0 ||
            interface_name.compare(arm_name.size(), 6, "_joint") != 0 ||
            interface_name[joint_digit_index] < '1' || interface_name[joint_digit_index] > '7' ||
            interface_name[joint_digit_index + 1] != '/') {
          continue;
        }

        size_t mode_offset = 0;
        if (interface_name.compare(mode_index, std::string::npos, "effort") == 0) {
          mode_offset = 0;
        } else if (interface_name.compare(mode_index, std::string::npos, "velocity") == 0) {
          mode_offset = 7;
        } else {
          continue;
        }
        const auto joint_index = static_cast<size_t>(interface_name[joint_digit_index] - '1');
        interface_bit = static_cast<uint32_t>(1U << (arm_index * 14 + mode_offset + joint_index));
        matched = true;
        break;
      }

      if ((!matched && claims_configured_arm) || (matched && (mask & interface_bit) != 0U)) {
        return false;
      }
      if (matched) {
        mask |= interface_bit;
      }
    }
    return true;
  };

  return encode(start_interfaces, signature.start_mask) &&
         encode(stop_interfaces, signature.stop_mask);
}

uint64_t FrankaMultiHardwareInterface::nextPreparedTransactionGeneration() noexcept {
  auto generation = preparedTransactionGenerationCandidate(
      next_prepared_generation_.fetch_add(1, std::memory_order_relaxed));
  if (generation == 0) {
    // Exactly one counter value in each 61-bit epoch encodes the reserved Empty state. Skip it
    // with one statically bounded extra fetch.
    generation = preparedTransactionGenerationCandidate(
        next_prepared_generation_.fetch_add(1, std::memory_order_relaxed));
  }
  return generation;
}

bool FrankaMultiHardwareInterface::beginPreparedTransactionPublication(
    uint64_t generation) noexcept {
  const uint64_t publishing =
      (generation << 3U) | static_cast<uint64_t>(PreparedTransactionStage::Publishing);
  uint64_t expected = 0;
  return prepared_transaction_state_.compare_exchange_strong(
      expected, publishing, std::memory_order_acq_rel, std::memory_order_acquire);
}

bool FrankaMultiHardwareInterface::finishPreparedTransactionPublication(
    uint64_t generation) noexcept {
  const uint64_t publishing =
      (generation << 3U) | static_cast<uint64_t>(PreparedTransactionStage::Publishing);
  const uint64_t ready =
      (generation << 3U) | static_cast<uint64_t>(PreparedTransactionStage::Ready);
  uint64_t expected = publishing;
  if (prepared_transaction_state_.compare_exchange_strong(
          expected, ready, std::memory_order_release, std::memory_order_acquire)) {
    return true;
  }

  const uint64_t invalidated =
      (generation << 3U) | static_cast<uint64_t>(PreparedTransactionStage::PublishingInvalidated);
  if (expected == invalidated) {
    (void)prepared_transaction_state_.compare_exchange_strong(
        expected, 0, std::memory_order_release, std::memory_order_relaxed);
  }
  return false;
}

bool FrankaMultiHardwareInterface::consumePreparedTransaction(
    PreparedModeTransaction& transaction) noexcept {
  auto observed = prepared_transaction_state_.load(std::memory_order_acquire);
  if (static_cast<PreparedTransactionStage>(observed & 0x7U) != PreparedTransactionStage::Ready) {
    return false;
  }
  const auto generation = observed >> 3U;
  const uint64_t consuming =
      (generation << 3U) | static_cast<uint64_t>(PreparedTransactionStage::Consuming);
  if (!prepared_transaction_state_.compare_exchange_strong(
          observed, consuming, std::memory_order_acq_rel, std::memory_order_acquire)) {
    return false;
  }
  transaction = prepared_transaction_;
  if (transaction.generation != generation) {
    finishPreparedTransactionConsumption(generation);
    return false;
  }
  return true;
}

void FrankaMultiHardwareInterface::finishPreparedTransactionConsumption(
    uint64_t generation) noexcept {
  uint64_t expected =
      (generation << 3U) | static_cast<uint64_t>(PreparedTransactionStage::Consuming);
  if (prepared_transaction_state_.compare_exchange_strong(expected, 0, std::memory_order_release,
                                                          std::memory_order_relaxed)) {
    return;
  }
  const uint64_t invalidated =
      (generation << 3U) | static_cast<uint64_t>(PreparedTransactionStage::ConsumingInvalidated);
  if (expected == invalidated) {
    (void)prepared_transaction_state_.compare_exchange_strong(
        expected, 0, std::memory_order_release, std::memory_order_relaxed);
  }
}

void FrankaMultiHardwareInterface::invalidatePreparedTransaction() noexcept {
  auto observed = prepared_transaction_state_.load(std::memory_order_acquire);
  const auto target_generation = observed >> 3U;

  // A transaction generation has only three forward transitions that can race this operation:
  // Publishing -> Ready -> Consuming -> Empty. Strong CAS does not fail spuriously, so four
  // observations are sufficient to either invalidate that generation or observe it completed.
  // Never follow a newer generation: a global fault makes its publisher self-invalidate, while an
  // off-owner perform is responsible only for the transaction it rejected.
  for (size_t attempt = 0; attempt < 4; ++attempt) {
    if ((observed >> 3U) != target_generation) {
      return;
    }
    const auto stage = static_cast<PreparedTransactionStage>(observed & 0x7U);
    uint64_t desired = observed;
    switch (stage) {
      case PreparedTransactionStage::Empty:
      case PreparedTransactionStage::PublishingInvalidated:
      case PreparedTransactionStage::ConsumingInvalidated:
        return;
      case PreparedTransactionStage::Publishing:
        desired = (observed & ~uint64_t{0x7U}) |
                  static_cast<uint64_t>(PreparedTransactionStage::PublishingInvalidated);
        break;
      case PreparedTransactionStage::Ready:
        desired = 0;
        break;
      case PreparedTransactionStage::Consuming:
        desired = (observed & ~uint64_t{0x7U}) |
                  static_cast<uint64_t>(PreparedTransactionStage::ConsumingInvalidated);
        break;
    }
    if (prepared_transaction_state_.compare_exchange_strong(
            observed, desired, std::memory_order_acq_rel, std::memory_order_acquire)) {
      return;
    }
  }
}

bool FrankaMultiHardwareInterface::discardReadyPreparedTransaction() noexcept {
  auto observed = prepared_transaction_state_.load(std::memory_order_acquire);
  const auto stage = static_cast<PreparedTransactionStage>(observed & 0x7U);
  if (stage == PreparedTransactionStage::Empty) {
    return true;
  }
  if (stage != PreparedTransactionStage::Ready) {
    return false;
  }
  return prepared_transaction_state_.compare_exchange_strong(observed, 0, std::memory_order_acq_rel,
                                                             std::memory_order_acquire);
}

bool FrankaMultiHardwareInterface::bindControlCycleOwner() noexcept {
  const auto token = static_cast<uintptr_t>(pthread_self());
  auto expected = uintptr_t{0};
  if (control_cycle_owner_.compare_exchange_strong(expected, token, std::memory_order_acq_rel,
                                                   std::memory_order_acquire)) {
    return true;
  }
  return expected == token;
}

bool FrankaMultiHardwareInterface::isControlCycleOwner() const noexcept {
  const auto owner = control_cycle_owner_.load(std::memory_order_acquire);
  return owner != 0 && owner == static_cast<uintptr_t>(pthread_self());
}

void FrankaMultiHardwareInterface::resetCurrentModeState() noexcept {
  invalidatePreparedTransaction();
  for (size_t arm_index = 0; arm_index < robot_count_; ++arm_index) {
    arm_slots_.at(arm_index)->control_mode_.store(ControlMode::None, std::memory_order_release);
  }
}

GlobalFaultDiagnostic FrankaMultiHardwareInterface::globalFaultDiagnostic() const noexcept {
  const uint64_t packed = global_fault_latch_.load(std::memory_order_acquire);
  return GlobalFaultDiagnostic{
      static_cast<uint8_t>(packed & 0xffU),
      static_cast<GlobalFaultCause>((packed >> 8U) & 0xffU),
      static_cast<uint8_t>((packed >> 16U) & 0xffU),
      static_cast<uint8_t>((packed >> 24U) & 0xffU),
  };
}

bool FrankaMultiHardwareInterface::globalFaultLatched() const noexcept {
  return global_fault_latch_.load(std::memory_order_acquire) != 0;
}

bool FrankaMultiHardwareInterface::tryClearRecoveredBackendFault() noexcept {
  const uint64_t observed_latch = global_fault_latch_.load(std::memory_order_acquire);
  if (static_cast<GlobalFaultCause>((observed_latch >> 8U) & 0xffU) !=
      GlobalFaultCause::BackendFault) {
    return observed_latch == 0;
  }

  for (size_t arm_index = 0; arm_index < robot_count_; ++arm_index) {
    const auto& arm = *arm_slots_.at(arm_index);
    const auto diagnostics = arm.backend_->diagnostics();
    const bool safe_mode =
        diagnostics.stopped || (diagnostics.requested_mode == ControlMode::None &&
                                diagnostics.active_mode == ControlMode::None);
    if (arm.backend_->hasFault() ||
        diagnostics.service_operation != BackendServiceOperation::Idle || !safe_mode) {
      return false;
    }
  }

  uint64_t expected = observed_latch;
  return global_fault_latch_.compare_exchange_strong(expected, 0, std::memory_order_acq_rel,
                                                     std::memory_order_acquire) ||
         expected == 0;
}

void FrankaMultiHardwareInterface::enterGlobalFault(uint8_t origin_arm_slot,
                                                    GlobalFaultCause cause) noexcept {
  if (cause == GlobalFaultCause::None || origin_arm_slot == 0 || origin_arm_slot > robot_count_) {
    return;
  }

  const uint8_t configured_mask = static_cast<uint8_t>((1U << robot_count_) - 1U);
  const uint64_t provisional =
      packGlobalFault(origin_arm_slot, cause, configured_mask, configured_mask);
  uint64_t expected = 0;
  if (!global_fault_latch_.compare_exchange_strong(expected, provisional, std::memory_order_acq_rel,
                                                   std::memory_order_acquire)) {
    return;
  }

  invalidatePreparedTransaction();
  std::array<RobotCommand, 2> safe_commands{};
  for (size_t arm_index = 0; arm_index < robot_count_; ++arm_index) {
    safe_commands.at(arm_index) =
        safeCommandForArm(*arm_slots_.at(arm_index), CommandInitialization::None);
  }

  uint8_t unsafe_safe_publish_mask = 0;
  for (size_t arm_index = 0; arm_index < robot_count_; ++arm_index) {
    auto& arm = *arm_slots_.at(arm_index);
    if (arm.backend_->hasFault() || !arm.backend_->canPublishCommand() ||
        !publishCommand(arm, safe_commands.at(arm_index))) {
      unsafe_safe_publish_mask =
          static_cast<uint8_t>(unsafe_safe_publish_mask | armMask(arm_index));
    }
  }

  uint8_t unsafe_none_request_mask = 0;
  for (size_t arm_index = 0; arm_index < robot_count_; ++arm_index) {
    auto& arm = *arm_slots_.at(arm_index);
    if (arm.backend_->requestControlMode(ControlMode::None)) {
      arm.control_mode_.store(ControlMode::None, std::memory_order_release);
    } else {
      unsafe_none_request_mask =
          static_cast<uint8_t>(unsafe_none_request_mask | armMask(arm_index));
      arm.control_mode_.store(arm.backend_->requestedControlMode(), std::memory_order_release);
    }
  }

  const uint64_t complete =
      packGlobalFault(origin_arm_slot, cause, unsafe_safe_publish_mask, unsafe_none_request_mask);
  expected = provisional;
  (void)global_fault_latch_.compare_exchange_strong(expected, complete, std::memory_order_release,
                                                    std::memory_order_relaxed);
}

bool FrankaMultiHardwareInterface::stopAllBackendsForRollback() noexcept {
  bool all_stopped = true;
  for (size_t arm_index = 0; arm_index < robot_count_; ++arm_index) {
    auto& arm = *arm_slots_.at(arm_index);
    try {
      if (!arm.backend_->stop()) {
        RCLCPP_ERROR(getLogger(), "Rollback stop returned false for arm '%s'",
                     arm.robot_name_.c_str());
      }
    } catch (const std::exception& exception) {
      RCLCPP_ERROR(getLogger(), "Rollback stop threw for arm '%s': %s", arm.robot_name_.c_str(),
                   exception.what());
    } catch (...) {
      RCLCPP_ERROR(getLogger(), "Rollback stop threw for arm '%s'", arm.robot_name_.c_str());
    }
    if (!arm.backend_->diagnostics().stopped) {
      all_stopped = false;
      RCLCPP_ERROR(getLogger(), "Rollback could not confirm arm '%s' is stopped",
                   arm.robot_name_.c_str());
    }
  }
  return all_stopped;
}

bool FrankaMultiHardwareInterface::driveAllArmsToFailSafeStop() noexcept {
  // The documented fail-safe state for this class: every configured arm's backend reports
  // stopped(), and the logical control-mode/prepared-transaction state is cleared to None so
  // a concurrently-running RT read()/write() cycle can never observe a live mode again.
  //
  // Order: clear the logical mode state first, then stop each backend. A worker thread
  // racing this call under read()/write() sees "no active mode" the instant this store
  // lands, even if the (comparatively slow) backend stop() below hasn't returned yet -- so
  // the RT-visible intent goes safe before the RT-invisible physical/simulated stop
  // completes, never the other way around. This mirrors the original on_deactivate
  // ordering exactly; nothing about the sequence changed, only that it is now shared.
  //
  // Every configured arm slot is always visited, even if an earlier one throws or fails to
  // confirm stopped: one arm's failure must never leave a sibling arm unattended. The
  // return value reports whether *every* arm was confirmed stopped, matching the original
  // on_deactivate all-or-nothing success contract.
  //
  // Idempotent: calling this repeatedly, from any lifecycle state, in any order relative to
  // itself, is safe. resetCurrentModeState() is a plain store + prepared-transaction
  // invalidation (safe on an already-None mode), and stop() on an already-stopped backend is
  // a no-op confirmed by RealFrankaArmBackend::stop() / Robot::stopRobot() (and mirrored by
  // the synthetic backend used in offline tests). robot_count_ == 0 (e.g. on_init never
  // completed) makes the loop below a no-op and this returns true trivially.
  //
  // This function is never called from read()/write()/update() and must stay that way: nothing
  // here is bounded for the 1 kHz control cycle (backend_->stop() can block on real hardware).
  resetCurrentModeState();
  bool all_stops_succeeded = true;
  for (size_t arm_index = 0; arm_index < robot_count_; ++arm_index) {
    auto& arm = *arm_slots_.at(arm_index);
    try {
      const bool stop_succeeded = arm.backend_->stop();
      const bool stopped = arm.backend_->diagnostics().stopped;
      all_stops_succeeded = stop_succeeded && stopped && all_stops_succeeded;
      if (!stop_succeeded || !stopped) {
        RCLCPP_ERROR(getLogger(), "Could not confirm that arm '%s' stopped",
                     arm.robot_name_.c_str());
      }
    } catch (const std::exception& exception) {
      all_stops_succeeded = false;
      RCLCPP_ERROR(getLogger(), "Stopping arm '%s' threw: %s", arm.robot_name_.c_str(),
                   exception.what());
    } catch (...) {
      all_stops_succeeded = false;
      RCLCPP_ERROR(getLogger(), "Stopping arm '%s' threw", arm.robot_name_.c_str());
    }
  }
  control_cycle_owner_.store(0, std::memory_order_release);
  // Nothing will ever service a pending handoff now that the owner token is cleared. Resetting
  // the slot to 0 here (rather than leaving it Requested) is what lets a caller blocked in
  // requestOwnerExecutedEffects() recognize that and fail immediately instead of spinning out its
  // full bounded timeout -- see the raw-zero check there.
  owner_handoff_state_.store(0, std::memory_order_release);
  return all_stops_succeeded;
}

CallbackReturn FrankaMultiHardwareInterface::rollbackActivation(const char* reason) noexcept {
  RCLCPP_ERROR(getLogger(), "Activation failed: %s", reason);
  resetCurrentModeState();
  return stopAllBackendsForRollback() ? CallbackReturn::FAILURE : CallbackReturn::ERROR;
}

rclcpp::Logger FrankaMultiHardwareInterface::getLogger() {
  return rclcpp::get_logger("FrankaMultiHardwareInterface");
}
}  // namespace franka_hardware

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_hardware::FrankaMultiHardwareInterface,
                       hardware_interface::SystemInterface)
