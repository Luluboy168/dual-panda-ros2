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
#include <cmath>
#include <exception>
#include <franka_hardware/real/franka_multi_hardware_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace franka_hardware
{

using StateInterface = hardware_interface::StateInterface;
using CommandInterface = hardware_interface::CommandInterface;

CallbackReturn FrankaMultiHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  // First get number of robots
  std::stringstream rc_stream(info_.hardware_parameters.at("robot_count"));
  rc_stream >> robot_count_;
  // if(robot_count_ < 2U){
  //   RCLCPP_FATAL(getLogger(), "Configured robot count is less than 2. Please use
  //   FrankaHardwareInterface instead."); return CallbackReturn::ERROR;
  // }

  if (info_.joints.size() != kNumberOfJoints * robot_count_) {
    RCLCPP_FATAL(
      getLogger(), "Got %ld joints. Expected %ld.", info_.joints.size(),
      kNumberOfJoints * robot_count_);
    return CallbackReturn::ERROR;
  }

  for (const auto & joint : info_.joints) {
    // Check number of command interfaces
    if (joint.command_interfaces.size() != 3) {
      RCLCPP_FATAL(
        getLogger(), "Joint '%s' has %ld command interfaces found. 3 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    // Check that the interfaces are named correctly
    for (const auto & cmd_interface : joint.command_interfaces) {
      if (
        cmd_interface.name != hardware_interface::HW_IF_EFFORT &&    // Effort "effort"
        cmd_interface.name != hardware_interface::HW_IF_POSITION &&  // Joint position "position"
        cmd_interface.name != hardware_interface::HW_IF_VELOCITY &&  // Joint velocity "velocity"
        cmd_interface.name != "cartesian_position" &&                // Cartesian position
        cmd_interface.name != "cartesian_velocity") {                // Cartesian velocity
        RCLCPP_FATAL(
          getLogger(), "Joint '%s' has unexpected command interface '%s'", joint.name.c_str(),
          cmd_interface.name.c_str());
        return CallbackReturn::ERROR;
      }
    }
    const auto has_one_command_interface = [&joint](const std::string & interface_name) {
      return std::count_if(
               joint.command_interfaces.begin(), joint.command_interfaces.end(),
               [&interface_name](const auto & interface) {
                 return interface.name == interface_name;
               }) == 1;
    };
    if (
      !has_one_command_interface(hardware_interface::HW_IF_EFFORT) ||
      !has_one_command_interface(hardware_interface::HW_IF_POSITION) ||
      !has_one_command_interface(hardware_interface::HW_IF_VELOCITY)) {
      RCLCPP_FATAL(
        getLogger(), "Joint '%s' must provide effort, position and velocity exactly once",
        joint.name.c_str());
      return CallbackReturn::ERROR;
    }

    // Check number of state interfaces
    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(
        getLogger(), "Joint '%s' has %zu state interfaces found. 3 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        getLogger(), "Joint '%s' has unexpected state interface '%s'. Expected '%s'",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        getLogger(), "Joint '%s' has unexpected state interface '%s'. Expected '%s'",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(
        getLogger(), "Joint '%s' has unexpected state interface '%s'. Expected '%s'",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_EFFORT);
      return CallbackReturn::ERROR;
    }
  }
  // only one executor for the class!
  executor_ = std::make_shared<FrankaExecutor>();

  for (size_t i = 1; i <= robot_count_; i++) {
    // Setup arm container
    std::string suffix = "_" + std::to_string(i);
    std::string robot_name;

    try {
      robot_name = info_.hardware_parameters.at("ns" + suffix);
    } catch (const std::out_of_range & ex) {
      RCLCPP_FATAL(
        getLogger(),
        "Parameber 'ns%s' ! set\nMake sure all multi-robot parameters follow the format "
        "{ns/robot_ip/etc.}_n",
        suffix.c_str());
      return CallbackReturn::ERROR;
    }
    if (!arms_.insert(std::make_pair(robot_name, ArmContainer())).second) {
      RCLCPP_FATAL(
        getLogger(), "The provided robot namespace %s already exists! Make sure they are unique.",
        robot_name.c_str());
      return CallbackReturn::ERROR;
    }
    auto & arm = arms_[robot_name];
    state_pointers_.insert(std::make_pair(robot_name, &arm.hw_franka_robot_state_));
    model_pointers_.insert(std::make_pair(robot_name, nullptr));
    arm.robot_name_ = robot_name;

    try {
      arm.robot_ip_ = info_.hardware_parameters.at("robot_ip" + suffix);
    } catch (const std::out_of_range & ex) {
      RCLCPP_FATAL(getLogger(), "Parameter 'robot_ip%s' ! set", suffix.c_str());
      return CallbackReturn::ERROR;
    }
    try {
      RCLCPP_INFO(getLogger(), "Connecting to robot at \"%s\" ...", arm.robot_ip_.c_str());
      arm.robot_ = std::make_unique<Robot>(arm.robot_ip_, getLogger());
    } catch (const franka::Exception & e) {
      RCLCPP_FATAL(getLogger(), "Could ! connect to robot");
      RCLCPP_FATAL(getLogger(), "%s", e.what());
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(
      getLogger(), "Successfully connected to robot %s at %s", robot_name.c_str(),
      arm.robot_ip_.c_str());
    arm.hw_franka_robot_state_ = arm.robot_->read();
    arm.hw_positions_ = arm.hw_franka_robot_state_.q;
    arm.hw_velocities_ = arm.hw_franka_robot_state_.dq;
    arm.hw_efforts_ = arm.hw_franka_robot_state_.tau_J;
    arm.hw_cartesian_positions_ = arm.hw_franka_robot_state_.O_T_EE;
    arm.hw_cartesian_velocities_ = arm.hw_franka_robot_state_.O_T_EE_d;
    // Start the service nodes
    arm.error_recovery_service_node_ = std::make_shared<FrankaErrorRecoveryServiceServer>(
      rclcpp::NodeOptions(), arm.robot_, robot_name + "_");
    arm.param_service_node_ = std::make_shared<FrankaParamServiceServer>(
      rclcpp::NodeOptions(), arm.robot_, robot_name + "_");

    executor_->add_node(arm.error_recovery_service_node_);
    executor_->add_node(arm.param_service_node_);

    initializeCommandsForMode(arm, CommandInitialization::None);
  }
  for (const auto & arm_entry : arms_) {
    for (size_t joint = 1; joint <= kNumberOfJoints; ++joint) {
      const auto expected_name = arm_entry.first + "_joint" + std::to_string(joint);
      const auto count = std::count_if(
        info_.joints.begin(), info_.joints.end(),
        [&expected_name](const auto & joint_info) { return joint_info.name == expected_name; });
      if (count != 1) {
        RCLCPP_FATAL(getLogger(), "Expected joint '%s' exactly once", expected_name.c_str());
        return CallbackReturn::ERROR;
      }
    }
  }
  RCLCPP_INFO(
    getLogger(), "All %ld robots have been intiialized (%ld)", robot_count_, arms_.size());
  if (arms_.size() != robot_count_) {
    RCLCPP_FATAL(
      getLogger(), "initialized arm container size %ld is not the same as robot_count %ld",
      arms_.size(), robot_count_);
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> FrankaMultiHardwareInterface::export_state_interfaces()
{
  std::vector<StateInterface> state_interfaces;
  for (auto i = 0U; i < info_.joints.size(); i++) {
    // std::cout << get_ns(info_.joints[i].name) << std::endl;
    state_interfaces.emplace_back(StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION,
      &arms_[get_ns(info_.joints[i].name)].hw_positions_.at(get_joint_no(info_.joints[i].name))));
    state_interfaces.emplace_back(StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
      &arms_[get_ns(info_.joints[i].name)].hw_velocities_.at(get_joint_no(info_.joints[i].name))));
    state_interfaces.emplace_back(StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
      &arms_[get_ns(info_.joints[i].name)].hw_efforts_.at(get_joint_no(info_.joints[i].name))));
  }

  for (auto & arm_container_pair : arms_) {
    auto & arm = arm_container_pair.second;

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
      reinterpret_cast<double *>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        &state_pointers_[arm_container_pair.first])));
    state_interfaces.emplace_back(StateInterface(
      arm.robot_name_, k_robot_model_interface_name,
      reinterpret_cast<double *>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        &model_pointers_[arm_container_pair.first])));
  }

  return state_interfaces;
}

std::vector<CommandInterface> FrankaMultiHardwareInterface::export_command_interfaces()
{
  std::vector<CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());
  // RCLCPP_INFO(getLogger(), "%ld", info_.joints.size());

  for (auto i = 0U; i < info_.joints.size(); i++) {
    // RCLCPP_INFO(getLogger(), "%s", info_.joints[i].name.c_str());

    command_interfaces.emplace_back(CommandInterface(  // JOINT EFFORT
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
      &arms_[get_ns(info_.joints[i].name)].hw_commands_joint_effort_.at(
        get_joint_no(info_.joints[i].name))));
    command_interfaces.emplace_back(CommandInterface(  // JOINT POSITION
      info_.joints[i].name, hardware_interface::HW_IF_POSITION,
      &arms_[get_ns(info_.joints[i].name)].hw_commands_joint_position_.at(
        get_joint_no(info_.joints[i].name))));
    command_interfaces.emplace_back(CommandInterface(  // JOINT VELOCITY
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
      &arms_[get_ns(info_.joints[i].name)].hw_commands_joint_velocity_.at(
        get_joint_no(info_.joints[i].name))));
  }

  for (auto & arm_container_pair : arms_) {
    auto & arm = arm_container_pair.second;
    std::string cartesian_position_prefix = arm.robot_name_ + "_ee_cartesian_position";
    std::string cartesian_velocity_prefix = arm.robot_name_ + "_ee_cartesian_velocity";

    for (auto i = 0; i < 16; i++) {
      command_interfaces.emplace_back(CommandInterface(
        cartesian_position_prefix, cartesian_matrix_names[i],
        &arm.hw_commands_cartesian_position_[i]));
    }
    for (auto i = 0; i < 6; i++) {
      command_interfaces.emplace_back(CommandInterface(
        cartesian_velocity_prefix, cartesian_velocity_command_names[i],
        &arm.hw_commands_cartesian_velocity_[i]));
    }
  }

  return command_interfaces;
}

CallbackReturn FrankaMultiHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  prepared_plan_valid_ = false;
  for (auto & arm_container_pair : arms_) {
    auto & arm = arm_container_pair.second;
    arm.control_mode_ = ControlMode::None;
    arm.pending_control_mode_ = ControlMode::None;
    arm.has_pending_control_mode_ = false;
    initializeCommandsForMode(arm, CommandInitialization::None);
    if (!arm.robot_->canWriteCommand() || !publishCommands(arm)) {
      RCLCPP_ERROR(
        getLogger(), "Could not publish safe initial commands for arm '%s'",
        arm.robot_name_.c_str());
      return CallbackReturn::ERROR;
    }
  }
  for (auto & arm_container_pair : arms_) {
    auto & arm = arm_container_pair.second;
    if (!arm.robot_->initializeContinuousReading()) {
      for (auto & cleanup_entry : arms_) {
        cleanup_entry.second.robot_->stopRobot();
      }
      RCLCPP_ERROR(
        getLogger(), "Could not start state reading for arm '%s'", arm.robot_name_.c_str());
      return CallbackReturn::ERROR;
    }
  }
  if (read(rclcpp::Time(0), rclcpp::Duration(0, 0)) != hardware_interface::return_type::OK) {
    for (auto & cleanup_entry : arms_) {
      cleanup_entry.second.robot_->stopRobot();
    }
    RCLCPP_ERROR(getLogger(), "Robot state initialization failed during activation");
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(getLogger(), "Started");
  return CallbackReturn::SUCCESS;
}

CallbackReturn FrankaMultiHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(getLogger(), "trying to Stop...");
  prepared_plan_valid_ = false;
  bool stopped = true;
  for (auto & arm_container_pair : arms_) {
    auto & arm = arm_container_pair.second;
    arm.control_mode_ = ControlMode::None;
    arm.pending_control_mode_ = ControlMode::None;
    arm.has_pending_control_mode_ = false;
    stopped = arm.robot_->stopRobot() && stopped;
  }
  RCLCPP_INFO(getLogger(), "Stopped");
  return stopped ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

hardware_interface::return_type FrankaMultiHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  bool any_robot_has_error = false;
  for (auto & arm_container_pair : arms_) {
    auto & arm = arm_container_pair.second;
    if (model_pointers_[arm_container_pair.first] == nullptr) {
      model_pointers_[arm_container_pair.first] = arm.robot_->getModel();
    }
    arm.hw_franka_robot_state_ = arm.robot_->read();
    // so this works as expected...
    // auto cor = arm.hw_franka_model_ptr_->coriolis(arm.hw_franka_robot_state_);
    // std::cout << cor[0] << " "
    //           << cor[1] << " "
    //           << cor[2] << " "
    //           << cor[3] << " "
    //           << cor[4] << " "
    //           << cor[5] << " "
    //           << cor[6] << " "
    //   << std::endl;

    arm.hw_positions_ = arm.hw_franka_robot_state_.q;
    arm.hw_velocities_ = arm.hw_franka_robot_state_.dq;
    arm.hw_efforts_ = arm.hw_franka_robot_state_.tau_J;
    arm.hw_cartesian_positions_ = arm.hw_franka_robot_state_.O_T_EE;
    arm.hw_cartesian_velocities_ = arm.hw_franka_robot_state_.O_T_EE_d;
    any_robot_has_error = arm.robot_->hasError() || any_robot_has_error;
  }

  return any_robot_has_error ? hardware_interface::return_type::ERROR
                             : hardware_interface::return_type::OK;
}

hardware_interface::return_type FrankaMultiHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  bool any_robot_has_error = false;
  bool any_command_is_invalid = false;
  for (auto & arm_container_pair : arms_) {
    auto & arm = arm_container_pair.second;
    if (std::any_of(
          arm.hw_commands_joint_effort_.begin(), arm.hw_commands_joint_effort_.end(),
          [](double c) { return !std::isfinite(c); })) {
      any_command_is_invalid = true;
    }
    if (std::any_of(
          arm.hw_commands_joint_position_.begin(), arm.hw_commands_joint_position_.end(),
          [](double c) { return !std::isfinite(c); })) {
      any_command_is_invalid = true;
    }
    if (std::any_of(
          arm.hw_commands_joint_velocity_.begin(), arm.hw_commands_joint_velocity_.end(),
          [](double c) { return !std::isfinite(c); })) {
      any_command_is_invalid = true;
    }
    if (std::any_of(
          arm.hw_commands_cartesian_position_.begin(), arm.hw_commands_cartesian_position_.end(),
          [](double c) { return !std::isfinite(c); })) {
      any_command_is_invalid = true;
    }
    if (std::any_of(
          arm.hw_commands_cartesian_velocity_.begin(), arm.hw_commands_cartesian_velocity_.end(),
          [](double c) { return !std::isfinite(c); })) {
      any_command_is_invalid = true;
    }
    any_robot_has_error = arm.robot_->hasError() || any_robot_has_error;
  }

  if (any_robot_has_error || any_command_is_invalid) {
    for (auto & arm_entry : arms_) {
      auto & arm = arm_entry.second;
      initializeCommandsForMode(arm, CommandInitialization::None);
      if (!arm.robot_->hasError() && arm.robot_->canWriteCommand()) {
        (void)publishCommands(arm);
      }
    }
    return hardware_interface::return_type::ERROR;
  }

  const bool all_arms_have_capacity = std::all_of(
    arms_.begin(), arms_.end(),
    [](const auto & arm_entry) { return arm_entry.second.robot_->canWriteCommand(); });
  if (!all_arms_have_capacity) {
    for (auto & arm_entry : arms_) {
      auto & arm = arm_entry.second;
      initializeCommandsForMode(arm, CommandInitialization::None);
      if (arm.robot_->canWriteCommand()) {
        (void)publishCommands(arm);
      }
    }
    return hardware_interface::return_type::ERROR;
  }
  for (auto & arm_entry : arms_) {
    if (!publishCommands(arm_entry.second)) {
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FrankaMultiHardwareInterface::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  prepared_plan_valid_ = false;
  std::vector<ArmCommandModeState> arm_states;
  arm_states.reserve(arms_.size());
  for (const auto & arm_entry : arms_) {
    arm_states.push_back({arm_entry.first, arm_entry.second.control_mode_});
  }

  const auto result =
    CommandModeSwitchPlanner::makePlan(arm_states, start_interfaces, stop_interfaces);
  if (!result) {
    RCLCPP_ERROR(getLogger(), "Cannot prepare command mode switch: %s", result.message.c_str());
    return hardware_interface::return_type::ERROR;
  }

  for (auto & arm_entry : arms_) {
    arm_entry.second.has_pending_control_mode_ = false;
  }
  for (const auto & request : result.plan->arms) {
    auto & arm = arms_.at(request.arm_name);
    arm.pending_control_mode_ = request.requested_mode;
    arm.pending_command_initialization_ = request.command_initialization;
    arm.has_pending_control_mode_ = request.has_request;
  }
  prepared_start_interfaces_ = start_interfaces;
  prepared_stop_interfaces_ = stop_interfaces;
  prepared_plan_valid_ = true;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FrankaMultiHardwareInterface::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  if (
    !prepared_plan_valid_ || start_interfaces != prepared_start_interfaces_ ||
    stop_interfaces != prepared_stop_interfaces_) {
    return hardware_interface::return_type::ERROR;
  }
  prepared_plan_valid_ = false;

  for (const auto & arm_entry : arms_) {
    const auto & arm = arm_entry.second;
    if (arm.has_pending_control_mode_ && !arm.robot_->canWriteCommand()) {
      return hardware_interface::return_type::ERROR;
    }
  }

  for (auto & arm_entry : arms_) {
    auto & arm = arm_entry.second;
    if (!arm.has_pending_control_mode_) {
      continue;
    }
    initializeCommandsForMode(arm, arm.pending_command_initialization_);
    if (!publishCommands(arm)) {
      return hardware_interface::return_type::ERROR;
    }
  }

  for (const auto & arm_entry : arms_) {
    const auto & arm = arm_entry.second;
    if (
      arm.has_pending_control_mode_ &&
      !arm.robot_->canRequestControlMode(arm.pending_control_mode_)) {
      return hardware_interface::return_type::ERROR;
    }
  }

  for (auto & arm_entry : arms_) {
    auto & arm = arm_entry.second;
    if (!arm.has_pending_control_mode_) {
      continue;
    }
    if (!arm.robot_->requestControlMode(arm.pending_control_mode_)) {
      for (auto & rollback_entry : arms_) {
        if (rollback_entry.second.has_pending_control_mode_) {
          rollback_entry.second.robot_->requestControlMode(ControlMode::None);
        }
      }
      return hardware_interface::return_type::ERROR;
    }
  }

  for (auto & arm_entry : arms_) {
    auto & arm = arm_entry.second;
    if (arm.has_pending_control_mode_) {
      arm.control_mode_ = arm.pending_control_mode_;
      arm.has_pending_control_mode_ = false;
    }
  }
  return hardware_interface::return_type::OK;
}

void FrankaMultiHardwareInterface::initializeCommandsForMode(
  ArmContainer & arm, CommandInitialization /*initialization*/)
{
  const auto command = makeSafeRobotCommand(arm.hw_franka_robot_state_);
  arm.hw_commands_joint_effort_ = command.efforts;
  arm.hw_commands_joint_position_ = command.joint_positions;
  arm.hw_commands_joint_velocity_ = command.joint_velocities;
  arm.hw_commands_cartesian_position_ = command.cartesian_positions;
  arm.hw_commands_cartesian_velocity_ = command.cartesian_velocities;
}

bool FrankaMultiHardwareInterface::publishCommands(ArmContainer & arm) noexcept
{
  return arm.robot_->write(
    arm.hw_commands_joint_effort_, arm.hw_commands_joint_position_, arm.hw_commands_joint_velocity_,
    arm.hw_commands_cartesian_position_, arm.hw_commands_cartesian_velocity_);
}

rclcpp::Logger FrankaMultiHardwareInterface::getLogger()
{
  return rclcpp::get_logger("FrankaMultiHardwareInterface");
}
}  // namespace franka_hardware

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(
  franka_hardware::FrankaMultiHardwareInterface, hardware_interface::SystemInterface)
