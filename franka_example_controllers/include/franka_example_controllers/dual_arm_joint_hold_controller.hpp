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
#include <cstddef>
#include <string>
#include <vector>

#include <franka/robot_state.h>
#include <controller_interface/controller_interface.hpp>
#include <franka_hardware/common/model_base.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

namespace franka_example_controllers {

class DualArmJointHoldController final : public controller_interface::ControllerInterface {
 public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  void release_interfaces() override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_error(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_shutdown(
      const rclcpp_lifecycle::State& previous_state) override;

 private:
  static constexpr size_t kArmCount = 2;
  static constexpr size_t kJointCount = 7;
  static constexpr double kVelocityFilterAlpha = 0.99;

  struct Arm {
    std::string arm_id;
    std::array<double, kJointCount> k_gains{};
    std::array<double, kJointCount> d_gains{};
    std::array<double, kJointCount> max_effort{};
    std::array<double, kJointCount> hold_position{};
    std::array<double, kJointCount> filtered_velocity{};
    std::array<hardware_interface::LoanedStateInterface*, kJointCount> position_interfaces{};
    std::array<hardware_interface::LoanedStateInterface*, kJointCount> velocity_interfaces{};
    std::array<hardware_interface::LoanedCommandInterface*, kJointCount> effort_interfaces{};
    hardware_interface::LoanedStateInterface* robot_state_interface{nullptr};
    hardware_interface::LoanedStateInterface* robot_model_interface{nullptr};
    franka::RobotState* robot_state{nullptr};
    franka_hardware::ModelBase* robot_model{nullptr};
  };

  bool bindInterfaces();
  bool bindArmInterfaces(Arm& arm);
  bool captureActivationState();
  bool computeCommands(
      std::array<std::array<double, kJointCount>, kArmCount>& efforts,
      std::array<std::array<double, kJointCount>, kArmCount>& next_filtered_velocity) const;
  bool writeCommands(
      const std::array<std::array<double, kJointCount>, kArmCount>& efforts) noexcept;
  bool writeZeroEffort() noexcept;
  bool attemptRequiredZero() noexcept;
  void resetBindings() noexcept;

  hardware_interface::LoanedStateInterface* findUniqueStateInterface(
      const std::string& name) noexcept;
  hardware_interface::LoanedCommandInterface* findUniqueCommandInterface(
      const std::string& name) noexcept;

  std::array<Arm, kArmCount> arms_{};
  size_t arm_count_{kArmCount};
  bool configured_{false};
  bool interfaces_bound_{false};
  bool zero_required_{false};
  bool release_zero_failed_{false};
  bool active_{false};
};

}  // namespace franka_example_controllers
