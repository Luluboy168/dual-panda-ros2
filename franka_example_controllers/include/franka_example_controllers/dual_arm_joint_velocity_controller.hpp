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

#include <controller_interface/controller_interface.hpp>
#include <memory>
#include <rclcpp_lifecycle/state.hpp>

namespace franka_example_controllers {

class DualArmJointVelocityControllerCore;
class DualArmJointVelocityControllerTestAccess;

/// Bounded two-arm velocity controller using named, timestamped JointJog commands.
///
/// Each arm accepts only a complete seven-joint velocity-only JointJog on its private topic.
/// Header stamps are mandatory and checked at receipt; frame_id and duration must be empty/zero.
/// A steady-clock watchdog independently stops each arm. Enabling an arm discards every command
/// received before the enable request, so enabling alone can never create motion.
/// Disable, stale, missing, and invalid inputs synchronously select a zero target; the controller
/// writes that zero directly on the next update cycle as an explicit safety exception to the
/// configured acceleration ramp. The realtime update reads its inbox through Jazzy's nonblocking
/// try-lock path; it is not lock-free. Subscription and service callbacks never write command
/// interfaces.
class DualArmJointVelocityController final : public controller_interface::ControllerInterface {
 public:
  DualArmJointVelocityController();
  ~DualArmJointVelocityController() override;

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
  friend class DualArmJointVelocityControllerCore;
  friend class DualArmJointVelocityControllerTestAccess;

  std::unique_ptr<DualArmJointVelocityControllerCore> core_;
};

}  // namespace franka_example_controllers
