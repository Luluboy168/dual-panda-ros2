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

class DualArmJointImpedanceControllerCore;
class DualArmJointImpedanceControllerTestAccess;

/// Bounded two-arm impedance controller using named, timestamped JointTrajectory targets.
///
/// Activation captures measured joint positions and commands zero effort before becoming active.
/// Both arms start disabled, and enabling requires a target received after that request. Fresh
/// targets are rate-limited into an internal target; missing, invalid, or stale input freezes that
/// last rate-limited value. A disable transition is observed by update(), which samples measured
/// position once and freezes the new internal target. Callbacks never access loaned interfaces.
///
/// Joint velocity uses the same bounded fixed coefficient as the reviewed hold controller:
/// `filtered = 0.01 * previous + 0.99 * measured`. This keeps update allocation-free and adds
/// minimal single-pole smoothing with low lag; the independently checked absolute effort ceiling
/// remains the safety bound. The realtime command-buffer read uses Jazzy's nonblocking try-lock
/// path; it is bounded/nonblocking, not lock-free.
///
/// Lifecycle and ROS callbacks share one non-RT mutex and an active-epoch token. `update()` never
/// acquires that mutex, and ROS callbacks never access loaned interfaces. A disable published after
/// an update's final generation read can finish that in-flight cycle; the next update observes the
/// new generation, samples measured position once, and freezes there. Stable enable generations
/// are even and advance by two modulo uint64_t;
/// correctness assumes fewer than 2^63 enable transitions can occur between two RT observations,
/// which excludes generation ABA while retaining defined near-wrap behavior. The nonzero lifecycle
/// epoch similarly assumes a callback cannot remain queued across 2^64 - 1 subsequent successful
/// activations.
class DualArmJointImpedanceController final : public controller_interface::ControllerInterface {
 public:
  DualArmJointImpedanceController();
  ~DualArmJointImpedanceController() override;

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
  friend class DualArmJointImpedanceControllerCore;
  friend class DualArmJointImpedanceControllerTestAccess;

  std::unique_ptr<DualArmJointImpedanceControllerCore> core_;
};

}  // namespace franka_example_controllers
