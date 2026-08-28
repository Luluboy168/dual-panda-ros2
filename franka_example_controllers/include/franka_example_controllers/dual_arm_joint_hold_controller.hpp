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
#include <atomic>
#include <cstddef>
#include <cstdint>
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
    // F-10c (design §3.3): bindArmInterfaces() now runs on the control-cycle owner thread, inside
    // update() -- see serviceFirstUpdateActivation() -- where an allocation (e.g. the std::string
    // concatenation jointInterfaceName() used to perform per joint, per activation) would violate
    // the RT wait-free/allocation-free contract. Precomputed once, on the service thread, in
    // on_configure() -- a lifecycle phase that provably cannot overlap update() for this instance,
    // the same precondition arm_id/k_gains/d_gains/max_effort above already rely on -- so the
    // owner thread's bind step only ever does string *comparisons* against these, no allocation.
    std::array<std::string, kJointCount> position_interface_names{};
    std::array<std::string, kJointCount> velocity_interface_names{};
    std::array<std::string, kJointCount> effort_interface_names{};
    std::string robot_state_interface_name;
    std::string robot_model_interface_name;
    std::array<hardware_interface::LoanedStateInterface*, kJointCount> position_interfaces{};
    std::array<hardware_interface::LoanedStateInterface*, kJointCount> velocity_interfaces{};
    std::array<hardware_interface::LoanedCommandInterface*, kJointCount> effort_interfaces{};
    hardware_interface::LoanedStateInterface* robot_state_interface{nullptr};
    hardware_interface::LoanedStateInterface* robot_model_interface{nullptr};
    franka::RobotState* robot_state{nullptr};
    franka_hardware::ModelBase* robot_model{nullptr};
  };

  // F-10c: on_activate() may run on controller_manager's service thread (Jazzy
  // activate_asap=false). Rather than binding interfaces and capturing the activation target
  // there -- which is what raced the owner thread's read()/write() in F-10a/F-10b -- on_activate()
  // only posts a request; bindInterfaces()/captureActivationState() then run on the control-cycle
  // owner thread's own first subsequent update() cycle, the same thread that owns arm.robot_state/
  // arm.robot_model and the effort command storage, so there is nothing left to race. See
  // update()/on_activate() in the .cpp for the full sequence.
  // This phase is the *sole* cross-thread signal for activation state: update() recomputes its
  // own function-local `active` from it, fresh, on every call, and no lifecycle callback holds or
  // writes an activation flag of its own -- doing so would just reintroduce the plain-bool race
  // this phase exists to remove.
  enum class RtActivationPhase : uint32_t {
    kIdle = 0,       // no activation in flight; update() must not compute/write real commands
    kRequested = 1,  // on_activate() asked update() to bind + capture on its next cycle
    kActive = 2,     // update() finished a successful first-cycle bind + capture + zero
    kFailed = 3,     // update() attempted the first-cycle bind + capture and it failed
  };

  bool validateInterfaceWiring() const noexcept;
  size_t countStateInterfaces(const std::string& name) const noexcept;
  size_t countCommandInterfaces(const std::string& name) const noexcept;
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
  void serviceFirstUpdateActivation() noexcept;

  hardware_interface::LoanedStateInterface* findUniqueStateInterface(
      const std::string& name) noexcept;
  hardware_interface::LoanedCommandInterface* findUniqueCommandInterface(
      const std::string& name) noexcept;

  std::array<Arm, kArmCount> arms_{};
  size_t arm_count_{kArmCount};
  bool configured_{false};
  bool interfaces_bound_{false};
  bool zero_required_{false};
  std::atomic<RtActivationPhase> rt_activation_phase_{RtActivationPhase::kIdle};
  // Owner-thread-written, lifecycle-thread-read: true from the moment bindInterfaces() last
  // succeeded until resetBindings() actually runs. on_configure() reads it (together with
  // rt_activation_phase_) to reject a reconfigure while this instance still holds bindings,
  // without reading the owner-thread-only interfaces_bound_ directly. See .cpp.
  std::atomic<bool> rt_ever_bound_{false};
};

}  // namespace franka_example_controllers
