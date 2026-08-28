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
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>

#include <franka/robot_state.h>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "franka_example_controllers/dual_arm_joint_impedance_controller.hpp"
#include "franka_example_controllers/panda_joint_limits.hpp"
#include "franka_hardware/common/model_base.hpp"

namespace franka_example_controllers {

constexpr size_t kImpedanceArmCount = 2;
constexpr size_t kImpedanceJointCount = kPandaJointCount;

enum class JointTargetValidationResult {
  Accepted,
  ControllerInactive,
  InvalidFrame,
  InvalidStamp,
  HeaderTooOld,
  HeaderTooFarInFuture,
  InvalidNameCount,
  InvalidPointCount,
  InvalidPositionCount,
  VelocityCommandNotAllowed,
  AccelerationCommandNotAllowed,
  EffortCommandNotAllowed,
  InvalidTimeFromStart,
  DuplicateOrUnknownJoint,
  NonfinitePosition,
  PositionLimitExceeded,
};

struct ImpedanceTargetPolicy {
  std::array<std::string, kImpedanceJointCount> joint_names{};
  std::array<double, kImpedanceJointCount> position_lower{};
  std::array<double, kImpedanceJointCount> position_upper{};
  int64_t max_header_age_ns{0};
  int64_t future_tolerance_ns{0};
};

struct BufferedImpedanceTarget {
  std::array<double, kImpedanceJointCount> positions{};
  int64_t header_ns{0};
  int64_t steady_receive_ns{0};
  bool valid{false};
};

class ArmImpedanceTargetInbox {
 public:
  void configure(const ImpedanceTargetPolicy& policy) noexcept;

  JointTargetValidationResult accept(const trajectory_msgs::msg::JointTrajectory& message,
                                     int64_t ros_now_ns,
                                     int64_t steady_receive_ns) noexcept;

  void setEnabled(bool enabled, int64_t steady_now_ns, int64_t ros_now_ns);
  bool readFresh(int64_t steady_now_ns,
                 int64_t watchdog_ns,
                 std::array<double, kImpedanceJointCount>& positions) noexcept;

  bool enabled() const noexcept { return enabled_.load(std::memory_order_acquire); }
  uint64_t enableGeneration() const noexcept {
    return enable_generation_.load(std::memory_order_acquire);
  }
  const BufferedImpedanceTarget& nonRealtimeTarget() const {
    return *target_buffer_.readFromNonRT();
  }

 private:
  friend class DualArmJointImpedanceControllerTestAccess;

  void invalidate(int64_t steady_receive_ns);

  ImpedanceTargetPolicy policy_{};
  realtime_tools::RealtimeBuffer<BufferedImpedanceTarget> target_buffer_{};
  std::atomic<bool> enabled_{false};
  std::atomic<int64_t> enabled_since_ns_{0};
  std::atomic<int64_t> enabled_ros_epoch_ns_{0};
  std::atomic<uint64_t> enable_generation_{0};
};

class DualArmJointImpedanceControllerCore {
 public:
  using SteadyClock = std::chrono::steady_clock;

  controller_interface::InterfaceConfiguration commandInterfaceConfiguration() const;
  controller_interface::InterfaceConfiguration stateInterfaceConfiguration() const;
  controller_interface::return_type update(DualArmJointImpedanceController& controller,
                                           const rclcpp::Duration& period) noexcept;

  controller_interface::CallbackReturn onInit(DualArmJointImpedanceController& controller);
  controller_interface::CallbackReturn onConfigure(DualArmJointImpedanceController& controller);
  controller_interface::CallbackReturn onActivate(DualArmJointImpedanceController& controller);
  controller_interface::CallbackReturn onDeactivate(DualArmJointImpedanceController& controller);
  controller_interface::CallbackReturn onCleanup(DualArmJointImpedanceController& controller);
  controller_interface::CallbackReturn onError(DualArmJointImpedanceController& controller);
  controller_interface::CallbackReturn onShutdown(DualArmJointImpedanceController& controller);
  void releaseInterfaces(DualArmJointImpedanceController& controller);

  JointTargetValidationResult acceptTarget(size_t arm,
                                           const trajectory_msgs::msg::JointTrajectory& message,
                                           int64_t ros_now_ns,
                                           int64_t steady_receive_ns);
  bool setArmEnabled(size_t arm, bool enabled, int64_t steady_now_ns, int64_t ros_now_ns);
  bool armEnabled(size_t arm) const noexcept;
  std::string subscriptionTopic(size_t arm) const;
  std::string enableServiceName(size_t arm) const;
  std::array<double, kImpedanceJointCount> internalTarget(size_t arm) const noexcept;

 private:
  friend class DualArmJointImpedanceControllerTestAccess;

  static constexpr double kVelocityFilterAlpha = 0.99;

  struct Arm {
    std::string arm_id;
    std::array<std::string, kImpedanceJointCount> joint_names{};
    std::array<double, kImpedanceJointCount> k_gains{};
    std::array<double, kImpedanceJointCount> d_gains{};
    std::array<double, kImpedanceJointCount> max_effort{};
    std::array<double, kImpedanceJointCount> position_lower{};
    std::array<double, kImpedanceJointCount> position_upper{};
    std::array<double, kImpedanceJointCount> max_target_velocity{};
    std::array<double, kImpedanceJointCount> internal_target{};
    std::array<double, kImpedanceJointCount> filtered_velocity{};
    uint64_t observed_enable_generation{0};
    // F-10c (design §3.3): precomputed once, on the service thread, in onConfigure() -- a
    // lifecycle phase that provably cannot overlap update() for this instance -- so the owner
    // thread's bindArmInterfaces() (called from update(), see serviceFirstUpdateActivation())
    // only ever does string *comparisons*, never an allocation.
    std::array<std::string, kImpedanceJointCount> position_interface_names{};
    std::array<std::string, kImpedanceJointCount> velocity_interface_names{};
    std::array<std::string, kImpedanceJointCount> effort_interface_names{};
    std::string robot_state_interface_name;
    std::string robot_model_interface_name;
    std::array<hardware_interface::LoanedStateInterface*, kImpedanceJointCount>
        position_interfaces{};
    std::array<hardware_interface::LoanedStateInterface*, kImpedanceJointCount>
        velocity_interfaces{};
    std::array<hardware_interface::LoanedCommandInterface*, kImpedanceJointCount>
        effort_interfaces{};
    hardware_interface::LoanedStateInterface* robot_state_interface{nullptr};
    hardware_interface::LoanedStateInterface* robot_model_interface{nullptr};
    franka::RobotState* robot_state{nullptr};
    franka_hardware::ModelBase* robot_model{nullptr};
    ArmImpedanceTargetInbox inbox{};
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr subscription;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service;
  };

  // F-10c: onActivate() may run on controller_manager's service thread (Jazzy activate_asap=
  // false). Rather than binding interfaces and capturing the activation target there -- which is
  // what raced the owner thread's read()/write() in F-10a/F-10b -- onActivate() only posts a
  // request; bindInterfaces()/captureActivationState() then run on the control-cycle owner
  // thread's own first subsequent update() cycle, the same thread that owns arm.robot_state/
  // arm.robot_model and the effort command storage, so there is nothing left to race. See
  // update()/onActivate() in the .cpp for the full sequence. This phase is the sole cross-thread
  // signal for RT activation state: it is otherwise touched only by update(), never by
  // onActivate()/onDeactivate()/onError()/onShutdown(), which would just reintroduce the race
  // this phase exists to remove.
  enum class RtActivationPhase : uint32_t {
    kIdle = 0,       // no activation in flight; update() must not compute/write real commands
    kRequested = 1,  // onActivate() asked update() to bind + capture on its next cycle
    kActive = 2,     // update() finished a successful first-cycle bind + capture + zero
    kFailed = 3,     // update() attempted the first-cycle bind + capture and it failed
  };

  bool bindInterfaces(DualArmJointImpedanceController& controller) noexcept;
  bool bindArmInterfaces(DualArmJointImpedanceController& controller, Arm& arm) noexcept;
  bool captureActivationState() noexcept;
  void serviceFirstUpdateActivation(DualArmJointImpedanceController& controller) noexcept;
  hardware_interface::LoanedStateInterface* findUniqueStateInterface(
      DualArmJointImpedanceController& controller,
      const std::string& name) noexcept;
  hardware_interface::LoanedCommandInterface* findUniqueCommandInterface(
      DualArmJointImpedanceController& controller,
      const std::string& name) noexcept;
  bool writeCommands(const std::array<std::array<double, kImpedanceJointCount>, kImpedanceArmCount>&
                         efforts) noexcept;
  bool writeZeroAll() noexcept;
  bool attemptRequiredZero() noexcept;
  bool validateInterfaceWiring(const DualArmJointImpedanceController& controller) const noexcept;
  void resetBindings() noexcept;
  void resetRosEndpoints() noexcept;
  void disableAndInvalidateAll(int64_t steady_now_ns, int64_t ros_now_ns);
  bool callbackEpochIsStableActive(uint64_t entry_epoch) const noexcept;

  std::array<Arm, kImpedanceArmCount> arms_{};
  size_t arm_count_{kImpedanceArmCount};
  int64_t watchdog_ns_{0};
  int64_t max_header_age_ns_{0};
  int64_t future_tolerance_ns_{0};
  mutable std::mutex non_rt_mutex_;
  // F-10c: the nonzero generation is published only by serviceFirstUpdateActivation(), on the
  // control-cycle owner thread, on a successful first-cycle activation. It is driven back to 0
  // both by the owner thread (update()'s inactive branch) and by every deactivation-class
  // lifecycle callback under non_rt_mutex_ -- an atomic store of a constant, not a command write,
  // so amendment A.1 does not touch it and the deactivation is visible to
  // acceptTarget()/setArmEnabled() (the ROS topic-callback thread, a third thread distinct from
  // both the service thread and the owner thread; see design §3.6) the moment the callback runs
  // rather than one control cycle later.
  uint64_t next_active_epoch_{0};
  std::atomic<uint64_t> active_epoch_{0};
  std::atomic<uint64_t> non_rt_callback_entries_{0};
  bool configured_{false};
  bool interfaces_bound_{false};
  bool zero_required_{false};
  std::atomic<RtActivationPhase> rt_activation_phase_{RtActivationPhase::kIdle};
  // Owner-thread-written, lifecycle-thread-read: true from the moment bindInterfaces() last
  // succeeded until resetBindings() actually runs. onConfigure() reads it together with
  // rt_activation_phase_ to reject a reconfigure while bindings are held or in flight, without
  // reading the owner-thread-only interfaces_bound_ directly.
  std::atomic<bool> rt_ever_bound_{false};
};

int64_t impedanceSteadyNowNanoseconds() noexcept;

}  // namespace franka_example_controllers
