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
#include <control_msgs/msg/joint_jog.hpp>
#include <cstddef>
#include <cstdint>
#include <hardware_interface/loaned_command_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <string>

#include "franka_example_controllers/dual_arm_joint_velocity_controller.hpp"

namespace franka_example_controllers {

constexpr size_t kVelocityArmCount = 2;
constexpr size_t kVelocityJointCount = 7;

enum class JointJogValidationResult {
  Accepted,
  InvalidFrame,
  InvalidStamp,
  HeaderTooOld,
  HeaderTooFarInFuture,
  InvalidDuration,
  DisplacementCommandNotAllowed,
  InvalidNameCount,
  InvalidVelocityCount,
  DuplicateOrUnknownJoint,
  NonfiniteVelocity,
  VelocityLimitExceeded,
};

struct VelocityCommandPolicy {
  std::array<std::string, kVelocityJointCount> joint_names{};
  std::array<double, kVelocityJointCount> max_velocity{};
  int64_t max_header_age_ns{0};
  int64_t future_tolerance_ns{0};
};

struct BufferedVelocityCommand {
  std::array<double, kVelocityJointCount> velocities{};
  int64_t steady_receive_ns{0};
  bool valid{false};
};

class ArmVelocityCommandInbox {
 public:
  void configure(const VelocityCommandPolicy& policy) noexcept;

  JointJogValidationResult accept(const control_msgs::msg::JointJog& message,
                                  int64_t ros_now_ns,
                                  int64_t steady_receive_ns) noexcept;

  void setEnabled(bool enabled, int64_t steady_now_ns);
  bool readFresh(int64_t steady_now_ns,
                 int64_t watchdog_ns,
                 std::array<double, kVelocityJointCount>& velocities) noexcept;

  bool enabled() const noexcept { return enabled_.load(std::memory_order_acquire); }
  const BufferedVelocityCommand& nonRealtimeCommand() const {
    return *command_buffer_.readFromNonRT();
  }

 private:
  void invalidate(int64_t steady_receive_ns);

  VelocityCommandPolicy policy_{};
  realtime_tools::RealtimeBuffer<BufferedVelocityCommand> command_buffer_{};
  std::atomic<bool> enabled_{false};
  std::atomic<int64_t> enabled_since_ns_{0};
};

class DualArmJointVelocityControllerCore {
 public:
  using SteadyClock = std::chrono::steady_clock;

  controller_interface::InterfaceConfiguration commandInterfaceConfiguration() const;
  controller_interface::InterfaceConfiguration stateInterfaceConfiguration() const;
  controller_interface::return_type update(DualArmJointVelocityController& controller,
                                           const rclcpp::Duration& period) noexcept;

  controller_interface::CallbackReturn onInit(DualArmJointVelocityController& controller);
  controller_interface::CallbackReturn onConfigure(DualArmJointVelocityController& controller);
  controller_interface::CallbackReturn onActivate(DualArmJointVelocityController& controller);
  controller_interface::CallbackReturn onDeactivate(DualArmJointVelocityController& controller);
  controller_interface::CallbackReturn onCleanup();
  controller_interface::CallbackReturn onError(DualArmJointVelocityController& controller);
  controller_interface::CallbackReturn onShutdown(DualArmJointVelocityController& controller);
  void releaseInterfaces() noexcept;

  JointJogValidationResult acceptCommand(size_t arm,
                                         const control_msgs::msg::JointJog& message,
                                         int64_t ros_now_ns,
                                         int64_t steady_receive_ns) noexcept;
  void setArmEnabled(size_t arm, bool enabled, int64_t steady_now_ns);
  bool armEnabled(size_t arm) const noexcept;
  std::string subscriptionTopic(size_t arm) const;
  std::string enableServiceName(size_t arm) const;

 private:
  struct Arm {
    std::string arm_id;
    std::array<std::string, kVelocityJointCount> joint_names{};
    std::array<double, kVelocityJointCount> max_velocity{};
    std::array<double, kVelocityJointCount> max_acceleration{};
    std::array<double, kVelocityJointCount> last_output{};
    std::array<hardware_interface::LoanedCommandInterface*, kVelocityJointCount>
        velocity_interfaces{};
    ArmVelocityCommandInbox inbox{};
    rclcpp::Subscription<control_msgs::msg::JointJog>::SharedPtr subscription;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service;
  };

  bool bindInterfaces(DualArmJointVelocityController& controller) noexcept;
  hardware_interface::LoanedCommandInterface* findUniqueCommandInterface(
      DualArmJointVelocityController& controller,
      const std::string& name) noexcept;
  bool writeCommands(const std::array<std::array<double, kVelocityJointCount>, kVelocityArmCount>&
                         commands) noexcept;
  bool writeZeroAll() noexcept;
  bool attemptRequiredZero() noexcept;
  void resetBindings() noexcept;
  void disableAndInvalidateAll(int64_t steady_now_ns);

  std::array<Arm, kVelocityArmCount> arms_{};
  size_t arm_count_{kVelocityArmCount};
  int64_t watchdog_ns_{0};
  int64_t max_header_age_ns_{0};
  int64_t future_tolerance_ns_{0};
  bool configured_{false};
  bool interfaces_bound_{false};
  bool zero_required_{false};
  bool release_zero_failed_{false};
  bool active_{false};
};

int64_t steadyNowNanoseconds() noexcept;

}  // namespace franka_example_controllers
