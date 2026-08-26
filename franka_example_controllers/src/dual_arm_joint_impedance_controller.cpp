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

#include "franka_example_controllers/dual_arm_joint_impedance_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <exception>
#include <limits>
#include <memory>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <type_traits>
#include <utility>
#include <vector>

#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "dual_arm_joint_impedance_controller_core.hpp"

namespace {

constexpr size_t kStateInterfacesPerArmOverhead = 2;  // robot_state + robot_model
constexpr long double kNanosecondsPerSecond = 1000000000.0L;

bool hasOverriddenParameterWithPrefix(rclcpp_lifecycle::LifecycleNode& node,
                                      const std::string& prefix) {
  for (const auto& [name, value] :
       node.get_node_parameters_interface()->get_parameter_overrides()) {
    (void)value;
    if (name.rfind(prefix, 0) == 0) {
      return true;
    }
  }
  return false;
}

bool isAsciiIdentifier(const std::string& value) {
  if (value.empty() || value.size() > franka_example_controllers::kPandaArmIdMaxLength) {
    return false;
  }
  const auto first = static_cast<unsigned char>(value.front());
  if (!((first >= 'A' && first <= 'Z') || (first >= 'a' && first <= 'z'))) {
    return false;
  }
  return std::all_of(value.begin() + 1, value.end(), [](const char item) {
    const auto character = static_cast<unsigned char>(item);
    return (character >= 'A' && character <= 'Z') || (character >= 'a' && character <= 'z') ||
           (character >= '0' && character <= '9') || character == '_';
  });
}

bool positiveSecondsToNanoseconds(const double seconds, int64_t& nanoseconds) {
  if (!std::isfinite(seconds) || seconds <= 0.0) {
    return false;
  }
  const long double value = static_cast<long double>(seconds) * kNanosecondsPerSecond;
  if (value < 1.0L || value > static_cast<long double>(std::numeric_limits<int64_t>::max())) {
    return false;
  }
  nanoseconds = static_cast<int64_t>(value);
  return true;
}

bool hasCanonicalJointNames(const std::string& arm_id,
                            const std::vector<std::string>& joint_names) {
  if (joint_names.size() != franka_example_controllers::kImpedanceJointCount) {
    return false;
  }
  for (size_t joint = 0; joint < joint_names.size(); ++joint) {
    if (joint_names[joint] != arm_id + "_joint" + std::to_string(joint + 1)) {
      return false;
    }
  }
  return true;
}

bool validGains(const std::vector<double>& gains) {
  return gains.size() == franka_example_controllers::kImpedanceJointCount &&
         std::all_of(gains.begin(), gains.end(),
                     [](const double gain) { return std::isfinite(gain) && gain >= 0.0; });
}

bool validEffortBounds(const std::vector<double>& bounds) {
  if (bounds.size() != franka_example_controllers::kPandaAbsoluteEffortCeilings.size()) {
    return false;
  }
  for (size_t joint = 0; joint < bounds.size(); ++joint) {
    if (!std::isfinite(bounds[joint]) || bounds[joint] <= 0.0 ||
        bounds[joint] > franka_example_controllers::kPandaAbsoluteEffortCeilings[joint]) {
      return false;
    }
  }
  return true;
}

bool validPositionBounds(const std::vector<double>& lower, const std::vector<double>& upper) {
  if (lower.size() != franka_example_controllers::kImpedanceJointCount ||
      upper.size() != franka_example_controllers::kImpedanceJointCount) {
    return false;
  }
  for (size_t joint = 0; joint < lower.size(); ++joint) {
    if (!std::isfinite(lower[joint]) || !std::isfinite(upper[joint]) ||
        lower[joint] >= upper[joint] ||
        lower[joint] < franka_example_controllers::kPandaPositionLowerLimits[joint] ||
        upper[joint] > franka_example_controllers::kPandaPositionUpperLimits[joint]) {
      return false;
    }
  }
  return true;
}

bool validTargetVelocity(const std::vector<double>& max_target_velocity) {
  if (max_target_velocity.size() != franka_example_controllers::kImpedanceJointCount) {
    return false;
  }
  for (size_t joint = 0; joint < max_target_velocity.size(); ++joint) {
    if (!std::isfinite(max_target_velocity[joint]) || max_target_velocity[joint] <= 0.0 ||
        max_target_velocity[joint] >
            franka_example_controllers::kPandaAbsoluteJointVelocityCeilings[joint]) {
      return false;
    }
  }
  return true;
}

std::string jointInterfaceName(const std::string& joint_name, const char* interface_name) {
  return joint_name + "/" + interface_name;
}

template <typename Pointer>
Pointer decodePointer(const double encoded) noexcept {
  static_assert(std::is_pointer<Pointer>::value, "decoded value must be a pointer");
  static_assert(sizeof(Pointer) == sizeof(encoded), "pointer interface must fit in a double");
  Pointer pointer = nullptr;
  std::memcpy(&pointer, &encoded, sizeof(pointer));
  return pointer;
}

template <typename Pointer>
bool decodeStablePointer(const hardware_interface::LoanedStateInterface& interface,
                         Pointer& pointer) noexcept {
  try {
    const auto first_value = interface.get_optional<double>(1);
    const auto second_value = interface.get_optional<double>(1);
    if (!first_value || !second_value) {
      return false;
    }
    const auto first_pointer = decodePointer<Pointer>(*first_value);
    const auto second_pointer = decodePointer<Pointer>(*second_value);
    if (first_pointer == nullptr || first_pointer != second_pointer) {
      return false;
    }
    pointer = first_pointer;
    return true;
  } catch (...) {
    return false;
  }
}

template <typename Pointer>
bool pointerStillMatches(const hardware_interface::LoanedStateInterface* interface,
                         const Pointer expected) noexcept {
  if (interface == nullptr || expected == nullptr) {
    return false;
  }
  try {
    const auto value = interface->get_optional<double>(1);
    return value && decodePointer<Pointer>(*value) == expected;
  } catch (...) {
    return false;
  }
}

bool finiteModelInput(const franka::RobotState& state) {
  const auto finite_array = [](const auto& values) {
    return std::all_of(values.begin(), values.end(),
                       [](const double value) { return std::isfinite(value); });
  };
  return finite_array(state.q) && finite_array(state.dq) && finite_array(state.I_total) &&
         finite_array(state.F_x_Ctotal) && std::isfinite(state.m_total);
}

bool readFinite(const hardware_interface::LoanedStateInterface* interface, double& value) noexcept {
  if (interface == nullptr) {
    return false;
  }
  try {
    const auto result = interface->get_optional<double>(1);
    if (!result || !std::isfinite(*result)) {
      return false;
    }
    value = *result;
    return true;
  } catch (...) {
    return false;
  }
}

bool writeEffort(hardware_interface::LoanedCommandInterface* interface,
                 const double effort) noexcept {
  if (interface == nullptr) {
    return false;
  }
  try {
    return interface->set_value(effort, 1);
  } catch (...) {
    return false;
  }
}

}  // namespace

namespace franka_example_controllers {

int64_t impedanceSteadyNowNanoseconds() noexcept {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             DualArmJointImpedanceControllerCore::SteadyClock::now().time_since_epoch())
      .count();
}

void ArmImpedanceTargetInbox::configure(const ImpedanceTargetPolicy& policy) noexcept {
  policy_ = policy;
  enabled_.store(false, std::memory_order_release);
  enabled_since_ns_.store(0, std::memory_order_release);
  enabled_ros_epoch_ns_.store(0, std::memory_order_release);
  enable_generation_.store(0, std::memory_order_release);
  target_buffer_.initRT(BufferedImpedanceTarget{});
}

JointTargetValidationResult ArmImpedanceTargetInbox::accept(
    const trajectory_msgs::msg::JointTrajectory& message,
    const int64_t ros_now_ns,
    const int64_t steady_receive_ns) noexcept {
  auto reject = [&](const JointTargetValidationResult result) {
    invalidate(steady_receive_ns);
    return result;
  };

  if (!message.header.frame_id.empty()) {
    return reject(JointTargetValidationResult::InvalidFrame);
  }
  if (message.header.stamp.sec < 0 || message.header.stamp.nanosec >= 1000000000U ||
      (message.header.stamp.sec == 0 && message.header.stamp.nanosec == 0)) {
    return reject(JointTargetValidationResult::InvalidStamp);
  }
  const int64_t header_ns = static_cast<int64_t>(message.header.stamp.sec) * 1000000000LL +
                            static_cast<int64_t>(message.header.stamp.nanosec);
  const long double header_age_ns =
      static_cast<long double>(ros_now_ns) - static_cast<long double>(header_ns);
  if (header_age_ns > static_cast<long double>(policy_.max_header_age_ns)) {
    return reject(JointTargetValidationResult::HeaderTooOld);
  }
  if (-header_age_ns > static_cast<long double>(policy_.future_tolerance_ns)) {
    return reject(JointTargetValidationResult::HeaderTooFarInFuture);
  }
  if (message.joint_names.size() != kImpedanceJointCount) {
    return reject(JointTargetValidationResult::InvalidNameCount);
  }
  if (message.points.size() != 1U) {
    return reject(JointTargetValidationResult::InvalidPointCount);
  }

  const auto& point = message.points.front();
  if (point.positions.size() != kImpedanceJointCount) {
    return reject(JointTargetValidationResult::InvalidPositionCount);
  }
  if (!point.velocities.empty()) {
    return reject(JointTargetValidationResult::VelocityCommandNotAllowed);
  }
  if (!point.accelerations.empty()) {
    return reject(JointTargetValidationResult::AccelerationCommandNotAllowed);
  }
  if (!point.effort.empty()) {
    return reject(JointTargetValidationResult::EffortCommandNotAllowed);
  }
  if (point.time_from_start.sec != 0 || point.time_from_start.nanosec != 0U) {
    return reject(JointTargetValidationResult::InvalidTimeFromStart);
  }

  BufferedImpedanceTarget target;
  target.valid = true;
  target.header_ns = header_ns;
  target.steady_receive_ns = steady_receive_ns;
  std::array<bool, kImpedanceJointCount> matched{};
  for (size_t message_index = 0; message_index < kImpedanceJointCount; ++message_index) {
    size_t configured_index = kImpedanceJointCount;
    for (size_t joint = 0; joint < kImpedanceJointCount; ++joint) {
      if (message.joint_names[message_index] == policy_.joint_names[joint]) {
        configured_index = joint;
        break;
      }
    }
    if (configured_index == kImpedanceJointCount || matched[configured_index]) {
      return reject(JointTargetValidationResult::DuplicateOrUnknownJoint);
    }
    const double position = point.positions[message_index];
    if (!std::isfinite(position)) {
      return reject(JointTargetValidationResult::NonfinitePosition);
    }
    if (position < policy_.position_lower[configured_index] ||
        position > policy_.position_upper[configured_index]) {
      return reject(JointTargetValidationResult::PositionLimitExceeded);
    }
    matched[configured_index] = true;
    target.positions[configured_index] = position;
  }

  target_buffer_.writeFromNonRT(target);
  return JointTargetValidationResult::Accepted;
}

void ArmImpedanceTargetInbox::setEnabled(const bool enabled,
                                         const int64_t steady_now_ns,
                                         const int64_t ros_now_ns) {
  // Odd generations denote an in-progress callback update. The RT side accepts a command-state
  // snapshot only when the same even generation brackets its reads.
  enable_generation_.fetch_add(1, std::memory_order_acq_rel);
  enabled_.store(false, std::memory_order_release);
  invalidate(steady_now_ns);
  enabled_since_ns_.store(steady_now_ns, std::memory_order_release);
  enabled_ros_epoch_ns_.store(ros_now_ns, std::memory_order_release);
  enabled_.store(enabled, std::memory_order_release);
  enable_generation_.fetch_add(1, std::memory_order_release);
}

bool ArmImpedanceTargetInbox::readFresh(
    const int64_t steady_now_ns,
    const int64_t watchdog_ns,
    std::array<double, kImpedanceJointCount>& positions) noexcept {
  if (!enabled_.load(std::memory_order_acquire)) {
    return false;
  }
  const BufferedImpedanceTarget target = *target_buffer_.readFromRT();
  const int64_t enabled_since_ns = enabled_since_ns_.load(std::memory_order_acquire);
  const int64_t enabled_ros_epoch_ns = enabled_ros_epoch_ns_.load(std::memory_order_acquire);
  const long double receipt_age_ns =
      static_cast<long double>(steady_now_ns) - static_cast<long double>(target.steady_receive_ns);
  if (!target.valid || target.steady_receive_ns <= enabled_since_ns ||
      target.header_ns <= enabled_ros_epoch_ns || target.steady_receive_ns > steady_now_ns ||
      receipt_age_ns > static_cast<long double>(watchdog_ns)) {
    return false;
  }
  positions = target.positions;
  return true;
}

void ArmImpedanceTargetInbox::invalidate(const int64_t steady_receive_ns) {
  BufferedImpedanceTarget target;
  target.steady_receive_ns = steady_receive_ns;
  target_buffer_.writeFromNonRT(target);
}

controller_interface::InterfaceConfiguration
DualArmJointImpedanceControllerCore::commandInterfaceConfiguration() const {
  std::lock_guard<std::mutex> lock(non_rt_mutex_);
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  if (!configured_) {
    return configuration;
  }
  configuration.names.reserve(arm_count_ * kImpedanceJointCount);
  for (size_t arm = 0; arm < arm_count_; ++arm) {
    for (const auto& joint_name : arms_[arm].joint_names) {
      configuration.names.push_back(
          jointInterfaceName(joint_name, hardware_interface::HW_IF_EFFORT));
    }
  }
  return configuration;
}

controller_interface::InterfaceConfiguration
DualArmJointImpedanceControllerCore::stateInterfaceConfiguration() const {
  std::lock_guard<std::mutex> lock(non_rt_mutex_);
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  if (!configured_) {
    return configuration;
  }
  configuration.names.reserve(arm_count_ *
                              (2 * kImpedanceJointCount + kStateInterfacesPerArmOverhead));
  for (size_t arm = 0; arm < arm_count_; ++arm) {
    for (const auto& joint_name : arms_[arm].joint_names) {
      configuration.names.push_back(
          jointInterfaceName(joint_name, hardware_interface::HW_IF_POSITION));
      configuration.names.push_back(
          jointInterfaceName(joint_name, hardware_interface::HW_IF_VELOCITY));
    }
    configuration.names.push_back(arms_[arm].arm_id + "/robot_state");
    configuration.names.push_back(arms_[arm].arm_id + "/robot_model");
  }
  return configuration;
}

controller_interface::return_type DualArmJointImpedanceControllerCore::update(
    DualArmJointImpedanceController& /*controller*/,
    const rclcpp::Duration& period) noexcept {
  if (active_epoch_.load(std::memory_order_acquire) == 0 || !interfaces_bound_) {
    if (interfaces_bound_) {
      attemptRequiredZero();
    }
    return controller_interface::return_type::ERROR;
  }

  const double period_seconds = period.seconds();
  if (!std::isfinite(period_seconds) || period_seconds <= 0.0) {
    attemptRequiredZero();
    return controller_interface::return_type::ERROR;
  }

  const int64_t steady_now_ns = impedanceSteadyNowNanoseconds();
  std::array<std::array<double, kImpedanceJointCount>, kImpedanceArmCount> efforts{};
  std::array<std::array<double, kImpedanceJointCount>, kImpedanceArmCount> next_targets{};
  std::array<std::array<double, kImpedanceJointCount>, kImpedanceArmCount> next_filtered_velocity{};
  std::array<uint64_t, kImpedanceArmCount> next_enable_generation{};

  for (size_t arm_index = 0; arm_index < arm_count_; ++arm_index) {
    auto& arm = arms_[arm_index];
    if (arm.robot_state == nullptr || arm.robot_model == nullptr ||
        !pointerStillMatches(arm.robot_state_interface, arm.robot_state) ||
        !pointerStillMatches(arm.robot_model_interface, arm.robot_model) ||
        !finiteModelInput(*arm.robot_state)) {
      attemptRequiredZero();
      return controller_interface::return_type::ERROR;
    }

    std::array<double, kImpedanceJointCount> positions{};
    std::array<double, kImpedanceJointCount> velocities{};
    for (size_t joint = 0; joint < kImpedanceJointCount; ++joint) {
      if (!readFinite(arm.position_interfaces[joint], positions[joint]) ||
          !readFinite(arm.velocity_interfaces[joint], velocities[joint])) {
        attemptRequiredZero();
        return controller_interface::return_type::ERROR;
      }
      next_filtered_velocity[arm_index][joint] =
          (1.0 - kVelocityFilterAlpha) * arm.filtered_velocity[joint] +
          kVelocityFilterAlpha * velocities[joint];
      if (!std::isfinite(next_filtered_velocity[arm_index][joint])) {
        attemptRequiredZero();
        return controller_interface::return_type::ERROR;
      }
    }

    next_targets[arm_index] = arm.internal_target;
    const uint64_t enable_generation_before = arm.inbox.enableGeneration();
    next_enable_generation[arm_index] = enable_generation_before;
    const bool enable_snapshot_stable = (enable_generation_before & 1U) == 0U;
    const bool enabled = enable_snapshot_stable && arm.inbox.enabled();
    if (!enable_snapshot_stable || enable_generation_before != arm.observed_enable_generation) {
      // Every enable/disable transition is consumed here in RT. This also catches a rapid
      // disable-then-enable pair between cycles and prevents retaining its older internal target.
      next_targets[arm_index] = positions;
    }

    std::array<double, kImpedanceJointCount> endpoint{};
    const bool fresh_target = enabled && arm.inbox.readFresh(steady_now_ns, watchdog_ns_, endpoint);
    const uint64_t enable_generation_after = arm.inbox.enableGeneration();
    const bool enable_snapshot_unchanged =
        enable_snapshot_stable && enable_generation_after == enable_generation_before;
    if (!enable_snapshot_unchanged) {
      // A callback overlapped this snapshot. Treat it conservatively as a transition and defer
      // any new endpoint until update observes a completed even generation.
      next_enable_generation[arm_index] = enable_generation_after;
      next_targets[arm_index] = positions;
    } else if (fresh_target) {
      for (size_t joint = 0; joint < kImpedanceJointCount; ++joint) {
        if (!std::isfinite(endpoint[joint]) || endpoint[joint] < arm.position_lower[joint] ||
            endpoint[joint] > arm.position_upper[joint]) {
          attemptRequiredZero();
          return controller_interface::return_type::ERROR;
        }
        const long double delta = static_cast<long double>(endpoint[joint]) -
                                  static_cast<long double>(next_targets[arm_index][joint]);
        const long double max_delta = static_cast<long double>(arm.max_target_velocity[joint]) *
                                      static_cast<long double>(period_seconds);
        long double limited_delta = delta;
        if (limited_delta > max_delta) {
          limited_delta = max_delta;
        } else if (limited_delta < -max_delta) {
          limited_delta = -max_delta;
        }
        const long double target =
            static_cast<long double>(next_targets[arm_index][joint]) + limited_delta;
        if (!std::isfinite(target)) {
          attemptRequiredZero();
          return controller_interface::return_type::ERROR;
        }
        next_targets[arm_index][joint] = static_cast<double>(target);
      }
    }

    std::array<double, kImpedanceJointCount> coriolis{};
    try {
      coriolis = arm.robot_model->coriolis(*arm.robot_state);
    } catch (...) {
      attemptRequiredZero();
      return controller_interface::return_type::ERROR;
    }
    for (size_t joint = 0; joint < kImpedanceJointCount; ++joint) {
      if (!std::isfinite(next_targets[arm_index][joint]) || !std::isfinite(coriolis[joint])) {
        attemptRequiredZero();
        return controller_interface::return_type::ERROR;
      }
      efforts[arm_index][joint] =
          arm.k_gains[joint] * (next_targets[arm_index][joint] - positions[joint]) -
          arm.d_gains[joint] * next_filtered_velocity[arm_index][joint] + coriolis[joint];
      if (!std::isfinite(efforts[arm_index][joint]) ||
          std::abs(efforts[arm_index][joint]) > arm.max_effort[joint]) {
        attemptRequiredZero();
        return controller_interface::return_type::ERROR;
      }
    }
  }

  zero_required_ = true;
  if (!writeCommands(efforts)) {
    attemptRequiredZero();
    return controller_interface::return_type::ERROR;
  }
  for (size_t arm = 0; arm < arm_count_; ++arm) {
    arms_[arm].internal_target = next_targets[arm];
    arms_[arm].filtered_velocity = next_filtered_velocity[arm];
    arms_[arm].observed_enable_generation = next_enable_generation[arm];
  }
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn DualArmJointImpedanceControllerCore::onInit(
    DualArmJointImpedanceController& controller) {
  std::lock_guard<std::mutex> lock(non_rt_mutex_);
  try {
    controller.auto_declare<int64_t>("arm_count", static_cast<int64_t>(kImpedanceArmCount));
    for (size_t arm = 0; arm < kImpedanceArmCount; ++arm) {
      const auto prefix = "arm_" + std::to_string(arm + 1) + ".";
      controller.auto_declare<std::string>(prefix + "arm_id", "");
      controller.auto_declare<std::vector<std::string>>(prefix + "joint_names", {});
      controller.auto_declare<std::vector<double>>(prefix + "k_gains", {});
      controller.auto_declare<std::vector<double>>(prefix + "d_gains", {});
      controller.auto_declare<std::vector<double>>(prefix + "max_effort", {});
      controller.auto_declare<std::vector<double>>(prefix + "position_lower", {});
      controller.auto_declare<std::vector<double>>(prefix + "position_upper", {});
      controller.auto_declare<std::vector<double>>(prefix + "max_target_velocity", {});
    }
    controller.auto_declare<double>("watchdog_timeout", 0.0);
    controller.auto_declare<double>("max_header_age", 0.0);
    controller.auto_declare<double>("future_tolerance", 0.0);
  } catch (const std::exception& error) {
    RCLCPP_ERROR(controller.get_node()->get_logger(),
                 "Failed to declare impedance-controller parameters: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DualArmJointImpedanceControllerCore::onConfigure(
    DualArmJointImpedanceController& controller) {
  std::lock_guard<std::mutex> lock(non_rt_mutex_);
  beginNonRealtimeTransition(NonRealtimePhase::Unconfigured);
  configured_ = false;
  if (interfaces_bound_) {
    return controller_interface::CallbackReturn::ERROR;
  }
  resetRosEndpoints();

  try {
    const auto node = controller.get_node();
    if (!positiveSecondsToNanoseconds(node->get_parameter("watchdog_timeout").as_double(),
                                      watchdog_ns_) ||
        !positiveSecondsToNanoseconds(node->get_parameter("max_header_age").as_double(),
                                      max_header_age_ns_) ||
        !positiveSecondsToNanoseconds(node->get_parameter("future_tolerance").as_double(),
                                      future_tolerance_ns_)) {
      RCLCPP_ERROR(node->get_logger(),
                   "watchdog_timeout, max_header_age, and future_tolerance must be finite, "
                   "positive, representable seconds");
      return controller_interface::CallbackReturn::FAILURE;
    }

    const int64_t requested_arm_count = node->get_parameter("arm_count").as_int();
    if (requested_arm_count != 1 &&
        static_cast<size_t>(requested_arm_count) != kImpedanceArmCount) {
      RCLCPP_ERROR(node->get_logger(), "arm_count must be exactly 1 or %zu, got %ld",
                   kImpedanceArmCount, static_cast<long>(requested_arm_count));
      return controller_interface::CallbackReturn::FAILURE;
    }
    const size_t arm_count = static_cast<size_t>(requested_arm_count);
    if (arm_count < kImpedanceArmCount && hasOverriddenParameterWithPrefix(*node, "arm_2.")) {
      RCLCPP_ERROR(node->get_logger(), "arm_2.* parameters must not be set when arm_count is 1");
      return controller_interface::CallbackReturn::FAILURE;
    }

    for (size_t arm_index = 0; arm_index < arm_count; ++arm_index) {
      auto& arm = arms_[arm_index];
      const auto prefix = "arm_" + std::to_string(arm_index + 1) + ".";
      const auto arm_id = node->get_parameter(prefix + "arm_id").as_string();
      const auto joint_names = node->get_parameter(prefix + "joint_names").as_string_array();
      const auto k_gains = node->get_parameter(prefix + "k_gains").as_double_array();
      const auto d_gains = node->get_parameter(prefix + "d_gains").as_double_array();
      const auto max_effort = node->get_parameter(prefix + "max_effort").as_double_array();
      const auto position_lower = node->get_parameter(prefix + "position_lower").as_double_array();
      const auto position_upper = node->get_parameter(prefix + "position_upper").as_double_array();
      const auto max_target_velocity =
          node->get_parameter(prefix + "max_target_velocity").as_double_array();

      if (!isAsciiIdentifier(arm_id)) {
        RCLCPP_ERROR(node->get_logger(),
                     "%sarm_id must contain 1..64 ASCII characters, start with a letter, and "
                     "then contain only letters, digits, or '_'",
                     prefix.c_str());
        return controller_interface::CallbackReturn::FAILURE;
      }
      if (!hasCanonicalJointNames(arm_id, joint_names)) {
        RCLCPP_ERROR(node->get_logger(),
                     "%sjoint_names must exactly equal arm_id + '_joint1' through "
                     "arm_id + '_joint7' in that order",
                     prefix.c_str());
        return controller_interface::CallbackReturn::FAILURE;
      }
      if (!validGains(k_gains) || !validGains(d_gains)) {
        RCLCPP_ERROR(node->get_logger(),
                     "%sk_gains and d_gains must each contain seven finite nonnegative values",
                     prefix.c_str());
        return controller_interface::CallbackReturn::FAILURE;
      }
      if (!validEffortBounds(max_effort)) {
        RCLCPP_ERROR(node->get_logger(),
                     "%smax_effort must contain seven finite positive values no greater than "
                     "[87, 87, 87, 87, 12, 12, 12]",
                     prefix.c_str());
        return controller_interface::CallbackReturn::FAILURE;
      }
      if (!validPositionBounds(position_lower, position_upper)) {
        RCLCPP_ERROR(node->get_logger(),
                     "%sposition bounds must be seven finite ordered pairs no looser than the "
                     "canonical Panda limits",
                     prefix.c_str());
        return controller_interface::CallbackReturn::FAILURE;
      }
      if (!validTargetVelocity(max_target_velocity)) {
        RCLCPP_ERROR(node->get_logger(),
                     "%smax_target_velocity must contain seven finite positive values no greater "
                     "than the canonical Panda velocity ceilings",
                     prefix.c_str());
        return controller_interface::CallbackReturn::FAILURE;
      }

      arm.arm_id = arm_id;
      std::copy(joint_names.begin(), joint_names.end(), arm.joint_names.begin());
      std::copy(k_gains.begin(), k_gains.end(), arm.k_gains.begin());
      std::copy(d_gains.begin(), d_gains.end(), arm.d_gains.begin());
      std::copy(max_effort.begin(), max_effort.end(), arm.max_effort.begin());
      std::copy(position_lower.begin(), position_lower.end(), arm.position_lower.begin());
      std::copy(position_upper.begin(), position_upper.end(), arm.position_upper.begin());
      std::copy(max_target_velocity.begin(), max_target_velocity.end(),
                arm.max_target_velocity.begin());
    }

    if (arm_count == kImpedanceArmCount && arms_[0].arm_id == arms_[1].arm_id) {
      RCLCPP_ERROR(node->get_logger(), "The two impedance-controller arm IDs must be unique");
      return controller_interface::CallbackReturn::FAILURE;
    }

    for (size_t arm_index = 0; arm_index < arm_count; ++arm_index) {
      auto& arm = arms_[arm_index];
      ImpedanceTargetPolicy policy;
      policy.joint_names = arm.joint_names;
      policy.position_lower = arm.position_lower;
      policy.position_upper = arm.position_upper;
      policy.max_header_age_ns = max_header_age_ns_;
      policy.future_tolerance_ns = future_tolerance_ns_;
      arm.inbox.configure(policy);

      const auto topic = "~/arm_" + std::to_string(arm_index + 1) + "/joint_target";
      const auto clock = node->get_clock();
      arm.subscription = node->create_subscription<trajectory_msgs::msg::JointTrajectory>(
          topic, rclcpp::QoS(1).reliable().durability_volatile(),
          [this, arm_index, clock](const trajectory_msgs::msg::JointTrajectory::SharedPtr message) {
            acceptTarget(arm_index, *message, clock->now().nanoseconds(),
                         impedanceSteadyNowNanoseconds());
          });
      const auto service_name = "~/arm_" + std::to_string(arm_index + 1) + "/enable";
      arm.enable_service = node->create_service<std_srvs::srv::SetBool>(
          service_name,
          [this, arm_index, clock](const std_srvs::srv::SetBool::Request::SharedPtr request,
                                   std_srvs::srv::SetBool::Response::SharedPtr response) {
            response->success =
                setArmEnabled(arm_index, request->data, impedanceSteadyNowNanoseconds(),
                              clock->now().nanoseconds());
            if (!response->success) {
              response->message = "rejected because the controller is not stably active";
            } else {
              response->message =
                  request->data
                      ? "enabled; measured target retained while awaiting a fresh valid target"
                      : "disabled; measured position sampled on the next update";
            }
          });
    }
    arm_count_ = arm_count;
  } catch (const std::exception& error) {
    RCLCPP_ERROR(controller.get_node()->get_logger(),
                 "Failed to configure dual-arm impedance controller: %s", error.what());
    return controller_interface::CallbackReturn::FAILURE;
  }

  configured_ = true;
  non_rt_phase_ = NonRealtimePhase::Inactive;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DualArmJointImpedanceControllerCore::onActivate(
    DualArmJointImpedanceController& controller) {
  std::lock_guard<std::mutex> lock(non_rt_mutex_);
  beginNonRealtimeTransition(NonRealtimePhase::Activating);
  disableAndInvalidateAll(impedanceSteadyNowNanoseconds(),
                          controller.get_node()->get_clock()->now().nanoseconds());
  if (!configured_ || release_zero_failed_ || !bindInterfaces(controller)) {
    non_rt_phase_ = NonRealtimePhase::Inactive;
    return controller_interface::CallbackReturn::FAILURE;
  }
  const bool activation_state_valid = captureActivationState();
  const bool zeroed = attemptRequiredZero();
  if (!activation_state_valid || !zeroed) {
    non_rt_phase_ = NonRealtimePhase::Inactive;
    return controller_interface::CallbackReturn::FAILURE;
  }
  zero_required_ = true;
  publishStableActiveEpoch();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DualArmJointImpedanceControllerCore::onDeactivate(
    DualArmJointImpedanceController& controller) {
  std::lock_guard<std::mutex> lock(non_rt_mutex_);
  beginNonRealtimeTransition(NonRealtimePhase::Inactive);
  disableAndInvalidateAll(impedanceSteadyNowNanoseconds(),
                          controller.get_node()->get_clock()->now().nanoseconds());
  const bool zeroed = attemptRequiredZero();
  return zeroed ? controller_interface::CallbackReturn::SUCCESS
                : controller_interface::CallbackReturn::ERROR;
}

controller_interface::CallbackReturn DualArmJointImpedanceControllerCore::onCleanup(
    DualArmJointImpedanceController& controller) {
  std::lock_guard<std::mutex> lock(non_rt_mutex_);
  beginNonRealtimeTransition(NonRealtimePhase::Unconfigured);
  configured_ = false;
  disableAndInvalidateAll(impedanceSteadyNowNanoseconds(),
                          controller.get_node()->get_clock()->now().nanoseconds());
  const bool zeroed = attemptRequiredZero();
  resetRosEndpoints();
  if (!zeroed) {
    return controller_interface::CallbackReturn::ERROR;
  }
  release_zero_failed_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DualArmJointImpedanceControllerCore::onError(
    DualArmJointImpedanceController& controller) {
  std::lock_guard<std::mutex> lock(non_rt_mutex_);
  beginNonRealtimeTransition(NonRealtimePhase::Unconfigured);
  configured_ = false;
  disableAndInvalidateAll(impedanceSteadyNowNanoseconds(),
                          controller.get_node()->get_clock()->now().nanoseconds());
  const bool zeroed = attemptRequiredZero();
  resetRosEndpoints();
  return zeroed ? controller_interface::CallbackReturn::SUCCESS
                : controller_interface::CallbackReturn::ERROR;
}

controller_interface::CallbackReturn DualArmJointImpedanceControllerCore::onShutdown(
    DualArmJointImpedanceController& controller) {
  std::lock_guard<std::mutex> lock(non_rt_mutex_);
  beginNonRealtimeTransition(NonRealtimePhase::Unconfigured);
  configured_ = false;
  disableAndInvalidateAll(impedanceSteadyNowNanoseconds(),
                          controller.get_node()->get_clock()->now().nanoseconds());
  const bool zeroed = attemptRequiredZero();
  resetRosEndpoints();
  if (!zeroed) {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

JointTargetValidationResult DualArmJointImpedanceControllerCore::acceptTarget(
    const size_t arm,
    const trajectory_msgs::msg::JointTrajectory& message,
    const int64_t ros_now_ns,
    const int64_t steady_receive_ns) {
  const uint64_t entry_epoch = active_epoch_.load(std::memory_order_acquire);
  non_rt_callback_entries_.fetch_add(1, std::memory_order_acq_rel);
  std::lock_guard<std::mutex> lock(non_rt_mutex_);
  if (!callbackEpochIsStableActive(entry_epoch)) {
    return JointTargetValidationResult::ControllerInactive;
  }
  if (arm >= kImpedanceArmCount) {
    return JointTargetValidationResult::DuplicateOrUnknownJoint;
  }
  return arms_[arm].inbox.accept(message, ros_now_ns, steady_receive_ns);
}

bool DualArmJointImpedanceControllerCore::setArmEnabled(const size_t arm,
                                                        const bool enabled,
                                                        const int64_t steady_now_ns,
                                                        const int64_t ros_now_ns) {
  const uint64_t entry_epoch = active_epoch_.load(std::memory_order_acquire);
  non_rt_callback_entries_.fetch_add(1, std::memory_order_acq_rel);
  std::lock_guard<std::mutex> lock(non_rt_mutex_);
  if (!callbackEpochIsStableActive(entry_epoch) || arm >= kImpedanceArmCount) {
    return false;
  }
  arms_[arm].inbox.setEnabled(enabled, steady_now_ns, ros_now_ns);
  return true;
}

bool DualArmJointImpedanceControllerCore::armEnabled(const size_t arm) const noexcept {
  return arm < kImpedanceArmCount && arms_[arm].inbox.enabled();
}

std::string DualArmJointImpedanceControllerCore::subscriptionTopic(const size_t arm) const {
  std::lock_guard<std::mutex> lock(non_rt_mutex_);
  return arm < kImpedanceArmCount && arms_[arm].subscription
             ? arms_[arm].subscription->get_topic_name()
             : std::string{};
}

std::string DualArmJointImpedanceControllerCore::enableServiceName(const size_t arm) const {
  std::lock_guard<std::mutex> lock(non_rt_mutex_);
  return arm < kImpedanceArmCount && arms_[arm].enable_service
             ? arms_[arm].enable_service->get_service_name()
             : std::string{};
}

std::array<double, kImpedanceJointCount> DualArmJointImpedanceControllerCore::internalTarget(
    const size_t arm) const noexcept {
  return arm < kImpedanceArmCount ? arms_[arm].internal_target
                                  : std::array<double, kImpedanceJointCount>{};
}

bool DualArmJointImpedanceControllerCore::bindInterfaces(
    DualArmJointImpedanceController& controller) noexcept {
  resetBindings();
  const size_t expected_command_count = arm_count_ * kImpedanceJointCount;
  const size_t expected_state_count =
      arm_count_ * (2 * kImpedanceJointCount + kStateInterfacesPerArmOverhead);
  if (controller.command_interfaces_.size() != expected_command_count ||
      controller.state_interfaces_.size() != expected_state_count) {
    return false;
  }
  for (size_t arm = 0; arm < arm_count_; ++arm) {
    if (!bindArmInterfaces(controller, arms_[arm])) {
      resetBindings();
      return false;
    }
  }
  if (arm_count_ == kImpedanceArmCount && (arms_[0].robot_state == arms_[1].robot_state ||
                                           arms_[0].robot_model == arms_[1].robot_model)) {
    resetBindings();
    return false;
  }
  interfaces_bound_ = true;
  zero_required_ = true;
  return true;
}

bool DualArmJointImpedanceControllerCore::bindArmInterfaces(
    DualArmJointImpedanceController& controller,
    Arm& arm) noexcept {
  for (size_t joint = 0; joint < kImpedanceJointCount; ++joint) {
    arm.position_interfaces[joint] = findUniqueStateInterface(
        controller, jointInterfaceName(arm.joint_names[joint], hardware_interface::HW_IF_POSITION));
    arm.velocity_interfaces[joint] = findUniqueStateInterface(
        controller, jointInterfaceName(arm.joint_names[joint], hardware_interface::HW_IF_VELOCITY));
    arm.effort_interfaces[joint] = findUniqueCommandInterface(
        controller, jointInterfaceName(arm.joint_names[joint], hardware_interface::HW_IF_EFFORT));
    if (arm.position_interfaces[joint] == nullptr || arm.velocity_interfaces[joint] == nullptr ||
        arm.effort_interfaces[joint] == nullptr) {
      return false;
    }
  }

  arm.robot_state_interface = findUniqueStateInterface(controller, arm.arm_id + "/robot_state");
  arm.robot_model_interface = findUniqueStateInterface(controller, arm.arm_id + "/robot_model");
  return arm.robot_state_interface != nullptr && arm.robot_model_interface != nullptr &&
         decodeStablePointer(*arm.robot_state_interface, arm.robot_state) &&
         decodeStablePointer(*arm.robot_model_interface, arm.robot_model);
}

bool DualArmJointImpedanceControllerCore::captureActivationState() noexcept {
  for (size_t arm_index = 0; arm_index < arm_count_; ++arm_index) {
    auto& arm = arms_[arm_index];
    if (arm.robot_state == nullptr || arm.robot_model == nullptr ||
        !pointerStillMatches(arm.robot_state_interface, arm.robot_state) ||
        !pointerStillMatches(arm.robot_model_interface, arm.robot_model) ||
        !finiteModelInput(*arm.robot_state)) {
      return false;
    }
    for (size_t joint = 0; joint < kImpedanceJointCount; ++joint) {
      double position = 0.0;
      double velocity = 0.0;
      if (!readFinite(arm.position_interfaces[joint], position) ||
          !readFinite(arm.velocity_interfaces[joint], velocity)) {
        return false;
      }
      arm.internal_target[joint] = position;
      arm.filtered_velocity[joint] = velocity;
    }
    arm.observed_enable_generation = arm.inbox.enableGeneration();
    try {
      const auto coriolis = arm.robot_model->coriolis(*arm.robot_state);
      if (!std::all_of(coriolis.begin(), coriolis.end(),
                       [](const double value) { return std::isfinite(value); })) {
        return false;
      }
    } catch (...) {
      return false;
    }
  }
  return true;
}

hardware_interface::LoanedStateInterface*
DualArmJointImpedanceControllerCore::findUniqueStateInterface(
    DualArmJointImpedanceController& controller,
    const std::string& name) noexcept {
  hardware_interface::LoanedStateInterface* match = nullptr;
  for (auto& interface : controller.state_interfaces_) {
    if (interface.get_name() == name) {
      if (match != nullptr) {
        return nullptr;
      }
      match = &interface;
    }
  }
  return match;
}

hardware_interface::LoanedCommandInterface*
DualArmJointImpedanceControllerCore::findUniqueCommandInterface(
    DualArmJointImpedanceController& controller,
    const std::string& name) noexcept {
  hardware_interface::LoanedCommandInterface* match = nullptr;
  for (auto& interface : controller.command_interfaces_) {
    if (interface.get_name() == name) {
      if (match != nullptr) {
        return nullptr;
      }
      match = &interface;
    }
  }
  return match;
}

bool DualArmJointImpedanceControllerCore::writeCommands(
    const std::array<std::array<double, kImpedanceJointCount>, kImpedanceArmCount>&
        efforts) noexcept {
  bool all_written = true;
  for (size_t arm = 0; arm < arm_count_; ++arm) {
    for (size_t joint = 0; joint < kImpedanceJointCount; ++joint) {
      const bool written = writeEffort(arms_[arm].effort_interfaces[joint], efforts[arm][joint]);
      all_written = written && all_written;
    }
  }
  return all_written;
}

bool DualArmJointImpedanceControllerCore::writeZeroAll() noexcept {
  bool all_written = true;
  for (size_t arm = 0; arm < arm_count_; ++arm) {
    for (auto* interface : arms_[arm].effort_interfaces) {
      const bool written = writeEffort(interface, 0.0);
      all_written = written && all_written;
    }
  }
  return all_written;
}

bool DualArmJointImpedanceControllerCore::attemptRequiredZero() noexcept {
  if (!interfaces_bound_ || !zero_required_) {
    return true;
  }
  if (!writeZeroAll()) {
    return false;
  }
  zero_required_ = false;
  return true;
}

void DualArmJointImpedanceControllerCore::releaseInterfaces(
    DualArmJointImpedanceController& controller) {
  std::lock_guard<std::mutex> lock(non_rt_mutex_);
  beginNonRealtimeTransition(NonRealtimePhase::Inactive);
  disableAndInvalidateAll(impedanceSteadyNowNanoseconds(),
                          controller.get_node()->get_clock()->now().nanoseconds());
  if (interfaces_bound_ && !writeZeroAll()) {
    release_zero_failed_ = true;
  }
  resetBindings();
  controller.controller_interface::ControllerInterface::release_interfaces();
}

void DualArmJointImpedanceControllerCore::resetBindings() noexcept {
  interfaces_bound_ = false;
  zero_required_ = false;
  for (auto& arm : arms_) {
    arm.position_interfaces.fill(nullptr);
    arm.velocity_interfaces.fill(nullptr);
    arm.effort_interfaces.fill(nullptr);
    arm.robot_state_interface = nullptr;
    arm.robot_model_interface = nullptr;
    arm.robot_state = nullptr;
    arm.robot_model = nullptr;
  }
}

void DualArmJointImpedanceControllerCore::resetRosEndpoints() noexcept {
  for (auto& arm : arms_) {
    arm.subscription.reset();
    arm.enable_service.reset();
  }
}

void DualArmJointImpedanceControllerCore::disableAndInvalidateAll(const int64_t steady_now_ns,
                                                                  const int64_t ros_now_ns) {
  for (auto& arm : arms_) {
    arm.inbox.setEnabled(false, steady_now_ns, ros_now_ns);
  }
}

void DualArmJointImpedanceControllerCore::beginNonRealtimeTransition(
    const NonRealtimePhase phase) noexcept {
  active_epoch_.store(0, std::memory_order_release);
  non_rt_phase_ = phase;
}

void DualArmJointImpedanceControllerCore::publishStableActiveEpoch() noexcept {
  non_rt_phase_ = NonRealtimePhase::Active;
  do {
    ++next_active_epoch_;
  } while (next_active_epoch_ == 0);
  active_epoch_.store(next_active_epoch_, std::memory_order_release);
}

bool DualArmJointImpedanceControllerCore::callbackEpochIsStableActive(
    const uint64_t entry_epoch) const noexcept {
  return entry_epoch != 0 && non_rt_phase_ == NonRealtimePhase::Active &&
         active_epoch_.load(std::memory_order_acquire) == entry_epoch;
}

DualArmJointImpedanceController::DualArmJointImpedanceController()
    : core_(std::make_unique<DualArmJointImpedanceControllerCore>()) {}

DualArmJointImpedanceController::~DualArmJointImpedanceController() = default;

controller_interface::InterfaceConfiguration
DualArmJointImpedanceController::command_interface_configuration() const {
  return core_->commandInterfaceConfiguration();
}

controller_interface::InterfaceConfiguration
DualArmJointImpedanceController::state_interface_configuration() const {
  return core_->stateInterfaceConfiguration();
}

controller_interface::return_type DualArmJointImpedanceController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  return core_->update(*this, period);
}

void DualArmJointImpedanceController::release_interfaces() {
  core_->releaseInterfaces(*this);
}

controller_interface::CallbackReturn DualArmJointImpedanceController::on_init() {
  return core_->onInit(*this);
}

controller_interface::CallbackReturn DualArmJointImpedanceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  return core_->onConfigure(*this);
}

controller_interface::CallbackReturn DualArmJointImpedanceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  return core_->onActivate(*this);
}

controller_interface::CallbackReturn DualArmJointImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  return core_->onDeactivate(*this);
}

controller_interface::CallbackReturn DualArmJointImpedanceController::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  return core_->onCleanup(*this);
}

controller_interface::CallbackReturn DualArmJointImpedanceController::on_error(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  return core_->onError(*this);
}

controller_interface::CallbackReturn DualArmJointImpedanceController::on_shutdown(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  return core_->onShutdown(*this);
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::DualArmJointImpedanceController,
                       controller_interface::ControllerInterface)
