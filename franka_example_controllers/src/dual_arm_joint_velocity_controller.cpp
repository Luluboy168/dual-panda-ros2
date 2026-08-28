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

#include "franka_example_controllers/dual_arm_joint_velocity_controller.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <limits>
#include <memory>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <utility>
#include <vector>

#include "dual_arm_joint_velocity_controller_core.hpp"
#include "franka_example_controllers/panda_joint_limits.hpp"

namespace {

constexpr size_t kExpectedVelocityCommandInterfaceCount = 14;
constexpr long double kNanosecondsPerSecond = 1000000000.0L;

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

bool validPositiveLimits(const std::vector<double>& limits) {
  return limits.size() == franka_example_controllers::kVelocityJointCount &&
         std::all_of(limits.begin(), limits.end(),
                     [](const double limit) { return std::isfinite(limit) && limit > 0.0; });
}

bool hasCanonicalJointNames(const std::string& arm_id,
                            const std::vector<std::string>& joint_names) {
  if (joint_names.size() != franka_example_controllers::kVelocityJointCount) {
    return false;
  }
  for (size_t joint = 0; joint < joint_names.size(); ++joint) {
    if (joint_names[joint] != arm_id + "_joint" + std::to_string(joint + 1)) {
      return false;
    }
  }
  return true;
}

bool withinFrankaJointLimits(const std::vector<double>& max_velocity,
                             const std::vector<double>& max_acceleration) {
  if (!validPositiveLimits(max_velocity) || !validPositiveLimits(max_acceleration)) {
    return false;
  }
  for (size_t joint = 0; joint < franka_example_controllers::kVelocityJointCount; ++joint) {
    if (max_velocity[joint] > franka_example_controllers::kPandaFciJointVelocityCeilings[joint] ||
        max_acceleration[joint] >
            franka_example_controllers::kPandaFciJointAccelerationCeilings[joint]) {
      return false;
    }
  }
  return true;
}

std::string commandInterfaceName(const std::string& joint_name) {
  return joint_name + "/" + hardware_interface::HW_IF_VELOCITY;
}

bool writeVelocity(hardware_interface::LoanedCommandInterface* interface,
                   const double velocity) noexcept {
  if (interface == nullptr) {
    return false;
  }
  try {
    return interface->set_value(velocity, 1);
  } catch (...) {
    return false;
  }
}

}  // namespace

namespace franka_example_controllers {

int64_t steadyNowNanoseconds() noexcept {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             DualArmJointVelocityControllerCore::SteadyClock::now().time_since_epoch())
      .count();
}

void ArmVelocityCommandInbox::configure(const VelocityCommandPolicy& policy) noexcept {
  policy_ = policy;
  enabled_.store(false, std::memory_order_release);
  enabled_since_ns_.store(0, std::memory_order_release);
  command_buffer_.initRT(BufferedVelocityCommand{});
}

JointJogValidationResult ArmVelocityCommandInbox::accept(const control_msgs::msg::JointJog& message,
                                                         const int64_t ros_now_ns,
                                                         const int64_t steady_receive_ns) noexcept {
  auto reject = [&](const JointJogValidationResult result) {
    invalidate(steady_receive_ns);
    return result;
  };

  if (!message.header.frame_id.empty()) {
    return reject(JointJogValidationResult::InvalidFrame);
  }
  if (message.header.stamp.sec < 0 || message.header.stamp.nanosec >= 1000000000U ||
      (message.header.stamp.sec == 0 && message.header.stamp.nanosec == 0)) {
    return reject(JointJogValidationResult::InvalidStamp);
  }
  const int64_t header_ns = static_cast<int64_t>(message.header.stamp.sec) * 1000000000LL +
                            static_cast<int64_t>(message.header.stamp.nanosec);
  const long double header_age_ns =
      static_cast<long double>(ros_now_ns) - static_cast<long double>(header_ns);
  if (header_age_ns > static_cast<long double>(policy_.max_header_age_ns)) {
    return reject(JointJogValidationResult::HeaderTooOld);
  }
  if (-header_age_ns > static_cast<long double>(policy_.future_tolerance_ns)) {
    return reject(JointJogValidationResult::HeaderTooFarInFuture);
  }
  if (!std::isfinite(message.duration) || message.duration != 0.0) {
    return reject(JointJogValidationResult::InvalidDuration);
  }
  if (!message.displacements.empty()) {
    return reject(JointJogValidationResult::DisplacementCommandNotAllowed);
  }
  if (message.joint_names.size() != kVelocityJointCount) {
    return reject(JointJogValidationResult::InvalidNameCount);
  }
  if (message.velocities.size() != kVelocityJointCount) {
    return reject(JointJogValidationResult::InvalidVelocityCount);
  }

  BufferedVelocityCommand command;
  command.valid = true;
  command.steady_receive_ns = steady_receive_ns;
  std::array<bool, kVelocityJointCount> matched{};
  for (size_t message_index = 0; message_index < kVelocityJointCount; ++message_index) {
    size_t configured_index = kVelocityJointCount;
    for (size_t joint = 0; joint < kVelocityJointCount; ++joint) {
      if (message.joint_names[message_index] == policy_.joint_names[joint]) {
        configured_index = joint;
        break;
      }
    }
    if (configured_index == kVelocityJointCount || matched[configured_index]) {
      return reject(JointJogValidationResult::DuplicateOrUnknownJoint);
    }
    const double velocity = message.velocities[message_index];
    if (!std::isfinite(velocity)) {
      return reject(JointJogValidationResult::NonfiniteVelocity);
    }
    if (std::abs(velocity) > policy_.max_velocity[configured_index]) {
      return reject(JointJogValidationResult::VelocityLimitExceeded);
    }
    matched[configured_index] = true;
    command.velocities[configured_index] = velocity;
  }

  command_buffer_.writeFromNonRT(command);
  return JointJogValidationResult::Accepted;
}

void ArmVelocityCommandInbox::setEnabled(const bool enabled, const int64_t steady_now_ns) {
  enabled_.store(false, std::memory_order_release);
  invalidate(steady_now_ns);
  enabled_since_ns_.store(steady_now_ns, std::memory_order_release);
  enabled_.store(enabled, std::memory_order_release);
}

bool ArmVelocityCommandInbox::readFresh(
    const int64_t steady_now_ns,
    const int64_t watchdog_ns,
    std::array<double, kVelocityJointCount>& velocities) noexcept {
  if (!enabled_.load(std::memory_order_acquire)) {
    return false;
  }
  const BufferedVelocityCommand command = *command_buffer_.readFromRT();
  const int64_t enabled_since_ns = enabled_since_ns_.load(std::memory_order_acquire);
  if (!command.valid || command.steady_receive_ns <= enabled_since_ns ||
      command.steady_receive_ns > steady_now_ns ||
      steady_now_ns - command.steady_receive_ns > watchdog_ns) {
    return false;
  }
  velocities = command.velocities;
  return true;
}

void ArmVelocityCommandInbox::invalidate(const int64_t steady_receive_ns) {
  BufferedVelocityCommand command;
  command.steady_receive_ns = steady_receive_ns;
  command_buffer_.writeFromNonRT(command);
}

controller_interface::InterfaceConfiguration
DualArmJointVelocityControllerCore::commandInterfaceConfiguration() const {
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  if (!configured_) {
    return configuration;
  }
  configuration.names.reserve(arm_count_ * kVelocityJointCount);
  for (size_t arm = 0; arm < arm_count_; ++arm) {
    for (const auto& joint_name : arms_[arm].joint_names) {
      configuration.names.push_back(commandInterfaceName(joint_name));
    }
  }
  return configuration;
}

controller_interface::InterfaceConfiguration
DualArmJointVelocityControllerCore::stateInterfaceConfiguration() const {
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::NONE;
  return configuration;
}

controller_interface::return_type DualArmJointVelocityControllerCore::update(
    DualArmJointVelocityController& controller,
    const rclcpp::Duration& period) noexcept {
  // First-update bind (F-10c): bindInterfaces() runs here, on the control-cycle owner thread, in
  // response to onActivate()'s request -- never on the thread onActivate() itself may run on.
  if (rt_activation_phase_.load(std::memory_order_acquire) == RtActivationPhase::kRequested) {
    serviceFirstUpdateActivation(controller);
  }
  const bool active =
      rt_activation_phase_.load(std::memory_order_acquire) == RtActivationPhase::kActive;

  if (!active || !interfaces_bound_) {
    // Owner-thread ramp reset: arm.last_output is RT-only state, so every deactivation-class
    // event resets it here, on the cycle after the lifecycle callback published kIdle, rather
    // than from the lifecycle thread itself. A subsequent reactivation therefore always starts
    // its acceleration ramp from zero.
    for (auto& arm : arms_) {
      arm.last_output.fill(0.0);
    }
    if (interfaces_bound_) {
      attemptRequiredZero();
    }
    return controller_interface::return_type::ERROR;
  }

  const double period_seconds = period.seconds();
  if (!std::isfinite(period_seconds) || period_seconds <= 0.0) {
    for (size_t arm_index = 0; arm_index < arm_count_; ++arm_index) {
      arms_[arm_index].last_output.fill(0.0);
    }
    attemptRequiredZero();
    return controller_interface::return_type::ERROR;
  }

  const int64_t steady_now_ns = steadyNowNanoseconds();
  std::array<std::array<double, kVelocityJointCount>, kVelocityArmCount> commands{};
  for (size_t arm_index = 0; arm_index < arm_count_; ++arm_index) {
    auto& arm = arms_[arm_index];
    std::array<double, kVelocityJointCount> target{};
    if (!arm.inbox.readFresh(steady_now_ns, watchdog_ns_, target)) {
      arm.last_output.fill(0.0);
      continue;
    }

    for (size_t joint = 0; joint < kVelocityJointCount; ++joint) {
      if (!std::isfinite(target[joint]) || std::abs(target[joint]) > arm.max_velocity[joint]) {
        arm.last_output.fill(0.0);
        commands[arm_index].fill(0.0);
        break;
      }
      const long double requested_delta = static_cast<long double>(target[joint]) -
                                          static_cast<long double>(arm.last_output[joint]);
      const long double max_delta = static_cast<long double>(arm.max_acceleration[joint]) *
                                    static_cast<long double>(period_seconds);
      long double limited_delta = requested_delta;
      if (limited_delta > max_delta) {
        limited_delta = max_delta;
      } else if (limited_delta < -max_delta) {
        limited_delta = -max_delta;
      }
      const long double output = static_cast<long double>(arm.last_output[joint]) + limited_delta;
      if (!std::isfinite(output) ||
          std::abs(output) > static_cast<long double>(arm.max_velocity[joint])) {
        arm.last_output.fill(0.0);
        commands[arm_index].fill(0.0);
        break;
      }
      commands[arm_index][joint] = static_cast<double>(output);
      arm.last_output[joint] = commands[arm_index][joint];
    }
  }

  zero_required_ = true;
  if (!writeCommands(commands)) {
    for (size_t arm_index = 0; arm_index < arm_count_; ++arm_index) {
      arms_[arm_index].last_output.fill(0.0);
    }
    attemptRequiredZero();
    return controller_interface::return_type::ERROR;
  }
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn DualArmJointVelocityControllerCore::onInit(
    DualArmJointVelocityController& controller) {
  try {
    controller.auto_declare<int64_t>("arm_count", static_cast<int64_t>(kVelocityArmCount));
    for (size_t arm = 0; arm < kVelocityArmCount; ++arm) {
      const auto prefix = "arm_" + std::to_string(arm + 1) + ".";
      controller.auto_declare<std::string>(prefix + "arm_id", "");
      controller.auto_declare<std::vector<std::string>>(prefix + "joint_names", {});
      controller.auto_declare<std::vector<double>>(prefix + "max_velocity", {});
      controller.auto_declare<std::vector<double>>(prefix + "max_acceleration", {});
    }
    controller.auto_declare<double>("watchdog_timeout", 0.0);
    controller.auto_declare<double>("max_header_age", 0.0);
    controller.auto_declare<double>("future_tolerance", 0.0);
  } catch (const std::exception& error) {
    RCLCPP_ERROR(controller.get_node()->get_logger(),
                 "Failed to declare velocity-controller parameters: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DualArmJointVelocityControllerCore::onConfigure(
    DualArmJointVelocityController& controller) {
  configured_ = false;
  // rt_ever_bound_ (owner-thread-written) rather than the owner-thread-only interfaces_bound_:
  // onConfigure() runs on the service thread, and F-10c's ownership rule (design 3.1) forbids it
  // from reading a field update() owns, even under the lifecycle-graph precondition that makes an
  // overlap impossible in practice.
  // rt_activation_phase_ closes the in-flight-activation window rt_ever_bound_ alone leaves open
  // -- see DualArmJointHoldController::on_configure() for the full rationale.
  if (rt_ever_bound_.load(std::memory_order_acquire) ||
      rt_activation_phase_.load(std::memory_order_acquire) != RtActivationPhase::kIdle) {
    return controller_interface::CallbackReturn::ERROR;
  }
  for (auto& arm : arms_) {
    arm.subscription.reset();
    arm.enable_service.reset();
  }

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
    if (requested_arm_count != 1 && static_cast<size_t>(requested_arm_count) != kVelocityArmCount) {
      RCLCPP_ERROR(node->get_logger(), "arm_count must be exactly 1 or %zu, got %ld",
                   kVelocityArmCount, static_cast<long>(requested_arm_count));
      return controller_interface::CallbackReturn::FAILURE;
    }
    const size_t arm_count = static_cast<size_t>(requested_arm_count);
    if (arm_count < kVelocityArmCount && hasOverriddenParameterWithPrefix(*node, "arm_2.")) {
      RCLCPP_ERROR(node->get_logger(), "arm_2.* parameters must not be set when arm_count is 1");
      return controller_interface::CallbackReturn::FAILURE;
    }

    for (size_t arm_index = 0; arm_index < arm_count; ++arm_index) {
      auto& arm = arms_[arm_index];
      const auto prefix = "arm_" + std::to_string(arm_index + 1) + ".";
      const auto arm_id = node->get_parameter(prefix + "arm_id").as_string();
      const auto joint_names = node->get_parameter(prefix + "joint_names").as_string_array();
      const auto max_velocity = node->get_parameter(prefix + "max_velocity").as_double_array();
      const auto max_acceleration =
          node->get_parameter(prefix + "max_acceleration").as_double_array();
      if (!isAsciiIdentifier(arm_id)) {
        RCLCPP_ERROR(node->get_logger(),
                     "%sarm_id must contain 1..64 ASCII characters, start with a letter, and then "
                     "contain only letters, digits, or '_'",
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
      if (!withinFrankaJointLimits(max_velocity, max_acceleration)) {
        RCLCPP_ERROR(node->get_logger(),
                     "%smax_velocity and max_acceleration must each contain seven finite positive "
                     "values no greater than the per-joint libfranka Panda ceilings",
                     prefix.c_str());
        return controller_interface::CallbackReturn::FAILURE;
      }
      arm.arm_id = arm_id;
      std::copy(joint_names.begin(), joint_names.end(), arm.joint_names.begin());
      std::copy(max_velocity.begin(), max_velocity.end(), arm.max_velocity.begin());
      std::copy(max_acceleration.begin(), max_acceleration.end(), arm.max_acceleration.begin());
      // F-10c (design §3.3): precompute here, once, on the service thread, so the owner thread's
      // first-update bind (bindInterfaces(), called from update()) never allocates.
      for (size_t joint = 0; joint < kVelocityJointCount; ++joint) {
        arm.velocity_interface_names[joint] = commandInterfaceName(arm.joint_names[joint]);
      }
    }

    if (arm_count == kVelocityArmCount && arms_[0].arm_id == arms_[1].arm_id) {
      RCLCPP_ERROR(node->get_logger(), "The two velocity-controller arm IDs must be unique");
      return controller_interface::CallbackReturn::FAILURE;
    }
    std::array<std::string, kExpectedVelocityCommandInterfaceCount> all_joint_names{};
    size_t next_name = 0;
    for (size_t arm_index = 0; arm_index < arm_count; ++arm_index) {
      for (const auto& joint_name : arms_[arm_index].joint_names) {
        if (std::find(all_joint_names.begin(), all_joint_names.begin() + next_name, joint_name) !=
            all_joint_names.begin() + next_name) {
          RCLCPP_ERROR(node->get_logger(), "Joint names must be unique across both arms");
          return controller_interface::CallbackReturn::FAILURE;
        }
        all_joint_names[next_name++] = joint_name;
      }
    }

    for (size_t arm_index = 0; arm_index < arm_count; ++arm_index) {
      auto& arm = arms_[arm_index];
      VelocityCommandPolicy policy;
      policy.joint_names = arm.joint_names;
      policy.max_velocity = arm.max_velocity;
      policy.max_header_age_ns = max_header_age_ns_;
      policy.future_tolerance_ns = future_tolerance_ns_;
      arm.inbox.configure(policy);
      arm.last_output.fill(0.0);

      const auto topic = "~/arm_" + std::to_string(arm_index + 1) + "/joint_jog";
      const auto clock = node->get_clock();
      arm.subscription = node->create_subscription<control_msgs::msg::JointJog>(
          topic, rclcpp::QoS(1).reliable().durability_volatile(),
          [this, arm_index, clock](const control_msgs::msg::JointJog::SharedPtr message) {
            acceptCommand(arm_index, *message, clock->now().nanoseconds(), steadyNowNanoseconds());
          });
      const auto service_name = "~/arm_" + std::to_string(arm_index + 1) + "/enable";
      arm.enable_service = node->create_service<std_srvs::srv::SetBool>(
          service_name, [this, arm_index](const std_srvs::srv::SetBool::Request::SharedPtr request,
                                          std_srvs::srv::SetBool::Response::SharedPtr response) {
            setArmEnabled(arm_index, request->data, steadyNowNanoseconds());
            response->success = true;
            response->message = request->data ? "enabled; awaiting a fresh valid command"
                                              : "disabled; zero commanded on the next update";
          });
    }
    arm_count_ = arm_count;
  } catch (const std::exception& error) {
    RCLCPP_ERROR(controller.get_node()->get_logger(),
                 "Failed to configure dual-arm velocity controller: %s", error.what());
    return controller_interface::CallbackReturn::FAILURE;
  }

  configured_ = true;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DualArmJointVelocityControllerCore::onActivate(
    DualArmJointVelocityController& controller) {
  disableAndInvalidateAll(steadyNowNanoseconds());
  if (!configured_) {
    return controller_interface::CallbackReturn::FAILURE;
  }
  // F-10c amendment A.4: read-only wiring validation -- exact count, exact names, each present
  // exactly once (which is also the no-aliasing property). Reads only interface names, stores
  // nothing, binds nothing; see DualArmJointHoldController::on_activate() for why this cannot
  // race the owner thread. bindInterfaces() itself stays deferred to update()'s first cycle after
  // this request; see RtActivationPhase above and serviceFirstUpdateActivation() below.
  if (!validateInterfaceWiring(controller)) {
    return controller_interface::CallbackReturn::FAILURE;
  }
  rt_activation_phase_.store(RtActivationPhase::kRequested, std::memory_order_release);
  return controller_interface::CallbackReturn::SUCCESS;
}

// F-10c amendment A (2026-08-28): onDeactivate()/onCleanup()/onError()/onShutdown() write no
// command interface. See DualArmJointHoldController's equivalent comment and design amendment A.1
// -- the hardware layer publishes the safe command on the control-cycle owner thread during the
// mode switch, before controller_manager reaches any of these callbacks, and this controller's
// own zero writes live where they belong: inside update(), on the owner thread.

controller_interface::CallbackReturn DualArmJointVelocityControllerCore::onDeactivate(
    DualArmJointVelocityController& /*controller*/) {
  disableAndInvalidateAll(steadyNowNanoseconds());
  rt_activation_phase_.store(RtActivationPhase::kIdle, std::memory_order_release);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DualArmJointVelocityControllerCore::onCleanup() {
  configured_ = false;
  disableAndInvalidateAll(steadyNowNanoseconds());
  rt_activation_phase_.store(RtActivationPhase::kIdle, std::memory_order_release);
  for (auto& arm : arms_) {
    arm.subscription.reset();
    arm.enable_service.reset();
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DualArmJointVelocityControllerCore::onError(
    DualArmJointVelocityController& /*controller*/) {
  disableAndInvalidateAll(steadyNowNanoseconds());
  rt_activation_phase_.store(RtActivationPhase::kIdle, std::memory_order_release);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DualArmJointVelocityControllerCore::onShutdown(
    DualArmJointVelocityController& /*controller*/) {
  configured_ = false;
  disableAndInvalidateAll(steadyNowNanoseconds());
  rt_activation_phase_.store(RtActivationPhase::kIdle, std::memory_order_release);
  for (auto& arm : arms_) {
    arm.subscription.reset();
    arm.enable_service.reset();
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

JointJogValidationResult DualArmJointVelocityControllerCore::acceptCommand(
    const size_t arm,
    const control_msgs::msg::JointJog& message,
    const int64_t ros_now_ns,
    const int64_t steady_receive_ns) noexcept {
  if (arm >= kVelocityArmCount) {
    return JointJogValidationResult::DuplicateOrUnknownJoint;
  }
  return arms_[arm].inbox.accept(message, ros_now_ns, steady_receive_ns);
}

void DualArmJointVelocityControllerCore::setArmEnabled(const size_t arm,
                                                       const bool enabled,
                                                       const int64_t steady_now_ns) {
  if (arm < kVelocityArmCount) {
    arms_[arm].inbox.setEnabled(enabled, steady_now_ns);
  }
}

bool DualArmJointVelocityControllerCore::armEnabled(const size_t arm) const noexcept {
  return arm < kVelocityArmCount && arms_[arm].inbox.enabled();
}

std::string DualArmJointVelocityControllerCore::subscriptionTopic(const size_t arm) const {
  return arm < kVelocityArmCount && arms_[arm].subscription
             ? arms_[arm].subscription->get_topic_name()
             : std::string{};
}

std::string DualArmJointVelocityControllerCore::enableServiceName(const size_t arm) const {
  return arm < kVelocityArmCount && arms_[arm].enable_service
             ? arms_[arm].enable_service->get_service_name()
             : std::string{};
}

bool DualArmJointVelocityControllerCore::validateInterfaceWiring(
    const DualArmJointVelocityController& controller) const noexcept {
  // Read-only (F-10c amendment A.4): exact count, exact names, each present exactly once. Over a
  // name set whose size equals the vector's size, "exactly once each" is also the no-aliasing
  // property -- no two arms and no two joints can share an interface.
  if (controller.command_interfaces_.size() != arm_count_ * kVelocityJointCount) {
    return false;
  }
  for (size_t arm = 0; arm < arm_count_; ++arm) {
    for (size_t joint = 0; joint < kVelocityJointCount; ++joint) {
      size_t matches = 0;
      for (const auto& interface : controller.command_interfaces_) {
        if (interface.get_name() == arms_[arm].velocity_interface_names[joint]) {
          ++matches;
        }
      }
      if (matches != 1) {
        return false;
      }
    }
  }
  return true;
}

bool DualArmJointVelocityControllerCore::bindInterfaces(
    DualArmJointVelocityController& controller) noexcept {
  resetBindings();
  if (controller.command_interfaces_.size() != arm_count_ * kVelocityJointCount) {
    return false;
  }
  // F-10c (design §3.3): only string *comparisons* against the names onConfigure() precomputed --
  // no allocation. This runs on the control-cycle owner thread (see
  // serviceFirstUpdateActivation()).
  for (size_t arm = 0; arm < arm_count_; ++arm) {
    for (size_t joint = 0; joint < kVelocityJointCount; ++joint) {
      arms_[arm].velocity_interfaces[joint] =
          findUniqueCommandInterface(controller, arms_[arm].velocity_interface_names[joint]);
      if (arms_[arm].velocity_interfaces[joint] == nullptr) {
        resetBindings();
        return false;
      }
    }
  }
  interfaces_bound_ = true;
  zero_required_ = true;
  rt_ever_bound_.store(true, std::memory_order_release);
  return true;
}

void DualArmJointVelocityControllerCore::serviceFirstUpdateActivation(
    DualArmJointVelocityController& controller) noexcept {
  // Only ever called from update() on the control-cycle owner thread.
  // The acceleration ramp is owner-thread-only RT state, so it is reset here rather than from
  // onActivate(): a fresh activation always starts from zero output.
  for (auto& arm : arms_) {
    arm.last_output.fill(0.0);
  }
  const bool bound = bindInterfaces(controller);
  const bool zeroed = bound && attemptRequiredZero();
  const RtActivationPhase desired =
      (bound && zeroed) ? RtActivationPhase::kActive : RtActivationPhase::kFailed;
  RtActivationPhase expected = RtActivationPhase::kRequested;
  // If this CAS loses, a concurrent onDeactivate()/onError()/onShutdown()/releaseInterfaces()
  // already reset the phase (to kIdle) while this bind was in flight -- see
  // DualArmJointHoldController::serviceFirstUpdateActivation() for the full rationale, which
  // applies identically here.
  (void)rt_activation_phase_.compare_exchange_strong(expected, desired, std::memory_order_acq_rel,
                                                     std::memory_order_acquire);
}

hardware_interface::LoanedCommandInterface*
DualArmJointVelocityControllerCore::findUniqueCommandInterface(
    DualArmJointVelocityController& controller,
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

bool DualArmJointVelocityControllerCore::writeCommands(
    const std::array<std::array<double, kVelocityJointCount>, kVelocityArmCount>&
        commands) noexcept {
  bool all_written = true;
  for (size_t arm = 0; arm < arm_count_; ++arm) {
    for (size_t joint = 0; joint < kVelocityJointCount; ++joint) {
      const bool written =
          writeVelocity(arms_[arm].velocity_interfaces[joint], commands[arm][joint]);
      all_written = written && all_written;
    }
  }
  return all_written;
}

bool DualArmJointVelocityControllerCore::writeZeroAll() noexcept {
  bool all_written = true;
  for (size_t arm = 0; arm < arm_count_; ++arm) {
    for (auto* interface : arms_[arm].velocity_interfaces) {
      const bool written = writeVelocity(interface, 0.0);
      all_written = written && all_written;
    }
  }
  return all_written;
}

bool DualArmJointVelocityControllerCore::attemptRequiredZero() noexcept {
  if (!interfaces_bound_ || !zero_required_) {
    return true;
  }
  if (!writeZeroAll()) {
    return false;
  }
  zero_required_ = false;
  return true;
}

void DualArmJointVelocityControllerCore::releaseInterfaces() noexcept {
  // F-10c amendment A: no command write here -- see DualArmJointHoldController::
  // release_interfaces() for the full rationale, same shape.
  rt_activation_phase_.store(RtActivationPhase::kIdle, std::memory_order_release);
  resetBindings();
}

void DualArmJointVelocityControllerCore::resetBindings() noexcept {
  interfaces_bound_ = false;
  zero_required_ = false;
  rt_ever_bound_.store(false, std::memory_order_release);
  for (auto& arm : arms_) {
    arm.velocity_interfaces.fill(nullptr);
  }
}

void DualArmJointVelocityControllerCore::disableAndInvalidateAll(const int64_t steady_now_ns) {
  // F-10c: arm.last_output is owner-thread-only RT ramp state (read/written every update() cycle)
  // -- it is not reset here (service thread). update() resets it on the owner thread: on the
  // first cycle after a deactivation-class event publishes kIdle, and again in
  // serviceFirstUpdateActivation() when a fresh activation binds.
  for (auto& arm : arms_) {
    arm.inbox.setEnabled(false, steady_now_ns);
  }
}

DualArmJointVelocityController::DualArmJointVelocityController()
    : core_(std::make_unique<DualArmJointVelocityControllerCore>()) {}

DualArmJointVelocityController::~DualArmJointVelocityController() = default;

controller_interface::InterfaceConfiguration
DualArmJointVelocityController::command_interface_configuration() const {
  return core_->commandInterfaceConfiguration();
}

controller_interface::InterfaceConfiguration
DualArmJointVelocityController::state_interface_configuration() const {
  return core_->stateInterfaceConfiguration();
}

controller_interface::return_type DualArmJointVelocityController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  return core_->update(*this, period);
}

void DualArmJointVelocityController::release_interfaces() {
  core_->releaseInterfaces();
  controller_interface::ControllerInterface::release_interfaces();
}

controller_interface::CallbackReturn DualArmJointVelocityController::on_init() {
  return core_->onInit(*this);
}

controller_interface::CallbackReturn DualArmJointVelocityController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  return core_->onConfigure(*this);
}

controller_interface::CallbackReturn DualArmJointVelocityController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  return core_->onActivate(*this);
}

controller_interface::CallbackReturn DualArmJointVelocityController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  return core_->onDeactivate(*this);
}

controller_interface::CallbackReturn DualArmJointVelocityController::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  return core_->onCleanup();
}

controller_interface::CallbackReturn DualArmJointVelocityController::on_error(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  return core_->onError(*this);
}

controller_interface::CallbackReturn DualArmJointVelocityController::on_shutdown(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  return core_->onShutdown(*this);
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::DualArmJointVelocityController,
                       controller_interface::ControllerInterface)
