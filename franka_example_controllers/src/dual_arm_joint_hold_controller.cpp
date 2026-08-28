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

#include "franka_example_controllers/dual_arm_joint_hold_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <exception>
#include <type_traits>
#include <utility>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>

#include "franka_example_controllers/panda_joint_limits.hpp"

namespace {

constexpr size_t kStateInterfacesPerArmOverhead = 2;  // robot_state + robot_model

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

bool isAsciiArmId(const std::string& arm_id) {
  if (arm_id.empty() || arm_id.size() > franka_example_controllers::kPandaArmIdMaxLength) {
    return false;
  }
  const auto first = static_cast<unsigned char>(arm_id.front());
  if (!((first >= 'A' && first <= 'Z') || (first >= 'a' && first <= 'z'))) {
    return false;
  }
  return std::all_of(arm_id.begin() + 1, arm_id.end(), [](const char value) {
    const auto character = static_cast<unsigned char>(value);
    return (character >= 'A' && character <= 'Z') || (character >= 'a' && character <= 'z') ||
           (character >= '0' && character <= '9') || character == '_';
  });
}

bool validGains(const std::vector<double>& gains) {
  return gains.size() == 7 && std::all_of(gains.begin(), gains.end(), [](const double gain) {
           return std::isfinite(gain) && gain >= 0.0;
         });
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

template <typename Fields>
bool finiteModelInputFields(const Fields& fields) {
  const auto finite_array = [](const auto& values) {
    return std::all_of(values.begin(), values.end(),
                       [](const double value) { return std::isfinite(value); });
  };
  return finite_array(fields.q) && finite_array(fields.dq) && finite_array(fields.I_total) &&
         finite_array(fields.F_x_Ctotal) && std::isfinite(fields.m_total);
}

bool finiteModelInput(const franka::RobotState& state) {
  return finiteModelInputFields(state);
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

std::string jointInterfaceName(const std::string& arm_id,
                               const size_t joint_index,
                               const char* interface_name) {
  return arm_id + "_joint" + std::to_string(joint_index + 1) + "/" + interface_name;
}

}  // namespace

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
DualArmJointHoldController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  if (!configured_) {
    return configuration;
  }
  configuration.names.reserve(arm_count_ * kJointCount);
  for (size_t arm = 0; arm < arm_count_; ++arm) {
    for (size_t joint = 0; joint < kJointCount; ++joint) {
      configuration.names.push_back(
          jointInterfaceName(arms_[arm].arm_id, joint, hardware_interface::HW_IF_EFFORT));
    }
  }
  return configuration;
}

controller_interface::InterfaceConfiguration
DualArmJointHoldController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  if (!configured_) {
    return configuration;
  }
  configuration.names.reserve(arm_count_ * (2 * kJointCount + kStateInterfacesPerArmOverhead));
  for (size_t arm = 0; arm < arm_count_; ++arm) {
    for (size_t joint = 0; joint < kJointCount; ++joint) {
      configuration.names.push_back(
          jointInterfaceName(arms_[arm].arm_id, joint, hardware_interface::HW_IF_POSITION));
      configuration.names.push_back(
          jointInterfaceName(arms_[arm].arm_id, joint, hardware_interface::HW_IF_VELOCITY));
    }
    configuration.names.push_back(arms_[arm].arm_id + "/robot_state");
    configuration.names.push_back(arms_[arm].arm_id + "/robot_model");
  }
  return configuration;
}

void DualArmJointHoldController::release_interfaces() {
  // F-10c amendment A (2026-08-28): this callback writes nothing. The zero that a mode switch
  // requires is published by the hardware layer on the control-cycle owner thread, before
  // controller_manager ever reaches deactivate_controllers()/release_interfaces() --
  // FrankaMultiHardwareInterface::perform_command_mode_switch() ->
  // applyPreparedTransactionEffects() -> safeCommandForArm() -> publishCommand(). A controller
  // zero here would only duplicate it, from the wrong thread, racing the owner thread's write()
  // over the very same arm.hw_commands_* storage.
  //
  // resetBindings() is not a command write and needs no handoff: controller_manager has already
  // taken this controller out of ACTIVE by the time it calls release_interfaces(), and the base
  // class call immediately below destroys the loan vectors on this same thread unconditionally.
  // Any topology in which nulling our pointers *into* those vectors could race update() is one in
  // which upstream's destruction of the vectors themselves is already a use-after-free. We rely
  // on exactly the guarantee upstream already relies on, and no more.
  rt_activation_phase_.store(RtActivationPhase::kIdle, std::memory_order_release);
  resetBindings();
  controller_interface::ControllerInterface::release_interfaces();
}

controller_interface::return_type DualArmJointHoldController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  // First-update capture (F-10c): bindArmInterfaces()/captureActivationState() run here, on the
  // control-cycle owner thread, in response to on_activate()'s request -- never on the thread
  // on_activate() itself may run on.
  if (rt_activation_phase_.load(std::memory_order_acquire) == RtActivationPhase::kRequested) {
    serviceFirstUpdateActivation();
  }
  // Activation state is derived, fresh, from the atomic phase on every cycle and kept in a
  // function-local: no lifecycle callback writes it, so there is nothing left to race on it.
  const bool active =
      rt_activation_phase_.load(std::memory_order_acquire) == RtActivationPhase::kActive;

  if (!active || !interfaces_bound_) {
    if (interfaces_bound_) {
      attemptRequiredZero();
    }
    return controller_interface::return_type::ERROR;
  }

  std::array<std::array<double, kJointCount>, kArmCount> efforts{};
  std::array<std::array<double, kJointCount>, kArmCount> next_filtered_velocity{};
  if (!computeCommands(efforts, next_filtered_velocity)) {
    attemptRequiredZero();
    return controller_interface::return_type::ERROR;
  }

  for (size_t arm = 0; arm < arm_count_; ++arm) {
    arms_[arm].filtered_velocity = next_filtered_velocity[arm];
  }
  zero_required_ = true;
  if (!writeCommands(efforts)) {
    attemptRequiredZero();
    return controller_interface::return_type::ERROR;
  }
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn DualArmJointHoldController::on_init() {
  try {
    auto_declare<int64_t>("arm_count", static_cast<int64_t>(kArmCount));
    for (size_t arm = 0; arm < kArmCount; ++arm) {
      const auto prefix = "arm_" + std::to_string(arm + 1) + ".";
      auto_declare<std::string>(prefix + "arm_id", "");
      auto_declare<std::vector<double>>(prefix + "k_gains", {});
      auto_declare<std::vector<double>>(prefix + "d_gains", {});
      auto_declare<std::vector<double>>(prefix + "max_effort", {});
    }
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare hold parameters: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DualArmJointHoldController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  configured_ = false;
  // rt_ever_bound_ (owner-thread-written) rather than the owner-thread-only interfaces_bound_:
  // on_configure() runs on the service thread, and F-10c's ownership rule (design 3.1) forbids it
  // from reading a field update() owns, even under the lifecycle-graph precondition that makes an
  // overlap impossible in practice.
  //
  // rt_activation_phase_ closes the in-flight-activation window rt_ever_bound_ alone leaves open:
  // between on_activate() publishing kRequested and the owner thread's first update() cycle
  // actually binding, rt_ever_bound_ is still false, so this guard would have let a reconfigure
  // through and rewritten arm_count_/the arm ids/the precomputed interface names out from under
  // the bind that is about to run. Anything other than kIdle means an activation is live or in
  // flight.
  if (rt_ever_bound_.load(std::memory_order_acquire) ||
      rt_activation_phase_.load(std::memory_order_acquire) != RtActivationPhase::kIdle) {
    return controller_interface::CallbackReturn::ERROR;
  }

  const int64_t requested_arm_count = get_node()->get_parameter("arm_count").as_int();
  if (requested_arm_count != 1 && static_cast<size_t>(requested_arm_count) != kArmCount) {
    RCLCPP_ERROR(get_node()->get_logger(), "arm_count must be exactly 1 or %zu, got %ld", kArmCount,
                 static_cast<long>(requested_arm_count));
    return controller_interface::CallbackReturn::FAILURE;
  }
  const size_t arm_count = static_cast<size_t>(requested_arm_count);
  if (arm_count < kArmCount && hasOverriddenParameterWithPrefix(*get_node(), "arm_2.")) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "arm_2.* parameters must not be set when arm_count is 1");
    return controller_interface::CallbackReturn::FAILURE;
  }

  for (size_t arm = 0; arm < arm_count; ++arm) {
    const auto prefix = "arm_" + std::to_string(arm + 1) + ".";
    const auto arm_id = get_node()->get_parameter(prefix + "arm_id").as_string();
    const auto k_gains = get_node()->get_parameter(prefix + "k_gains").as_double_array();
    const auto d_gains = get_node()->get_parameter(prefix + "d_gains").as_double_array();
    const auto max_effort = get_node()->get_parameter(prefix + "max_effort").as_double_array();
    if (!isAsciiArmId(arm_id)) {
      RCLCPP_ERROR(
          get_node()->get_logger(),
          "%sarm_id must contain 1..64 ASCII characters, start with a letter, and then contain "
          "only letters, digits, or '_'",
          prefix.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    if (!validGains(k_gains) || !validGains(d_gains)) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "%sk_gains and d_gains must each contain seven finite nonnegative values",
                   prefix.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    if (!validEffortBounds(max_effort)) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "%smax_effort must contain seven finite positive values no greater than "
                   "[87, 87, 87, 87, 12, 12, 12]",
                   prefix.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
    arms_[arm].arm_id = arm_id;
    std::copy(k_gains.begin(), k_gains.end(), arms_[arm].k_gains.begin());
    std::copy(d_gains.begin(), d_gains.end(), arms_[arm].d_gains.begin());
    std::copy(max_effort.begin(), max_effort.end(), arms_[arm].max_effort.begin());
    // F-10c (design §3.3): precompute here, once, on the service thread, so the owner thread's
    // first-update bind (bindArmInterfaces(), called from update()) never allocates.
    for (size_t joint = 0; joint < kJointCount; ++joint) {
      arms_[arm].position_interface_names[joint] =
          jointInterfaceName(arm_id, joint, hardware_interface::HW_IF_POSITION);
      arms_[arm].velocity_interface_names[joint] =
          jointInterfaceName(arm_id, joint, hardware_interface::HW_IF_VELOCITY);
      arms_[arm].effort_interface_names[joint] =
          jointInterfaceName(arm_id, joint, hardware_interface::HW_IF_EFFORT);
    }
    arms_[arm].robot_state_interface_name = arm_id + "/robot_state";
    arms_[arm].robot_model_interface_name = arm_id + "/robot_model";
  }
  if (arm_count == kArmCount && arms_[0].arm_id == arms_[1].arm_id) {
    RCLCPP_ERROR(get_node()->get_logger(), "The two hold-controller arm IDs must be unique");
    return controller_interface::CallbackReturn::FAILURE;
  }

  arm_count_ = arm_count;
  configured_ = true;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DualArmJointHoldController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (!configured_) {
    return controller_interface::CallbackReturn::FAILURE;
  }
  // F-10c amendment A.4: read-only wiring validation, restored here so a mis-wired controller
  // fails activation *externally* again (controller_manager reports the failure to the caller)
  // instead of activating and then erroring out one update() cycle later. This reads
  // command_interfaces_/state_interfaces_ and each entry's name; it stores nothing, binds no
  // pointer, decodes no robot_state/robot_model pointer, and dereferences nothing the owner
  // thread writes. controller_manager populates both vectors once, before on_activate(), and
  // update() never resizes them, so the read cannot race the owner thread -- and reading a loaned
  // interface is not a write, so it does not cross the ownership rule (design 3.1).
  //
  // Binding and activation capture stay exactly where design 3.2 put them: on the owner thread's
  // first update() cycle (RtActivationPhase above, serviceFirstUpdateActivation() below).
  if (!validateInterfaceWiring()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Hold controller activation rejected: the loaned interfaces do not match the "
                 "configured arms exactly");
    return controller_interface::CallbackReturn::FAILURE;
  }
  rt_activation_phase_.store(RtActivationPhase::kRequested, std::memory_order_release);
  return controller_interface::CallbackReturn::SUCCESS;
}

// F-10c amendment A (2026-08-28): none of on_deactivate()/on_cleanup()/on_error()/on_shutdown()
// writes a command interface any more -- see A.1 of the design. Each may run on
// controller_manager's service thread, concurrently with the owner thread's update() and with
// FrankaMultiHardwareInterface::write(); a zero written from here would race write()'s read of
// the same arm.hw_commands_* storage (measured: 42-44 ThreadSanitizer reports per production run)
// while duplicating a safe command the hardware layer has already published on the owner thread,
// before controller_manager reached this callback at all (perform_command_mode_switch() ->
// applyPreparedTransactionEffects() -> safeCommandForArm() -> publishCommand() ->
// requestControlMode(None)). The controller's own zero-on-error writes are retained where they
// belong: inside update(), on the owner thread.
//
// All four therefore only publish kIdle so update() stops computing real commands, and report
// SUCCESS. There is no longer any zero-write failure for them to surface, which is why the old
// release-zero failure flag is gone.

controller_interface::CallbackReturn DualArmJointHoldController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  rt_activation_phase_.store(RtActivationPhase::kIdle, std::memory_order_release);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DualArmJointHoldController::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  configured_ = false;
  rt_activation_phase_.store(RtActivationPhase::kIdle, std::memory_order_release);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DualArmJointHoldController::on_error(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  rt_activation_phase_.store(RtActivationPhase::kIdle, std::memory_order_release);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DualArmJointHoldController::on_shutdown(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  configured_ = false;
  rt_activation_phase_.store(RtActivationPhase::kIdle, std::memory_order_release);
  return controller_interface::CallbackReturn::SUCCESS;
}

size_t DualArmJointHoldController::countStateInterfaces(const std::string& name) const noexcept {
  size_t matches = 0;
  for (const auto& interface : state_interfaces_) {
    if (interface.get_name() == name) {
      ++matches;
    }
  }
  return matches;
}

size_t DualArmJointHoldController::countCommandInterfaces(const std::string& name) const noexcept {
  size_t matches = 0;
  for (const auto& interface : command_interfaces_) {
    if (interface.get_name() == name) {
      ++matches;
    }
  }
  return matches;
}

bool DualArmJointHoldController::validateInterfaceWiring() const noexcept {
  // Read-only (F-10c amendment A.4). Exact counts, exact names, each present exactly once.
  // "Exactly once each" over a set of names whose size equals the vector's size is also the
  // no-aliasing property: every configured name resolves to its own distinct interface, so no two
  // arms -- and no two joints -- can share one. LoanedStateInterface::get_name() returns a const
  // reference, so the comparisons below allocate nothing.
  const size_t expected_command_count = arm_count_ * kJointCount;
  const size_t expected_state_count =
      arm_count_ * (2 * kJointCount + kStateInterfacesPerArmOverhead);
  if (command_interfaces_.size() != expected_command_count ||
      state_interfaces_.size() != expected_state_count) {
    return false;
  }
  for (size_t arm = 0; arm < arm_count_; ++arm) {
    for (size_t joint = 0; joint < kJointCount; ++joint) {
      if (countStateInterfaces(arms_[arm].position_interface_names[joint]) != 1 ||
          countStateInterfaces(arms_[arm].velocity_interface_names[joint]) != 1 ||
          countCommandInterfaces(arms_[arm].effort_interface_names[joint]) != 1) {
        return false;
      }
    }
    if (countStateInterfaces(arms_[arm].robot_state_interface_name) != 1 ||
        countStateInterfaces(arms_[arm].robot_model_interface_name) != 1) {
      return false;
    }
  }
  return true;
}

bool DualArmJointHoldController::bindInterfaces() {
  resetBindings();
  const size_t expected_command_count = arm_count_ * kJointCount;
  const size_t expected_state_count =
      arm_count_ * (2 * kJointCount + kStateInterfacesPerArmOverhead);
  if (command_interfaces_.size() != expected_command_count ||
      state_interfaces_.size() != expected_state_count) {
    return false;
  }
  for (size_t arm = 0; arm < arm_count_; ++arm) {
    if (!bindArmInterfaces(arms_[arm])) {
      resetBindings();
      return false;
    }
  }
  if (arm_count_ == kArmCount && (arms_[0].robot_state == arms_[1].robot_state ||
                                  arms_[0].robot_model == arms_[1].robot_model)) {
    resetBindings();
    return false;
  }
  interfaces_bound_ = true;
  zero_required_ = true;
  rt_ever_bound_.store(true, std::memory_order_release);
  return true;
}

bool DualArmJointHoldController::bindArmInterfaces(Arm& arm) {
  // F-10c (design §3.3): only string *comparisons* against the names on_configure() precomputed
  // -- see findUniqueStateInterface()/findUniqueCommandInterface() -- no allocation. This runs on
  // the control-cycle owner thread (see serviceFirstUpdateActivation()).
  for (size_t joint = 0; joint < kJointCount; ++joint) {
    arm.position_interfaces[joint] = findUniqueStateInterface(arm.position_interface_names[joint]);
    arm.velocity_interfaces[joint] = findUniqueStateInterface(arm.velocity_interface_names[joint]);
    arm.effort_interfaces[joint] = findUniqueCommandInterface(arm.effort_interface_names[joint]);
    if (arm.position_interfaces[joint] == nullptr || arm.velocity_interfaces[joint] == nullptr ||
        arm.effort_interfaces[joint] == nullptr) {
      return false;
    }
  }

  arm.robot_state_interface = findUniqueStateInterface(arm.robot_state_interface_name);
  arm.robot_model_interface = findUniqueStateInterface(arm.robot_model_interface_name);
  if (arm.robot_state_interface == nullptr || arm.robot_model_interface == nullptr ||
      !decodeStablePointer(*arm.robot_state_interface, arm.robot_state) ||
      !decodeStablePointer(*arm.robot_model_interface, arm.robot_model)) {
    return false;
  }
  return true;
}

bool DualArmJointHoldController::captureActivationState() {
  // F-10c: bindInterfaces()/captureActivationState() now run exclusively from
  // serviceFirstUpdateActivation(), i.e. only ever on the control-cycle owner thread's own first
  // update() cycle after activation -- the same thread that assignState() uses to write
  // *arm.robot_state every read() cycle. Reading it directly here is therefore no longer a
  // cross-thread race (see F-10b's now-removed RtActivationSnapshot for the previous, off-owner
  // version of this function and why it needed a seqlock).
  for (size_t arm_index = 0; arm_index < arm_count_; ++arm_index) {
    auto& arm = arms_[arm_index];
    if (arm.robot_state == nullptr || arm.robot_model == nullptr ||
        !finiteModelInput(*arm.robot_state)) {
      return false;
    }
    for (size_t joint = 0; joint < kJointCount; ++joint) {
      arm.hold_position[joint] = arm.robot_state->q[joint];
      arm.filtered_velocity[joint] = arm.robot_state->dq[joint];
    }
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

void DualArmJointHoldController::serviceFirstUpdateActivation() noexcept {
  // Only ever called from update() on the control-cycle owner thread.
  const bool bound = bindInterfaces();
  const bool activation_state_valid = bound && captureActivationState();
  const bool zeroed = bound && attemptRequiredZero();
  const RtActivationPhase desired = (bound && activation_state_valid && zeroed)
                                        ? RtActivationPhase::kActive
                                        : RtActivationPhase::kFailed;
  RtActivationPhase expected = RtActivationPhase::kRequested;
  if (!rt_activation_phase_.compare_exchange_strong(expected, desired, std::memory_order_acq_rel,
                                                    std::memory_order_acquire)) {
    // A concurrent on_deactivate()/on_error()/on_shutdown()/release_interfaces() already reset
    // the phase (to kIdle) while this bind/capture was in flight -- e.g. an off-owner activate
    // immediately followed by an off-owner deactivate before this, the owner thread's first
    // cycle, ran. Do not resurrect activity by overwriting whatever that callback published:
    // leave the phase exactly as it left it. If bindInterfaces() did succeed above,
    // interfaces_bound_/zero_required_ already reflect that, so update()'s own
    // `interfaces_bound_ && !active` fallthrough (this function's caller) re-zeros on this very
    // cycle regardless of the outcome here.
  }
}

bool DualArmJointHoldController::computeCommands(
    std::array<std::array<double, kJointCount>, kArmCount>& efforts,
    std::array<std::array<double, kJointCount>, kArmCount>& next_filtered_velocity) const {
  for (size_t arm_index = 0; arm_index < arm_count_; ++arm_index) {
    const auto& arm = arms_[arm_index];
    if (arm.robot_state == nullptr || arm.robot_model == nullptr ||
        !pointerStillMatches(arm.robot_state_interface, arm.robot_state) ||
        !pointerStillMatches(arm.robot_model_interface, arm.robot_model) ||
        !finiteModelInput(*arm.robot_state)) {
      return false;
    }

    std::array<double, kJointCount> positions{};
    std::array<double, kJointCount> velocities{};
    for (size_t joint = 0; joint < kJointCount; ++joint) {
      if (!readFinite(arm.position_interfaces[joint], positions[joint]) ||
          !readFinite(arm.velocity_interfaces[joint], velocities[joint])) {
        return false;
      }
      next_filtered_velocity[arm_index][joint] =
          (1.0 - kVelocityFilterAlpha) * arm.filtered_velocity[joint] +
          kVelocityFilterAlpha * velocities[joint];
    }

    std::array<double, kJointCount> coriolis{};
    try {
      coriolis = arm.robot_model->coriolis(*arm.robot_state);
    } catch (...) {
      return false;
    }
    for (size_t joint = 0; joint < kJointCount; ++joint) {
      if (!std::isfinite(coriolis[joint])) {
        return false;
      }
      efforts[arm_index][joint] =
          arm.k_gains[joint] * (arm.hold_position[joint] - positions[joint]) -
          arm.d_gains[joint] * next_filtered_velocity[arm_index][joint] + coriolis[joint];
      if (!std::isfinite(efforts[arm_index][joint])) {
        return false;
      }
      if (std::abs(efforts[arm_index][joint]) > arm.max_effort[joint]) {
        return false;
      }
    }
  }
  return true;
}

bool DualArmJointHoldController::writeCommands(
    const std::array<std::array<double, kJointCount>, kArmCount>& efforts) noexcept {
  bool all_written = true;
  for (size_t arm = 0; arm < arm_count_; ++arm) {
    for (size_t joint = 0; joint < kJointCount; ++joint) {
      const bool written = writeEffort(arms_[arm].effort_interfaces[joint], efforts[arm][joint]);
      all_written = written && all_written;
    }
  }
  return all_written;
}

bool DualArmJointHoldController::writeZeroEffort() noexcept {
  bool all_written = true;
  for (size_t arm = 0; arm < arm_count_; ++arm) {
    for (auto* interface : arms_[arm].effort_interfaces) {
      const bool written = writeEffort(interface, 0.0);
      all_written = written && all_written;
    }
  }
  return all_written;
}

bool DualArmJointHoldController::attemptRequiredZero() noexcept {
  if (!interfaces_bound_ || !zero_required_) {
    return true;
  }
  if (!writeZeroEffort()) {
    return false;
  }
  zero_required_ = false;
  return true;
}

void DualArmJointHoldController::resetBindings() noexcept {
  interfaces_bound_ = false;
  zero_required_ = false;
  rt_ever_bound_.store(false, std::memory_order_release);
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

hardware_interface::LoanedStateInterface* DualArmJointHoldController::findUniqueStateInterface(
    const std::string& name) noexcept {
  hardware_interface::LoanedStateInterface* match = nullptr;
  for (auto& interface : state_interfaces_) {
    if (interface.get_name() == name) {
      if (match != nullptr) {
        return nullptr;
      }
      match = &interface;
    }
  }
  return match;
}

hardware_interface::LoanedCommandInterface* DualArmJointHoldController::findUniqueCommandInterface(
    const std::string& name) noexcept {
  hardware_interface::LoanedCommandInterface* match = nullptr;
  for (auto& interface : command_interfaces_) {
    if (interface.get_name() == name) {
      if (match != nullptr) {
        return nullptr;
      }
      match = &interface;
    }
  }
  return match;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::DualArmJointHoldController,
                       controller_interface::ControllerInterface)
