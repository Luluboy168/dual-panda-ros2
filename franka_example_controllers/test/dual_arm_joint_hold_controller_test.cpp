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

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <franka/model.h>
#include <franka/robot_state.h>
#include <controller_interface/test_utils.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include "franka_example_controllers/dual_arm_joint_hold_controller.hpp"
#include "franka_example_controllers/panda_joint_limits.hpp"
#include "franka_hardware/common/model_base.hpp"

namespace franka_example_controllers {
namespace {

constexpr size_t kArmCount = 2;
constexpr size_t kJointCount = 7;
constexpr size_t kCommandCount = kArmCount * kJointCount;

using JointArray = std::array<double, kJointCount>;
using ArmIds = std::array<std::string, kArmCount>;

class FixedCoriolisModel final : public franka_hardware::ModelBase {
 public:
  std::array<double, 7> coriolis(const franka::RobotState& /*robot_state*/) const override {
    return coriolis_;
  }

  void setCoriolis(const JointArray& coriolis) { coriolis_ = coriolis; }

 private:
  std::array<double, 16> poseImpl(franka::Frame /*frame*/,
                                  const std::array<double, 7>& /*q*/,
                                  const std::array<double, 16>& /*f_t_ee*/,
                                  const std::array<double, 16>& /*ee_t_k*/) const override {
    return {};
  }

  std::array<double, 42> bodyJacobianImpl(franka::Frame /*frame*/,
                                          const std::array<double, 7>& /*q*/,
                                          const std::array<double, 16>& /*f_t_ee*/,
                                          const std::array<double, 16>& /*ee_t_k*/) const override {
    return {};
  }

  std::array<double, 42> zeroJacobianImpl(franka::Frame /*frame*/,
                                          const std::array<double, 7>& /*q*/,
                                          const std::array<double, 16>& /*f_t_ee*/,
                                          const std::array<double, 16>& /*ee_t_k*/) const override {
    return {};
  }

  std::array<double, 49> massImpl(const std::array<double, 7>& /*q*/,
                                  const std::array<double, 9>& /*i_total*/,
                                  double /*m_total*/,
                                  const std::array<double, 3>& /*f_x_ctotal*/) const override {
    return {};
  }

  std::array<double, 7> coriolisImpl(const std::array<double, 7>& /*q*/,
                                     const std::array<double, 7>& /*dq*/,
                                     const std::array<double, 9>& /*i_total*/,
                                     double /*m_total*/,
                                     const std::array<double, 3>& /*f_x_ctotal*/) const override {
    return coriolis_;
  }

  std::array<double, 7> gravityImpl(const std::array<double, 7>& /*q*/,
                                    double /*m_total*/,
                                    const std::array<double, 3>& /*f_x_ctotal*/,
                                    const std::array<double, 3>& /*gravity_earth*/) const override {
    return {};
  }

  JointArray coriolis_{};
};

std::string jointName(const std::string& arm_id, const size_t joint) {
  return arm_id + "_joint" + std::to_string(joint + 1);
}

// Stable, comparable identity for the OS thread that performed a command write. Used by
// OffOwnerDeactivateWritesNothingAndTheOwnerThreadDoesTheZeroing below to prove -- rather than
// infer from timing -- that every command write really happened on the control-cycle owner
// thread and never on the deactivating lifecycle thread (F-10c amendment A.1).
unsigned long currentThreadIdentity() noexcept {
  return static_cast<unsigned long>(std::hash<std::thread::id>{}(std::this_thread::get_id()));
}

class ControllerHardwareFixture {
 public:
  explicit ControllerHardwareFixture(ArmIds arm_ids) : arm_ids_(std::move(arm_ids)) {
    state_handles_.reserve(32);
    command_handles_.reserve(kCommandCount);
    for (size_t arm = 0; arm < kArmCount; ++arm) {
      state_pointers_[arm] = &robot_states_[arm];
      model_pointers_[arm] = &models_[arm];
      JointArray coriolis{};
      for (size_t joint = 0; joint < kJointCount; ++joint) {
        coriolis[joint] = static_cast<double>(arm + 1) + 0.1 * static_cast<double>(joint);
        state_handles_.push_back(std::make_shared<hardware_interface::StateInterface>(
            jointName(arm_ids_[arm], joint), "position", &positions_[arm][joint]));
        state_handles_.push_back(std::make_shared<hardware_interface::StateInterface>(
            jointName(arm_ids_[arm], joint), "velocity", &velocities_[arm][joint]));

        const auto command_index = command_handles_.size();
        command_handles_.push_back(std::make_shared<hardware_interface::CommandInterface>(
            jointName(arm_ids_[arm], joint), "effort", &commands_[arm][joint]));
        command_handles_.back()->set_on_set_command_limiter(
            [this, command_index](const double value, bool& limited) {
              write_counts_[command_index].fetch_add(1, std::memory_order_relaxed);
              write_thread_ids_[command_index].store(currentThreadIdentity(),
                                                     std::memory_order_relaxed);
              limited = false;
              return value;
            });
      }
      models_[arm].setCoriolis(coriolis);
      state_handles_.push_back(std::make_shared<hardware_interface::StateInterface>(
          arm_ids_[arm], "robot_state",
          reinterpret_cast<double*>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
              &state_pointers_[arm])));
      state_handles_.push_back(std::make_shared<hardware_interface::StateInterface>(
          arm_ids_[arm], "robot_model",
          reinterpret_cast<double*>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
              &model_pointers_[arm])));
    }
  }

  void setJointState(const size_t arm, const JointArray& position, const JointArray& velocity) {
    positions_[arm] = position;
    velocities_[arm] = velocity;
    robot_states_[arm].q = position;
    robot_states_[arm].dq = velocity;
  }

  void setPosition(const size_t arm, const size_t joint, const double position) {
    positions_[arm][joint] = position;
    robot_states_[arm].q[joint] = position;
  }

  void setCoriolis(const size_t arm, const JointArray& coriolis) {
    models_[arm].setCoriolis(coriolis);
  }

  franka::RobotState& robotState(const size_t arm) { return robot_states_[arm]; }

  void setNullStatePointer(const size_t arm) { state_pointers_[arm] = nullptr; }

  void setNullModelPointer(const size_t arm) { model_pointers_[arm] = nullptr; }

  void aliasStatePointer(const size_t destination_arm, const size_t source_arm) {
    state_pointers_[destination_arm] = state_pointers_[source_arm];
  }

  void replaceStateInterface(const std::string& expected_name,
                             const std::string& replacement_prefix,
                             const std::string& replacement_interface) {
    const auto iterator =
        std::find_if(state_handles_.begin(), state_handles_.end(),
                     [&](const auto& interface) { return interface->get_name() == expected_name; });
    if (iterator == state_handles_.end()) {
      throw std::runtime_error("state interface to replace was not found");
    }
    *iterator = std::make_shared<hardware_interface::StateInterface>(
        replacement_prefix, replacement_interface, &replacement_state_value_);
  }

  void duplicateStateInterface(const std::string& missing_name, const std::string& duplicate_name) {
    const auto missing =
        std::find_if(state_handles_.begin(), state_handles_.end(),
                     [&](const auto& interface) { return interface->get_name() == missing_name; });
    const auto duplicate = std::find_if(
        state_handles_.begin(), state_handles_.end(),
        [&](const auto& interface) { return interface->get_name() == duplicate_name; });
    if (missing == state_handles_.end() || duplicate == state_handles_.end()) {
      throw std::runtime_error("state interface for duplicate injection was not found");
    }
    *missing = *duplicate;
  }

  void replaceCommandInterface(const std::string& expected_name,
                               const std::string& replacement_prefix) {
    const auto iterator =
        std::find_if(command_handles_.begin(), command_handles_.end(),
                     [&](const auto& interface) { return interface->get_name() == expected_name; });
    if (iterator == command_handles_.end()) {
      throw std::runtime_error("command interface to replace was not found");
    }
    *iterator = std::make_shared<hardware_interface::CommandInterface>(replacement_prefix, "effort",
                                                                       &replacement_command_value_);
  }

  void assignTo(DualArmJointHoldController& controller, const bool shuffled) const {
    auto command_order = command_handles_;
    auto state_order = state_handles_;
    if (shuffled) {
      std::reverse(command_order.begin(), command_order.end());
      std::rotate(state_order.begin(), state_order.begin() + 11, state_order.end());
      std::reverse(state_order.begin(), state_order.end());
    }
    std::vector<hardware_interface::LoanedCommandInterface> command_interfaces;
    std::vector<hardware_interface::LoanedStateInterface> state_interfaces;
    command_interfaces.reserve(command_order.size());
    state_interfaces.reserve(state_order.size());
    for (const auto& interface : command_order) {
      command_interfaces.emplace_back(interface,
                                      hardware_interface::LoanedCommandInterface::Deleter{});
    }
    for (const auto& interface : state_order) {
      state_interfaces.emplace_back(interface);
    }
    controller.assign_interfaces(std::move(command_interfaces), std::move(state_interfaces));
  }

  void fillCommands(const double value) {
    for (auto& arm_commands : commands_) {
      arm_commands.fill(value);
    }
  }

  double command(const size_t arm, const size_t joint) const { return commands_[arm][joint]; }

  bool allCommandsEqual(const double expected) const {
    for (const auto& arm_commands : commands_) {
      if (!std::all_of(arm_commands.begin(), arm_commands.end(),
                       [&](const double command) { return command == expected; })) {
        return false;
      }
    }
    return true;
  }

  void resetWriteCounts() {
    for (size_t command = 0; command < kCommandCount; ++command) {
      write_counts_[command].store(0, std::memory_order_relaxed);
      write_thread_ids_[command].store(0, std::memory_order_relaxed);
    }
  }

  bool everyCommandWasWrittenTwice() const { return everyCommandWasWritten(2); }

  bool everyCommandWasWritten(const size_t expected_count) const {
    for (size_t command = 0; command < kCommandCount; ++command) {
      if (writeCount(command) != expected_count) {
        return false;
      }
    }
    return true;
  }

  size_t writeCount(const size_t command) const {
    return write_counts_.at(command).load(std::memory_order_relaxed);
  }

  // Identity of the thread that most recently wrote this command interface (0 if none since the
  // last resetWriteCounts()).
  unsigned long writeThreadIdentity(const size_t command) const {
    return write_thread_ids_.at(command).load(std::memory_order_relaxed);
  }

  hardware_interface::CommandInterface::SharedPtr commandHandle(const std::string& name) const {
    const auto iterator =
        std::find_if(command_handles_.begin(), command_handles_.end(),
                     [&](const auto& interface) { return interface->get_name() == name; });
    if (iterator == command_handles_.end()) {
      throw std::runtime_error("command interface was not found");
    }
    return *iterator;
  }

 private:
  ArmIds arm_ids_;
  std::array<JointArray, kArmCount> positions_{};
  std::array<JointArray, kArmCount> velocities_{};
  std::array<JointArray, kArmCount> commands_{};
  std::array<franka::RobotState, kArmCount> robot_states_{};
  std::array<FixedCoriolisModel, kArmCount> models_{};
  std::array<franka::RobotState*, kArmCount> state_pointers_{};
  std::array<franka_hardware::ModelBase*, kArmCount> model_pointers_{};
  std::vector<hardware_interface::StateInterface::SharedPtr> state_handles_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_handles_;
  std::array<std::atomic<size_t>, kCommandCount> write_counts_{};
  std::array<std::atomic<unsigned long>, kCommandCount> write_thread_ids_{};
  double replacement_state_value_{0.0};
  double replacement_command_value_{0.0};
};

struct ControllerParameters {
  ArmIds arm_ids{"arm", "arm_extra"};
  std::array<std::vector<double>, kArmCount> k_gains{std::vector<double>(kJointCount, 10.0),
                                                     std::vector<double>(kJointCount, 20.0)};
  std::array<std::vector<double>, kArmCount> d_gains{std::vector<double>(kJointCount, 1.0),
                                                     std::vector<double>(kJointCount, 2.0)};
  std::array<std::vector<double>, kArmCount> max_effort{
      std::vector<double>{87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0},
      std::vector<double>{87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0}};
};

std::unique_ptr<DualArmJointHoldController> makeController(const ControllerParameters& parameters) {
  rclcpp::NodeOptions node_options;
  node_options.enable_rosout(false);
  node_options.start_parameter_event_publisher(false);
  node_options.start_parameter_services(false);
  node_options.parameter_overrides({
      rclcpp::Parameter("arm_1.arm_id", parameters.arm_ids[0]),
      rclcpp::Parameter("arm_1.k_gains", parameters.k_gains[0]),
      rclcpp::Parameter("arm_1.d_gains", parameters.d_gains[0]),
      rclcpp::Parameter("arm_1.max_effort", parameters.max_effort[0]),
      rclcpp::Parameter("arm_2.arm_id", parameters.arm_ids[1]),
      rclcpp::Parameter("arm_2.k_gains", parameters.k_gains[1]),
      rclcpp::Parameter("arm_2.d_gains", parameters.d_gains[1]),
      rclcpp::Parameter("arm_2.max_effort", parameters.max_effort[1]),
  });
  controller_interface::ControllerInterfaceParams controller_parameters;
  controller_parameters.controller_name = "dual_arm_joint_hold_controller_test";
  controller_parameters.update_rate = 1000;
  controller_parameters.controller_manager_update_rate = 1000;
  controller_parameters.node_options = node_options;

  auto controller = std::make_unique<DualArmJointHoldController>();
  if (controller->init(controller_parameters) != controller_interface::return_type::OK) {
    throw std::runtime_error("controller initialization failed");
  }
  return controller;
}

bool configure(const std::unique_ptr<DualArmJointHoldController>& controller) {
  return controller_interface::configure_succeeds(controller);
}

bool activate(const std::unique_ptr<DualArmJointHoldController>& controller) {
  return controller_interface::activate_succeeds(controller);
}

controller_interface::return_type update(DualArmJointHoldController& controller) {
  return controller.update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration(0, 1000000));
}

class DualArmJointHoldControllerTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(DualArmJointHoldControllerTest, DeclaresExactPerArmInterfaceNames) {
  ControllerParameters parameters;
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));

  const auto commands = controller->command_interface_configuration();
  const auto states = controller->state_interface_configuration();
  ASSERT_EQ(commands.names.size(), 14U);
  // 32 = 2 arms * (2*7 joint interfaces + 2 [robot_state, robot_model]).
  ASSERT_EQ(states.names.size(), 32U);
  for (size_t arm = 0; arm < kArmCount; ++arm) {
    for (size_t joint = 0; joint < kJointCount; ++joint) {
      const auto joint_name = jointName(parameters.arm_ids[arm], joint);
      EXPECT_EQ(std::count(commands.names.begin(), commands.names.end(), joint_name + "/effort"),
                1);
      EXPECT_EQ(std::count(states.names.begin(), states.names.end(), joint_name + "/position"), 1);
      EXPECT_EQ(std::count(states.names.begin(), states.names.end(), joint_name + "/velocity"), 1);
    }
    EXPECT_EQ(std::count(states.names.begin(), states.names.end(),
                         parameters.arm_ids[arm] + "/robot_state"),
              1);
    EXPECT_EQ(std::count(states.names.begin(), states.names.end(),
                         parameters.arm_ids[arm] + "/robot_model"),
              1);
  }
}

TEST_F(DualArmJointHoldControllerTest, RejectsDuplicateAndInvalidArmIds) {
  const std::array<ArmIds, 7> invalid_ids{{
      {"same", "same"},
      {"", "valid"},
      {"1leading_digit", "valid"},
      {"_leading_underscore", "valid"},
      {"slash/name", "valid"},
      {"white space", "valid"},
      {"non_ascii_\xC3\xA9", "valid"},
  }};
  for (const auto& arm_ids : invalid_ids) {
    SCOPED_TRACE(arm_ids[0] + "," + arm_ids[1]);
    ControllerParameters parameters;
    parameters.arm_ids = arm_ids;
    auto controller = makeController(parameters);
    EXPECT_FALSE(configure(controller));
  }
}

TEST_F(DualArmJointHoldControllerTest, Accepts64CharacterArmIdAndRejects65) {
  ControllerParameters maximum_length;
  maximum_length.arm_ids[0] = "a" + std::string(kPandaArmIdMaxLength - 1U, 'x');
  EXPECT_TRUE(configure(makeController(maximum_length)));

  ControllerParameters too_long;
  too_long.arm_ids[0] = "a" + std::string(kPandaArmIdMaxLength, 'x');
  EXPECT_FALSE(configure(makeController(too_long)));
}

TEST_F(DualArmJointHoldControllerTest, RejectsWrongLengthNonfiniteAndNegativeGains) {
  for (size_t scenario = 0; scenario < 6; ++scenario) {
    SCOPED_TRACE(scenario);
    ControllerParameters parameters;
    switch (scenario) {
      case 0:
        parameters.k_gains[0].pop_back();
        break;
      case 1:
        parameters.d_gains[1].push_back(1.0);
        break;
      case 2:
        parameters.k_gains[1][3] = std::numeric_limits<double>::quiet_NaN();
        break;
      case 3:
        parameters.d_gains[0][4] = std::numeric_limits<double>::infinity();
        break;
      case 4:
        parameters.k_gains[0][0] = -0.01;
        break;
      case 5:
        parameters.d_gains[1][6] = -0.01;
        break;
      default:
        FAIL() << "unhandled gain scenario";
    }
    auto controller = makeController(parameters);
    EXPECT_FALSE(configure(controller));
  }
}

TEST_F(DualArmJointHoldControllerTest, RejectsInvalidAndAbovePandaCeilingEffortBounds) {
  for (size_t scenario = 0; scenario < 8; ++scenario) {
    SCOPED_TRACE(scenario);
    ControllerParameters parameters;
    switch (scenario) {
      case 0:
        parameters.max_effort[0].pop_back();
        break;
      case 1:
        parameters.max_effort[1].push_back(1.0);
        break;
      case 2:
        parameters.max_effort[0][0] = 0.0;
        break;
      case 3:
        parameters.max_effort[1][2] = -1.0;
        break;
      case 4:
        parameters.max_effort[0][3] = std::numeric_limits<double>::quiet_NaN();
        break;
      case 5:
        parameters.max_effort[1][4] = std::numeric_limits<double>::infinity();
        break;
      case 6:
        parameters.max_effort[0][0] = std::nextafter(87.0, 88.0);
        break;
      case 7:
        parameters.max_effort[1][6] = std::nextafter(12.0, 13.0);
        break;
      default:
        FAIL() << "unhandled effort-bound scenario";
    }
    EXPECT_FALSE(configure(makeController(parameters)));
  }
}

TEST_F(DualArmJointHoldControllerTest,
       ShuffledInterfacesAndSubstringArmIdsHoldArbitraryActivationPoses) {
  ControllerParameters parameters;
  ControllerHardwareFixture hardware(parameters.arm_ids);
  const JointArray first_position{-0.4, 0.2, 0.7, -1.1, 0.3, 1.0, -0.8};
  const JointArray second_position{0.6, -0.1, -0.5, 1.2, -0.7, 0.9, 0.4};
  hardware.setJointState(0, first_position, {});
  hardware.setJointState(1, second_position, {});
  hardware.fillCommands(-999.0);

  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, true);
  // First-update capture (F-10c): on_activate() only posts a request now -- bindArmInterfaces()/
  // captureActivationState() and the resulting zero-effort write all happen inside this first
  // update() call, on the owner thread, immediately followed by the real command computed from
  // the just-captured hold position (both writes land within this one call, so only the final,
  // real-valued state is externally observable here).
  ASSERT_TRUE(activate(controller));
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);
  for (size_t arm = 0; arm < kArmCount; ++arm) {
    for (size_t joint = 0; joint < kJointCount; ++joint) {
      EXPECT_DOUBLE_EQ(hardware.command(arm, joint),
                       static_cast<double>(arm + 1) + 0.1 * static_cast<double>(joint));
    }
  }

  ASSERT_EQ(controller->update(rclcpp::Time(1000, 0, RCL_ROS_TIME), rclcpp::Duration(10, 0)),
            controller_interface::return_type::OK);
  for (size_t arm = 0; arm < kArmCount; ++arm) {
    for (size_t joint = 0; joint < kJointCount; ++joint) {
      EXPECT_DOUBLE_EQ(hardware.command(arm, joint),
                       static_cast<double>(arm + 1) + 0.1 * static_cast<double>(joint));
    }
  }
}

TEST_F(DualArmJointHoldControllerTest, RejectsMissingDuplicateAndInexactBindings) {
  for (size_t scenario = 0; scenario < 3; ++scenario) {
    SCOPED_TRACE(scenario);
    ControllerParameters parameters;
    ControllerHardwareFixture hardware(parameters.arm_ids);
    if (scenario == 0) {
      hardware.replaceStateInterface("arm_joint1/position", "arm_extra_joint1", "effort");
    } else if (scenario == 1) {
      hardware.duplicateStateInterface("arm/robot_model", "arm/robot_state");
    } else {
      hardware.replaceCommandInterface("arm_extra_joint7/effort", "arm_joint7");
    }
    auto controller = makeController(parameters);
    ASSERT_TRUE(configure(controller));
    hardware.assignTo(*controller, true);
    // F-10c amendment A.4: activation-time wiring validation is back, as a read-only check, so
    // every one of these substitutions is rejected by on_activate() itself again -- externally
    // visible to controller_manager -- rather than one update() cycle later.
    EXPECT_FALSE(activate(controller));
    EXPECT_EQ(update(*controller), controller_interface::return_type::ERROR);
  }
}

TEST_F(DualArmJointHoldControllerTest, RejectsNullAndCrossArmAliasedDecodedPointers) {
  for (size_t scenario = 0; scenario < 2; ++scenario) {
    SCOPED_TRACE(scenario);
    ControllerParameters parameters;
    ControllerHardwareFixture hardware(parameters.arm_ids);
    if (scenario == 0) {
      hardware.setNullStatePointer(0);
    } else {
      hardware.aliasStatePointer(1, 0);
    }
    auto controller = makeController(parameters);
    ASSERT_TRUE(configure(controller));
    hardware.assignTo(*controller, true);
    // First-update capture (F-10c): decoding happens on the first update() cycle now; see the
    // comment in RejectsMissingDuplicateAndInexactBindings above.
    ASSERT_TRUE(activate(controller));
    EXPECT_EQ(update(*controller), controller_interface::return_type::ERROR);
  }
}

TEST_F(DualArmJointHoldControllerTest, ChangedDecodedPointersFailUpdateToZeroEffort) {
  for (size_t scenario = 0; scenario < 2; ++scenario) {
    SCOPED_TRACE(scenario);
    ControllerParameters parameters;
    ControllerHardwareFixture hardware(parameters.arm_ids);
    auto controller = makeController(parameters);
    ASSERT_TRUE(configure(controller));
    hardware.assignTo(*controller, true);
    ASSERT_TRUE(activate(controller));
    // First-update capture (F-10c): this first update() completes the bind so the pointer
    // corruption below is only visible to computeCommands()'s *runtime* pointerStillMatches()
    // check on the following cycle, which is what this test exercises.
    ASSERT_EQ(update(*controller), controller_interface::return_type::OK);
    hardware.fillCommands(55.0);
    if (scenario == 0) {
      hardware.setNullStatePointer(0);
    } else {
      hardware.setNullModelPointer(1);
    }

    EXPECT_EQ(update(*controller), controller_interface::return_type::ERROR);
    EXPECT_TRUE(hardware.allCommandsEqual(0.0));
  }
}

TEST_F(DualArmJointHoldControllerTest, InitializesVelocityFilterAndLeavesZeroingToTheOwnerThread) {
  ControllerParameters parameters;
  parameters.k_gains = {std::vector<double>(kJointCount, 0.0),
                        std::vector<double>(kJointCount, 0.0)};
  parameters.d_gains = {std::vector<double>(kJointCount, 2.0),
                        std::vector<double>(kJointCount, 3.0)};
  ControllerHardwareFixture hardware(parameters.arm_ids);
  const JointArray first_velocity{0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7};
  const JointArray second_velocity{-0.7, 0.6, -0.5, 0.4, -0.3, 0.2, -0.1};
  hardware.setJointState(0, {}, first_velocity);
  hardware.setJointState(1, {}, second_velocity);

  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, true);
  ASSERT_TRUE(activate(controller));
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);
  for (size_t joint = 0; joint < kJointCount; ++joint) {
    EXPECT_DOUBLE_EQ(hardware.command(0, joint),
                     1.0 + 0.1 * static_cast<double>(joint) - 2.0 * first_velocity[joint]);
    EXPECT_DOUBLE_EQ(hardware.command(1, joint),
                     2.0 + 0.1 * static_cast<double>(joint) - 3.0 * second_velocity[joint]);
  }

  // F-10c amendment A: on_deactivate() writes nothing at all. The safe command that a mode
  // switch requires is published by the hardware layer, on the control-cycle owner thread,
  // before controller_manager reaches this callback (see
  // franka_multi_hardware_interface_mode_fault_test.cpp's
  // DeactivateSwitchPublishesTheOwnerThreadSafeCommandBeforeControllerDeactivation). The
  // command storage therefore still holds the last real command right after deactivation...
  hardware.resetWriteCounts();
  EXPECT_TRUE(controller_interface::deactivate_succeeds(controller));
  EXPECT_TRUE(hardware.everyCommandWasWritten(0));
  for (size_t joint = 0; joint < kJointCount; ++joint) {
    EXPECT_DOUBLE_EQ(hardware.command(0, joint),
                     1.0 + 0.1 * static_cast<double>(joint) - 2.0 * first_velocity[joint]);
  }
  // ...and the controller's own zero lands on the very next owner-thread update() cycle, which
  // is the only thread it is ever allowed to write from.
  EXPECT_EQ(update(*controller), controller_interface::return_type::ERROR);
  EXPECT_TRUE(hardware.allCommandsEqual(0.0));
  EXPECT_TRUE(hardware.everyCommandWasWritten(1));
}

TEST_F(DualArmJointHoldControllerTest, NonfiniteStateModelAndOutputFailToZeroEffort) {
  for (size_t scenario = 0; scenario < 4; ++scenario) {
    SCOPED_TRACE(scenario);
    ControllerParameters parameters;
    ControllerHardwareFixture hardware(parameters.arm_ids);
    JointArray initial_position{};
    initial_position.fill(-std::numeric_limits<double>::max());
    if (scenario == 3) {
      hardware.setJointState(0, initial_position, {});
    }
    auto controller = makeController(parameters);
    ASSERT_TRUE(configure(controller));
    hardware.assignTo(*controller, true);
    ASSERT_TRUE(activate(controller));
    // First-update capture (F-10c): capture the hold target from the *pre-corruption* state --
    // scenario 3 in particular depends on hold_position having been captured before its position
    // is corrupted, exactly like the old synchronous on_activate() used to guarantee.
    ASSERT_EQ(update(*controller), controller_interface::return_type::OK);
    hardware.fillCommands(123.0);

    if (scenario == 0) {
      hardware.setPosition(0, 2, std::numeric_limits<double>::quiet_NaN());
    } else if (scenario == 1) {
      hardware.robotState(1).m_total = std::numeric_limits<double>::infinity();
    } else if (scenario == 2) {
      JointArray coriolis{};
      coriolis[5] = std::numeric_limits<double>::quiet_NaN();
      hardware.setCoriolis(0, coriolis);
    } else {
      hardware.setPosition(0, 0, std::numeric_limits<double>::max());
    }

    EXPECT_EQ(update(*controller), controller_interface::return_type::ERROR);
    EXPECT_TRUE(hardware.allCommandsEqual(0.0));
  }
}

TEST_F(DualArmJointHoldControllerTest, AggregatesAllWritesThenAttemptsZeroOnSetFailure) {
  ControllerParameters parameters;
  ControllerHardwareFixture hardware(parameters.arm_ids);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, true);
  ASSERT_TRUE(activate(controller));
  hardware.fillCommands(77.0);
  hardware.resetWriteCounts();

  auto failing_handle = hardware.commandHandle("arm_joint4/effort");
  std::unique_lock<std::shared_mutex> lock(failing_handle->get_mutex());
  EXPECT_EQ(update(*controller), controller_interface::return_type::ERROR);
  EXPECT_TRUE(hardware.everyCommandWasWrittenTwice());
  lock.unlock();

  for (size_t arm = 0; arm < kArmCount; ++arm) {
    for (size_t joint = 0; joint < kJointCount; ++joint) {
      if (arm == 0 && joint == 3) {
        EXPECT_DOUBLE_EQ(hardware.command(arm, joint), 77.0);
      } else {
        EXPECT_DOUBLE_EQ(hardware.command(arm, joint), 0.0);
      }
    }
  }
}

TEST_F(DualArmJointHoldControllerTest, OverLimitEffortAttemptsAllFourteenZerosAndReturnsError) {
  ControllerParameters parameters;
  parameters.max_effort[0] = std::vector<double>(kJointCount, 0.5);
  ControllerHardwareFixture hardware(parameters.arm_ids);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, true);
  ASSERT_TRUE(activate(controller));
  hardware.fillCommands(77.0);
  hardware.resetWriteCounts();

  EXPECT_EQ(update(*controller), controller_interface::return_type::ERROR);
  EXPECT_TRUE(hardware.everyCommandWasWritten(1));
  EXPECT_TRUE(hardware.allCommandsEqual(0.0));
}

TEST_F(DualArmJointHoldControllerTest,
       ActivationFailureReleaseRequiresCleanupReconfigureAndFreshAssignment) {
  ControllerParameters parameters;
  ControllerHardwareFixture hardware(parameters.arm_ids);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, true);
  hardware.fillCommands(4.0);
  hardware.resetWriteCounts();

  auto failing_handle = hardware.commandHandle("arm_joint4/effort");
  std::unique_lock<std::shared_mutex> lock(failing_handle->get_mutex());
  // First-update capture (F-10c): on_activate()'s restored validation is read-only (amendment
  // A.4), so a *locked* handle -- which only fails writes -- does not affect it and activation
  // still succeeds; the locked joint4 write failure surfaces once the first update() cycle
  // attempts the post-bind zero effort (twice, exactly as in
  // AggregatesAllWritesThenAttemptsZeroOnSetFailure above -- once from
  // serviceFirstUpdateActivation()'s own attemptRequiredZero(), once more from update()'s
  // interfaces_bound_-but-inactive fallthrough).
  ASSERT_TRUE(activate(controller));
  EXPECT_EQ(update(*controller), controller_interface::return_type::ERROR);
  EXPECT_TRUE(hardware.everyCommandWasWrittenTwice());
  // F-10c amendment A: release_interfaces() writes nothing -- no third attempt, no matter what
  // the handles do.
  controller->release_interfaces();
  for (size_t command = 0; command < kCommandCount; ++command) {
    EXPECT_EQ(hardware.writeCount(command), 2U) << "command " << command;
  }
  lock.unlock();
  EXPECT_EQ(update(*controller), controller_interface::return_type::ERROR);
  // First-update capture (F-10c): on_activate() itself no longer detects a *write* failure
  // synchronously (see above), so the lifecycle node is still ACTIVE at this point -- exactly as
  // controller_manager would see it until it notices update() returning ERROR and deactivates the
  // controller itself. Do that explicitly here (a raw unit test has no controller_manager to do
  // it) before re-attempting activation, matching that real-system recovery path.
  ASSERT_TRUE(controller_interface::deactivate_succeeds(controller));
  // The loans were already released above, so the restored activation-time wiring validation
  // rejects this attempt outright -- a fresh assignment is required.
  EXPECT_FALSE(activate(controller));

  ASSERT_TRUE(controller_interface::cleanup_succeeds(controller));
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, false);
  ASSERT_TRUE(activate(controller));
  // First-update capture (F-10c): this call binds fresh (joint4 is no longer locked), captures
  // the (default, zero) pose as the hold target, and -- since position/velocity error is zero --
  // writes the coriolis-only steady-state command, exactly like
  // ShuffledInterfacesAndSubstringArmIdsHoldArbitraryActivationPoses above.
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);
  for (size_t arm = 0; arm < kArmCount; ++arm) {
    for (size_t joint = 0; joint < kJointCount; ++joint) {
      EXPECT_DOUBLE_EQ(hardware.command(arm, joint),
                       static_cast<double>(arm + 1) + 0.1 * static_cast<double>(joint));
    }
  }
  ASSERT_TRUE(controller_interface::deactivate_succeeds(controller));
  controller->release_interfaces();
}

TEST_F(DualArmJointHoldControllerTest, DeactivationCascadeWritesNothingEvenWithAFailingHandle) {
  // F-10c amendment A: the whole deactivate/error/release cascade performs no command-interface
  // write, so a handle that would fail every write no longer has anything to fail -- and the
  // cascade no longer reports ERROR for a zero-write reason. The zero the controller is
  // responsible for is written by update(), on the owner thread, and by nothing else.
  ControllerParameters parameters;
  ControllerHardwareFixture hardware(parameters.arm_ids);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, true);
  ASSERT_TRUE(activate(controller));
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);
  hardware.fillCommands(5.0);
  hardware.resetWriteCounts();

  auto failing_handle = hardware.commandHandle("arm_joint4/effort");
  std::unique_lock<std::shared_mutex> lock(failing_handle->get_mutex());
  ASSERT_TRUE(controller_interface::deactivate_succeeds(controller));
  EXPECT_TRUE(hardware.everyCommandWasWritten(0));
  controller->release_interfaces();
  EXPECT_TRUE(hardware.everyCommandWasWritten(0));
  lock.unlock();
  EXPECT_EQ(update(*controller), controller_interface::return_type::ERROR);
}

TEST_F(DualArmJointHoldControllerTest, ShutdownErrorReleaseLeavesNoDanglingDereference) {
  ControllerParameters parameters;
  auto hardware = std::make_unique<ControllerHardwareFixture>(parameters.arm_ids);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware->assignTo(*controller, true);
  ASSERT_TRUE(activate(controller));
  // First-update capture (F-10c): establish a fully bound, active controller before exercising
  // the failing-shutdown cascade below.
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);
  hardware->fillCommands(6.0);
  hardware->resetWriteCounts();

  auto failing_handle = hardware->commandHandle("arm_joint4/effort");
  std::unique_lock<std::shared_mutex> lock(failing_handle->get_mutex());
  const auto state = controller->get_node()->shutdown();
  // F-10c amendment A: on_shutdown() writes nothing, so it can no longer fail; Jazzy therefore
  // finalizes cleanly instead of routing through on_error. What this test is really about --
  // that nothing dereferences a released loan afterwards -- is unchanged.
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
  EXPECT_TRUE(hardware->everyCommandWasWritten(0));
  controller->release_interfaces();
  EXPECT_TRUE(hardware->everyCommandWasWritten(0));
  lock.unlock();
  failing_handle.reset();
  hardware.reset();

  EXPECT_EQ(update(*controller), controller_interface::return_type::ERROR);
  controller->release_interfaces();
}

TEST_F(DualArmJointHoldControllerTest, CleanupAfterCmStyleReleaseAllowsFreshConfigure) {
  ControllerParameters parameters;
  ControllerHardwareFixture hardware(parameters.arm_ids);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, true);
  ASSERT_TRUE(activate(controller));
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);
  hardware.fillCommands(6.0);
  hardware.resetWriteCounts();

  // F-10c amendment A: neither the deactivate nor the release writes a command interface.
  ASSERT_TRUE(controller_interface::deactivate_succeeds(controller));
  EXPECT_TRUE(hardware.everyCommandWasWritten(0));
  controller->release_interfaces();
  EXPECT_TRUE(hardware.everyCommandWasWritten(0));
  ASSERT_TRUE(controller_interface::cleanup_succeeds(controller));
  ASSERT_TRUE(configure(controller));
}

TEST_F(DualArmJointHoldControllerTest,
       OffOwnerDeactivateWritesNothingAndTheOwnerThreadDoesTheZeroing) {
  // F-10c amendment A regression, live topology at the unit level: on_deactivate() runs on a
  // different thread while the control-cycle owner thread keeps calling update() -- exactly what
  // controller_manager produces with activate_asap=false (see
  // production_controller_manager_integration_test.cpp for the same shape through the real
  // controller_manager, and phase8_evidence/f10c/'s topology probe for the measured thread
  // identities).
  //
  // The rule under test: the deactivating thread writes NO command interface, ever. The zero is
  // written by the owner thread's own next update() cycle, and the arm's real safe command comes
  // from the hardware layer's mode switch, which has already run by this point in production.
  // Reintroduce any lifecycle-thread write and the per-command thread-identity assertions below
  // fail; reintroduce the bounded handshake wait and the latency assertion fails.
  ControllerParameters parameters;
  ControllerHardwareFixture hardware(parameters.arm_ids);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, false);
  ASSERT_TRUE(activate(controller));
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);

  // Everything the main thread does to the fixture happens either before the owner thread starts
  // or after it is joined: the only cross-thread contact this test deliberately makes is the
  // lifecycle callback itself, so any ThreadSanitizer report it produces is a production defect
  // and not harness noise. Progress is observed through an atomic cycle counter, never by
  // reading the fixture's command storage while the owner thread is writing it.
  hardware.fillCommands(7.0);
  hardware.resetWriteCounts();

  std::atomic<bool> stop_owner{false};
  std::atomic<unsigned long> owner_thread_identity{0};
  std::atomic<size_t> owner_cycles{0};
  std::thread owner_thread([&]() {
    owner_thread_identity.store(currentThreadIdentity(), std::memory_order_release);
    while (!stop_owner.load(std::memory_order_acquire)) {
      (void)update(*controller);
      owner_cycles.fetch_add(1, std::memory_order_acq_rel);
      std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
  });
  // Bounded: let the owner thread take over the update cycle before deactivating off-thread.
  for (size_t attempt = 0; attempt < 2000 && owner_cycles.load(std::memory_order_acquire) < 5;
       ++attempt) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_GE(owner_cycles.load(std::memory_order_acquire), 5U);

  const auto lifecycle_thread_identity = currentThreadIdentity();
  const auto started = std::chrono::steady_clock::now();
  ASSERT_TRUE(controller_interface::deactivate_succeeds(controller));
  const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                              std::chrono::steady_clock::now() - started)
                              .count();
  // Bounded: give the owner thread a few more cycles to observe the deactivation and zero.
  const size_t cycles_at_deactivate = owner_cycles.load(std::memory_order_acquire);
  for (size_t attempt = 0;
       attempt < 2000 && owner_cycles.load(std::memory_order_acquire) < cycles_at_deactivate + 3U;
       ++attempt) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  stop_owner.store(true, std::memory_order_release);
  owner_thread.join();

  // No bounded wait exists any more: an off-owner deactivate is as cheap as an atomic store.
  EXPECT_LT(elapsed_ms, 50) << "off-owner deactivate blocked for " << elapsed_ms << " ms";
  EXPECT_TRUE(hardware.allCommandsEqual(0.0));
  for (size_t command = 0; command < kCommandCount; ++command) {
    ASSERT_GT(hardware.writeCount(command), 0U) << "command " << command;
    EXPECT_EQ(hardware.writeThreadIdentity(command),
              owner_thread_identity.load(std::memory_order_acquire))
        << "command " << command;
    EXPECT_NE(hardware.writeThreadIdentity(command), lifecycle_thread_identity)
        << "command " << command;
  }

  controller->release_interfaces();
}

TEST_F(DualArmJointHoldControllerTest, LifecycleCallbacksNeverWriteACommandInterface) {
  // F-10c amendment A.1, stated directly: no lifecycle callback -- deactivate, error, shutdown,
  // cleanup, or the release hook -- writes a command interface, on any thread, in any order.
  // Everything the controller writes, it writes from update().
  ControllerParameters parameters;
  ControllerHardwareFixture hardware(parameters.arm_ids);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, false);
  ASSERT_TRUE(activate(controller));
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);
  hardware.fillCommands(8.0);
  hardware.resetWriteCounts();

  const auto started = std::chrono::steady_clock::now();
  ASSERT_TRUE(controller_interface::deactivate_succeeds(controller));
  controller->release_interfaces();
  const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                              std::chrono::steady_clock::now() - started)
                              .count();

  EXPECT_TRUE(hardware.everyCommandWasWritten(0));
  EXPECT_TRUE(hardware.allCommandsEqual(8.0));
  // The old bounded wait was ~100-220 ms per callback and dominated every controller switch.
  EXPECT_LT(elapsed_ms, 20) << "the lifecycle cascade blocked for " << elapsed_ms << " ms";
}

TEST_F(DualArmJointHoldControllerTest, PluginHasDistinctLoadableName) {
  pluginlib::ClassLoader<controller_interface::ControllerInterface> loader(
      "controller_interface", "controller_interface::ControllerInterface");
  EXPECT_TRUE(loader.isClassAvailable("franka_example_controllers/DualArmJointHoldController"));
  auto controller =
      loader.createUniqueInstance("franka_example_controllers/DualArmJointHoldController");
  EXPECT_NE(controller, nullptr);
}

}  // namespace
}  // namespace franka_example_controllers
