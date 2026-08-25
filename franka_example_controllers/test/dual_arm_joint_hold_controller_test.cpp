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
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
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
              ++write_counts_[command_index];
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

  void resetWriteCounts() { write_counts_.fill(0); }

  bool everyCommandWasWrittenTwice() const {
    return std::all_of(write_counts_.begin(), write_counts_.end(),
                       [](const size_t count) { return count == 2; });
  }

  bool everyCommandWasWritten(const size_t expected_count) const {
    return std::all_of(write_counts_.begin(), write_counts_.end(),
                       [expected_count](const size_t count) { return count == expected_count; });
  }

  size_t writeCount(const size_t command) const { return write_counts_.at(command); }

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
  std::array<size_t, kCommandCount> write_counts_{};
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
  ASSERT_TRUE(activate(controller));
  EXPECT_TRUE(hardware.allCommandsEqual(0.0));
  EXPECT_TRUE(hardware.everyCommandWasWritten(1));

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
    EXPECT_FALSE(activate(controller));
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
    EXPECT_FALSE(activate(controller));
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

TEST_F(DualArmJointHoldControllerTest, InitializesVelocityFilterAndZerosOnDeactivation) {
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

  EXPECT_TRUE(controller_interface::deactivate_succeeds(controller));
  EXPECT_TRUE(hardware.allCommandsEqual(0.0));
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
  EXPECT_FALSE(activate(controller));
  EXPECT_TRUE(hardware.everyCommandWasWritten(1));
  controller->release_interfaces();
  for (size_t command = 0; command < kCommandCount; ++command) {
    EXPECT_EQ(hardware.writeCount(command), 2U) << "command " << command;
  }
  lock.unlock();
  EXPECT_EQ(update(*controller), controller_interface::return_type::ERROR);
  EXPECT_FALSE(activate(controller));

  ASSERT_TRUE(controller_interface::cleanup_succeeds(controller));
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, false);
  ASSERT_TRUE(activate(controller));
  EXPECT_TRUE(hardware.allCommandsEqual(0.0));
  ASSERT_TRUE(controller_interface::deactivate_succeeds(controller));
  controller->release_interfaces();
}

TEST_F(DualArmJointHoldControllerTest, DeactivationErrorRunsOnErrorThenReleaseFinalZeroAttempt) {
  ControllerParameters parameters;
  ControllerHardwareFixture hardware(parameters.arm_ids);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, true);
  ASSERT_TRUE(activate(controller));
  hardware.fillCommands(5.0);
  hardware.resetWriteCounts();

  auto failing_handle = hardware.commandHandle("arm_joint4/effort");
  std::unique_lock<std::shared_mutex> lock(failing_handle->get_mutex());
  const auto state = controller->get_node()->deactivate();
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
  // Jazzy runs on_deactivate, on_error with loans, and the release hook before returning.
  EXPECT_TRUE(hardware.everyCommandWasWritten(3));
  controller->release_interfaces();
  EXPECT_TRUE(hardware.everyCommandWasWritten(3));
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
  hardware->fillCommands(6.0);
  hardware->resetWriteCounts();

  auto failing_handle = hardware->commandHandle("arm_joint4/effort");
  std::unique_lock<std::shared_mutex> lock(failing_handle->get_mutex());
  const auto state = controller->get_node()->shutdown();
  // Jazzy releases after the failing shutdown callback before on_error, so on_error sees no loans.
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  for (size_t command = 0; command < kCommandCount; ++command) {
    EXPECT_EQ(hardware->writeCount(command), 2U) << "command " << command;
  }
  controller->release_interfaces();
  EXPECT_TRUE(hardware->everyCommandWasWritten(2));
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
  hardware.fillCommands(6.0);
  hardware.resetWriteCounts();

  ASSERT_TRUE(controller_interface::deactivate_succeeds(controller));
  EXPECT_TRUE(hardware.everyCommandWasWritten(1));
  controller->release_interfaces();
  EXPECT_TRUE(hardware.everyCommandWasWritten(1));
  ASSERT_TRUE(controller_interface::cleanup_succeeds(controller));
  ASSERT_TRUE(configure(controller));
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
