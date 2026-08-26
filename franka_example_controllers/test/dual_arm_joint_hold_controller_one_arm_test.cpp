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
//
// One-arm-mode coverage for DualArmJointHoldController (arm_count == 1). The dual-mode behaviour
// exercised by dual_arm_joint_hold_controller_test.cpp is untouched by this file: every test here
// builds its own single-arm controller instance and hardware fixture sized for exactly one arm.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <franka/model.h>
#include <franka/robot_state.h>
#include <controller_interface/test_utils.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include "franka_example_controllers/dual_arm_joint_hold_controller.hpp"
#include "franka_example_controllers/panda_joint_limits.hpp"
#include "franka_hardware/common/model_base.hpp"

namespace franka_example_controllers {
namespace {

constexpr size_t kJointCount = 7;
// The arm_id deliberately differs from the dual-mode test file's "arm"/"arm_extra" fixtures, and
// is one of the two production one-arm-mode IDs, to prove arm_id is a genuine runtime choice and
// not hardcoded to arm 1 of the compile-time-2 capacity.
const char* const kArmId = "panda2";

using JointArray = std::array<double, kJointCount>;

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

// Hardware sized for exactly one arm: bindInterfaces() requires the assigned interface count to
// match arm_count_ * kJointCount / arm_count_ * (2 * kJointCount + 2) exactly, so a one-arm
// controller must never be handed the dual-mode fixture's 14/32 interfaces.
class SingleArmHoldHardwareFixture {
 public:
  explicit SingleArmHoldHardwareFixture(std::string arm_id) : arm_id_(std::move(arm_id)) {
    state_pointer_ = &robot_state_;
    model_pointer_ = &model_;
    for (size_t joint = 0; joint < kJointCount; ++joint) {
      const double coriolis_value = 1.0 + 0.1 * static_cast<double>(joint);
      coriolis_[joint] = coriolis_value;
      state_handles_.push_back(std::make_shared<hardware_interface::StateInterface>(
          jointName(arm_id_, joint), "position", &positions_[joint]));
      state_handles_.push_back(std::make_shared<hardware_interface::StateInterface>(
          jointName(arm_id_, joint), "velocity", &velocities_[joint]));
      const auto command_index = command_handles_.size();
      command_handles_.push_back(std::make_shared<hardware_interface::CommandInterface>(
          jointName(arm_id_, joint), "effort", &commands_[joint]));
      command_handles_.back()->set_on_set_command_limiter(
          [this, command_index](const double value, bool& limited) {
            ++write_counts_[command_index];
            limited = false;
            return value;
          });
    }
    model_.setCoriolis(coriolis_);
    state_handles_.push_back(std::make_shared<hardware_interface::StateInterface>(
        arm_id_, "robot_state",
        reinterpret_cast<double*>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
            &state_pointer_)));
    state_handles_.push_back(std::make_shared<hardware_interface::StateInterface>(
        arm_id_, "robot_model",
        reinterpret_cast<double*>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
            &model_pointer_)));
  }

  void setJointState(const JointArray& position, const JointArray& velocity) {
    positions_ = position;
    velocities_ = velocity;
    robot_state_.q = position;
    robot_state_.dq = velocity;
  }

  void assignTo(DualArmJointHoldController& controller) const {
    std::vector<hardware_interface::LoanedCommandInterface> command_interfaces;
    std::vector<hardware_interface::LoanedStateInterface> state_interfaces;
    command_interfaces.reserve(command_handles_.size());
    state_interfaces.reserve(state_handles_.size());
    for (const auto& interface : command_handles_) {
      command_interfaces.emplace_back(interface,
                                      hardware_interface::LoanedCommandInterface::Deleter{});
    }
    for (const auto& interface : state_handles_) {
      state_interfaces.emplace_back(interface);
    }
    controller.assign_interfaces(std::move(command_interfaces), std::move(state_interfaces));
  }

  void fillCommands(const double value) { commands_.fill(value); }
  double command(const size_t joint) const { return commands_[joint]; }
  bool allCommandsEqual(const double expected) const {
    return std::all_of(commands_.begin(), commands_.end(),
                       [&](const double value) { return value == expected; });
  }

 private:
  std::string arm_id_;
  JointArray positions_{};
  JointArray velocities_{};
  JointArray commands_{};
  JointArray coriolis_{};
  franka::RobotState robot_state_{};
  FixedCoriolisModel model_{};
  franka::RobotState* state_pointer_{nullptr};
  franka_hardware::ModelBase* model_pointer_{nullptr};
  std::vector<hardware_interface::StateInterface::SharedPtr> state_handles_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_handles_;
  std::array<size_t, kJointCount> write_counts_{};
};

struct OneArmParameters {
  int64_t arm_count{1};
  std::string arm_id{kArmId};
  std::vector<double> k_gains{std::vector<double>(kJointCount, 10.0)};
  std::vector<double> d_gains{std::vector<double>(kJointCount, 1.0)};
  std::vector<double> max_effort{87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0};
  bool set_arm_1{true};
  // Only used by the negative "arm_2.* present" test; left unset otherwise.
  bool set_arm_2_arm_id{false};
};

std::unique_ptr<DualArmJointHoldController> makeController(const OneArmParameters& parameters) {
  rclcpp::NodeOptions node_options;
  node_options.enable_rosout(false);
  node_options.start_parameter_event_publisher(false);
  node_options.start_parameter_services(false);
  std::vector<rclcpp::Parameter> overrides{
      rclcpp::Parameter("arm_count", parameters.arm_count),
  };
  if (parameters.set_arm_1) {
    overrides.emplace_back("arm_1.arm_id", parameters.arm_id);
    overrides.emplace_back("arm_1.k_gains", parameters.k_gains);
    overrides.emplace_back("arm_1.d_gains", parameters.d_gains);
    overrides.emplace_back("arm_1.max_effort", parameters.max_effort);
  }
  if (parameters.set_arm_2_arm_id) {
    overrides.emplace_back("arm_2.arm_id", std::string("panda1"));
  }
  node_options.parameter_overrides(overrides);

  controller_interface::ControllerInterfaceParams controller_parameters;
  controller_parameters.controller_name = "dual_arm_joint_hold_controller_one_arm_test";
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

class DualArmJointHoldControllerOneArmTest : public ::testing::Test {
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

TEST_F(DualArmJointHoldControllerOneArmTest, ClaimsExactlySevenJointsUnderConfiguredArmId) {
  OneArmParameters parameters;
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));

  const auto commands = controller->command_interface_configuration();
  const auto states = controller->state_interface_configuration();
  ASSERT_EQ(commands.names.size(), kJointCount);
  ASSERT_EQ(states.names.size(), 2 * kJointCount + 2);
  for (size_t joint = 0; joint < kJointCount; ++joint) {
    const auto joint_name = jointName(kArmId, joint);
    EXPECT_EQ(std::count(commands.names.begin(), commands.names.end(), joint_name + "/effort"), 1);
    EXPECT_EQ(std::count(states.names.begin(), states.names.end(), joint_name + "/position"), 1);
    EXPECT_EQ(std::count(states.names.begin(), states.names.end(), joint_name + "/velocity"), 1);
  }
  EXPECT_EQ(
      std::count(states.names.begin(), states.names.end(), std::string(kArmId) + "/robot_state"),
      1);
  EXPECT_EQ(
      std::count(states.names.begin(), states.names.end(), std::string(kArmId) + "/robot_model"),
      1);
  // No trace of the compile-time-capacity second arm slot may leak into the wire contract.
  for (const auto& name : commands.names) {
    EXPECT_EQ(name.rfind("arm_extra", 0), std::string::npos);
  }
}

TEST_F(DualArmJointHoldControllerOneArmTest,
       ConfigureActivateDeactivateSucceedAndCaptureMeasuredPose) {
  OneArmParameters parameters;
  SingleArmHoldHardwareFixture hardware(kArmId);
  const JointArray activation_pose{-0.4, 0.2, 0.7, -1.1, 0.3, 1.0, -0.8};
  hardware.setJointState(activation_pose, {});
  hardware.fillCommands(-999.0);

  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller);
  ASSERT_TRUE(activate(controller));
  // Activation immediately writes a required zero before any update() call.
  EXPECT_TRUE(hardware.allCommandsEqual(0.0));

  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);
  for (size_t joint = 0; joint < kJointCount; ++joint) {
    // k_gains * (hold_position - position) + coriolis == coriolis at the captured pose, since the
    // measured position has not moved and initial filtered velocity is the measured velocity (0).
    EXPECT_DOUBLE_EQ(hardware.command(joint), 1.0 + 0.1 * static_cast<double>(joint));
  }

  EXPECT_TRUE(controller_interface::deactivate_succeeds(controller));
  EXPECT_TRUE(hardware.allCommandsEqual(0.0));
}

TEST_F(DualArmJointHoldControllerOneArmTest, OverLimitEffortIsRejectedAndZeroed) {
  OneArmParameters parameters;
  parameters.max_effort = std::vector<double>(kJointCount, 0.5);
  SingleArmHoldHardwareFixture hardware(kArmId);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller);
  ASSERT_TRUE(activate(controller));
  hardware.fillCommands(77.0);

  EXPECT_EQ(update(*controller), controller_interface::return_type::ERROR);
  EXPECT_TRUE(hardware.allCommandsEqual(0.0));
}

TEST_F(DualArmJointHoldControllerOneArmTest, RejectsArmCountThree) {
  OneArmParameters parameters;
  parameters.arm_count = 3;
  EXPECT_FALSE(configure(makeController(parameters)));
}

TEST_F(DualArmJointHoldControllerOneArmTest, RejectsArmTwoParametersWhenArmCountIsOne) {
  OneArmParameters parameters;
  parameters.set_arm_2_arm_id = true;
  EXPECT_FALSE(configure(makeController(parameters)));
}

TEST_F(DualArmJointHoldControllerOneArmTest, RejectsMissingArmOneBlockWhenArmCountDeclared) {
  OneArmParameters parameters;
  parameters.set_arm_1 = false;
  EXPECT_FALSE(configure(makeController(parameters)));
}

}  // namespace
}  // namespace franka_example_controllers
