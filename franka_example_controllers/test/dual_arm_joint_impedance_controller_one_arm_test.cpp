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
// One-arm-mode coverage for DualArmJointImpedanceController (arm_count == 1). The dual-mode
// behaviour exercised by dual_arm_joint_impedance_controller_test.cpp is untouched by this file:
// every test here builds its own single-arm controller instance and hardware fixture sized for
// exactly one arm.

#include "franka_example_controllers/dual_arm_joint_impedance_controller.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <controller_interface/test_utils.hpp>
#include <cstddef>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <vector>

#include <franka/model.h>
#include <franka/robot_state.h>

#include "dual_arm_joint_impedance_controller_core.hpp"
#include "franka_example_controllers/panda_joint_limits.hpp"
#include "franka_hardware/common/model_base.hpp"

namespace franka_example_controllers {

// Reuses the production friend declaration; a distinct type from the dual-mode test file's
// same-named class is fine because each ament_add_gtest target links a separate binary.
class DualArmJointImpedanceControllerTestAccess {
 public:
  static JointTargetValidationResult accept(DualArmJointImpedanceController& controller,
                                            const size_t arm,
                                            const trajectory_msgs::msg::JointTrajectory& message,
                                            const int64_t ros_now_ns,
                                            const int64_t steady_receive_ns) {
    return controller.core_->acceptTarget(arm, message, ros_now_ns, steady_receive_ns);
  }
  static bool enable(DualArmJointImpedanceController& controller,
                     const size_t arm,
                     const bool enabled,
                     const int64_t steady_now_ns,
                     const int64_t ros_now_ns = 0) {
    return controller.core_->setArmEnabled(arm, enabled, steady_now_ns, ros_now_ns);
  }
  static std::array<double, kImpedanceJointCount> target(
      const DualArmJointImpedanceController& controller,
      const size_t arm) {
    return controller.core_->internalTarget(arm);
  }
};

namespace {

constexpr size_t kJointCount = 7;
const char* const kArmId = "panda2";
constexpr int64_t kRosNowNs = 10000000000LL;

using JointArray = std::array<double, kJointCount>;
using JointNames = std::array<std::string, kJointCount>;

JointArray basePose() {
  return JointArray{{0.0, -0.5, 0.0, -1.0, 0.0, 1.0, 0.0}};
}

JointNames makeJointNames(const std::string& arm_id) {
  JointNames names;
  for (size_t joint = 0; joint < kJointCount; ++joint) {
    names[joint] = arm_id + "_joint" + std::to_string(joint + 1);
  }
  return names;
}

std::vector<std::string> asVector(const JointNames& names) {
  return {names.begin(), names.end()};
}
std::vector<double> asVector(const JointArray& values) {
  return {values.begin(), values.end()};
}

class FixedCoriolisModel final : public franka_hardware::ModelBase {
 public:
  JointArray coriolis(const franka::RobotState& /*robot_state*/) const override {
    return coriolis_;
  }
  void setCoriolis(const JointArray& values) { coriolis_ = values; }

 private:
  std::array<double, 16> poseImpl(franka::Frame /*frame*/,
                                  const JointArray& /*q*/,
                                  const std::array<double, 16>& /*f_t_ee*/,
                                  const std::array<double, 16>& /*ee_t_k*/) const override {
    return {};
  }
  std::array<double, 42> bodyJacobianImpl(franka::Frame /*frame*/,
                                          const JointArray& /*q*/,
                                          const std::array<double, 16>& /*f_t_ee*/,
                                          const std::array<double, 16>& /*ee_t_k*/) const override {
    return {};
  }
  std::array<double, 42> zeroJacobianImpl(franka::Frame /*frame*/,
                                          const JointArray& /*q*/,
                                          const std::array<double, 16>& /*f_t_ee*/,
                                          const std::array<double, 16>& /*ee_t_k*/) const override {
    return {};
  }
  std::array<double, 49> massImpl(const JointArray& /*q*/,
                                  const std::array<double, 9>& /*i_total*/,
                                  double /*m_total*/,
                                  const std::array<double, 3>& /*f_x_ctotal*/) const override {
    return {};
  }
  JointArray coriolisImpl(const JointArray& /*q*/,
                          const JointArray& /*dq*/,
                          const std::array<double, 9>& /*i_total*/,
                          double /*m_total*/,
                          const std::array<double, 3>& /*f_x_ctotal*/) const override {
    return coriolis_;
  }
  JointArray gravityImpl(const JointArray& /*q*/,
                         double /*m_total*/,
                         const std::array<double, 3>& /*f_x_ctotal*/,
                         const std::array<double, 3>& /*gravity_earth*/) const override {
    return {};
  }
  JointArray coriolis_{};
};

// Hardware sized for exactly one arm.
class SingleArmImpedanceHardwareFixture {
 public:
  explicit SingleArmImpedanceHardwareFixture(const JointNames& joint_names, std::string arm_id)
      : arm_id_(std::move(arm_id)) {
    state_pointer_ = &robot_state_;
    model_pointer_ = &model_;
    setJointState(basePose(), {});
    for (size_t joint = 0; joint < kJointCount; ++joint) {
      coriolis_[joint] = 0.1 + 0.01 * static_cast<double>(joint);
      state_handles_.push_back(std::make_shared<hardware_interface::StateInterface>(
          joint_names[joint], "position", &positions_[joint]));
      state_handles_.push_back(std::make_shared<hardware_interface::StateInterface>(
          joint_names[joint], "velocity", &velocities_[joint]));
      command_handles_.push_back(std::make_shared<hardware_interface::CommandInterface>(
          joint_names[joint], "effort", &commands_[joint]));
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

  void setJointState(const JointArray& positions, const JointArray& velocities) {
    positions_ = positions;
    velocities_ = velocities;
    robot_state_.q = positions;
    robot_state_.dq = velocities;
  }

  void assignTo(DualArmJointImpedanceController& controller) const {
    std::vector<hardware_interface::LoanedCommandInterface> commands;
    std::vector<hardware_interface::LoanedStateInterface> states;
    commands.reserve(command_handles_.size());
    states.reserve(state_handles_.size());
    for (const auto& handle : command_handles_) {
      commands.emplace_back(handle, hardware_interface::LoanedCommandInterface::Deleter{});
    }
    for (const auto& handle : state_handles_) {
      states.emplace_back(handle);
    }
    controller.assign_interfaces(std::move(commands), std::move(states));
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
};

struct OneArmParameters {
  int64_t arm_count{1};
  std::string arm_id{kArmId};
  JointNames joint_names{makeJointNames(kArmId)};
  std::vector<double> k_gains{std::vector<double>(kJointCount, 10.0)};
  std::vector<double> d_gains{std::vector<double>(kJointCount, 1.0)};
  std::vector<double> max_effort{asVector(kPandaAbsoluteEffortCeilings)};
  std::vector<double> position_lower{asVector(kPandaPositionLowerLimits)};
  std::vector<double> position_upper{asVector(kPandaPositionUpperLimits)};
  std::vector<double> max_target_velocity{std::vector<double>(kJointCount, 0.5)};
  double watchdog_timeout{10.0};
  double max_header_age{1.0};
  double future_tolerance{0.1};
  bool set_arm_1{true};
  bool set_arm_2_arm_id{false};
};

std::unique_ptr<DualArmJointImpedanceController> makeController(
    const OneArmParameters& parameters) {
  rclcpp::NodeOptions node_options;
  node_options.enable_rosout(false);
  node_options.start_parameter_event_publisher(false);
  node_options.start_parameter_services(false);
  std::vector<rclcpp::Parameter> overrides{
      rclcpp::Parameter("arm_count", parameters.arm_count),
      rclcpp::Parameter("watchdog_timeout", parameters.watchdog_timeout),
      rclcpp::Parameter("max_header_age", parameters.max_header_age),
      rclcpp::Parameter("future_tolerance", parameters.future_tolerance),
  };
  if (parameters.set_arm_1) {
    overrides.emplace_back("arm_1.arm_id", parameters.arm_id);
    overrides.emplace_back("arm_1.joint_names", asVector(parameters.joint_names));
    overrides.emplace_back("arm_1.k_gains", parameters.k_gains);
    overrides.emplace_back("arm_1.d_gains", parameters.d_gains);
    overrides.emplace_back("arm_1.max_effort", parameters.max_effort);
    overrides.emplace_back("arm_1.position_lower", parameters.position_lower);
    overrides.emplace_back("arm_1.position_upper", parameters.position_upper);
    overrides.emplace_back("arm_1.max_target_velocity", parameters.max_target_velocity);
  }
  if (parameters.set_arm_2_arm_id) {
    overrides.emplace_back("arm_2.arm_id", std::string("panda1"));
  }
  node_options.parameter_overrides(overrides);

  controller_interface::ControllerInterfaceParams controller_parameters;
  controller_parameters.controller_name = "dual_arm_joint_impedance_controller_one_arm_test";
  controller_parameters.update_rate = 1000;
  controller_parameters.controller_manager_update_rate = 1000;
  controller_parameters.node_options = node_options;

  auto controller = std::make_unique<DualArmJointImpedanceController>();
  if (controller->init(controller_parameters) != controller_interface::return_type::OK) {
    throw std::runtime_error("controller initialization failed");
  }
  return controller;
}

bool configure(const std::unique_ptr<DualArmJointImpedanceController>& controller) {
  return controller_interface::configure_succeeds(controller);
}

bool activate(const std::unique_ptr<DualArmJointImpedanceController>& controller) {
  return controller_interface::activate_succeeds(controller);
}

controller_interface::return_type update(DualArmJointImpedanceController& controller,
                                         const double seconds = 0.1) {
  return controller.update(rclcpp::Time(0, 0, RCL_ROS_TIME),
                           rclcpp::Duration::from_seconds(seconds));
}

trajectory_msgs::msg::JointTrajectory makeMessage(const JointNames& names,
                                                  const JointArray& positions,
                                                  const int64_t stamp_ns = kRosNowNs) {
  trajectory_msgs::msg::JointTrajectory message;
  message.header.stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
  message.header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);
  message.points.resize(1);
  for (size_t joint = 0; joint < kJointCount; ++joint) {
    message.joint_names.push_back(names[joint]);
    message.points.front().positions.push_back(positions[joint]);
  }
  return message;
}

class DualArmJointImpedanceControllerOneArmTest : public ::testing::Test {
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

TEST_F(DualArmJointImpedanceControllerOneArmTest, ClaimsExactlySevenJointsUnderConfiguredArmId) {
  OneArmParameters parameters;
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));

  const auto commands = controller->command_interface_configuration();
  const auto states = controller->state_interface_configuration();
  ASSERT_EQ(commands.names.size(), kJointCount);
  ASSERT_EQ(states.names.size(), 2 * kJointCount + 2);
  for (const auto& name : parameters.joint_names) {
    EXPECT_EQ(std::count(commands.names.begin(), commands.names.end(), name + "/effort"), 1);
    EXPECT_EQ(std::count(states.names.begin(), states.names.end(), name + "/position"), 1);
    EXPECT_EQ(std::count(states.names.begin(), states.names.end(), name + "/velocity"), 1);
  }
  EXPECT_EQ(
      std::count(states.names.begin(), states.names.end(), std::string(kArmId) + "/robot_state"),
      1);
  EXPECT_EQ(
      std::count(states.names.begin(), states.names.end(), std::string(kArmId) + "/robot_model"),
      1);
}

TEST_F(DualArmJointImpedanceControllerOneArmTest,
       ActivationCapturesMeasuredPoseDisablesAndWritesZero) {
  OneArmParameters parameters;
  SingleArmImpedanceHardwareFixture hardware(parameters.joint_names, kArmId);
  hardware.fillCommands(99.0);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller);
  ASSERT_TRUE(activate(controller));
  EXPECT_TRUE(hardware.allCommandsEqual(0.0));
  EXPECT_EQ(DualArmJointImpedanceControllerTestAccess::target(*controller, 0), basePose());

  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);
  for (size_t joint = 0; joint < kJointCount; ++joint) {
    EXPECT_DOUBLE_EQ(hardware.command(joint), 0.1 + 0.01 * static_cast<double>(joint));
  }
}

TEST_F(DualArmJointImpedanceControllerOneArmTest, TargetWithinBoundsIsAcceptedAndAppliedGradually) {
  OneArmParameters parameters;
  parameters.max_target_velocity = std::vector<double>(kJointCount, 0.5);
  SingleArmImpedanceHardwareFixture hardware(parameters.joint_names, kArmId);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller);
  ASSERT_TRUE(activate(controller));

  JointArray requested = basePose();
  for (auto& value : requested) {
    // +0.2 stays comfortably inside every joint's configured [position_lower, position_upper]
    // range, including joint 4's narrow all-negative range ([-3.0718, -0.0698]: base -1.0 + 0.2
    // = -0.8), and is still far enough from the base pose to exceed one target-velocity step.
    value += 0.2;
  }
  const int64_t now = impedanceSteadyNowNanoseconds();
  EXPECT_TRUE(DualArmJointImpedanceControllerTestAccess::enable(*controller, 0, true, now - 1,
                                                               kRosNowNs - 1));
  ASSERT_EQ(DualArmJointImpedanceControllerTestAccess::accept(
                *controller, 0, makeMessage(parameters.joint_names, requested), kRosNowNs, now),
            JointTargetValidationResult::Accepted);

  ASSERT_EQ(update(*controller, 0.1), controller_interface::return_type::OK);
  const auto after_one_step = DualArmJointImpedanceControllerTestAccess::target(*controller, 0);
  const auto base = basePose();
  for (size_t joint = 0; joint < kJointCount; ++joint) {
    // max_target_velocity (0.5) * period (0.1s) = 0.05: the internal target moves partway
    // toward the requested position, clamped by the configured target-velocity limit.
    EXPECT_NEAR(after_one_step[joint], base[joint] + 0.05, 1e-12);
    EXPECT_LT(after_one_step[joint], requested[joint]);
  }
}

TEST_F(DualArmJointImpedanceControllerOneArmTest, TargetOutOfBoundsIsRejectedAndTargetUnchanged) {
  OneArmParameters parameters;
  SingleArmImpedanceHardwareFixture hardware(parameters.joint_names, kArmId);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller);
  ASSERT_TRUE(activate(controller));

  JointArray out_of_bounds = basePose();
  out_of_bounds[0] = kPandaPositionUpperLimits[0] + 0.1;  // Joint 1 above its configured ceiling.
  const int64_t now = impedanceSteadyNowNanoseconds();
  EXPECT_TRUE(DualArmJointImpedanceControllerTestAccess::enable(*controller, 0, true, now - 1,
                                                               kRosNowNs - 1));
  EXPECT_EQ(DualArmJointImpedanceControllerTestAccess::accept(
                *controller, 0, makeMessage(parameters.joint_names, out_of_bounds), kRosNowNs, now),
            JointTargetValidationResult::PositionLimitExceeded);

  ASSERT_EQ(update(*controller, 0.1), controller_interface::return_type::OK);
  EXPECT_EQ(DualArmJointImpedanceControllerTestAccess::target(*controller, 0), basePose());
}

TEST_F(DualArmJointImpedanceControllerOneArmTest, RejectsArmCountThree) {
  OneArmParameters parameters;
  parameters.arm_count = 3;
  EXPECT_FALSE(configure(makeController(parameters)));
}

TEST_F(DualArmJointImpedanceControllerOneArmTest, RejectsArmTwoParametersWhenArmCountIsOne) {
  OneArmParameters parameters;
  parameters.set_arm_2_arm_id = true;
  EXPECT_FALSE(configure(makeController(parameters)));
}

TEST_F(DualArmJointImpedanceControllerOneArmTest, RejectsMissingArmOneBlockWhenArmCountDeclared) {
  OneArmParameters parameters;
  parameters.set_arm_1 = false;
  EXPECT_FALSE(configure(makeController(parameters)));
}

}  // namespace
}  // namespace franka_example_controllers
