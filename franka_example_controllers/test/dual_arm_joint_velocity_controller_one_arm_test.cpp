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
// One-arm-mode coverage for DualArmJointVelocityController (arm_count == 1). The dual-mode
// behaviour exercised by dual_arm_joint_velocity_controller_test.cpp is untouched by this file:
// every test here builds its own single-arm controller instance and hardware fixture sized for
// exactly one arm.

#include "franka_example_controllers/dual_arm_joint_velocity_controller.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <control_msgs/msg/joint_jog.hpp>
#include <controller_interface/test_utils.hpp>
#include <cstddef>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "dual_arm_joint_velocity_controller_core.hpp"
#include "franka_example_controllers/panda_joint_limits.hpp"

namespace franka_example_controllers {

// Reuses the production friend declaration; a distinct type from the dual-mode test file's
// same-named class is fine because each ament_add_gtest target links a separate binary.
class DualArmJointVelocityControllerTestAccess {
 public:
  static JointJogValidationResult accept(DualArmJointVelocityController& controller,
                                         const size_t arm,
                                         const control_msgs::msg::JointJog& message,
                                         const int64_t ros_now_ns,
                                         const int64_t steady_receive_ns) {
    return controller.core_->acceptCommand(arm, message, ros_now_ns, steady_receive_ns);
  }
  static void enable(DualArmJointVelocityController& controller,
                     const size_t arm,
                     const bool enabled,
                     const int64_t steady_now_ns) {
    controller.core_->setArmEnabled(arm, enabled, steady_now_ns);
  }
  static std::string topic(const DualArmJointVelocityController& controller, const size_t arm) {
    return controller.core_->subscriptionTopic(arm);
  }
  static std::string service(const DualArmJointVelocityController& controller, const size_t arm) {
    return controller.core_->enableServiceName(arm);
  }
};

namespace {

constexpr size_t kJointCount = 7;
const char* const kArmId = "panda2";
constexpr int64_t kRosNowNs = 10000000000LL;

using JointArray = std::array<double, kJointCount>;
using JointNames = std::array<std::string, kJointCount>;

JointNames makeJointNames(const std::string& prefix) {
  JointNames names;
  for (size_t joint = 0; joint < kJointCount; ++joint) {
    names[joint] = prefix + "_joint" + std::to_string(joint + 1);
  }
  return names;
}

std::vector<std::string> asVector(const JointNames& names) {
  return {names.begin(), names.end()};
}

struct OneArmParameters {
  int64_t arm_count{1};
  std::string arm_id{kArmId};
  JointNames joint_names{makeJointNames(kArmId)};
  std::vector<double> max_velocity{std::vector<double>(kJointCount, 1.0)};
  std::vector<double> max_acceleration{std::vector<double>(kJointCount, 7.0)};
  double watchdog_timeout{10.0};
  double max_header_age{1.0};
  double future_tolerance{0.1};
  bool set_arm_1{true};
  bool set_arm_2_arm_id{false};
};

std::unique_ptr<DualArmJointVelocityController> makeController(const OneArmParameters& parameters) {
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
    overrides.emplace_back("arm_1.max_velocity", parameters.max_velocity);
    overrides.emplace_back("arm_1.max_acceleration", parameters.max_acceleration);
  }
  if (parameters.set_arm_2_arm_id) {
    overrides.emplace_back("arm_2.arm_id", std::string("panda1"));
  }
  node_options.parameter_overrides(overrides);

  controller_interface::ControllerInterfaceParams controller_parameters;
  controller_parameters.controller_name = "dual_arm_joint_velocity_controller_one_arm_test";
  controller_parameters.update_rate = 1000;
  controller_parameters.controller_manager_update_rate = 1000;
  controller_parameters.node_options = node_options;

  auto controller = std::make_unique<DualArmJointVelocityController>();
  if (controller->init(controller_parameters) != controller_interface::return_type::OK) {
    throw std::runtime_error("controller initialization failed");
  }
  return controller;
}

bool configure(const std::unique_ptr<DualArmJointVelocityController>& controller) {
  return controller_interface::configure_succeeds(controller);
}

bool activate(const std::unique_ptr<DualArmJointVelocityController>& controller) {
  return controller_interface::activate_succeeds(controller);
}

controller_interface::return_type update(DualArmJointVelocityController& controller,
                                         const double seconds = 0.1) {
  return controller.update(rclcpp::Time(0, 0, RCL_ROS_TIME),
                           rclcpp::Duration::from_seconds(seconds));
}

control_msgs::msg::JointJog makeMessage(const JointNames& names,
                                        const JointArray& velocities,
                                        const int64_t stamp_ns = kRosNowNs) {
  control_msgs::msg::JointJog message;
  message.header.stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
  message.header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);
  for (size_t joint = 0; joint < kJointCount; ++joint) {
    message.joint_names.push_back(names[joint]);
    message.velocities.push_back(velocities[joint]);
  }
  return message;
}

// Hardware sized for exactly one arm's worth of velocity command interfaces.
class SingleArmVelocityHardwareFixture {
 public:
  explicit SingleArmVelocityHardwareFixture(const JointNames& names) {
    handles_.reserve(kJointCount);
    for (size_t joint = 0; joint < kJointCount; ++joint) {
      handles_.push_back(std::make_shared<hardware_interface::CommandInterface>(
          names[joint], "velocity", &commands_[joint]));
    }
  }

  void assignTo(DualArmJointVelocityController& controller) const {
    std::vector<hardware_interface::LoanedCommandInterface> interfaces;
    interfaces.reserve(handles_.size());
    for (const auto& handle : handles_) {
      interfaces.emplace_back(handle, hardware_interface::LoanedCommandInterface::Deleter{});
    }
    controller.assign_interfaces(std::move(interfaces), {});
  }

  const JointArray& commands() const { return commands_; }
  bool allEqual(const double expected) const {
    return std::all_of(commands_.begin(), commands_.end(),
                       [&](const double value) { return value == expected; });
  }

 private:
  JointArray commands_{};
  std::vector<hardware_interface::CommandInterface::SharedPtr> handles_;
};

class DualArmJointVelocityControllerOneArmTest : public ::testing::Test {
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

TEST_F(DualArmJointVelocityControllerOneArmTest,
       ClaimsExactlySevenJointsUnderConfiguredArmIdAndArm1Topics) {
  OneArmParameters parameters;
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));

  const auto commands = controller->command_interface_configuration();
  const auto states = controller->state_interface_configuration();
  ASSERT_EQ(commands.names.size(), kJointCount);
  EXPECT_EQ(states.type, controller_interface::interface_configuration_type::NONE);
  for (const auto& name : parameters.joint_names) {
    EXPECT_EQ(std::count(commands.names.begin(), commands.names.end(), name + "/velocity"), 1);
  }
  // Command topics/services keep the arm_1 convention regardless of the configured arm_id.
  EXPECT_EQ(DualArmJointVelocityControllerTestAccess::topic(*controller, 0),
           "/dual_arm_joint_velocity_controller_one_arm_test/arm_1/joint_jog");
  EXPECT_EQ(DualArmJointVelocityControllerTestAccess::service(*controller, 0),
           "/dual_arm_joint_velocity_controller_one_arm_test/arm_1/enable");
  // arm_count == 1 means the second arm's subscription/service were never created: the arm_2
  // topic and service simply do not exist.
  EXPECT_TRUE(DualArmJointVelocityControllerTestAccess::topic(*controller, 1).empty());
  EXPECT_TRUE(DualArmJointVelocityControllerTestAccess::service(*controller, 1).empty());
}

TEST_F(DualArmJointVelocityControllerOneArmTest, ValidJointJogIsAcceptedAndMovesCommandedValues) {
  OneArmParameters parameters;
  parameters.max_velocity = std::vector<double>(kJointCount, 0.8);
  parameters.max_acceleration = std::vector<double>(kJointCount, 0.5);
  SingleArmVelocityHardwareFixture hardware(parameters.joint_names);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller);
  ASSERT_TRUE(activate(controller));
  EXPECT_TRUE(hardware.allEqual(0.0));

  const JointArray target{0.4, -0.4, 0.3, -0.3, 0.2, -0.2, 0.1};
  const int64_t now = steadyNowNanoseconds();
  DualArmJointVelocityControllerTestAccess::enable(*controller, 0, true, now - 2);
  ASSERT_EQ(DualArmJointVelocityControllerTestAccess::accept(
                *controller, 0, makeMessage(parameters.joint_names, target), kRosNowNs, now - 1),
            JointJogValidationResult::Accepted);

  ASSERT_EQ(update(*controller, 0.1), controller_interface::return_type::OK);
  for (size_t joint = 0; joint < kJointCount; ++joint) {
    // max_acceleration (0.5) * period (0.1s) = 0.05: the first cycle is acceleration-limited, so
    // the command moves partway toward the target rather than jumping straight to it.
    const double expected = target[joint] > 0.0 ? 0.05 : -0.05;
    EXPECT_NEAR(hardware.commands()[joint], expected, 1e-12);
  }
  ASSERT_EQ(update(*controller, 0.1), controller_interface::return_type::OK);
  for (size_t joint = 0; joint < kJointCount; ++joint) {
    // A second identical cycle advances by another 0.05 toward the target (joint 7's target of
    // 0.1 is exactly reached; every other joint is still short of its target).
    const double expected = target[joint] > 0.0 ? 0.10 : -0.10;
    EXPECT_NEAR(hardware.commands()[joint], expected, 1e-12);
    EXPECT_LE(std::abs(hardware.commands()[joint]), std::abs(target[joint]));
  }
}

TEST_F(DualArmJointVelocityControllerOneArmTest, StaleCommandTriggersWatchdogZero) {
  OneArmParameters parameters;
  parameters.watchdog_timeout = 0.05;
  SingleArmVelocityHardwareFixture hardware(parameters.joint_names);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller);
  ASSERT_TRUE(activate(controller));
  const JointArray target{0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2};

  const int64_t now = steadyNowNanoseconds();
  DualArmJointVelocityControllerTestAccess::enable(*controller, 0, true, now - 2000000000LL);
  ASSERT_EQ(DualArmJointVelocityControllerTestAccess::accept(
                *controller, 0, makeMessage(parameters.joint_names, target), kRosNowNs, now - 1),
            JointJogValidationResult::Accepted);
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);
  for (size_t joint = 0; joint < kJointCount; ++joint) {
    EXPECT_NEAR(hardware.commands()[joint], target[joint], 1e-12);
  }

  // Let the accepted command age past the watchdog window with no fresh publication.
  ASSERT_EQ(DualArmJointVelocityControllerTestAccess::accept(
                *controller, 0, makeMessage(parameters.joint_names, target), kRosNowNs,
                now - 1000000000LL),
            JointJogValidationResult::Accepted);
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);
  EXPECT_TRUE(hardware.allEqual(0.0));
}

TEST_F(DualArmJointVelocityControllerOneArmTest, RejectsArmCountThree) {
  OneArmParameters parameters;
  parameters.arm_count = 3;
  EXPECT_FALSE(configure(makeController(parameters)));
}

TEST_F(DualArmJointVelocityControllerOneArmTest, RejectsArmTwoParametersWhenArmCountIsOne) {
  OneArmParameters parameters;
  parameters.set_arm_2_arm_id = true;
  EXPECT_FALSE(configure(makeController(parameters)));
}

TEST_F(DualArmJointVelocityControllerOneArmTest, RejectsMissingArmOneBlockWhenArmCountDeclared) {
  OneArmParameters parameters;
  parameters.set_arm_1 = false;
  EXPECT_FALSE(configure(makeController(parameters)));
}

}  // namespace
}  // namespace franka_example_controllers
