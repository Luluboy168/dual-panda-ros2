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

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include "franka_hardware/real/franka_multi_hardware_interface.hpp"
#include "franka_hardware/real/real_franka_arm_backend.hpp"

namespace franka_hardware {
namespace {

static_assert(std::is_default_constructible_v<FrankaMultiHardwareInterface>);
static_assert(std::is_constructible_v<FrankaMultiHardwareInterface, BackendFactory>);
static_assert(!std::is_constructible_v<FrankaMultiHardwareInterface, std::string>);

hardware_interface::InterfaceInfo makeInterfaceInfo(const std::string& name) {
  hardware_interface::InterfaceInfo interface{};
  interface.name = name;
  return interface;
}

hardware_interface::HardwareInfo makeSingleArmHardwareInfo() {
  hardware_interface::HardwareInfo info{};
  info.name = "FrankaMultiHardwareInterface";
  info.type = "system";
  info.hardware_plugin_name = "franka_hardware/FrankaMultiHardwareInterface";
  info.hardware_parameters = {
      {"robot_count", "1"}, {"ns_1", "panda"}, {"robot_ip_1", "offline-placeholder"}};

  for (size_t joint_index = 1; joint_index <= FrankaMultiHardwareInterface::kNumberOfJoints;
       ++joint_index) {
    hardware_interface::ComponentInfo joint{};
    joint.name = "panda_joint" + std::to_string(joint_index);
    joint.type = "joint";
    joint.command_interfaces = {makeInterfaceInfo("effort"), makeInterfaceInfo("position"),
                                makeInterfaceInfo("velocity")};
    joint.state_interfaces = {makeInterfaceInfo("position"), makeInterfaceInfo("velocity"),
                              makeInterfaceInfo("effort")};
    info.joints.push_back(std::move(joint));
  }
  return info;
}

class RclcppScope {
 public:
  RclcppScope() {
    if (!rclcpp::ok()) {
      int argc = 0;
      char** argv = nullptr;
      rclcpp::init(argc, argv);
      owns_context_ = true;
    }
  }

  RclcppScope(const RclcppScope&) = delete;
  RclcppScope& operator=(const RclcppScope&) = delete;

  ~RclcppScope() {
    if (owns_context_ && rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

 private:
  bool owns_context_{false};
};

TEST(BackendFactorySeamTest, DefaultConstructionDoesNotOpenAConnection) {
  EXPECT_NO_THROW({ FrankaMultiHardwareInterface hardware; });
}

TEST(BackendFactorySeamTest, InjectedFactoryIsLazyAndDirectConstructionOnly) {
  size_t factory_calls = 0;
  BackendFactory factory = [&factory_calls](
                               const std::string&, const std::string&,
                               const rclcpp::Logger&) -> std::shared_ptr<FrankaArmBackend> {
    ++factory_calls;
    return nullptr;
  };

  EXPECT_NO_THROW({ FrankaMultiHardwareInterface hardware(std::move(factory)); });
  EXPECT_EQ(factory_calls, 0U);
}

TEST(BackendFactorySeamTest, EmptyInjectedFactoryIsRejectedWithoutHardwareAccess) {
  EXPECT_THROW(FrankaMultiHardwareInterface(BackendFactory{}), std::invalid_argument);
}

TEST(BackendFactorySeamTest, InjectedFactoryIsRetainedAndReceivesArmMetadata) {
  RclcppScope rclcpp_scope;
  size_t factory_calls = 0;
  std::string received_arm_name;
  std::string received_address;
  std::string received_logger_name;
  BackendFactory factory = [&](const std::string& arm_name, const std::string& robot_address,
                               const rclcpp::Logger& logger) -> std::shared_ptr<FrankaArmBackend> {
    ++factory_calls;
    received_arm_name = arm_name;
    received_address = robot_address;
    received_logger_name = logger.get_name();
    return nullptr;
  };

  {
    FrankaMultiHardwareInterface hardware(std::move(factory));
    EXPECT_EQ(hardware.on_init(makeSingleArmHardwareInfo()), CallbackReturn::ERROR);
  }

  EXPECT_EQ(factory_calls, 1U);
  EXPECT_EQ(received_arm_name, "panda");
  EXPECT_EQ(received_address, "offline-placeholder");
  EXPECT_EQ(received_logger_name, "FrankaMultiHardwareInterface");
}

TEST(BackendFactorySeamTest, PluginlibConstructsTheExportedDefaultClass) {
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
      "hardware_interface", "hardware_interface::SystemInterface");
  auto hardware = loader.createUniqueInstance("franka_hardware/FrankaMultiHardwareInterface");

  ASSERT_NE(hardware, nullptr);
  EXPECT_NE(dynamic_cast<FrankaMultiHardwareInterface*>(hardware.get()), nullptr);
}

TEST(BackendOperationPolicyTest, RejectsParameterCallDuringControlToReadingTransition) {
  EXPECT_FALSE(
      detail::isParameterOperationSafe(false, false, ControlMode::None, ControlMode::JointTorque));
  EXPECT_FALSE(detail::isParameterOperationSafe(false, false, ControlMode::JointVelocity,
                                                ControlMode::None));
  EXPECT_TRUE(detail::isParameterOperationSafe(false, false, ControlMode::None, ControlMode::None));
  EXPECT_TRUE(detail::isParameterOperationSafe(false, true, ControlMode::JointTorque,
                                               ControlMode::JointTorque));
  EXPECT_FALSE(detail::isParameterOperationSafe(true, true, ControlMode::None, ControlMode::None));
}

}  // namespace
}  // namespace franka_hardware
