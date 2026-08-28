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

#include <franka/rate_limiting.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <control_msgs/msg/joint_jog.hpp>
#include <controller_interface/test_utils.hpp>
#include <cstddef>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <pluginlib/class_loader.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "dual_arm_joint_velocity_controller_core.hpp"
#include "franka_example_controllers/panda_joint_limits.hpp"

namespace franka_example_controllers {

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

  static bool enabled(const DualArmJointVelocityController& controller, const size_t arm) {
    return controller.core_->armEnabled(arm);
  }

  static std::string topic(const DualArmJointVelocityController& controller, const size_t arm) {
    return controller.core_->subscriptionTopic(arm);
  }

  static std::string service(const DualArmJointVelocityController& controller, const size_t arm) {
    return controller.core_->enableServiceName(arm);
  }
};

namespace {

constexpr size_t kArmCount = 2;
constexpr size_t kJointCount = 7;
constexpr size_t kCommandCount = kArmCount * kJointCount;
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

struct ControllerParameters {
  std::array<std::string, kArmCount> arm_ids{"arm", "arm_extra"};
  std::array<JointNames, kArmCount> joint_names{makeJointNames("arm"), makeJointNames("arm_extra")};
  std::array<std::vector<double>, kArmCount> max_velocity{std::vector<double>(kJointCount, 1.0),
                                                          std::vector<double>(kJointCount, 1.5)};
  std::array<std::vector<double>, kArmCount> max_acceleration{
      std::vector<double>(kJointCount, 7.0), std::vector<double>(kJointCount, 7.0)};
  std::array<std::vector<std::string>, kArmCount> joint_name_overrides{};
  bool use_joint_name_overrides{false};
  double watchdog_timeout{10.0};
  double max_header_age{1.0};
  double future_tolerance{0.1};
};

std::vector<std::string> asVector(const JointNames& names) {
  return {names.begin(), names.end()};
}

std::unique_ptr<DualArmJointVelocityController> makeController(
    const ControllerParameters& parameters) {
  rclcpp::NodeOptions node_options;
  node_options.enable_rosout(false);
  node_options.start_parameter_event_publisher(false);
  node_options.start_parameter_services(false);
  node_options.parameter_overrides({
      rclcpp::Parameter("arm_1.arm_id", parameters.arm_ids[0]),
      rclcpp::Parameter("arm_1.joint_names", parameters.use_joint_name_overrides
                                                 ? parameters.joint_name_overrides[0]
                                                 : asVector(parameters.joint_names[0])),
      rclcpp::Parameter("arm_1.max_velocity", parameters.max_velocity[0]),
      rclcpp::Parameter("arm_1.max_acceleration", parameters.max_acceleration[0]),
      rclcpp::Parameter("arm_2.arm_id", parameters.arm_ids[1]),
      rclcpp::Parameter("arm_2.joint_names", parameters.use_joint_name_overrides
                                                 ? parameters.joint_name_overrides[1]
                                                 : asVector(parameters.joint_names[1])),
      rclcpp::Parameter("arm_2.max_velocity", parameters.max_velocity[1]),
      rclcpp::Parameter("arm_2.max_acceleration", parameters.max_acceleration[1]),
      rclcpp::Parameter("watchdog_timeout", parameters.watchdog_timeout),
      rclcpp::Parameter("max_header_age", parameters.max_header_age),
      rclcpp::Parameter("future_tolerance", parameters.future_tolerance),
  });

  controller_interface::ControllerInterfaceParams controller_parameters;
  controller_parameters.controller_name = "dual_arm_joint_velocity_controller_test";
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
                                        const int64_t stamp_ns = kRosNowNs,
                                        const bool reverse = false) {
  control_msgs::msg::JointJog message;
  message.header.stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
  message.header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);
  for (size_t joint = 0; joint < kJointCount; ++joint) {
    const size_t source = reverse ? kJointCount - joint - 1 : joint;
    message.joint_names.push_back(names[source]);
    message.velocities.push_back(velocities[source]);
  }
  return message;
}

class VelocityHardwareFixture {
 public:
  explicit VelocityHardwareFixture(const std::array<JointNames, kArmCount>& names) {
    handles_.reserve(kCommandCount);
    for (size_t arm = 0; arm < kArmCount; ++arm) {
      for (size_t joint = 0; joint < kJointCount; ++joint) {
        const size_t command_index = handles_.size();
        handles_.push_back(std::make_shared<hardware_interface::CommandInterface>(
            names[arm][joint], "velocity", &commands_[arm][joint]));
        handles_.back()->set_on_set_command_limiter(
            [this, command_index](const double value, bool& limited) {
              ++write_counts_[command_index];
              limited = false;
              return value;
            });
      }
    }
  }

  void assignTo(DualArmJointVelocityController& controller, const bool shuffled) const {
    auto order = handles_;
    if (shuffled) {
      std::rotate(order.begin(), order.begin() + 5, order.end());
      std::reverse(order.begin(), order.end());
    }
    std::vector<hardware_interface::LoanedCommandInterface> interfaces;
    interfaces.reserve(order.size());
    for (const auto& handle : order) {
      interfaces.emplace_back(handle, hardware_interface::LoanedCommandInterface::Deleter{});
    }
    controller.assign_interfaces(std::move(interfaces), {});
  }

  void replace(const std::string& expected_name, const std::string& replacement_name) {
    auto iterator = std::find_if(handles_.begin(), handles_.end(), [&](const auto& handle) {
      return handle->get_name() == expected_name;
    });
    if (iterator == handles_.end()) {
      throw std::runtime_error("command interface to replace was not found");
    }
    *iterator = std::make_shared<hardware_interface::CommandInterface>(replacement_name, "velocity",
                                                                       &replacement_command_);
  }

  void duplicate(const std::string& missing_name, const std::string& duplicate_name) {
    auto missing = std::find_if(handles_.begin(), handles_.end(), [&](const auto& handle) {
      return handle->get_name() == missing_name;
    });
    auto duplicate = std::find_if(handles_.begin(), handles_.end(), [&](const auto& handle) {
      return handle->get_name() == duplicate_name;
    });
    if (missing == handles_.end() || duplicate == handles_.end()) {
      throw std::runtime_error("command interface for duplicate injection was not found");
    }
    *missing = *duplicate;
  }

  void fill(const double value) {
    for (auto& commands : commands_) {
      commands.fill(value);
    }
  }

  double command(const size_t arm, const size_t joint) const { return commands_[arm][joint]; }

  bool armEquals(const size_t arm, const JointArray& expected) const {
    return commands_[arm] == expected;
  }

  bool allEqual(const double expected) const {
    for (const auto& commands : commands_) {
      if (!std::all_of(commands.begin(), commands.end(),
                       [&](const double value) { return value == expected; })) {
        return false;
      }
    }
    return true;
  }

  hardware_interface::CommandInterface::SharedPtr handle(const std::string& name) const {
    const auto iterator = std::find_if(handles_.begin(), handles_.end(),
                                       [&](const auto& item) { return item->get_name() == name; });
    if (iterator == handles_.end()) {
      throw std::runtime_error("command interface was not found");
    }
    return *iterator;
  }

  void resetWriteCounts() { write_counts_.fill(0); }

  bool everyInterfaceWritten(const size_t expected_count) const {
    return std::all_of(write_counts_.begin(), write_counts_.end(),
                       [&](const size_t count) { return count == expected_count; });
  }

  bool everyInterfaceWrittenTwice() const { return everyInterfaceWritten(2); }

  size_t writeCount(const size_t command) const { return write_counts_.at(command); }

 private:
  std::array<JointArray, kArmCount> commands_{};
  std::vector<hardware_interface::CommandInterface::SharedPtr> handles_;
  std::array<size_t, kCommandCount> write_counts_{};
  double replacement_command_{0.0};
};

VelocityCommandPolicy makePolicy(const JointNames& names) {
  VelocityCommandPolicy policy;
  policy.joint_names = names;
  policy.max_velocity.fill(1.0);
  policy.max_header_age_ns = 1000000000LL;
  policy.future_tolerance_ns = 100000000LL;
  return policy;
}

struct ReferenceJointJogResult {
  JointJogValidationResult result{JointJogValidationResult::Accepted};
  JointArray velocities{};
};

ReferenceJointJogResult referenceJointJogValidation(const control_msgs::msg::JointJog& message,
                                                    const JointNames& names,
                                                    const JointArray& limits,
                                                    const int64_t ros_now_ns,
                                                    const int64_t maximum_age_ns,
                                                    const int64_t future_tolerance_ns) {
  if (!message.header.frame_id.empty()) {
    return {JointJogValidationResult::InvalidFrame, {}};
  }
  if (message.header.stamp.sec < 0 || message.header.stamp.nanosec >= 1000000000U ||
      (message.header.stamp.sec == 0 && message.header.stamp.nanosec == 0)) {
    return {JointJogValidationResult::InvalidStamp, {}};
  }
  const int64_t stamp_ns = static_cast<int64_t>(message.header.stamp.sec) * 1000000000LL +
                           static_cast<int64_t>(message.header.stamp.nanosec);
  const long double age = static_cast<long double>(ros_now_ns) - static_cast<long double>(stamp_ns);
  if (age > static_cast<long double>(maximum_age_ns)) {
    return {JointJogValidationResult::HeaderTooOld, {}};
  }
  if (-age > static_cast<long double>(future_tolerance_ns)) {
    return {JointJogValidationResult::HeaderTooFarInFuture, {}};
  }
  if (!std::isfinite(message.duration) || message.duration != 0.0) {
    return {JointJogValidationResult::InvalidDuration, {}};
  }
  if (!message.displacements.empty()) {
    return {JointJogValidationResult::DisplacementCommandNotAllowed, {}};
  }
  if (message.joint_names.size() != kJointCount) {
    return {JointJogValidationResult::InvalidNameCount, {}};
  }
  if (message.velocities.size() != kJointCount) {
    return {JointJogValidationResult::InvalidVelocityCount, {}};
  }

  ReferenceJointJogResult expected;
  std::array<bool, kJointCount> matched{};
  for (std::size_t message_index = 0; message_index < kJointCount; ++message_index) {
    const auto found = std::find(names.begin(), names.end(), message.joint_names[message_index]);
    if (found == names.end()) {
      return {JointJogValidationResult::DuplicateOrUnknownJoint, {}};
    }
    const auto joint = static_cast<std::size_t>(std::distance(names.begin(), found));
    if (matched[joint]) {
      return {JointJogValidationResult::DuplicateOrUnknownJoint, {}};
    }
    const auto velocity = message.velocities[message_index];
    if (!std::isfinite(velocity)) {
      return {JointJogValidationResult::NonfiniteVelocity, {}};
    }
    if (std::abs(velocity) > limits[joint]) {
      return {JointJogValidationResult::VelocityLimitExceeded, {}};
    }
    matched[joint] = true;
    expected.velocities[joint] = velocity;
  }
  return expected;
}

std::string mutateGeneratedJointJog(control_msgs::msg::JointJog& message,
                                    const std::size_t category,
                                    std::mt19937_64& engine) {
  const auto index = static_cast<std::size_t>(engine() % kJointCount);
  switch (category) {
    case 0:
      return "valid canonical";
    case 1:
      message.header.frame_id = "base";
      return "nonempty frame";
    case 2:
      message.header.stamp.sec = 0;
      message.header.stamp.nanosec = 0;
      return "zero stamp";
    case 3:
      message.header.stamp.sec = -1;
      return "negative stamp";
    case 4:
      message.header.stamp.nanosec = 1000000000U;
      return "invalid stamp nanoseconds";
    case 5:
      message.header.stamp.sec = 8;
      message.header.stamp.nanosec = 999999999U;
      return "header one nanosecond too old";
    case 6:
      message.header.stamp.sec = 10;
      message.header.stamp.nanosec = 100000001U;
      return "header one nanosecond too far in future";
    case 7: {
      constexpr std::array<double, 4> kInvalidDurations{-0.1, 0.1,
                                                        std::numeric_limits<double>::infinity(),
                                                        std::numeric_limits<double>::quiet_NaN()};
      const auto variant = engine() % kInvalidDurations.size();
      message.duration = kInvalidDurations[variant];
      return "invalid duration variant=" + std::to_string(variant);
    }
    case 8:
      message.displacements.push_back(0.0);
      return "displacement present";
    case 9:
      message.joint_names.erase(message.joint_names.begin() + static_cast<std::ptrdiff_t>(index));
      return "missing joint name index=" + std::to_string(index);
    case 10:
      message.velocities.erase(message.velocities.begin() + static_cast<std::ptrdiff_t>(index));
      return "missing velocity index=" + std::to_string(index);
    case 11:
      message.joint_names[index] = message.joint_names[(index + 1U) % kJointCount];
      return "duplicate joint name index=" + std::to_string(index);
    case 12:
      message.joint_names[index] = "unknown_joint";
      return "unknown joint name index=" + std::to_string(index);
    case 13:
      message.velocities[index] = std::numeric_limits<double>::quiet_NaN();
      return "nan velocity index=" + std::to_string(index);
    case 14:
      message.velocities[index] = (engine() & 1U) != 0U ? std::numeric_limits<double>::infinity()
                                                        : -std::numeric_limits<double>::infinity();
      return "infinite velocity index=" + std::to_string(index);
    case 15:
      message.velocities[index] = (engine() & 1U) != 0U ? 1.0 : -1.0;
      return "exact velocity bound index=" + std::to_string(index);
    case 16:
      message.velocities[index] =
          (engine() & 1U) != 0U ? std::nextafter(1.0, std::numeric_limits<double>::infinity())
                                : std::nextafter(-1.0, -std::numeric_limits<double>::infinity());
      return "velocity beyond bound index=" + std::to_string(index);
    default: {
      std::vector<std::size_t> order(kJointCount);
      for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        order[joint] = joint;
      }
      std::shuffle(order.begin(), order.end(), engine);
      const auto original_names = message.joint_names;
      const auto original_velocities = message.velocities;
      for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        message.joint_names[joint] = original_names[order[joint]];
        message.velocities[joint] = original_velocities[order[joint]];
      }
      return "valid permutation";
    }
  }
}

class DualArmJointVelocityControllerTest : public ::testing::Test {
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

TEST_F(DualArmJointVelocityControllerTest, DeclaresExactInterfacesTopicsAndServices) {
  ControllerParameters parameters;
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));

  const auto commands = controller->command_interface_configuration();
  const auto states = controller->state_interface_configuration();
  ASSERT_EQ(commands.names.size(), kCommandCount);
  EXPECT_EQ(states.type, controller_interface::interface_configuration_type::NONE);
  for (size_t arm = 0; arm < kArmCount; ++arm) {
    for (const auto& name : parameters.joint_names[arm]) {
      EXPECT_EQ(std::count(commands.names.begin(), commands.names.end(), name + "/velocity"), 1);
    }
    EXPECT_EQ(
        DualArmJointVelocityControllerTestAccess::topic(*controller, arm),
        "/dual_arm_joint_velocity_controller_test/arm_" + std::to_string(arm + 1) + "/joint_jog");
    EXPECT_EQ(
        DualArmJointVelocityControllerTestAccess::service(*controller, arm),
        "/dual_arm_joint_velocity_controller_test/arm_" + std::to_string(arm + 1) + "/enable");
    EXPECT_FALSE(DualArmJointVelocityControllerTestAccess::enabled(*controller, arm));
  }
}

TEST_F(DualArmJointVelocityControllerTest, RejectsInvalidIdsJointNamesAndDuplicateBindings) {
  for (size_t scenario = 0; scenario < 10; ++scenario) {
    SCOPED_TRACE(scenario);
    ControllerParameters parameters;
    switch (scenario) {
      case 0:
        parameters.arm_ids[0] = parameters.arm_ids[1];
        break;
      case 1:
        parameters.arm_ids[0] = "1arm";
        break;
      case 2:
        parameters.arm_ids[0] = "_arm";
        break;
      case 3:
        parameters.arm_ids[0] = "bad/arm";
        break;
      case 4:
        parameters.joint_names[0][0] = "bad/joint";
        break;
      case 5:
        parameters.joint_names[0][0] = "white space";
        break;
      case 6:
        parameters.joint_names[0][0].clear();
        break;
      case 7:
        parameters.joint_names[0][1] = parameters.joint_names[0][0];
        break;
      case 8:
        parameters.joint_names[1][2] = parameters.joint_names[0][2];
        break;
      case 9:
        parameters.use_joint_name_overrides = true;
        parameters.joint_name_overrides[0] = asVector(parameters.joint_names[0]);
        parameters.joint_name_overrides[0].pop_back();
        parameters.joint_name_overrides[1] = asVector(parameters.joint_names[1]);
        break;
      default:
        FAIL() << "unhandled scenario";
    }
    EXPECT_FALSE(configure(makeController(parameters)));
  }
}

TEST_F(DualArmJointVelocityControllerTest, Accepts64CharacterArmIdAndRejects65) {
  ControllerParameters maximum_length;
  maximum_length.arm_ids[0] = "a" + std::string(kPandaArmIdMaxLength - 1U, 'x');
  maximum_length.joint_names[0] = makeJointNames(maximum_length.arm_ids[0]);
  EXPECT_TRUE(configure(makeController(maximum_length)));

  ControllerParameters too_long;
  too_long.arm_ids[0] = "a" + std::string(kPandaArmIdMaxLength, 'x');
  too_long.joint_names[0] = makeJointNames(too_long.arm_ids[0]);
  EXPECT_FALSE(configure(makeController(too_long)));
}

TEST_F(DualArmJointVelocityControllerTest, RejectsNoncanonicalConfiguredJointMappings) {
  for (size_t scenario = 0; scenario < 4; ++scenario) {
    SCOPED_TRACE(scenario);
    ControllerParameters parameters;
    switch (scenario) {
      case 0:
        std::swap(parameters.joint_names[0], parameters.joint_names[1]);
        break;
      case 1:
        std::swap(parameters.joint_names[0][0], parameters.joint_names[0][1]);
        break;
      case 2:
        parameters.joint_names[0][6] = parameters.arm_ids[0] + "_joint8";
        break;
      case 3:
        parameters.joint_names[1][0] = "wrong_joint1";
        break;
      default:
        FAIL() << "unhandled scenario";
    }
    EXPECT_FALSE(configure(makeController(parameters)));
  }
}

TEST_F(DualArmJointVelocityControllerTest, AcceptsCanonicalConfigurableArmMappingThroughLifecycle) {
  ControllerParameters parameters;
  parameters.arm_ids = {"panda_alpha", "pandaBeta2"};
  parameters.joint_names = {makeJointNames(parameters.arm_ids[0]),
                            makeJointNames(parameters.arm_ids[1])};
  VelocityHardwareFixture hardware(parameters.joint_names);
  auto controller = makeController(parameters);

  ASSERT_TRUE(configure(controller));
  const auto commands = controller->command_interface_configuration();
  ASSERT_EQ(commands.names.size(), kCommandCount);
  for (size_t arm = 0; arm < kArmCount; ++arm) {
    for (size_t joint = 0; joint < kJointCount; ++joint) {
      EXPECT_EQ(commands.names[arm * kJointCount + joint],
                parameters.arm_ids[arm] + "_joint" + std::to_string(joint + 1) + "/velocity");
    }
  }

  hardware.assignTo(*controller, true);
  ASSERT_TRUE(activate(controller));
  EXPECT_TRUE(hardware.allEqual(0.0));
  EXPECT_TRUE(controller_interface::deactivate_succeeds(controller));
  controller->release_interfaces();
}

TEST_F(DualArmJointVelocityControllerTest, RejectsInvalidRequiredLimitsAndTiming) {
  for (size_t scenario = 0; scenario < 14; ++scenario) {
    SCOPED_TRACE(scenario);
    ControllerParameters parameters;
    switch (scenario) {
      case 0:
        parameters.max_velocity[0].pop_back();
        break;
      case 1:
        parameters.max_acceleration[1].push_back(1.0);
        break;
      case 2:
        parameters.max_velocity[0][2] = 0.0;
        break;
      case 3:
        parameters.max_acceleration[1][3] = -1.0;
        break;
      case 4:
        parameters.max_velocity[1][0] = std::numeric_limits<double>::quiet_NaN();
        break;
      case 5:
        parameters.max_acceleration[0][6] = std::numeric_limits<double>::infinity();
        break;
      case 6:
        parameters.watchdog_timeout = 0.0;
        break;
      case 7:
        parameters.max_header_age = -1.0;
        break;
      case 8:
        parameters.future_tolerance = std::numeric_limits<double>::infinity();
        break;
      case 9:
        parameters.watchdog_timeout = std::numeric_limits<double>::denorm_min();
        break;
      case 10:
        parameters.max_velocity[0][0] =
            std::nextafter(franka::kMaxJointVelocity[0], std::numeric_limits<double>::infinity());
        break;
      case 11:
        parameters.max_velocity[1][6] =
            std::nextafter(franka::kMaxJointVelocity[6], std::numeric_limits<double>::infinity());
        break;
      case 12:
        parameters.max_acceleration[0][1] = std::nextafter(franka::kMaxJointAcceleration[1],
                                                           std::numeric_limits<double>::infinity());
        break;
      case 13:
        parameters.max_acceleration[1][5] = std::nextafter(franka::kMaxJointAcceleration[5],
                                                           std::numeric_limits<double>::infinity());
        break;
      default:
        FAIL() << "unhandled scenario";
    }
    EXPECT_FALSE(configure(makeController(parameters)));
  }
}

TEST_F(DualArmJointVelocityControllerTest, AcceptsExactLibfrankaPerJointCeilings) {
  ControllerParameters parameters;
  for (size_t arm = 0; arm < kArmCount; ++arm) {
    parameters.max_velocity[arm].assign(franka::kMaxJointVelocity.begin(),
                                        franka::kMaxJointVelocity.end());
    parameters.max_acceleration[arm].assign(franka::kMaxJointAcceleration.begin(),
                                            franka::kMaxJointAcceleration.end());
  }
  EXPECT_TRUE(configure(makeController(parameters)));
}

TEST_F(DualArmJointVelocityControllerTest,
       ActivationAndDisabledUpdateZeroShuffledSubstringArmBindings) {
  ControllerParameters parameters;
  VelocityHardwareFixture hardware(parameters.joint_names);
  hardware.fill(99.0);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, true);
  ASSERT_TRUE(activate(controller));
  // First-update bind (F-10c): bindInterfaces() and the initial required zero both happen on
  // this first update() cycle now (no target enabled, so it also writes the steady-state zero).
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);
  EXPECT_TRUE(hardware.allEqual(0.0));

  hardware.fill(42.0);
  EXPECT_EQ(update(*controller), controller_interface::return_type::OK);
  EXPECT_TRUE(hardware.allEqual(0.0));
}

TEST_F(DualArmJointVelocityControllerTest, ReordersNamesAndCommandsEachArmIndependentlyThenBoth) {
  ControllerParameters parameters;
  VelocityHardwareFixture hardware(parameters.joint_names);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, true);
  ASSERT_TRUE(activate(controller));

  const JointArray first{0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7};
  const JointArray second{-0.7, 0.6, -0.5, 0.4, -0.3, 0.2, -0.1};
  int64_t now = steadyNowNanoseconds();
  DualArmJointVelocityControllerTestAccess::enable(*controller, 0, true, now - 2);
  EXPECT_EQ(DualArmJointVelocityControllerTestAccess::accept(
                *controller, 0, makeMessage(parameters.joint_names[0], first, kRosNowNs, true),
                kRosNowNs, now - 1),
            JointJogValidationResult::Accepted);
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);
  EXPECT_TRUE(hardware.armEquals(0, first));
  EXPECT_TRUE(hardware.armEquals(1, JointArray{}));

  now = steadyNowNanoseconds();
  DualArmJointVelocityControllerTestAccess::enable(*controller, 1, true, now - 2);
  EXPECT_EQ(DualArmJointVelocityControllerTestAccess::accept(
                *controller, 1, makeMessage(parameters.joint_names[1], second), kRosNowNs, now - 1),
            JointJogValidationResult::Accepted);
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);
  EXPECT_TRUE(hardware.armEquals(0, first));
  EXPECT_TRUE(hardware.armEquals(1, second));

  DualArmJointVelocityControllerTestAccess::enable(*controller, 0, false, steadyNowNanoseconds());
  EXPECT_TRUE(hardware.armEquals(0, first));
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);
  EXPECT_TRUE(hardware.armEquals(0, JointArray{}));
  EXPECT_TRUE(hardware.armEquals(1, second));
}

TEST_F(DualArmJointVelocityControllerTest, RejectsEveryJointJogContractViolation) {
  const JointNames names = makeJointNames("arm");
  const JointArray velocities{0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7};
  ArmVelocityCommandInbox inbox;
  inbox.configure(makePolicy(names));

  for (size_t scenario = 0; scenario < 17; ++scenario) {
    SCOPED_TRACE(scenario);
    auto message = makeMessage(names, velocities);
    JointJogValidationResult expected = JointJogValidationResult::Accepted;
    switch (scenario) {
      case 0:
        message.header.frame_id = "arm";
        expected = JointJogValidationResult::InvalidFrame;
        break;
      case 1:
        message.header.stamp.sec = 0;
        message.header.stamp.nanosec = 0;
        expected = JointJogValidationResult::InvalidStamp;
        break;
      case 2:
        message.duration = 0.1;
        expected = JointJogValidationResult::InvalidDuration;
        break;
      case 3:
        message.duration = std::numeric_limits<double>::quiet_NaN();
        expected = JointJogValidationResult::InvalidDuration;
        break;
      case 4:
        message.displacements.push_back(0.0);
        expected = JointJogValidationResult::DisplacementCommandNotAllowed;
        break;
      case 5:
        message.joint_names.pop_back();
        expected = JointJogValidationResult::InvalidNameCount;
        break;
      case 6:
        message.velocities.pop_back();
        expected = JointJogValidationResult::InvalidVelocityCount;
        break;
      case 7:
        message.joint_names[1] = message.joint_names[0];
        expected = JointJogValidationResult::DuplicateOrUnknownJoint;
        break;
      case 8:
        message.joint_names[3] = "unknown_joint";
        expected = JointJogValidationResult::DuplicateOrUnknownJoint;
        break;
      case 9:
        message.velocities[2] = std::numeric_limits<double>::quiet_NaN();
        expected = JointJogValidationResult::NonfiniteVelocity;
        break;
      case 10:
        message.velocities[4] = std::numeric_limits<double>::infinity();
        expected = JointJogValidationResult::NonfiniteVelocity;
        break;
      case 11:
        message.velocities[5] = 1.01;
        expected = JointJogValidationResult::VelocityLimitExceeded;
        break;
      case 12:
        message.header.stamp.sec = -1;
        expected = JointJogValidationResult::InvalidStamp;
        break;
      case 13:
        message.header.stamp.nanosec = 1000000000U;
        expected = JointJogValidationResult::InvalidStamp;
        break;
      case 14:
        message.duration = -0.1;
        expected = JointJogValidationResult::InvalidDuration;
        break;
      case 15:
        message.duration = std::numeric_limits<double>::infinity();
        expected = JointJogValidationResult::InvalidDuration;
        break;
      case 16:
        message.duration = -std::numeric_limits<double>::infinity();
        expected = JointJogValidationResult::InvalidDuration;
        break;
      default:
        FAIL() << "unhandled scenario";
    }
    EXPECT_EQ(inbox.accept(message, kRosNowNs, 100 + static_cast<int64_t>(scenario)), expected);
    EXPECT_FALSE(inbox.nonRealtimeCommand().valid);
  }
}

TEST_F(DualArmJointVelocityControllerTest, EnforcesDocumentedHeaderAgeAndFuturePolicy) {
  const JointNames names = makeJointNames("arm");
  ArmVelocityCommandInbox inbox;
  inbox.configure(makePolicy(names));
  const JointArray velocities{};
  constexpr int64_t kLargestRosStampNs =
      static_cast<int64_t>(std::numeric_limits<int32_t>::max()) * 1000000000LL + 999999999LL;

  EXPECT_EQ(inbox.accept(makeMessage(names, velocities, kRosNowNs - 1000000000LL), kRosNowNs, 1),
            JointJogValidationResult::Accepted);
  EXPECT_EQ(inbox.accept(makeMessage(names, velocities, kRosNowNs - 1000000001LL), kRosNowNs, 2),
            JointJogValidationResult::HeaderTooOld);
  EXPECT_EQ(inbox.accept(makeMessage(names, velocities, kRosNowNs + 100000000LL), kRosNowNs, 3),
            JointJogValidationResult::Accepted);
  EXPECT_EQ(inbox.accept(makeMessage(names, velocities, kRosNowNs + 100000001LL), kRosNowNs, 4),
            JointJogValidationResult::HeaderTooFarInFuture);
  EXPECT_EQ(inbox.accept(makeMessage(names, velocities, kLargestRosStampNs), kLargestRosStampNs, 5),
            JointJogValidationResult::Accepted);
  EXPECT_EQ(inbox.accept(makeMessage(names, velocities, kLargestRosStampNs),
                         std::numeric_limits<int64_t>::max(), 6),
            JointJogValidationResult::HeaderTooOld);
}

TEST_F(DualArmJointVelocityControllerTest,
       FixedSeedCommandsMatchReferenceValidationAndRejectedCallbacksPreserveAppliedState) {
  constexpr std::array<std::uint64_t, 3> kSeeds{0x56454c4fU, 0x600dcafeU, 0x72656a6563746564ULL};
  constexpr std::size_t kCasesPerSeed = 4000;
  constexpr double kPeriodSeconds = 0.001;
  constexpr double kMaximumDelta = 7.0 * kPeriodSeconds;
  const JointArray limits{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

  ControllerParameters parameters;
  VelocityHardwareFixture hardware(parameters.joint_names);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, true);
  ASSERT_TRUE(activate(controller));
  DualArmJointVelocityControllerTestAccess::enable(*controller, 0, true,
                                                   steadyNowNanoseconds() - 1);

  for (const auto seed : kSeeds) {
    std::cout << "Dual-arm velocity property seed=" << seed << " cases=" << kCasesPerSeed << '\n';
    std::mt19937_64 engine(seed);
    for (std::size_t case_index = 0; case_index < kCasesPerSeed; ++case_index) {
      JointArray generated{};
      for (auto& velocity : generated) {
        const auto signed_units = static_cast<std::int64_t>(engine() % 2000001U) - 1000000LL;
        velocity = static_cast<double>(signed_units) / 1000000.0;
      }
      auto message = makeMessage(parameters.joint_names[0], generated);
      const auto category = case_index % 18U;
      const auto operation = mutateGeneratedJointJog(message, category, engine);
      SCOPED_TRACE("seed=" + std::to_string(seed) + " case=" + std::to_string(case_index) +
                   " operation=" + operation);

      JointArray before{};
      for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        before[joint] = hardware.command(0, joint);
      }
      const bool enabled_before = DualArmJointVelocityControllerTestAccess::enabled(*controller, 0);
      const auto expected = referenceJointJogValidation(message, parameters.joint_names[0], limits,
                                                        kRosNowNs, 1000000000LL, 100000000LL);
      const auto actual = DualArmJointVelocityControllerTestAccess::accept(
          *controller, 0, message, kRosNowNs, steadyNowNanoseconds());
      ASSERT_EQ(actual, expected.result);

      // Reception is non-RT: neither valid nor rejected payloads may directly alter applied
      // hardware state, and rejection must not silently toggle the arm's enable state.
      EXPECT_EQ(DualArmJointVelocityControllerTestAccess::enabled(*controller, 0), enabled_before);
      for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        EXPECT_DOUBLE_EQ(hardware.command(0, joint), before[joint]);
      }

      ASSERT_EQ(update(*controller, kPeriodSeconds), controller_interface::return_type::OK);
      for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        double expected_output = 0.0;
        if (expected.result == JointJogValidationResult::Accepted) {
          expected_output = std::clamp(expected.velocities[joint], before[joint] - kMaximumDelta,
                                       before[joint] + kMaximumDelta);
        }
        EXPECT_NEAR(hardware.command(0, joint), expected_output, 1e-15);
        EXPECT_DOUBLE_EQ(hardware.command(1, joint), 0.0);
      }
    }
  }

  ASSERT_TRUE(controller_interface::deactivate_succeeds(controller));
  controller->release_interfaces();
}

TEST_F(DualArmJointVelocityControllerTest,
       EnableRequiresNewCommandAndWatchdogOrInvalidCommandZerosArm) {
  ControllerParameters parameters;
  parameters.watchdog_timeout = 0.05;
  VelocityHardwareFixture hardware(parameters.joint_names);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, true);
  ASSERT_TRUE(activate(controller));
  const JointArray target{0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2};

  int64_t now = steadyNowNanoseconds();
  ASSERT_EQ(DualArmJointVelocityControllerTestAccess::accept(
                *controller, 0, makeMessage(parameters.joint_names[0], target), kRosNowNs, now - 2),
            JointJogValidationResult::Accepted);
  DualArmJointVelocityControllerTestAccess::enable(*controller, 0, true, now - 1);
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);
  EXPECT_TRUE(hardware.armEquals(0, JointArray{}));

  now = steadyNowNanoseconds();
  ASSERT_EQ(DualArmJointVelocityControllerTestAccess::accept(
                *controller, 0, makeMessage(parameters.joint_names[0], target), kRosNowNs, now - 1),
            JointJogValidationResult::Accepted);
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);
  EXPECT_TRUE(hardware.armEquals(0, target));

  auto invalid = makeMessage(parameters.joint_names[0], target);
  invalid.velocities[0] = 2.0;
  ASSERT_EQ(DualArmJointVelocityControllerTestAccess::accept(*controller, 0, invalid, kRosNowNs,
                                                             steadyNowNanoseconds()),
            JointJogValidationResult::VelocityLimitExceeded);
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);
  EXPECT_TRUE(hardware.armEquals(0, JointArray{}));

  now = steadyNowNanoseconds();
  DualArmJointVelocityControllerTestAccess::enable(*controller, 0, true, now - 2000000000LL);
  ASSERT_EQ(DualArmJointVelocityControllerTestAccess::accept(
                *controller, 0, makeMessage(parameters.joint_names[0], target), kRosNowNs, now - 1),
            JointJogValidationResult::Accepted);
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);
  EXPECT_TRUE(hardware.armEquals(0, target));

  ASSERT_EQ(DualArmJointVelocityControllerTestAccess::accept(
                *controller, 0, makeMessage(parameters.joint_names[0], target), kRosNowNs,
                now - 1000000000LL),
            JointJogValidationResult::Accepted);
  EXPECT_TRUE(hardware.armEquals(0, target));
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);
  EXPECT_TRUE(hardware.armEquals(0, JointArray{}));
}

TEST_F(DualArmJointVelocityControllerTest, AppliesConfiguredAccelerationAndVelocityBounds) {
  ControllerParameters parameters;
  parameters.max_velocity[0] = std::vector<double>(kJointCount, 0.8);
  parameters.max_acceleration[0] = std::vector<double>(kJointCount, 0.5);
  VelocityHardwareFixture hardware(parameters.joint_names);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, false);
  ASSERT_TRUE(activate(controller));

  const JointArray target{0.8, -0.8, 0.6, -0.6, 0.4, -0.4, 0.2};
  const int64_t now = steadyNowNanoseconds();
  DualArmJointVelocityControllerTestAccess::enable(*controller, 0, true, now - 2);
  ASSERT_EQ(DualArmJointVelocityControllerTestAccess::accept(
                *controller, 0, makeMessage(parameters.joint_names[0], target), kRosNowNs, now - 1),
            JointJogValidationResult::Accepted);

  ASSERT_EQ(update(*controller, 0.1), controller_interface::return_type::OK);
  for (size_t joint = 0; joint < kJointCount; ++joint) {
    const double expected = target[joint] > 0.0 ? 0.05 : -0.05;
    EXPECT_NEAR(hardware.command(0, joint), expected, 1e-12);
  }
  ASSERT_EQ(update(*controller, 0.1), controller_interface::return_type::OK);
  for (size_t joint = 0; joint < kJointCount; ++joint) {
    const double expected = target[joint] > 0.0 ? 0.1 : -0.1;
    EXPECT_NEAR(hardware.command(0, joint), expected, 1e-12);
    EXPECT_LE(std::abs(hardware.command(0, joint)), parameters.max_velocity[0][joint]);
  }

  // Disable/stale/invalid input is a safety stop: it bypasses the normal acceleration ramp.
  DualArmJointVelocityControllerTestAccess::enable(*controller, 0, false, steadyNowNanoseconds());
  ASSERT_EQ(update(*controller, 0.1), controller_interface::return_type::OK);
  EXPECT_TRUE(hardware.allEqual(0.0));

  EXPECT_EQ(update(*controller, 0.0), controller_interface::return_type::ERROR);
  EXPECT_TRUE(hardware.allEqual(0.0));
}

TEST_F(DualArmJointVelocityControllerTest, RejectsMissingDuplicateAndInexactInterfaceBindings) {
  for (size_t scenario = 0; scenario < 2; ++scenario) {
    SCOPED_TRACE(scenario);
    ControllerParameters parameters;
    VelocityHardwareFixture hardware(parameters.joint_names);
    if (scenario == 0) {
      hardware.replace("arm_joint1/velocity", "arm_extra_joint1_extra");
    } else {
      hardware.duplicate("arm_extra_joint7/velocity", "arm_joint7/velocity");
    }
    auto controller = makeController(parameters);
    ASSERT_TRUE(configure(controller));
    hardware.assignTo(*controller, true);
    // F-10c amendment A.4: on_activate()'s restored read-only wiring validation rejects both
    // substitutions again, externally visible to controller_manager, instead of activating and
    // then erroring out one update() cycle later.
    EXPECT_FALSE(activate(controller));
    EXPECT_EQ(update(*controller), controller_interface::return_type::ERROR);
  }
}

TEST_F(DualArmJointVelocityControllerTest, AggregatesEveryWriteAndZerosAfterOneFailure) {
  ControllerParameters parameters;
  VelocityHardwareFixture hardware(parameters.joint_names);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, true);
  ASSERT_TRUE(activate(controller));
  const JointArray target{0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
  const int64_t now = steadyNowNanoseconds();
  DualArmJointVelocityControllerTestAccess::enable(*controller, 0, true, now - 2);
  ASSERT_EQ(DualArmJointVelocityControllerTestAccess::accept(
                *controller, 0, makeMessage(parameters.joint_names[0], target), kRosNowNs, now - 1),
            JointJogValidationResult::Accepted);
  hardware.fill(77.0);
  hardware.resetWriteCounts();

  auto failing_handle = hardware.handle("arm_joint4/velocity");
  std::unique_lock<std::shared_mutex> lock(failing_handle->get_mutex());
  EXPECT_EQ(update(*controller), controller_interface::return_type::ERROR);
  EXPECT_TRUE(hardware.everyInterfaceWrittenTwice());
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

TEST_F(DualArmJointVelocityControllerTest,
       ActivationFailureReleaseRequiresCleanupReconfigureAndFreshAssignment) {
  ControllerParameters parameters;
  VelocityHardwareFixture hardware(parameters.joint_names);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, true);
  hardware.fill(4.0);
  hardware.resetWriteCounts();

  auto failing_handle = hardware.handle("arm_joint4/velocity");
  std::unique_lock<std::shared_mutex> lock(failing_handle->get_mutex());
  // First-update bind (F-10c): on_activate()'s restored validation is read-only (amendment A.4),
  // so a *locked* handle -- which only fails writes -- does not affect it and activation still
  // succeeds; the locked joint4 write failure surfaces once the first update() cycle attempts the
  // post-bind zero (twice: once from serviceFirstUpdateActivation()'s own attemptRequiredZero(),
  // once more from update()'s interfaces_bound_-but-inactive fallthrough).
  ASSERT_TRUE(activate(controller));
  EXPECT_EQ(update(*controller), controller_interface::return_type::ERROR);
  EXPECT_TRUE(hardware.everyInterfaceWrittenTwice());

  // Match ControllerManager: release claims after the failed transition. F-10c amendment A: the
  // override writes nothing -- it only clears every cached raw interface pointer.
  controller->release_interfaces();
  for (size_t command = 0; command < kCommandCount; ++command) {
    EXPECT_EQ(hardware.writeCount(command), 2U) << "command " << command;
  }
  lock.unlock();
  EXPECT_EQ(update(*controller), controller_interface::return_type::ERROR);
  // First-update bind (F-10c): on_activate() itself no longer detects this failure
  // synchronously, so the lifecycle node is still ACTIVE at this point -- exactly as
  // controller_manager would see it until it notices update() returning ERROR and deactivates the
  // controller itself. Do that explicitly here (a raw unit test has no controller_manager to do
  // it) before re-attempting activation, matching that real-system recovery path.
  ASSERT_TRUE(controller_interface::deactivate_succeeds(controller));
  EXPECT_FALSE(activate(controller));

  ASSERT_TRUE(controller_interface::cleanup_succeeds(controller));
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, false);
  ASSERT_TRUE(activate(controller));
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);
  EXPECT_TRUE(hardware.allEqual(0.0));
  ASSERT_TRUE(controller_interface::deactivate_succeeds(controller));
  controller->release_interfaces();
}

TEST_F(DualArmJointVelocityControllerTest, DeactivationCascadeWritesNothingEvenWithALockedHandle) {
  ControllerParameters parameters;
  VelocityHardwareFixture hardware(parameters.joint_names);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, true);
  ASSERT_TRUE(activate(controller));
  // First-update bind (F-10c): establish a fully bound, active controller before exercising the
  // failing-deactivation cascade below.
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);
  hardware.fill(5.0);
  hardware.resetWriteCounts();

  auto failing_handle = hardware.handle("arm_joint4/velocity");
  std::unique_lock<std::shared_mutex> lock(failing_handle->get_mutex());
  const auto state = controller->get_node()->deactivate();
  // F-10c amendment A: on_deactivate() writes nothing, so a locked handle can no longer make it
  // fail, and the node reaches INACTIVE instead of cascading through on_error to FINALIZED.
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_TRUE(hardware.everyInterfaceWritten(0));
  controller->release_interfaces();
  EXPECT_TRUE(hardware.everyInterfaceWritten(0));
  lock.unlock();
  EXPECT_EQ(update(*controller), controller_interface::return_type::ERROR);
}

TEST_F(DualArmJointVelocityControllerTest, ShutdownErrorReleaseLeavesNoDanglingDereference) {
  ControllerParameters parameters;
  auto hardware = std::make_unique<VelocityHardwareFixture>(parameters.joint_names);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware->assignTo(*controller, true);
  ASSERT_TRUE(activate(controller));
  // First-update bind (F-10c): establish a fully bound, active controller before exercising the
  // failing-shutdown cascade below.
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);
  hardware->fill(6.0);
  hardware->resetWriteCounts();

  auto failing_handle = hardware->handle("arm_joint4/velocity");
  std::unique_lock<std::shared_mutex> lock(failing_handle->get_mutex());
  const auto state = controller->get_node()->shutdown();
  // F-10c amendment A: on_shutdown() writes nothing, so it can no longer fail on a locked handle
  // and the node finalizes cleanly. What this test is about -- no dereference of a released loan
  // afterwards -- is unchanged.
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
  EXPECT_TRUE(hardware->everyInterfaceWritten(0));
  controller->release_interfaces();
  EXPECT_TRUE(hardware->everyInterfaceWritten(0));
  lock.unlock();
  failing_handle.reset();
  hardware.reset();

  EXPECT_EQ(update(*controller), controller_interface::return_type::ERROR);
  controller->release_interfaces();
}

TEST_F(DualArmJointVelocityControllerTest, CleanupAfterCmStyleReleaseAllowsFreshConfigure) {
  ControllerParameters parameters;
  VelocityHardwareFixture hardware(parameters.joint_names);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, true);
  ASSERT_TRUE(activate(controller));
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);
  hardware.fill(6.0);
  hardware.resetWriteCounts();

  // F-10c amendment A: neither the deactivate nor the release writes a command interface.
  ASSERT_TRUE(controller_interface::deactivate_succeeds(controller));
  EXPECT_TRUE(hardware.everyInterfaceWritten(0));
  controller->release_interfaces();
  EXPECT_TRUE(hardware.everyInterfaceWritten(0));
  ASSERT_TRUE(controller_interface::cleanup_succeeds(controller));
  ASSERT_TRUE(configure(controller));
  EXPECT_FALSE(DualArmJointVelocityControllerTestAccess::topic(*controller, 0).empty());
  EXPECT_FALSE(DualArmJointVelocityControllerTestAccess::service(*controller, 1).empty());
}

TEST_F(DualArmJointVelocityControllerTest,
       RejectsNegativeDurationAndAcceptsMaximumRepresentableDuration) {
  ControllerParameters parameters;
  VelocityHardwareFixture hardware(parameters.joint_names);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, true);
  ASSERT_TRUE(activate(controller));
  const JointArray target{0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7};
  const int64_t now = steadyNowNanoseconds();
  DualArmJointVelocityControllerTestAccess::enable(*controller, 0, true, now - 2);
  ASSERT_EQ(DualArmJointVelocityControllerTestAccess::accept(
                *controller, 0, makeMessage(parameters.joint_names[0], target), kRosNowNs, now - 1),
            JointJogValidationResult::Accepted);

  EXPECT_EQ(
      controller->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_nanoseconds(-1)),
      controller_interface::return_type::ERROR);
  EXPECT_TRUE(hardware.allEqual(0.0));

  // rclcpp::Duration stores signed integer nanoseconds, so NaN/Inf cannot be constructed through
  // its public API. Construct the largest period representable by its int64 nanosecond API.
  const auto maximum_period =
      rclcpp::Duration::from_nanoseconds(std::numeric_limits<int64_t>::max());
  EXPECT_EQ(maximum_period.nanoseconds(), std::numeric_limits<int64_t>::max());
  ASSERT_TRUE(std::isfinite(maximum_period.seconds()));
  EXPECT_EQ(controller->update(rclcpp::Time(0, 0, RCL_ROS_TIME), maximum_period),
            controller_interface::return_type::OK);
  EXPECT_TRUE(hardware.armEquals(0, target));
  EXPECT_TRUE(hardware.armEquals(1, JointArray{}));
}

TEST_F(DualArmJointVelocityControllerTest, DeactivationDisablesBothArmsAndTheOwnerThreadZeros) {
  ControllerParameters parameters;
  VelocityHardwareFixture hardware(parameters.joint_names);
  auto controller = makeController(parameters);
  ASSERT_TRUE(configure(controller));
  hardware.assignTo(*controller, true);
  ASSERT_TRUE(activate(controller));
  ASSERT_EQ(update(*controller), controller_interface::return_type::OK);
  hardware.fill(3.0);
  // F-10c amendment A: on_deactivate() disables both arms but writes nothing; the zero lands on
  // the owner thread's next update() cycle, before release_interfaces() unbinds.
  EXPECT_TRUE(controller_interface::deactivate_succeeds(controller));
  EXPECT_TRUE(hardware.allEqual(3.0));
  EXPECT_EQ(update(*controller), controller_interface::return_type::ERROR);
  EXPECT_TRUE(hardware.allEqual(0.0));
  controller->release_interfaces();
  EXPECT_TRUE(hardware.allEqual(0.0));
  EXPECT_FALSE(DualArmJointVelocityControllerTestAccess::enabled(*controller, 0));
  EXPECT_FALSE(DualArmJointVelocityControllerTestAccess::enabled(*controller, 1));
}

TEST_F(DualArmJointVelocityControllerTest, PluginHasDistinctLoadableName) {
  pluginlib::ClassLoader<controller_interface::ControllerInterface> loader(
      "controller_interface", "controller_interface::ControllerInterface");
  EXPECT_TRUE(loader.isClassAvailable("franka_example_controllers/DualArmJointVelocityController"));
  auto controller =
      loader.createUniqueInstance("franka_example_controllers/DualArmJointVelocityController");
  EXPECT_NE(controller, nullptr);
}

}  // namespace
}  // namespace franka_example_controllers
