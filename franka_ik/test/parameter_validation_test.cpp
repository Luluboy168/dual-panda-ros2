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

#include "service_test_support.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "franka_ik/service_node.hpp"

#ifndef PANDA_IK_SINGLE_TEST_URDF
#error "PANDA_IK_SINGLE_TEST_URDF must name the rendered single-arm IK wrapper"
#endif

namespace franka_ik {
namespace {

class ParameterValidationTest : public ::testing::Test {
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

  void SetUp() override { urdf_ = test::readTextFile(PANDA_IK_SINGLE_TEST_URDF); }

  std::vector<rclcpp::Parameter> validOverrides() const {
    return {
        rclcpp::Parameter("robot_description", urdf_),
        rclcpp::Parameter("arm_ids", std::vector<std::string>{"panda"}),
        rclcpp::Parameter("default_solver", "numeric"),
        rclcpp::Parameter("position_tolerance", 1.0e-4),
        rclcpp::Parameter("orientation_tolerance", 1.0e-3),
        rclcpp::Parameter("numeric_max_iterations", static_cast<std::int64_t>(200)),
        rclcpp::Parameter("numeric_eps", 1.0e-6),
        rclcpp::Parameter("joint_limit_margin_max", 0.1),
    };
  }

  rclcpp::NodeOptions optionsWith(const rclcpp::Parameter& replacement) const {
    auto overrides = validOverrides();
    overrides.erase(std::remove_if(overrides.begin(), overrides.end(),
                                   [&](const auto& parameter) {
                                     return parameter.get_name() == replacement.get_name();
                                   }),
                    overrides.end());
    overrides.push_back(replacement);
    rclcpp::NodeOptions options;
    options.use_global_arguments(false);
    options.parameter_overrides(std::move(overrides));
    return options;
  }

  void expectInvalid(const std::string& parameter_name,
                     const rclcpp::Parameter& invalid_parameter) const {
    try {
      auto node = std::make_shared<ServiceNode>(optionsWith(invalid_parameter));
      (void)node;
      FAIL() << "invalid parameter '" << parameter_name << "' was accepted";
    } catch (const std::exception& error) {
      EXPECT_NE(std::string(error.what()).find(parameter_name), std::string::npos)
          << "exception did not name parameter '" << parameter_name << "': " << error.what();
    }
  }

  std::string urdf_;
};

TEST_F(ParameterValidationTest, ValidDefaultsAndInclusiveBoundariesConstruct) {
  rclcpp::NodeOptions default_options;
  default_options.use_global_arguments(false);
  default_options.parameter_overrides(validOverrides());
  EXPECT_NO_THROW({ auto node = std::make_shared<ServiceNode>(default_options); });

  auto boundary_overrides = validOverrides();
  for (auto& parameter : boundary_overrides) {
    if (parameter.get_name() == "position_tolerance") {
      parameter = rclcpp::Parameter(parameter.get_name(), 1.0e-2);
    } else if (parameter.get_name() == "orientation_tolerance") {
      parameter = rclcpp::Parameter(parameter.get_name(), 1.0e-1);
    } else if (parameter.get_name() == "numeric_max_iterations") {
      parameter = rclcpp::Parameter(parameter.get_name(), static_cast<std::int64_t>(2000));
    } else if (parameter.get_name() == "numeric_eps") {
      parameter = rclcpp::Parameter(parameter.get_name(), 1.0e-2);
    } else if (parameter.get_name() == "joint_limit_margin_max") {
      parameter = rclcpp::Parameter(parameter.get_name(), 0.5);
    }
  }
  rclcpp::NodeOptions upper_options;
  upper_options.use_global_arguments(false);
  upper_options.parameter_overrides(boundary_overrides);
  EXPECT_NO_THROW({ auto node = std::make_shared<ServiceNode>(upper_options); });

  for (auto& parameter : boundary_overrides) {
    if (parameter.get_name() == "numeric_max_iterations") {
      parameter = rclcpp::Parameter(parameter.get_name(), static_cast<std::int64_t>(10));
    } else if (parameter.get_name() == "joint_limit_margin_max") {
      parameter = rclcpp::Parameter(parameter.get_name(), 0.0);
    }
  }
  rclcpp::NodeOptions lower_options;
  lower_options.use_global_arguments(false);
  lower_options.parameter_overrides(boundary_overrides);
  EXPECT_NO_THROW({ auto node = std::make_shared<ServiceNode>(lower_options); });
}

TEST_F(ParameterValidationTest, RobotDescriptionAndArmIdsFailHardWithNamedMessages) {
  expectInvalid("robot_description", rclcpp::Parameter("robot_description", ""));
  expectInvalid("robot_description", rclcpp::Parameter("robot_description", "not urdf"));
  expectInvalid("arm_ids", rclcpp::Parameter("arm_ids", std::vector<std::string>{}));
  expectInvalid("arm_ids", rclcpp::Parameter(
                               "arm_ids", std::vector<std::string>{"panda", "a", "b", "c", "d"}));
  expectInvalid("arm_ids", rclcpp::Parameter("arm_ids", std::vector<std::string>{"1panda"}));
  expectInvalid("arm_ids",
                rclcpp::Parameter("arm_ids", std::vector<std::string>{"panda", "panda"}));
  expectInvalid("arm_ids", rclcpp::Parameter("arm_ids", std::vector<std::string>{"panda2"}));
}

TEST_F(ParameterValidationTest, DefaultSolverRejectsUnknownAndUnavailableValues) {
  expectInvalid("default_solver", rclcpp::Parameter("default_solver", "other"));
  expectInvalid("default_solver", rclcpp::Parameter("default_solver", "analytic"));
}

TEST_F(ParameterValidationTest, PositionToleranceRejectsBothBoundsAndNonFiniteValues) {
  const std::array invalid_values{0.0, -1.0e-6, 1.0e-2 + 1.0e-9,
                                  std::numeric_limits<double>::infinity(),
                                  std::numeric_limits<double>::quiet_NaN()};
  for (const double invalid : invalid_values) {
    SCOPED_TRACE(invalid);
    expectInvalid("position_tolerance", rclcpp::Parameter("position_tolerance", invalid));
  }
}

TEST_F(ParameterValidationTest, OrientationToleranceRejectsBothBoundsAndNonFiniteValues) {
  const std::array invalid_values{0.0, -1.0e-6, 1.0e-1 + 1.0e-9,
                                  std::numeric_limits<double>::infinity(),
                                  std::numeric_limits<double>::quiet_NaN()};
  for (const double invalid : invalid_values) {
    SCOPED_TRACE(invalid);
    expectInvalid("orientation_tolerance", rclcpp::Parameter("orientation_tolerance", invalid));
  }
}

TEST_F(ParameterValidationTest, NumericIterationBudgetRejectsBothBounds) {
  for (const std::int64_t invalid : {9, 2001}) {
    SCOPED_TRACE(invalid);
    expectInvalid("numeric_max_iterations", rclcpp::Parameter("numeric_max_iterations", invalid));
  }
}

TEST_F(ParameterValidationTest, NumericEpsilonRejectsBothBoundsAndNonFiniteValues) {
  const std::array invalid_values{0.0, -1.0e-6, 1.0e-2 + 1.0e-9,
                                  std::numeric_limits<double>::infinity(),
                                  std::numeric_limits<double>::quiet_NaN()};
  for (const double invalid : invalid_values) {
    SCOPED_TRACE(invalid);
    expectInvalid("numeric_eps", rclcpp::Parameter("numeric_eps", invalid));
  }
}

TEST_F(ParameterValidationTest, MaximumJointMarginRejectsBothBoundsAndNonFiniteValues) {
  const std::array invalid_values{-1.0e-9, 0.5 + 1.0e-9, std::numeric_limits<double>::infinity(),
                                  std::numeric_limits<double>::quiet_NaN()};
  for (const double invalid : invalid_values) {
    SCOPED_TRACE(invalid);
    expectInvalid("joint_limit_margin_max", rclcpp::Parameter("joint_limit_margin_max", invalid));
  }
}

TEST_F(ParameterValidationTest, EveryConfigurationParameterIsReadOnlyAfterConstruction) {
  rclcpp::NodeOptions options;
  options.use_global_arguments(false);
  options.parameter_overrides(validOverrides());
  auto node = std::make_shared<ServiceNode>(options);
  for (const auto& parameter : validOverrides()) {
    const auto result = node->set_parameter(parameter);
    EXPECT_FALSE(result.successful) << parameter.get_name();
    EXPECT_NE(result.reason.find("read-only"), std::string::npos) << parameter.get_name();
  }
}

}  // namespace
}  // namespace franka_ik
