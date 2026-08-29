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

#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <rclcpp/logger.hpp>

#include "franka_hardware/real/franka_arm_backend.hpp"

namespace franka_hardware {

namespace detail {

constexpr bool isParameterOperationSafe(bool has_fault,
                                        bool is_stopped,
                                        ControlMode requested_mode,
                                        ControlMode active_mode) noexcept {
  return !has_fault &&
         (is_stopped || (requested_mode == ControlMode::None && active_mode == ControlMode::None));
}

}  // namespace detail

class Robot;

class RealFrankaArmBackend final : public FrankaArmBackend {
 public:
  RealFrankaArmBackend(const std::string& arm_name,
                       const std::string& robot_address,
                       const rclcpp::Logger& logger);

  bool startStateReading() override;
  bool stop() override;
  franka::RobotState readLatestState() override;
  ModelBase* model() noexcept override;

  bool canPublishCommand() const noexcept override;
  bool publishCommand(const RobotCommand& command) noexcept override;
  bool canRequestControlMode(ControlMode control_mode) const noexcept override;
  bool requestControlMode(ControlMode control_mode) noexcept override;
  ControlMode requestedControlMode() const noexcept override;
  ControlMode activeControlMode() const noexcept override;
  bool modeEntryInFlight() const noexcept override;

  bool hasFault() const noexcept override;
  bool recoverToReading() override;
  FrankaArmBackendDiagnostics diagnostics() const noexcept override;

  void setJointStiffness(
      const franka_msgs::srv::SetJointStiffness::Request::SharedPtr& request) override;
  void setCartesianStiffness(
      const franka_msgs::srv::SetCartesianStiffness::Request::SharedPtr& request) override;
  void setLoad(const franka_msgs::srv::SetLoad::Request::SharedPtr& request) override;
  void setTCPFrame(const franka_msgs::srv::SetTCPFrame::Request::SharedPtr& request) override;
  void setStiffnessFrame(
      const franka_msgs::srv::SetStiffnessFrame::Request::SharedPtr& request) override;
  void setForceTorqueCollisionBehavior(
      const franka_msgs::srv::SetForceTorqueCollisionBehavior::Request::SharedPtr& request)
      override;
  void setFullCollisionBehavior(
      const franka_msgs::srv::SetFullCollisionBehavior::Request::SharedPtr& request) override;

 private:
  bool parameterOperationAllowed() const noexcept;
  void beginParameterOperation();

  std::shared_ptr<Robot> robot_;
  std::atomic<BackendServiceOperation> service_operation_{BackendServiceOperation::Idle};
};

}  // namespace franka_hardware
