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

#include "franka_hardware/real/real_franka_arm_backend.hpp"

#include <stdexcept>
#include <utility>

#include "franka_hardware/real/robot.hpp"

namespace franka_hardware {
namespace {

static_assert(std::atomic<BackendServiceOperation>::is_always_lock_free,
              "The backend service-operation gate must be lock-free");

class ServiceOperationReset {
 public:
  explicit ServiceOperationReset(std::atomic<BackendServiceOperation>& operation) noexcept
      : operation_(operation) {}
  ServiceOperationReset(const ServiceOperationReset&) = delete;
  ServiceOperationReset& operator=(const ServiceOperationReset&) = delete;
  ~ServiceOperationReset() { operation_.store(BackendServiceOperation::Idle); }

 private:
  std::atomic<BackendServiceOperation>& operation_;
};

BackendWorkerState toBackendWorkerState(ControlLoopWorker::State state) noexcept {
  switch (state) {
    case ControlLoopWorker::State::Stopped:
      return BackendWorkerState::Stopped;
    case ControlLoopWorker::State::Starting:
      return BackendWorkerState::Starting;
    case ControlLoopWorker::State::Running:
      return BackendWorkerState::Running;
    case ControlLoopWorker::State::StopRequested:
      return BackendWorkerState::StopRequested;
    case ControlLoopWorker::State::Faulted:
      return BackendWorkerState::Faulted;
  }
  return BackendWorkerState::Faulted;
}

}  // namespace

RealFrankaArmBackend::RealFrankaArmBackend(const std::string& /*arm_name*/,
                                           const std::string& robot_address,
                                           const rclcpp::Logger& logger)
    : robot_(std::make_shared<Robot>(robot_address, logger)) {}

bool RealFrankaArmBackend::startStateReading() {
  auto expected = BackendServiceOperation::Idle;
  if (!service_operation_.compare_exchange_strong(expected, BackendServiceOperation::Lifecycle)) {
    return false;
  }
  ServiceOperationReset reset(service_operation_);
  return robot_->initializeContinuousReading();
}

bool RealFrankaArmBackend::stop() {
  auto expected = BackendServiceOperation::Idle;
  if (!service_operation_.compare_exchange_strong(expected, BackendServiceOperation::Lifecycle)) {
    return false;
  }
  ServiceOperationReset reset(service_operation_);
  return robot_->stopRobot();
}

franka::RobotState RealFrankaArmBackend::readLatestState() {
  return robot_->read();
}

ModelBase* RealFrankaArmBackend::model() noexcept {
  return robot_->getModel();
}

bool RealFrankaArmBackend::canPublishCommand() const noexcept {
  return robot_->canWriteCommand();
}

bool RealFrankaArmBackend::publishCommand(const RobotCommand& command) noexcept {
  return robot_->write(command.efforts, command.joint_positions, command.joint_velocities,
                       command.cartesian_positions, command.cartesian_velocities);
}

bool RealFrankaArmBackend::canRequestControlMode(ControlMode control_mode) const noexcept {
  return service_operation_.load() == BackendServiceOperation::Idle &&
         robot_->canRequestControlMode(control_mode);
}

bool RealFrankaArmBackend::requestControlMode(ControlMode control_mode) noexcept {
  auto expected = BackendServiceOperation::Idle;
  if (!service_operation_.compare_exchange_strong(expected, BackendServiceOperation::ModeRequest)) {
    return false;
  }
  ServiceOperationReset reset(service_operation_);
  return robot_->canRequestControlMode(control_mode) && robot_->requestControlMode(control_mode);
}

ControlMode RealFrankaArmBackend::requestedControlMode() const noexcept {
  return robot_->getControlMode();
}

ControlMode RealFrankaArmBackend::activeControlMode() const noexcept {
  return robot_->getActiveControlMode();
}

bool RealFrankaArmBackend::modeEntryInFlight() const noexcept {
  return robot_->modeEntryInFlight();
}

bool RealFrankaArmBackend::hasFault() const noexcept {
  return robot_->hasError();
}

bool RealFrankaArmBackend::recoverToReading() {
  auto expected = BackendServiceOperation::Idle;
  if (!service_operation_.compare_exchange_strong(expected, BackendServiceOperation::Recovery)) {
    return false;
  }
  ServiceOperationReset reset(service_operation_);
  return robot_->recoverToReading();
}

FrankaArmBackendDiagnostics RealFrankaArmBackend::diagnostics() const noexcept {
  FrankaArmBackendDiagnostics result;
  result.requested_mode = robot_->getControlMode();
  result.active_mode = robot_->getActiveControlMode();
  result.worker_state = toBackendWorkerState(robot_->getWorkerState());
  result.fault_category =
      robot_->hasError() ? BackendFaultCategory::Worker : BackendFaultCategory::None;
  result.failure_reason = robot_->failureReason();
  result.service_operation = service_operation_.load();
  result.stopped = robot_->isStopped();
  result.recovering = result.service_operation == BackendServiceOperation::Recovery;
  result.has_state_sample = robot_->hasStateSample();
  result.accepted_state_samples = robot_->acceptedStateSamples();
  result.last_accepted_state_steady_ns = robot_->lastAcceptedStateSteadyNanoseconds();
  result.dropped_state_samples = robot_->droppedStateSamples();
  result.rejected_command_samples = robot_->rejectedCommandSamples();
  result.state_queue_saturated = robot_->stateQueueSaturated();
  result.command_queue_saturated = robot_->commandQueueSaturated();
  result.recovery_attempts = robot_->recoveryAttempts();
  result.recovery_successes = robot_->recoverySuccesses();
  result.recovery_failures = robot_->recoveryFailures();
  result.last_recovery_result = robot_->lastRecoveryResult();
  return result;
}

bool RealFrankaArmBackend::parameterOperationAllowed() const noexcept {
  return detail::isParameterOperationSafe(robot_->hasError(), robot_->isStopped(),
                                          robot_->getControlMode(), robot_->getActiveControlMode());
}

void RealFrankaArmBackend::beginParameterOperation() {
  auto expected = BackendServiceOperation::Idle;
  if (!service_operation_.compare_exchange_strong(expected, BackendServiceOperation::Parameter)) {
    throw std::logic_error("another backend service operation is already active");
  }
  if (!parameterOperationAllowed()) {
    service_operation_.store(BackendServiceOperation::Idle);
    throw std::logic_error("parameter changes require stopped or state-reading mode");
  }
}

void RealFrankaArmBackend::setJointStiffness(
    const franka_msgs::srv::SetJointStiffness::Request::SharedPtr& request) {
  beginParameterOperation();
  ServiceOperationReset reset(service_operation_);
  robot_->setJointStiffness(request);
}

void RealFrankaArmBackend::setCartesianStiffness(
    const franka_msgs::srv::SetCartesianStiffness::Request::SharedPtr& request) {
  beginParameterOperation();
  ServiceOperationReset reset(service_operation_);
  robot_->setCartesianStiffness(request);
}

void RealFrankaArmBackend::setLoad(const franka_msgs::srv::SetLoad::Request::SharedPtr& request) {
  beginParameterOperation();
  ServiceOperationReset reset(service_operation_);
  robot_->setLoad(request);
}

void RealFrankaArmBackend::setTCPFrame(
    const franka_msgs::srv::SetTCPFrame::Request::SharedPtr& request) {
  beginParameterOperation();
  ServiceOperationReset reset(service_operation_);
  robot_->setTCPFrame(request);
}

void RealFrankaArmBackend::setStiffnessFrame(
    const franka_msgs::srv::SetStiffnessFrame::Request::SharedPtr& request) {
  beginParameterOperation();
  ServiceOperationReset reset(service_operation_);
  robot_->setStiffnessFrame(request);
}

void RealFrankaArmBackend::setForceTorqueCollisionBehavior(
    const franka_msgs::srv::SetForceTorqueCollisionBehavior::Request::SharedPtr& request) {
  beginParameterOperation();
  ServiceOperationReset reset(service_operation_);
  robot_->setForceTorqueCollisionBehavior(request);
}

void RealFrankaArmBackend::setFullCollisionBehavior(
    const franka_msgs::srv::SetFullCollisionBehavior::Request::SharedPtr& request) {
  beginParameterOperation();
  ServiceOperationReset reset(service_operation_);
  robot_->setFullCollisionBehavior(request);
}

}  // namespace franka_hardware
