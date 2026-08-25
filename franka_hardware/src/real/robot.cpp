// Copyright (c) 2021 Franka Emika GmbH
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

#include <franka/control_tools.h>

#include <chrono>
#include <franka_hardware/real/robot.hpp>
#include <iostream>
#include <rclcpp/logging.hpp>
#include <stdexcept>

namespace franka_hardware {

Robot::Robot(const std::string& robot_ip, const rclcpp::Logger& logger) {
  franka::RealtimeConfig rt_config = franka::RealtimeConfig::kEnforce;

  // measurement code
  robot_ip_ = robot_ip;
  tau_msmt_.reserve(max_count_);
  // measurement code

  if (!franka::hasRealtimeKernel()) {
    rt_config = franka::RealtimeConfig::kIgnore;
    RCLCPP_WARN(
        logger,
        "You are not using a real-time kernel. Using a real-time kernel is strongly recommended!");
  }

  try {
    robot_ = std::make_unique<franka::Robot>(robot_ip, rt_config);
  } catch (const franka::Exception& exception) {
    RCLCPP_ERROR(logger, "Could not connect to the robot: %s", exception.what());
    throw;
  }

  try {
    setDefaultParams();
  } catch (const franka::ControlException& exception) {
    RCLCPP_ERROR(logger,
                 "Robot is in control error state! Please trigger automatic recovery first.");
    RCLCPP_ERROR(logger, "Error: %s", exception.what());
    detail::recordBackendFault(control_worker_, has_error_,
                               BackendFailureReason::FrankaControlException);
  } catch (const franka::CommandException& exception) {
    RCLCPP_ERROR(logger,
                 "Robot is in command error state! Please trigger automatic recovery first.");
    RCLCPP_ERROR(logger, "Error: %s", exception.what());
    detail::recordBackendFault(control_worker_, has_error_,
                               BackendFailureReason::FrankaCommandException);
  }

  current_state_ = robot_->readOnce();
  worker_command_ = makeSafeRobotCommand(current_state_);
  model_ = std::make_unique<franka::Model>(robot_->loadModel());
  franka_hardware_model_ = std::make_unique<ModelFranka>(model_.get());
}

Robot::~Robot() {
  try {
    stopRobot();
  } catch (...) {
    setError(true);
  }
}

bool Robot::write(const std::array<double, 7>& efforts,
                  const std::array<double, 7>& joint_positions,
                  const std::array<double, 7>& joint_velocities,
                  const std::array<double, 16>& cartesian_positions,
                  const std::array<double, 6>& cartesian_velocities) noexcept {
  RobotCommand command;
  command.efforts = efforts;
  command.joint_positions = joint_positions;
  command.joint_velocities = joint_velocities;
  command.cartesian_positions = cartesian_positions;
  command.cartesian_velocities = cartesian_velocities;
  if (!command_buffer_.tryPush(command)) {
    rejected_command_samples_.fetch_add(1);
    command_queue_saturated_.store(true, std::memory_order_release);
    return false;
  }
  command_queue_saturated_.store(false, std::memory_order_release);
  return true;
}

franka::RobotState Robot::read() {
  state_buffer_.popLatest(current_state_);
  return current_state_;
}

franka_hardware::ModelFranka* Robot::getModel() {
  return franka_hardware_model_.get();
}

bool Robot::startLoop(ControlMode initial_mode) {
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  if (detail::hasBackendFault(control_worker_, has_error_)) {
    return false;
  }

  bool started = false;
  try {
    started = detail::startOrRequestWorkerUnlessFaulted(
        control_worker_, has_error_,
        [this](ControlMode control_mode) {
          try {
            runLoop(control_mode);
          } catch (...) {
            setError(true);
            throw;
          }
        },
        initial_mode);
  } catch (...) {
    setError(true);
    return false;
  }
  if (!started) {
    detail::recordBackendFault(control_worker_, has_error_,
                               BackendFailureReason::WorkerStateTransitionFailure);
  }
  if (started) {
    lifecycle_active_.store(true);
  }
  return started;
}

bool Robot::initializeTorqueControl() {
  return startLoop(ControlMode::JointTorque);
}

bool Robot::initializeJointPositionControl() {
  return startLoop(ControlMode::JointPosition);
}

bool Robot::initializeJointVelocityControl() {
  return startLoop(ControlMode::JointVelocity);
}

bool Robot::initializeCartesianPositionControl() {
  return startLoop(ControlMode::CartesianPose);
}

bool Robot::initializeCartesianVelocityControl() {
  return startLoop(ControlMode::CartesianVelocity);
}

bool Robot::initializeContinuousReading() {
  return startLoop(ControlMode::None);
}

bool Robot::requestControlMode(ControlMode control_mode) noexcept {
  return detail::requestWorkerModeUnlessFaulted(control_worker_, has_error_, control_mode);
}

bool Robot::canRequestControlMode(ControlMode control_mode) const noexcept {
  return !detail::hasBackendFault(control_worker_, has_error_) &&
         control_worker_.canRequestMode(control_mode);
}

ControlMode Robot::getControlMode() const noexcept {
  return control_worker_.requestedMode();
}

bool Robot::stopRobot() {
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  lifecycle_active_.store(false);
  return detail::stopWorkerAndClearCommandBuffer(control_worker_, command_buffer_);
}

bool Robot::recoverToReading() {
  recovery_attempts_.fetch_add(1, std::memory_order_relaxed);
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  if (!hasError()) {
    finishRecoveryAttempt(false);
    return false;
  }

  try {
    const bool restart_reading = lifecycle_active_.load();
    if (!detail::stopWorkerAndClearCommandBuffer(control_worker_, command_buffer_)) {
      control_worker_.recordFailure(BackendFailureReason::WorkerStateTransitionFailure);
      finishRecoveryAttempt(false);
      return false;
    }
    franka::RobotState recovered_state;
    {
      std::lock_guard<std::mutex> parameter_lock(parameter_mutex_);
      robot_->automaticErrorRecovery();
      if (!init_params_set_.load()) {
        setDefaultParamsUnlocked();
      }
      recovered_state = robot_->readOnce();
    }

    if (control_worker_.state() == ControlLoopWorker::State::Faulted &&
        !control_worker_.clearFault()) {
      control_worker_.recordFailure(BackendFailureReason::WorkerStateTransitionFailure);
      finishRecoveryAttempt(false);
      return false;
    }
    if (control_worker_.state() != ControlLoopWorker::State::Stopped) {
      control_worker_.recordFailure(BackendFailureReason::WorkerStateTransitionFailure);
      finishRecoveryAttempt(false);
      return false;
    }

    (void)publishState(recovered_state);
    has_error_.store(false);
    if (!restart_reading) {
      detail::finalizeSuccessfulRecoveryFailureReason(control_worker_, false);
      finishRecoveryAttempt(true);
      return true;
    }
    if (control_worker_.start(
            [this](ControlMode control_mode) {
              try {
                runLoop(control_mode);
              } catch (...) {
                setError(true);
                throw;
              }
            },
            ControlMode::None)) {
      detail::finalizeSuccessfulRecoveryFailureReason(control_worker_, true);
      finishRecoveryAttempt(true);
      return true;
    }
    has_error_.store(true);
    control_worker_.recordFailure(BackendFailureReason::WorkerStartupFailure);
    finishRecoveryAttempt(false);
    return false;
  } catch (...) {
    detail::recordCurrentBackendFault(control_worker_, has_error_);
    finishRecoveryAttempt(false);
    throw;
  }
}

bool Robot::hasError() const noexcept {
  return detail::hasBackendFault(control_worker_, has_error_);
}

bool Robot::publishState(const franka::RobotState& state) noexcept {
  if (!state_buffer_.tryPush(state)) {
    dropped_state_samples_.fetch_add(1);
    state_queue_saturated_.store(true, std::memory_order_release);
    return false;
  }
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
  last_accepted_state_steady_ns_.store(static_cast<uint64_t>(now_ns), std::memory_order_relaxed);
  accepted_state_samples_.fetch_add(1, std::memory_order_release);
  has_state_sample_.store(true, std::memory_order_release);
  state_queue_saturated_.store(false, std::memory_order_release);
  return true;
}

void Robot::finishRecoveryAttempt(bool succeeded) noexcept {
  if (succeeded) {
    recovery_successes_.fetch_add(1, std::memory_order_relaxed);
    last_recovery_result_.store(BackendRecoveryResult::Succeeded, std::memory_order_release);
  } else {
    recovery_failures_.fetch_add(1, std::memory_order_relaxed);
    last_recovery_result_.store(BackendRecoveryResult::Failed, std::memory_order_release);
  }
}

void Robot::updateCommandSnapshot() noexcept {
  command_buffer_.popLatest(worker_command_);
}

void Robot::runLoop(ControlMode control_mode) {
  switch (control_mode) {
    case ControlMode::JointTorque:
      robot_->control(
          [this, control_mode](const franka::RobotState& state,
                               const franka::Duration& /*period*/) {
            publishState(state);
            updateCommandSnapshot();
            franka::Torques out(worker_command_.efforts);
            out.motion_finished = control_worker_.shouldExitMode(control_mode);
            return out;
          },
          true, franka::kMaxCutoffFrequency);
      return;
    case ControlMode::JointPosition:
      robot_->control([this, control_mode](const franka::RobotState& state,
                                           const franka::Duration& /*period*/) {
        publishState(state);
        updateCommandSnapshot();
        franka::JointPositions out(worker_command_.joint_positions);
        out.motion_finished = control_worker_.shouldExitMode(control_mode);
        return out;
      });
      return;
    case ControlMode::JointVelocity:
      robot_->control([this, control_mode](const franka::RobotState& state,
                                           const franka::Duration& /*period*/) {
        publishState(state);
        updateCommandSnapshot();
        franka::JointVelocities out(worker_command_.joint_velocities);
        out.motion_finished = control_worker_.shouldExitMode(control_mode);
        return out;
      });
      return;
    case ControlMode::CartesianPose:
      robot_->control([this, control_mode](const franka::RobotState& state,
                                           const franka::Duration& /*period*/) {
        publishState(state);
        updateCommandSnapshot();
        franka::CartesianPose out(worker_command_.cartesian_positions);
        out.motion_finished = control_worker_.shouldExitMode(control_mode);
        return out;
      });
      return;
    case ControlMode::CartesianVelocity:
      robot_->control([this, control_mode](const franka::RobotState& state,
                                           const franka::Duration& /*period*/) {
        publishState(state);
        updateCommandSnapshot();
        franka::CartesianVelocities out(worker_command_.cartesian_velocities);
        out.motion_finished = control_worker_.shouldExitMode(control_mode);
        return out;
      });
      return;
    case ControlMode::None:
      robot_->read([this, control_mode](const franka::RobotState& state) {
        publishState(state);
        updateCommandSnapshot();
        if (state.robot_mode == franka::RobotMode::kReflex) {
          control_worker_.recordFailure(BackendFailureReason::RobotReflex);
          setError(true);
          return false;
        }
        return !control_worker_.shouldExitMode(control_mode);
      });
      return;
  }
  throw std::invalid_argument("Unsupported control mode");
}

bool Robot::isStopped() const noexcept {
  const auto state = control_worker_.state();
  return state == ControlLoopWorker::State::Stopped || state == ControlLoopWorker::State::Faulted;
}

// ##############################//
//  Internal param setters       //
// ##############################//

void Robot::setJointStiffness(const franka_msgs::srv::SetJointStiffness::Request::SharedPtr& req) {
  std::lock_guard<std::mutex> lock(parameter_mutex_);
  std::array<double, 7> joint_stiffness{};
  std::copy(req->joint_stiffness.cbegin(), req->joint_stiffness.cend(), joint_stiffness.begin());
  detail::runBackendOperationWithFailureRecording(
      control_worker_, has_error_,
      [this, &joint_stiffness]() { robot_->setJointImpedance(joint_stiffness); });
}

void Robot::setCartesianStiffness(
    const franka_msgs::srv::SetCartesianStiffness::Request::SharedPtr& req) {
  std::lock_guard<std::mutex> lock(parameter_mutex_);
  std::array<double, 6> cartesian_stiffness{};
  std::copy(req->cartesian_stiffness.cbegin(), req->cartesian_stiffness.cend(),
            cartesian_stiffness.begin());
  detail::runBackendOperationWithFailureRecording(
      control_worker_, has_error_,
      [this, &cartesian_stiffness]() { robot_->setCartesianImpedance(cartesian_stiffness); });
}

void Robot::setLoad(const franka_msgs::srv::SetLoad::Request::SharedPtr& req) {
  std::lock_guard<std::mutex> lock(parameter_mutex_);
  double mass(req->mass);
  std::array<double, 3> center_of_mass{};  // NOLINT [readability-identifier-naming]
  std::copy(req->center_of_mass.cbegin(), req->center_of_mass.cend(), center_of_mass.begin());
  std::array<double, 9> load_inertia{};
  std::copy(req->load_inertia.cbegin(), req->load_inertia.cend(), load_inertia.begin());

  detail::runBackendOperationWithFailureRecording(
      control_worker_, has_error_, [this, mass, &center_of_mass, &load_inertia]() {
        robot_->setLoad(mass, center_of_mass, load_inertia);
      });
}

void Robot::setTCPFrame(const franka_msgs::srv::SetTCPFrame::Request::SharedPtr& req) {
  std::lock_guard<std::mutex> lock(parameter_mutex_);

  std::array<double, 16> transformation{};  // NOLINT [readability-identifier-naming]
  std::copy(req->transformation.cbegin(), req->transformation.cend(), transformation.begin());
  detail::runBackendOperationWithFailureRecording(
      control_worker_, has_error_, [this, &transformation]() { robot_->setEE(transformation); });
}

void Robot::setStiffnessFrame(const franka_msgs::srv::SetStiffnessFrame::Request::SharedPtr& req) {
  std::lock_guard<std::mutex> lock(parameter_mutex_);

  std::array<double, 16> transformation{};
  std::copy(req->transformation.cbegin(), req->transformation.cend(), transformation.begin());
  detail::runBackendOperationWithFailureRecording(
      control_worker_, has_error_, [this, &transformation]() { robot_->setK(transformation); });
}

void Robot::setForceTorqueCollisionBehavior(
    const franka_msgs::srv::SetForceTorqueCollisionBehavior::Request::SharedPtr& req) {
  std::lock_guard<std::mutex> lock(parameter_mutex_);

  std::array<double, 7> lower_torque_thresholds_nominal{};
  std::copy(req->lower_torque_thresholds_nominal.cbegin(),
            req->lower_torque_thresholds_nominal.cend(), lower_torque_thresholds_nominal.begin());
  std::array<double, 7> upper_torque_thresholds_nominal{};
  std::copy(req->upper_torque_thresholds_nominal.cbegin(),
            req->upper_torque_thresholds_nominal.cend(), upper_torque_thresholds_nominal.begin());
  std::array<double, 6> lower_force_thresholds_nominal{};
  std::copy(req->lower_force_thresholds_nominal.cbegin(),
            req->lower_force_thresholds_nominal.cend(), lower_force_thresholds_nominal.begin());
  std::array<double, 6> upper_force_thresholds_nominal{};
  std::copy(req->upper_force_thresholds_nominal.cbegin(),
            req->upper_force_thresholds_nominal.cend(), upper_force_thresholds_nominal.begin());

  detail::runBackendOperationWithFailureRecording(
      control_worker_, has_error_,
      [this, &lower_torque_thresholds_nominal, &upper_torque_thresholds_nominal,
       &lower_force_thresholds_nominal, &upper_force_thresholds_nominal]() {
        robot_->setCollisionBehavior(
            lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
            lower_force_thresholds_nominal, upper_force_thresholds_nominal);
      });
}

void Robot::setFullCollisionBehavior(
    const franka_msgs::srv::SetFullCollisionBehavior::Request::SharedPtr& req) {
  std::lock_guard<std::mutex> lock(parameter_mutex_);

  std::array<double, 7> lower_torque_thresholds_acceleration{};
  std::copy(req->lower_torque_thresholds_acceleration.cbegin(),
            req->lower_torque_thresholds_acceleration.cend(),
            lower_torque_thresholds_acceleration.begin());
  std::array<double, 7> upper_torque_thresholds_acceleration{};
  std::copy(req->upper_torque_thresholds_acceleration.cbegin(),
            req->upper_torque_thresholds_acceleration.cend(),
            upper_torque_thresholds_acceleration.begin());
  std::array<double, 7> lower_torque_thresholds_nominal{};
  std::copy(req->lower_torque_thresholds_nominal.cbegin(),
            req->lower_torque_thresholds_nominal.cend(), lower_torque_thresholds_nominal.begin());
  std::array<double, 7> upper_torque_thresholds_nominal{};
  std::copy(req->upper_torque_thresholds_nominal.cbegin(),
            req->upper_torque_thresholds_nominal.cend(), upper_torque_thresholds_nominal.begin());
  std::array<double, 6> lower_force_thresholds_acceleration{};
  std::copy(req->lower_force_thresholds_acceleration.cbegin(),
            req->lower_force_thresholds_acceleration.cend(),
            lower_force_thresholds_acceleration.begin());
  std::array<double, 6> upper_force_thresholds_acceleration{};
  std::copy(req->upper_force_thresholds_acceleration.cbegin(),
            req->upper_force_thresholds_acceleration.cend(),
            upper_force_thresholds_acceleration.begin());
  std::array<double, 6> lower_force_thresholds_nominal{};
  std::copy(req->lower_force_thresholds_nominal.cbegin(),
            req->lower_force_thresholds_nominal.cend(), lower_force_thresholds_nominal.begin());
  std::array<double, 6> upper_force_thresholds_nominal{};
  std::copy(req->upper_force_thresholds_nominal.cbegin(),
            req->upper_force_thresholds_nominal.cend(), upper_force_thresholds_nominal.begin());
  detail::runBackendOperationWithFailureRecording(
      control_worker_, has_error_,
      [this, &lower_torque_thresholds_acceleration, &upper_torque_thresholds_acceleration,
       &lower_torque_thresholds_nominal, &upper_torque_thresholds_nominal,
       &lower_force_thresholds_acceleration, &upper_force_thresholds_acceleration,
       &lower_force_thresholds_nominal, &upper_force_thresholds_nominal]() {
        robot_->setCollisionBehavior(
            lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration,
            lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
            lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
            lower_force_thresholds_nominal, upper_force_thresholds_nominal);
      });
}

void Robot::setDefaultParams() {
  std::lock_guard<std::mutex> lock(parameter_mutex_);
  setDefaultParamsUnlocked();
}

void Robot::setDefaultParamsUnlocked() {
  robot_->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  robot_->setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
  robot_->setCollisionBehavior(
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
  init_params_set_.store(true);
}

// ##############################//
//  Internal param setters       //
// ##############################//

}  // namespace franka_hardware
