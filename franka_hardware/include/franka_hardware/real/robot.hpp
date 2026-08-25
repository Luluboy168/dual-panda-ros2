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

#pragma once

#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <sys/time.h>  // measurement include
#include <time.h>      // measurement include

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <franka_hardware/real/model_franka.hpp>
#include <franka_msgs/srv/set_cartesian_stiffness.hpp>
#include <franka_msgs/srv/set_force_torque_collision_behavior.hpp>
#include <franka_msgs/srv/set_full_collision_behavior.hpp>
#include <franka_msgs/srv/set_joint_stiffness.hpp>
#include <franka_msgs/srv/set_load.hpp>
#include <franka_msgs/srv/set_stiffness_frame.hpp>
#include <franka_msgs/srv/set_tcp_frame.hpp>
#include <fstream>  // measurement include
#include <iostream>
#include <memory>
#include <mutex>
#include <rclcpp/logger.hpp>
#include <string>

#include "franka_hardware/common/control_mode.h"
#include "franka_hardware/real/control_loop_worker.hpp"
#include "franka_hardware/real/robot_command.hpp"
#include "franka_hardware/real/spsc_ring_buffer.hpp"

namespace franka_hardware {

// data measurement
struct tau_measurement {
  tau_measurement(const std::array<double, 7> tau, double wall_time)
      : tau_(tau), wall_time_(wall_time){};
  std::array<double, 7> tau_;
  double wall_time_;
};
// data measurement

namespace detail {

template <std::size_t Capacity>
bool stopWorkerAndClearCommandBuffer(ControlLoopWorker& worker,
                                     SpscRingBuffer<RobotCommand, Capacity>& command_buffer) {
  if (worker.state() == ControlLoopWorker::State::Stopped) {
    command_buffer.clear();
    return true;
  }
  if (worker.state() == ControlLoopWorker::State::Faulted) {
    // Faulted is terminal for the consumer loop. shutdown() joins it when it is still joinable and
    // returns false when a prior stop already joined it; either way the consumer is quiescent and
    // the Faulted state is intentionally preserved.
    (void)worker.shutdown();
    command_buffer.clear();
    return true;
  }
  if (!worker.shutdown()) {
    return false;
  }
  // shutdown() has joined the sole consumer. The lifecycle caller is the sole producer, so both
  // sides are quiescent before clear(). A faulted worker deliberately remains faulted.
  command_buffer.clear();
  return true;
}

inline void finalizeSuccessfulRecoveryFailureReason(ControlLoopWorker& worker,
                                                    bool restarted_worker) noexcept {
  // A restarted worker clears the prior reason at its quiescent prelaunch point. It may then record
  // a new typed failure before the recovery caller returns, which must not be erased here.
  // A non-restarting success has joined the worker and is called while the real backend Recovery
  // gate excludes lifecycle, parameter, and mode writers, so this is a quiescent clear.
  if (!restarted_worker) {
    worker.clearFailureReason();
  }
}

}  // namespace detail

class Robot {
 public:
  /**
   * Connects to the robot. This method can block for up to one minute if the robot is not
   * responding. An exception will be thrown if the connection cannot be established.
   *
   * @param[in] robot_ip IP address or hostname of the robot.
   * @param[im] logger ROS Logger to print eventual warnings.
   */
  explicit Robot(const std::string& robot_ip, const rclcpp::Logger& logger);
  Robot(const Robot&) = delete;
  Robot& operator=(const Robot& other) = delete;
  Robot& operator=(Robot&& other) = delete;
  Robot(Robot&& other) = delete;

  /// Stops the currently running loop and closes the connection with the robot.
  virtual ~Robot();

  bool requestControlMode(ControlMode control_mode) noexcept;
  bool canRequestControlMode(ControlMode control_mode) const noexcept;
  ControlMode getControlMode() const noexcept;
  ControlMode getActiveControlMode() const noexcept { return control_worker_.runningMode(); }
  ControlLoopWorker::State getWorkerState() const noexcept { return control_worker_.state(); }
  /**
   * Starts a torque control loop. Before using this method make sure that no other
   * control or reading loop is currently active.
   */
  bool initializeTorqueControl();

  bool initializeJointPositionControl();
  bool initializeJointVelocityControl();

  bool initializeCartesianPositionControl();
  bool initializeCartesianVelocityControl();

  /**
   * Starts a reading loop of the robot state. Before using this method make sure that no other
   * control or reading loop is currently active.
   */
  bool initializeContinuousReading();

  /// stops the control or reading loop of the robot.
  bool stopRobot();

  /// Performs automatic recovery and starts state-only reading. Never resumes a command mode.
  bool recoverToReading();
  /**
   * Return pointer to the franka robot model object .
   * @return pointer to the current robot model.
   */
  virtual franka_hardware::ModelFranka* getModel();
  /**
   * Get the current robot state in a thread-safe way.
   * @return current robot state.
   */
  franka::RobotState read();

  bool hasError() const noexcept;

  void setError(bool new_state) noexcept { has_error_.store(new_state); }

  /**
   * Sends new desired torque commands to the control loop in a thread-safe way.
   * The robot will use these torques until a different set of torques are commanded.
   * @param[in] efforts torque command for each joint.
   */
  bool write(const std::array<double, 7>& efforts,
             const std::array<double, 7>& joint_positions,
             const std::array<double, 7>& joint_velocities,
             const std::array<double, 16>& cartesian_positions,
             const std::array<double, 6>& cartesian_velocities) noexcept;
  bool canWriteCommand() const noexcept { return command_buffer_.canPush(); }

  /// @return true if there is no control or reading loop running.
  bool isStopped() const noexcept;

  uint64_t droppedStateSamples() const noexcept { return dropped_state_samples_.load(); }
  uint64_t rejectedCommandSamples() const noexcept { return rejected_command_samples_.load(); }
  BackendFailureReason failureReason() const noexcept { return control_worker_.failureReason(); }
  bool hasStateSample() const noexcept { return has_state_sample_.load(std::memory_order_acquire); }
  uint64_t acceptedStateSamples() const noexcept {
    return accepted_state_samples_.load(std::memory_order_acquire);
  }
  uint64_t lastAcceptedStateSteadyNanoseconds() const noexcept {
    return last_accepted_state_steady_ns_.load(std::memory_order_acquire);
  }
  bool stateQueueSaturated() const noexcept {
    return state_queue_saturated_.load(std::memory_order_acquire);
  }
  bool commandQueueSaturated() const noexcept {
    return command_queue_saturated_.load(std::memory_order_acquire);
  }
  uint64_t recoveryAttempts() const noexcept { return recovery_attempts_.load(); }
  uint64_t recoverySuccesses() const noexcept { return recovery_successes_.load(); }
  uint64_t recoveryFailures() const noexcept { return recovery_failures_.load(); }
  BackendRecoveryResult lastRecoveryResult() const noexcept {
    return last_recovery_result_.load(std::memory_order_acquire);
  }

  // ##############################//
  //  Internal param setters       //
  // ##############################//

  /**
   * Sets the impedance for each joint in the internal controller.
   *
   * User-provided torques are not affected by this setting.
   *
   * @param[in] franka_msgs::srv::SetJointStiffness::Request::SharedPtr requests with JointStiffness
   * values
   *
   * @throw CommandException if the Control reports an error.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  virtual void setJointStiffness(
      const franka_msgs::srv::SetJointStiffness::Request::SharedPtr& req);

  /**
   * Sets the Cartesian stiffness (for x, y, z, roll, pitch, yaw) in the internal
   * controller.
   *
   * The values set using Robot::SetCartesianStiffness are used in the direction of the
   * stiffness frame, which can be set with Robot::setK.
   *
   * Inputs received by the torque controller are not affected by this setting.
   *
   * @param[in] franka_msgs::srv::SetCartesianStiffness::Request::SharedPtr request
   * @throw CommandException if the Control reports an error.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  virtual void setCartesianStiffness(
      const franka_msgs::srv::SetCartesianStiffness::Request::SharedPtr& req);

  /**
   * Sets dynamic parameters of a payload.
   *
   * @note
   * This is not for setting end effector parameters, which have to be set in the administrator's
   * interface.
   *
   * @param[in] franka_msgs::srv::SetLoad::Request::SharedPtr request
   *
   * @throw CommandException if the Control reports an error.
   * @throw
   */
  virtual void setLoad(const franka_msgs::srv::SetLoad::Request::SharedPtr& req);

  /**
   * Sets the transformation \f$^{NE}T_{EE}\f$ from nominal end effector to end effector frame.
   *
   * The transformation matrix is represented as a vectorized 4x4 matrix in column-major format.
   *
   * @param[in] franka_msgs::srv::SetTCPFrame::Request::SharedPtr req
   *
   * @throw CommandException if the Control reports an error.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  virtual void setTCPFrame(const franka_msgs::srv::SetTCPFrame::Request::SharedPtr& req);

  /**
   * Sets the transformation \f$^{EE}T_K\f$ from end effector frame to stiffness frame.
   *
   * The transformation matrix is represented as a vectorized 4x4 matrix in column-major format.
   *
   * @param[in] franka_msgs::srv::SetStiffnessFrame::Request::SharedPtr req.
   *
   * @throw CommandException if the Control reports an error.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   *
   */
  virtual void setStiffnessFrame(
      const franka_msgs::srv::SetStiffnessFrame::Request::SharedPtr& req);

  /**
   * Changes the collision behavior.
   *
   * Set common torque and force boundaries for acceleration/deceleration and constant velocity
   * movement phases.
   *
   * Forces or torques between lower and upper threshold are shown as contacts in the RobotState.
   * Forces or torques above the upper threshold are registered as collision and cause the robot to
   * stop moving.
   *
   * @param[in] franka_msgs::srv::SetForceTorqueCollisionBehavior::Request::SharedPtr req
   *
   * @throw CommandException if the Control reports an error.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  virtual void setForceTorqueCollisionBehavior(
      const franka_msgs::srv::SetForceTorqueCollisionBehavior::Request::SharedPtr& req);

  /**
   * Changes the collision behavior.
   *
   * Set separate torque and force boundaries for acceleration/deceleration and constant velocity
   * movement phases.
   *
   * Forces or torques between lower and upper threshold are shown as contacts in the RobotState.
   * Forces or torques above the upper threshold are registered as collision and cause the robot to
   * stop moving.
   *
   * @param[in] franka_msgs::srv::SetFullCollisionBehavior::Request::SharedPtr request msg
   *
   * @throw CommandException if the Control reports an error.
   * @throw NetworkException if the connection is lost, e.g. after a timeout.
   */
  virtual void setFullCollisionBehavior(
      const franka_msgs::srv::SetFullCollisionBehavior::Request::SharedPtr& req);

  void setDefaultParams();
  bool getInitParamsSet() const noexcept { return init_params_set_.load(); }

  // ##############################//
  //  Internal param setters end   //
  // ##############################//
  //  Measurement functions //
  double get_wall_time() {
    struct timeval time;
    if (gettimeofday(&time, NULL)) {
      //  Handle error
      return 0;
    }
    return (double)time.tv_sec + (double)time.tv_usec * .000001;
  }
  void measureTau(const std::array<double, 7>& tau, double end_time) {
    tau_msmt_[cycle_count_] = tau_measurement(tau, end_time);
  };
  void increaseCounter() { cycle_count_++; }
  void write_tau_to_file(std::string file_name) {
    std::ofstream logfile;
    logfile.open(file_name + "_tau.txt");
    // populate header

    logfile << std::fixed << "time," << "arm," << "tau1," << "tau2," << "tau3," << "tau4,"
            << "tau5," << "tau6," << "tau7\n";
    for (int i = 0; i < cycle_count_; i++) {
      logfile << tau_msmt_[i].wall_time_ << "," << 0 << "," << tau_msmt_[i].tau_[0] << ","
              << tau_msmt_[i].tau_[1] << "," << tau_msmt_[i].tau_[2] << "," << tau_msmt_[i].tau_[3]
              << "," << tau_msmt_[i].tau_[4] << "," << tau_msmt_[i].tau_[5] << ","
              << tau_msmt_[i].tau_[6] << "\n";
    }
    logfile.close();
  }
  std::vector<tau_measurement> tau_msmt_;
  int cycle_count_;
  const int max_count_ = 30000;
  bool logged_ = false;
  std::string robot_ip_;
  // Measurement functions //
 private:
  static constexpr size_t kRealtimeBufferCapacity = 64;

  bool startLoop(ControlMode initial_mode);
  void runLoop(ControlMode control_mode);
  bool publishState(const franka::RobotState& state) noexcept;
  void updateCommandSnapshot() noexcept;
  void setDefaultParamsUnlocked();
  void finishRecoveryAttempt(bool succeeded) noexcept;

  std::unique_ptr<franka::Robot> robot_;
  std::unique_ptr<franka::Model> model_;
  std::unique_ptr<ModelFranka> franka_hardware_model_;
  ControlLoopWorker control_worker_;
  // SPSC ownership: the libfranka worker produces state and consumes commands; the
  // controller-manager thread consumes state and produces commands. Recovery may publish one
  // state only after the worker has been joined.
  SpscRingBuffer<franka::RobotState, kRealtimeBufferCapacity> state_buffer_;
  SpscRingBuffer<RobotCommand, kRealtimeBufferCapacity> command_buffer_;
  // Lock order is lifecycle_mutex_ then parameter_mutex_. The real-time worker takes neither.
  // Parameter-only service calls never acquire lifecycle_mutex_.
  std::mutex lifecycle_mutex_;
  std::mutex parameter_mutex_;
  std::atomic_bool lifecycle_active_{false};
  std::atomic_bool has_error_{false};
  std::atomic_bool init_params_set_{false};
  std::atomic_bool has_state_sample_{false};
  std::atomic_bool state_queue_saturated_{false};
  std::atomic_bool command_queue_saturated_{false};
  std::atomic_uint64_t accepted_state_samples_{0};
  std::atomic_uint64_t last_accepted_state_steady_ns_{0};
  std::atomic_uint64_t dropped_state_samples_{0};
  std::atomic_uint64_t rejected_command_samples_{0};
  std::atomic_uint64_t recovery_attempts_{0};
  std::atomic_uint64_t recovery_successes_{0};
  std::atomic_uint64_t recovery_failures_{0};
  std::atomic<BackendRecoveryResult> last_recovery_result_{BackendRecoveryResult::NeverAttempted};
  franka::RobotState current_state_;
  RobotCommand worker_command_;
};
}  // namespace franka_hardware
