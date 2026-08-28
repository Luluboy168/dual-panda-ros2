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

#include <dirent.h>
#include <gtest/gtest.h>
#include <sys/wait.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <control_msgs/msg/joint_jog.hpp>
#include <controller_interface/controller_interface.hpp>
#include <controller_interface/controller_interface_params.hpp>
#include <controller_interface/test_utils.hpp>
#include <controller_manager/controller_manager.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/list_hardware_interfaces.hpp>
#include <controller_manager_msgs/srv/set_hardware_component_state.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <future>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/types/hardware_component_params.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/resource_manager_params.hpp>
#include <iterator>
#include <lifecycle_msgs/msg/state.hpp>
#include <map>
#include <memory>
#include <mutex>
#include <random>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <shared_mutex>
#include <std_srvs/srv/set_bool.hpp>
#include <stdexcept>
#include <string>
#include <thread>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <utility>
#include <vector>

#include "franka_hardware/common/model_base.hpp"
#include "franka_hardware/real/franka_arm_backend.hpp"
#include "franka_hardware/real/franka_multi_hardware_interface.hpp"
#include "franka_hardware/real/robot_command.hpp"
#include "franka_msgs/msg/franka_model.hpp"
#include "franka_msgs/msg/franka_state.hpp"
#include "franka_robot_state_broadcaster/franka_robot_model_broadcaster.hpp"
#include "franka_robot_state_broadcaster/franka_robot_state_broadcaster.hpp"
#include "support/offline_controller_manager_harness.hpp"
#include "support/offline_release_stress.hpp"
#include "support/synthetic_model.hpp"

namespace franka_example_controllers {
namespace {

using namespace std::chrono_literals;
using controller_interface::return_type;
using franka_hardware::BackendFactory;
using franka_hardware::BackendFaultCategory;
using franka_hardware::BackendServiceOperation;
using franka_hardware::BackendWorkerState;
using franka_hardware::ControlMode;
using franka_hardware::FrankaArmBackend;
using franka_hardware::FrankaArmBackendDiagnostics;
using franka_hardware::FrankaMultiHardwareInterface;
using franka_hardware::GlobalFaultCause;
using franka_hardware::InitializationCheckpointHook;
using franka_hardware::ModelBase;
using franka_hardware::ModeSwitchCheckpoint;
using franka_hardware::ModeSwitchCheckpointHook;
using franka_hardware::RobotCommand;

constexpr char kEmptyRobotDescription[] =
    R"(<?xml version="1.0"?>
<robot name="empty_production_cm_test">
  <link name="base_link"/>
  <ros2_control name="EmptyBootstrapSystem" type="system">
    <hardware>
      <plugin>mock_components/GenericSystem</plugin>
    </hardware>
  </ros2_control>
</robot>)";
constexpr size_t kArmCount = 2;
constexpr size_t kJointCount = 7;
constexpr size_t kDualJointInterfaceCount = kArmCount * kJointCount;
constexpr auto kCyclePeriod = std::chrono::milliseconds(1);

size_t processThreadCount() {
  DIR* directory = opendir("/proc/self/task");
  if (directory == nullptr) {
    throw std::runtime_error("failed to inspect process thread count");
  }
  size_t count = 0;
  while (const auto* entry = readdir(directory)) {
    const std::string name(entry->d_name);
    if (name != "." && name != "..") {
      ++count;
    }
  }
  (void)closedir(directory);
  return count;
}

int runStressSubprocess(const std::vector<std::string>& arguments) {
  std::vector<std::string> storage;
  storage.reserve(arguments.size() + 1U);
  storage.emplace_back(OFFLINE_STRESS_EXECUTABLE);
  storage.insert(storage.end(), arguments.begin(), arguments.end());
  std::vector<char*> argv;
  argv.reserve(storage.size() + 1U);
  for (auto& value : storage) {
    argv.push_back(value.data());
  }
  argv.push_back(nullptr);
  const pid_t child = fork();
  if (child < 0) {
    throw std::runtime_error("failed to fork offline stress subprocess");
  }
  if (child == 0) {
    execv(argv[0], argv.data());
    _exit(127);
  }
  int status = 0;
  while (waitpid(child, &status, 0) < 0) {
    if (errno != EINTR) {
      throw std::runtime_error("failed to wait for offline stress subprocess");
    }
  }
  if (!WIFEXITED(status)) {
    return 128;
  }
  return WEXITSTATUS(status);
}

std::string readWholeFile(const std::string& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("failed to read stress subprocess artifact");
  }
  return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

std::string sourceSegment(const std::string& path,
                          const std::string& begin_marker,
                          const std::string& end_marker) {
  const auto source = readWholeFile(path);
  const auto begin = source.find(begin_marker);
  if (begin == std::string::npos) {
    throw std::runtime_error("source gate begin marker missing");
  }
  const auto end = source.find(end_marker, begin + begin_marker.size());
  if (end == std::string::npos) {
    throw std::runtime_error("source gate end marker missing");
  }
  return source.substr(begin, end - begin);
}

YAML::Node parseAndCheckStressJson(const std::string& json,
                                   uint64_t expected_cycles,
                                   uint64_t expected_transactions) {
  const auto document = YAML::Load(json);
  EXPECT_TRUE(document.IsMap());
  EXPECT_EQ(document.size(), 14U);
  EXPECT_EQ(document["schema_version"].as<uint64_t>(), 2U);
  EXPECT_TRUE(document["success"].as<bool>());
  EXPECT_EQ(document["activation"].size(), 3U);
  EXPECT_EQ(document["transactions"].size(), 7U);
  EXPECT_EQ(document["transactions"]["requested"].as<uint64_t>(), expected_transactions);
  EXPECT_GT(document["mode_requests"]["accepted"].as<uint64_t>(), 0U);
  EXPECT_EQ(document["mode_requests"]["rejected"].as<uint64_t>(), 0U);
  EXPECT_GT(document["mode_requests"]["accepted_non_none"].as<uint64_t>(), 0U);
  EXPECT_EQ(document["mode_requests"]["rejected_non_none"].as<uint64_t>(), 0U);
  EXPECT_EQ(document["broadcasters"].size(), 3U);
  EXPECT_EQ(document["timed_run"].size(), 9U);
  EXPECT_EQ(document["timed_run"]["scheduled_cycles"].as<uint64_t>(), expected_cycles);
  EXPECT_EQ(document["timed_run"]["completed_cycles"].as<uint64_t>(), expected_cycles);
  EXPECT_TRUE(document["timed_run"]["functional_deadline_gate_applicable"].as<bool>());
  EXPECT_TRUE(document["timed_run"]["functional_deadline_gate_passed"].as<bool>());
  EXPECT_EQ(document["timed_run"]["functional_deadline_miss_limit"].as<uint64_t>(),
            (expected_cycles + 3U) / 4U);
  EXPECT_EQ(document["timing"].size(), 2U);
  EXPECT_TRUE(document["timing"]["update"]["mean_ns"].IsScalar());
  EXPECT_EQ(document["resources"].size(), 11U);
  EXPECT_EQ(document["resources"]["sanitizer_instrumented"].as<bool>(),
            test_support::kStressSanitizerInstrumented);
  EXPECT_EQ(document["resources"]["rss_threshold_applicable"].as<bool>(),
            !test_support::kStressSanitizerInstrumented);
  EXPECT_EQ(document["resources"]["rss_threshold_passed"].as<bool>(),
            !test_support::kStressSanitizerInstrumented);
  EXPECT_EQ(document["resources"]["post_cleanup_ros_discovery_tolerance"].as<uint64_t>(), 1U);
  EXPECT_EQ(document["resources"]["cold_retained_rss_limit_kib"].as<uint64_t>(), 64U * 1024U);
  EXPECT_TRUE(document["resources"]["cold_retained_rss_kib"].IsScalar());
  if constexpr (!test_support::kStressSanitizerInstrumented) {
    EXPECT_LE(document["resources"]["cold_retained_rss_kib"].as<int64_t>(), 64 * 1024);
  }
  EXPECT_FALSE(document["resources"]["periodic_high_water"]["sampling_failed"].as<bool>());
  EXPECT_EQ(document["resources"]["periodic_high_water"]["rss_sample_period_ms"].as<uint64_t>(),
            1000U);
  EXPECT_FALSE(
      document["resources"]["periodic_high_water"]["rss_steady_gate_applicable"].as<bool>());
  EXPECT_EQ(document["resources"]["periodic_high_water"]["rss_steady_gate_passed"].as<bool>(),
            !test_support::kStressSanitizerInstrumented);
  EXPECT_EQ(
      document["resources"]["periodic_high_water"]["rss_late_growth_limit_kib"].as<uint64_t>(),
      8U * 1024U);
  EXPECT_EQ(document["backends"].size(), 2U);
  uint64_t accepted_mode_requests = 0;
  uint64_t rejected_mode_requests = 0;
  uint64_t accepted_non_none_mode_requests = 0;
  uint64_t rejected_non_none_mode_requests = 0;
  uint64_t accepted_safe_snapshots = 0;
  uint64_t accepted_non_safe_snapshots = 0;
  uint64_t accepted_commands = 0;
  for (const auto& backend : document["backends"]) {
    EXPECT_GT(backend["accepted_states"].as<uint64_t>(), 0U);
    EXPECT_EQ(backend["dropped_states"].as<uint64_t>(), 0U);
    EXPECT_GT(backend["accepted_commands"].as<uint64_t>(), 0U);
    EXPECT_EQ(backend["rejected_commands"].as<uint64_t>(), 0U);
    EXPECT_GT(backend["accepted_mode_requests"].as<uint64_t>(), 0U);
    EXPECT_EQ(backend["rejected_mode_requests"].as<uint64_t>(), 0U);
    EXPECT_GT(backend["accepted_non_none_mode_requests"].as<uint64_t>(), 0U);
    EXPECT_EQ(backend["rejected_non_none_mode_requests"].as<uint64_t>(), 0U);
    EXPECT_GT(backend["accepted_safe_snapshots"].as<uint64_t>(), 0U);
    EXPECT_GT(backend["accepted_non_safe_snapshots"].as<uint64_t>(), 0U);
    EXPECT_EQ(backend["recovery_failures"].as<uint64_t>(), 0U);
    EXPECT_EQ(backend["command_queue_depth"].as<uint64_t>(), 0U);
    EXPECT_FALSE(backend["state_queue_saturated"].as<bool>());
    EXPECT_FALSE(backend["command_queue_saturated"].as<bool>());
    EXPECT_EQ(backend["requested_mode"].as<uint64_t>(), 0U);
    EXPECT_EQ(backend["active_mode"].as<uint64_t>(), 0U);
    EXPECT_FALSE(backend["faulted"].as<bool>());
    EXPECT_EQ(backend["accepted_safe_snapshots"].as<uint64_t>() +
                  backend["accepted_non_safe_snapshots"].as<uint64_t>(),
              backend["accepted_commands"].as<uint64_t>());
    accepted_mode_requests += backend["accepted_mode_requests"].as<uint64_t>();
    rejected_mode_requests += backend["rejected_mode_requests"].as<uint64_t>();
    accepted_non_none_mode_requests += backend["accepted_non_none_mode_requests"].as<uint64_t>();
    rejected_non_none_mode_requests += backend["rejected_non_none_mode_requests"].as<uint64_t>();
    accepted_safe_snapshots += backend["accepted_safe_snapshots"].as<uint64_t>();
    accepted_non_safe_snapshots += backend["accepted_non_safe_snapshots"].as<uint64_t>();
    accepted_commands += backend["accepted_commands"].as<uint64_t>();
  }
  EXPECT_EQ(accepted_mode_requests, document["mode_requests"]["accepted"].as<uint64_t>());
  EXPECT_EQ(rejected_mode_requests, document["mode_requests"]["rejected"].as<uint64_t>());
  EXPECT_EQ(accepted_non_none_mode_requests,
            document["mode_requests"]["accepted_non_none"].as<uint64_t>());
  EXPECT_EQ(rejected_non_none_mode_requests,
            document["mode_requests"]["rejected_non_none"].as<uint64_t>());
  EXPECT_EQ(accepted_safe_snapshots, document["command_snapshots"]["safe"].as<uint64_t>());
  EXPECT_EQ(accepted_non_safe_snapshots, document["command_snapshots"]["non_safe"].as<uint64_t>());
  EXPECT_GT(accepted_safe_snapshots, 0U);
  EXPECT_GT(accepted_non_safe_snapshots, 0U);
  EXPECT_EQ(accepted_safe_snapshots + accepted_non_safe_snapshots, accepted_commands);
  EXPECT_EQ(document["cleanup"]["controllers_unloaded"].as<uint64_t>(), 7U);
  EXPECT_EQ(document["cleanup"]["backends_destroyed"].as<uint64_t>(), 2U);
  return document;
}

std::array<double, 16> identityTransform(double x, double y, double z) {
  std::array<double, 16> transform{};
  transform[0] = 1.0;
  transform[5] = 1.0;
  transform[10] = 1.0;
  transform[15] = 1.0;
  transform[12] = x;
  transform[13] = y;
  transform[14] = z;
  return transform;
}

franka::RobotState makeState(uint8_t arm_slot) {
  franka::RobotState state{};
  const double marker = static_cast<double>(arm_slot);
  for (size_t joint = 0; joint < kJointCount; ++joint) {
    state.q[joint] = marker * 0.31 + static_cast<double>(joint) * 0.017;
    state.q_d[joint] = state.q[joint];
    state.theta[joint] = state.q[joint];
  }
  state.O_T_EE = identityTransform(0.40 + marker * 0.03, marker * 0.02, 0.50);
  state.O_T_EE_d = state.O_T_EE;
  state.O_T_EE_c = state.O_T_EE;
  state.F_T_EE = identityTransform(0.0, 0.0, 0.1);
  state.F_T_NE = state.F_T_EE;
  state.NE_T_EE = identityTransform(0.0, 0.0, 0.0);
  state.EE_T_K = identityTransform(0.0, 0.0, 0.0);
  state.control_command_success_rate = 1.0;
  state.robot_mode = franka::RobotMode::kIdle;
  state.time = franka::Duration(static_cast<uint64_t>(arm_slot) * 1000U);
  return state;
}

template <size_t Size>
bool allZero(const std::array<double, Size>& values) {
  return std::all_of(values.begin(), values.end(), [](double value) { return value == 0.0; });
}

bool isSafeSnapshot(const RobotCommand& command, const franka::RobotState& state) {
  return allZero(command.efforts) && allZero(command.joint_velocities) &&
         allZero(command.cartesian_velocities) && command.joint_positions == state.q &&
         command.cartesian_positions == state.O_T_EE;
}

enum class BackendEventKind : uint8_t { CommandAccepted, CommandRejected, ModeAttempt, Stop };

struct BackendEvent {
  BackendEventKind kind{BackendEventKind::CommandAccepted};
  RobotCommand command{};
  ControlMode mode{ControlMode::None};
  bool accepted{true};
};

class MinimalModel final : public ModelBase {
 public:
  [[nodiscard]] size_t coriolisCallCount() const noexcept { return coriolis_call_count_; }

 private:
  std::array<double, 16> poseImpl(franka::Frame /*frame*/,
                                  const std::array<double, 7>& /*q*/,
                                  const std::array<double, 16>& /*F_T_EE*/,
                                  const std::array<double, 16>& /*EE_T_K*/) const override {
    return identityTransform(0.0, 0.0, 0.0);
  }

  std::array<double, 42> bodyJacobianImpl(franka::Frame /*frame*/,
                                          const std::array<double, 7>& /*q*/,
                                          const std::array<double, 16>& /*F_T_EE*/,
                                          const std::array<double, 16>& /*EE_T_K*/) const override {
    return {};
  }

  std::array<double, 42> zeroJacobianImpl(franka::Frame /*frame*/,
                                          const std::array<double, 7>& /*q*/,
                                          const std::array<double, 16>& /*F_T_EE*/,
                                          const std::array<double, 16>& /*EE_T_K*/) const override {
    return {};
  }

  std::array<double, 49> massImpl(const std::array<double, 7>& /*q*/,
                                  const std::array<double, 9>& /*I_total*/,
                                  double /*m_total*/,
                                  const std::array<double, 3>& /*F_x_Ctotal*/) const override {
    std::array<double, 49> mass{};
    for (size_t joint = 0; joint < kJointCount; ++joint) {
      mass[joint * kJointCount + joint] = 1.0;
    }
    return mass;
  }

  std::array<double, 7> coriolisImpl(const std::array<double, 7>& /*q*/,
                                     const std::array<double, 7>& /*dq*/,
                                     const std::array<double, 9>& /*I_total*/,
                                     double /*m_total*/,
                                     const std::array<double, 3>& /*F_x_Ctotal*/) const override {
    ++coriolis_call_count_;
    return {};
  }

  std::array<double, 7> gravityImpl(const std::array<double, 7>& /*q*/,
                                    double /*m_total*/,
                                    const std::array<double, 3>& /*F_x_Ctotal*/,
                                    const std::array<double, 3>& /*gravity_earth*/) const override {
    return {};
  }

  mutable size_t coriolis_call_count_{0};
};

// This backend is deliberately local to this test translation unit. It owns no thread and performs
// no I/O. The only construction route used below is the production class's direct factory seam.
class MinimalBackend final : public FrankaArmBackend {
 public:
  explicit MinimalBackend(uint8_t arm_slot) : arm_slot_(arm_slot), state_(makeState(arm_slot)) {}

  bool startStateReading() override {
    ++start_count_;
    stopped_.store(false);
    worker_state_.store(BackendWorkerState::Running);
    requested_mode_.store(ControlMode::None);
    active_mode_.store(ControlMode::None);
    return true;
  }

  bool stop() override {
    ++stop_count_;
    stopped_.store(true);
    worker_state_.store(BackendWorkerState::Stopped);
    requested_mode_.store(ControlMode::None);
    active_mode_.store(ControlMode::None);
    BackendEvent event{};
    event.kind = BackendEventKind::Stop;
    event.mode = ControlMode::None;
    const std::lock_guard<std::mutex> lock(capture_mutex_);
    events_.push_back(event);
    return true;
  }

  franka::RobotState readLatestState() override {
    if (fail_next_read_.exchange(false)) {
      throw std::runtime_error("injected production-manager read failure");
    }
    ++read_count_;
    state_.time = franka::Duration(static_cast<uint64_t>(arm_slot_) * 1000U + read_count_);
    return state_;
  }

  ModelBase* model() noexcept override { return &model_; }

  bool canPublishCommand() const noexcept override { return true; }

  bool publishCommand(const RobotCommand& command) noexcept override {
    ++publish_attempt_count_;
    BackendEvent event{};
    event.command = command;
    const std::lock_guard<std::mutex> lock(capture_mutex_);
    if (reject_command_publish_.load()) {
      ++rejected_command_count_;
      event.kind = BackendEventKind::CommandRejected;
      event.accepted = false;
      events_.push_back(event);
      return false;
    }
    event.kind = BackendEventKind::CommandAccepted;
    events_.push_back(event);
    commands_.push_back(command);
    return true;
  }

  bool canRequestControlMode(ControlMode /*control_mode*/) const noexcept override {
    return !stopped_.load();
  }

  bool requestControlMode(ControlMode control_mode) noexcept override {
    const std::lock_guard<std::mutex> lock(capture_mutex_);
    mode_request_attempts_.push_back(control_mode);
    BackendEvent event{};
    event.kind = BackendEventKind::ModeAttempt;
    event.mode = control_mode;
    if (reject_non_none_mode_.load() && control_mode != ControlMode::None) {
      event.accepted = false;
      events_.push_back(event);
      return false;
    }
    events_.push_back(event);
    requested_mode_.store(control_mode);
    active_mode_.store(control_mode);
    return true;
  }

  ControlMode requestedControlMode() const noexcept override { return requested_mode_.load(); }
  ControlMode activeControlMode() const noexcept override { return active_mode_.load(); }
  bool hasFault() const noexcept override { return false; }
  bool recoverToReading() override { return false; }

  FrankaArmBackendDiagnostics diagnostics() const noexcept override {
    FrankaArmBackendDiagnostics diagnostics{};
    diagnostics.requested_mode = requested_mode_.load();
    diagnostics.active_mode = active_mode_.load();
    diagnostics.worker_state = worker_state_.load();
    diagnostics.fault_category = BackendFaultCategory::None;
    diagnostics.service_operation = BackendServiceOperation::Idle;
    diagnostics.stopped = stopped_.load();
    return diagnostics;
  }

  void setJointStiffness(
      const franka_msgs::srv::SetJointStiffness::Request::SharedPtr& /*request*/) override {}
  void setCartesianStiffness(
      const franka_msgs::srv::SetCartesianStiffness::Request::SharedPtr& /*request*/) override {}
  void setLoad(const franka_msgs::srv::SetLoad::Request::SharedPtr& /*request*/) override {}
  void setTCPFrame(const franka_msgs::srv::SetTCPFrame::Request::SharedPtr& /*request*/) override {}
  void setStiffnessFrame(
      const franka_msgs::srv::SetStiffnessFrame::Request::SharedPtr& /*request*/) override {}
  void setForceTorqueCollisionBehavior(
      const franka_msgs::srv::SetForceTorqueCollisionBehavior::Request::SharedPtr& /*request*/)
      override {}
  void setFullCollisionBehavior(
      const franka_msgs::srv::SetFullCollisionBehavior::Request::SharedPtr& /*request*/) override {}

  void rejectCommandPublish(bool reject) noexcept { reject_command_publish_.store(reject); }
  void rejectNonNoneMode(bool reject) noexcept { reject_non_none_mode_.store(reject); }
  void failNextRead() noexcept { fail_next_read_.store(true); }
  [[nodiscard]] const franka::RobotState& state() const noexcept { return state_; }
  [[nodiscard]] const MinimalModel& modelInstance() const noexcept { return model_; }
  [[nodiscard]] size_t startCount() const noexcept { return start_count_.load(); }
  [[nodiscard]] size_t stopCount() const noexcept { return stop_count_.load(); }
  [[nodiscard]] size_t readCount() const noexcept { return read_count_.load(); }
  [[nodiscard]] size_t publishAttemptCount() const noexcept {
    return publish_attempt_count_.load();
  }
  [[nodiscard]] size_t rejectedCommandCount() const noexcept {
    return rejected_command_count_.load();
  }
  [[nodiscard]] std::vector<RobotCommand> commands() const {
    const std::lock_guard<std::mutex> lock(capture_mutex_);
    return commands_;
  }
  [[nodiscard]] std::vector<ControlMode> modeRequestAttempts() const {
    const std::lock_guard<std::mutex> lock(capture_mutex_);
    return mode_request_attempts_;
  }
  [[nodiscard]] std::vector<BackendEvent> events() const {
    const std::lock_guard<std::mutex> lock(capture_mutex_);
    return events_;
  }

  [[nodiscard]] RobotCommand lastCommand() const {
    const std::lock_guard<std::mutex> lock(capture_mutex_);
    if (commands_.empty()) {
      throw std::logic_error("no captured backend command");
    }
    return commands_.back();
  }

 private:
  uint8_t arm_slot_;
  franka::RobotState state_{};
  MinimalModel model_{};
  std::atomic<ControlMode> requested_mode_{ControlMode::None};
  std::atomic<ControlMode> active_mode_{ControlMode::None};
  std::atomic<BackendWorkerState> worker_state_{BackendWorkerState::Stopped};
  std::atomic<bool> stopped_{true};
  std::atomic<bool> reject_command_publish_{false};
  std::atomic<bool> reject_non_none_mode_{false};
  std::atomic<bool> fail_next_read_{false};
  std::atomic<size_t> start_count_{0};
  std::atomic<size_t> stop_count_{0};
  std::atomic<size_t> read_count_{0};
  std::atomic<size_t> publish_attempt_count_{0};
  std::atomic<size_t> rejected_command_count_{0};
  mutable std::mutex capture_mutex_{};
  std::vector<RobotCommand> commands_{};
  std::vector<ControlMode> mode_request_attempts_{};
  std::vector<BackendEvent> events_{};
};

hardware_interface::InterfaceInfo makeInterface(const std::string& name) {
  hardware_interface::InterfaceInfo interface{};
  interface.name = name;
  interface.data_type = "double";
  return interface;
}

hardware_interface::HardwareInfo makeHardwareInfo() {
  hardware_interface::HardwareInfo info{};
  info.name = "FrankaMultiHardwareInterface";
  info.type = "system";
  info.rw_rate = 1000;
  info.is_async = false;
  info.thread_priority = 0;
  info.hardware_plugin_name = "franka_hardware/FrankaMultiHardwareInterface";
  info.hardware_parameters["robot_count"] = "2";
  info.original_xml = kEmptyRobotDescription;

  for (size_t arm = 0; arm < kArmCount; ++arm) {
    const auto slot = std::to_string(arm + 1);
    const auto arm_id = "panda" + slot;
    info.hardware_parameters["ns_" + slot] = arm_id;
    info.hardware_parameters["robot_ip_" + slot] = "offline-test-only-" + slot;
    for (size_t joint = 0; joint < kJointCount; ++joint) {
      hardware_interface::ComponentInfo component{};
      component.name = arm_id + "_joint" + std::to_string(joint + 1);
      component.type = "joint";
      component.command_interfaces = {makeInterface("effort"), makeInterface("position"),
                                      makeInterface("velocity")};
      component.state_interfaces = {makeInterface("position"), makeInterface("velocity"),
                                    makeInterface("effort")};
      info.joints.push_back(std::move(component));
    }
  }
  return info;
}

class BackendStore {
 public:
  BackendFactory factory() {
    return [this](const std::string& arm_name, const std::string& robot_address,
                  const rclcpp::Logger& /*logger*/) -> std::shared_ptr<FrankaArmBackend> {
      const auto expected_slot = arm_name == "panda1" ? 1U : arm_name == "panda2" ? 2U : 0U;
      if (expected_slot == 0U ||
          robot_address != "offline-test-only-" + std::to_string(expected_slot)) {
        throw std::invalid_argument("unexpected offline backend metadata");
      }
      auto backend = std::make_shared<MinimalBackend>(static_cast<uint8_t>(expected_slot));
      backends_.emplace(arm_name, backend);
      return backend;
    };
  }

  std::shared_ptr<MinimalBackend> backend(const std::string& arm_name) const {
    return backends_.at(arm_name);
  }

 private:
  std::map<std::string, std::shared_ptr<MinimalBackend>> backends_{};
};

// Test-only one-arm claim controller. It is added directly to ControllerManager and is neither
// pluginlib-registered nor installed, so no production input can select it.
class OneArmVelocityClaimController final : public controller_interface::ControllerInterface {
 public:
  explicit OneArmVelocityClaimController(std::string arm_id,
                                         size_t claim_count = kJointCount,
                                         bool fail_activation_after_write = false)
      : arm_id_(std::move(arm_id)),
        claim_count_(claim_count),
        fail_activation_after_write_(fail_activation_after_write) {
    if (claim_count_ == 0 || claim_count_ > kJointCount) {
      throw std::invalid_argument("test-only claim count must be between one and seven");
    }
    for (size_t joint = 0; joint < kJointCount; ++joint) {
      interface_names_[joint] = arm_id_ + "_joint" + std::to_string(joint + 1) + "/velocity";
    }
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override {
    controller_interface::InterfaceConfiguration configuration{};
    configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    configuration.names.assign(interface_names_.begin(), interface_names_.begin() + claim_count_);
    return configuration;
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
    return {};
  }

  return_type update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override {
    if (!active_) {
      // F-10c amendment A (test-only counterpart): the deactivation zero is written here, on the
      // control-cycle owner thread, on the first cycle after on_deactivate() cleared active_ --
      // never from on_deactivate() itself, which controller_manager may run on its service
      // thread concurrently with FrankaMultiHardwareInterface::write() reading the very same
      // exported command storage. See dual_arm_joint_hold_controller.cpp's update() for the
      // production shape this mirrors.
      if (pending_deactivation_zero_.exchange(false)) {
        const std::array<double, kJointCount> zero{};
        (void)write(zero);
      }
      return return_type::ERROR;
    }
    // F-10b (test-only counterpart): the pre-activation zero write used to happen directly inside
    // on_activate(), which -- exactly like the production controllers' captureActivationState()
    // this test exists to exercise -- controller_manager may run on its own service thread
    // (Jazzy activate_asap=false) while this test's harness keeps driving read()/write() on a
    // different thread. Writing command_interfaces_ from that service thread raced
    // FrankaMultiHardwareInterface::write()'s publishCommands() reading the very same exported
    // command storage on the owner thread. update() is always called in lockstep with
    // read()/write() on the control-cycle owner thread, so deferring the pending zero write to
    // this, its first post-activation call, keeps the "zero before any real command" behavior
    // on_activate() used to provide while touching the shared command storage only from the one
    // thread that is ever allowed to.
    if (pending_zero_write_) {
      pending_zero_write_ = false;
      const std::array<double, kJointCount> zero{};
      if (!write(zero)) {
        return return_type::ERROR;
      }
    }
    return write(command_) ? return_type::OK : return_type::ERROR;
  }

  controller_interface::CallbackReturn on_init() override {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& /*previous_state*/) override {
    ++activation_count_;
    // Deliberately no interface write here -- see update()'s comment above. Activation success
    // depends only on this controller's own claim bookkeeping (command_interfaces_.size(), set by
    // controller_manager before on_activate() runs) plus the test's injected failure flag, never
    // on touching hardware-owned command storage off the control-cycle owner thread.
    const bool claimed_as_expected = command_interfaces_.size() == claim_count_;
    const bool activated = claimed_as_expected && !fail_activation_after_write_;
    active_ = activated;
    pending_zero_write_ = activated;
    return active_ ? controller_interface::CallbackReturn::SUCCESS
                   : controller_interface::CallbackReturn::FAILURE;
  }

  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& /*previous_state*/) override {
    ++deactivation_count_;
    active_ = false;
    // F-10c amendment A (test-only counterpart): no command-interface write from a lifecycle
    // callback. The hardware layer publishes the owner-thread safe command during the mode
    // switch that precedes this callback; the owner thread's own next update() cycle writes the
    // controller-side zero (see update() above).
    pending_deactivation_zero_.store(true);
    return controller_interface::CallbackReturn::SUCCESS;
  }

  void setCommand(const std::array<double, kJointCount>& command) noexcept { command_ = command; }
  [[nodiscard]] size_t activationCount() const noexcept { return activation_count_.load(); }
  [[nodiscard]] size_t deactivationCount() const noexcept { return deactivation_count_.load(); }

 private:
  bool write(const std::array<double, kJointCount>& values) noexcept {
    if (command_interfaces_.size() != claim_count_) {
      return false;
    }
    bool success = true;
    for (size_t joint = 0; joint < claim_count_; ++joint) {
      success = command_interfaces_[joint].set_value(values[joint], 1) && success;
    }
    return success;
  }

  std::string arm_id_;
  size_t claim_count_;
  bool fail_activation_after_write_{false};
  std::array<std::string, kJointCount> interface_names_{};
  std::array<double, kJointCount> command_{};
  // Plain bools would race the same way the interface writes used to: on_activate()/on_deactivate()
  // may run on controller_manager's service thread while update() runs on the owner thread.
  std::atomic<bool> active_{false};
  std::atomic<bool> pending_zero_write_{false};
  std::atomic<size_t> activation_count_{0};
  std::atomic<size_t> deactivation_count_{0};
  std::atomic<bool> pending_deactivation_zero_{false};
};

class PreparedPublicationBarrier {
 public:
  ModeSwitchCheckpointHook hook() {
    return [this](ModeSwitchCheckpoint checkpoint) {
      if (checkpoint != ModeSwitchCheckpoint::PreparedPayloadWritten) {
        return;
      }
      std::unique_lock<std::mutex> lock(mutex_);
      entered_ = true;
      condition_.notify_all();
      (void)condition_.wait_for(lock, 2s, [this]() { return released_; });
    };
  }

  bool waitUntilEntered(std::chrono::milliseconds timeout = 1s) {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, timeout, [this]() { return entered_; });
  }

  void release() noexcept {
    const std::lock_guard<std::mutex> lock(mutex_);
    released_ = true;
    condition_.notify_all();
  }

 private:
  std::mutex mutex_{};
  std::condition_variable condition_{};
  bool entered_{false};
  bool released_{false};
};

class ManagerHarness {
 public:
  explicit ManagerHarness(ModeSwitchCheckpointHook mode_switch_checkpoint_hook = {}) {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    hardware_interface::ResourceManagerParams resource_params{};
    resource_params.robot_description = kEmptyRobotDescription;
    resource_params.clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
    resource_params.logger = rclcpp::get_logger("production_cm_test_resource_manager");
    resource_params.node_namespace = "";
    resource_params.executor = executor_;
    resource_params.activate_all = false;
    resource_params.update_rate = 1000;
    auto resource_manager =
        std::make_unique<hardware_interface::ResourceManager>(resource_params, true);
    if (!resource_manager->are_components_initialized()) {
      throw std::runtime_error("empty robot URDF did not initialize ResourceManager");
    }

    auto hardware = std::make_unique<FrankaMultiHardwareInterface>(
        backend_store_.factory(), InitializationCheckpointHook{},
        std::move(mode_switch_checkpoint_hook));
    production_hardware_ = hardware.get();
    hardware_interface::HardwareComponentParams component_params{};
    component_params.hardware_info = makeHardwareInfo();
    component_params.logger = resource_params.logger;
    component_params.clock = resource_params.clock;
    component_params.node_namespace = resource_params.node_namespace;
    component_params.executor = executor_;
    resource_manager->import_component(std::move(hardware), component_params);
    if (resource_manager->system_components_size() != 2U) {
      throw std::runtime_error(
          "production import plus interface-empty bootstrap did not register two components");
    }

    rclcpp_lifecycle::State active_state(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
                                         "active");
    if (resource_manager->set_component_state("FrankaMultiHardwareInterface", active_state) !=
        hardware_interface::return_type::OK) {
      throw std::runtime_error("imported production hardware did not reach ACTIVE");
    }
    const auto& status =
        resource_manager->get_components_status().at("FrankaMultiHardwareInterface");
    if (status.state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      throw std::runtime_error("resource manager status did not report ACTIVE");
    }

    auto manager_options = controller_manager::get_cm_node_options();
    manager_options.arguments({"--ros-args", "--params-file", PRODUCTION_CM_TEST_PARAMS_FILE});
    manager_ = std::make_shared<controller_manager::ControllerManager>(
        std::move(resource_manager), executor_, "controller_manager", "", manager_options);
    if (!manager_->is_resource_manager_initialized()) {
      throw std::runtime_error("controller manager rejected initialized ResourceManager");
    }

    client_node_ = std::make_shared<rclcpp::Node>("production_cm_test_client");
    executor_->add_node(manager_);
    executor_->add_node(client_node_);
    executor_thread_ = std::thread([this]() { executor_->spin(); });
  }

  ManagerHarness(const ManagerHarness&) = delete;
  ManagerHarness& operator=(const ManagerHarness&) = delete;

  ~ManagerHarness() { shutdown(); }

  controller_manager::ControllerManager& manager() { return *manager_; }
  FrankaMultiHardwareInterface& productionHardware() { return *production_hardware_; }
  std::shared_ptr<MinimalBackend> backend(const std::string& arm_id) const {
    return backend_store_.backend(arm_id);
  }

  void loadAndConfigureProductionControllers() {
    loadAndConfigure("hold_controller", "franka_example_controllers/DualArmJointHoldController");
    loadAndConfigure("impedance_controller",
                     "franka_example_controllers/DualArmJointImpedanceController");
    loadAndConfigure("velocity_controller",
                     "franka_example_controllers/DualArmJointVelocityController");
  }

  std::shared_ptr<OneArmVelocityClaimController> addOneArmController(
      const std::string& controller_name,
      const std::string& arm_id,
      size_t claim_count = kJointCount,
      bool fail_activation_after_write = false) {
    auto controller = std::make_shared<OneArmVelocityClaimController>(arm_id, claim_count,
                                                                      fail_activation_after_write);
    controller_interface::ControllerInterfaceParams params{};
    params.controller_name = controller_name;
    params.robot_description = kEmptyRobotDescription;
    params.update_rate = 1000;
    params.controller_manager_update_rate = 1000;
    params.node_namespace = "";
    params.node_options = rclcpp::NodeOptions().use_global_arguments(false);
    if (controller->init(params) != return_type::OK) {
      throw std::runtime_error("test-only one-arm controller init failed");
    }
    if (!manager_->add_controller(controller, controller_name,
                                  "test_only/OneArmVelocityClaimController")) {
      throw std::runtime_error("test-only one-arm controller add failed");
    }
    if (manager_->configure_controller(controller_name) != return_type::OK) {
      throw std::runtime_error("test-only one-arm controller configure failed");
    }
    return controller;
  }

  return_type switchControllers(const std::vector<std::string>& activate,
                                const std::vector<std::string>& deactivate,
                                bool activate_asap = true) {
    auto result = std::async(std::launch::async, [this, activate, deactivate, activate_asap]() {
      return manager_->switch_controller(
          activate, deactivate, controller_manager_msgs::srv::SwitchController::Request::STRICT,
          activate_asap, rclcpp::Duration::from_seconds(1.0));
    });
    for (size_t attempt = 0; attempt < 1000; ++attempt) {
      cycle();
      if (result.wait_for(0ms) == std::future_status::ready) {
        return result.get();
      }
      std::this_thread::sleep_for(1ms);
    }
    throw std::runtime_error("bounded controller switch update pump timed out");
  }

  return_type unloadController(const std::string& controller_name) {
    auto result = std::async(std::launch::async, [this, controller_name]() {
      return manager_->unload_controller(controller_name);
    });
    for (size_t attempt = 0; attempt < 1000; ++attempt) {
      (void)cycle();
      if (result.wait_for(0ms) == std::future_status::ready) {
        return result.get();
      }
      std::this_thread::sleep_for(1ms);
    }
    throw std::runtime_error("bounded controller unload update pump timed out");
  }

  return_type cycle() {
    const auto time = manager_->get_trigger_clock()->now();
    const auto period = rclcpp::Duration(kCyclePeriod);
    manager_->read(time, period);
    const auto result = manager_->update(time, period);
    manager_->write(time, period);
    return result;
  }

  template <typename Predicate>
  bool pumpUntil(Predicate predicate, size_t maximum_cycles = 500) {
    for (size_t cycle_index = 0; cycle_index < maximum_cycles; ++cycle_index) {
      cycle();
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(1ms);
    }
    return false;
  }

  std::vector<std::string> claimedInterfaces(const std::string& controller_name) const {
    for (const auto& specification : manager_->get_loaded_controllers()) {
      if (specification.info.name == controller_name) {
        auto claims = specification.info.claimed_interfaces;
        std::sort(claims.begin(), claims.end());
        return claims;
      }
    }
    throw std::out_of_range("controller not loaded: " + controller_name);
  }

  uint8_t lifecycleId(const std::string& controller_name) const {
    for (const auto& specification : manager_->get_loaded_controllers()) {
      if (specification.info.name == controller_name) {
        return specification.c->get_lifecycle_id();
      }
    }
    throw std::out_of_range("controller not loaded: " + controller_name);
  }

  controller_manager_msgs::srv::ListHardwareInterfaces::Response::SharedPtr hardwareInterfaces() {
    auto client = client_node_->create_client<controller_manager_msgs::srv::ListHardwareInterfaces>(
        "/controller_manager/list_hardware_interfaces");
    if (!client->wait_for_service(2s)) {
      throw std::runtime_error("list_hardware_interfaces service unavailable");
    }
    auto request =
        std::make_shared<controller_manager_msgs::srv::ListHardwareInterfaces::Request>();
    auto result = client->async_send_request(request);
    if (result.wait_for(2s) != std::future_status::ready) {
      throw std::runtime_error("list_hardware_interfaces service timed out");
    }
    return result.get();
  }

  controller_manager_msgs::srv::ListControllers::Response::SharedPtr controllers() {
    auto client = client_node_->create_client<controller_manager_msgs::srv::ListControllers>(
        "/controller_manager/list_controllers");
    if (!client->wait_for_service(2s)) {
      throw std::runtime_error("list_controllers service unavailable");
    }
    auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
    auto result = client->async_send_request(request);
    if (result.wait_for(2s) != std::future_status::ready) {
      throw std::runtime_error("list_controllers service timed out");
    }
    return result.get();
  }

  void setVelocityArmEnabled(size_t arm, bool enabled) {
    auto client = client_node_->create_client<std_srvs::srv::SetBool>(
        "/velocity_controller/arm_" + std::to_string(arm) + "/enable");
    if (!client->wait_for_service(2s)) {
      throw std::runtime_error("velocity enable service unavailable");
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = enabled;
    auto result = client->async_send_request(request);
    if (result.wait_for(2s) != std::future_status::ready || !result.get()->success) {
      throw std::runtime_error("velocity enable request failed");
    }
  }

  void publishVelocity(size_t arm, const std::array<double, kJointCount>& velocities) {
    const auto topic = "/velocity_controller/arm_" + std::to_string(arm) + "/joint_jog";
    auto publisher = client_node_->create_publisher<control_msgs::msg::JointJog>(
        topic, rclcpp::QoS(1).reliable().durability_volatile());
    for (size_t attempt = 0; attempt < 200 && publisher->get_subscription_count() == 0; ++attempt) {
      std::this_thread::sleep_for(1ms);
    }
    if (publisher->get_subscription_count() == 0) {
      throw std::runtime_error("velocity subscriber discovery timed out");
    }

    control_msgs::msg::JointJog message{};
    message.header.stamp = client_node_->now();
    message.duration = 0.0;
    // Reversed names prove the controller honors the named-message contract instead of order.
    for (size_t reverse_index = 0; reverse_index < kJointCount; ++reverse_index) {
      const size_t joint = kJointCount - reverse_index - 1;
      message.joint_names.push_back("panda" + std::to_string(arm) + "_joint" +
                                    std::to_string(joint + 1));
      message.velocities.push_back(velocities[joint]);
    }
    publisher->publish(message);
    std::this_thread::sleep_for(2ms);
  }

  void setImpedanceArmEnabled(size_t arm, bool enabled) {
    auto client = client_node_->create_client<std_srvs::srv::SetBool>(
        "/impedance_controller/arm_" + std::to_string(arm) + "/enable");
    if (!client->wait_for_service(2s)) {
      throw std::runtime_error("impedance enable service unavailable");
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = enabled;
    auto result = client->async_send_request(request);
    if (result.wait_for(2s) != std::future_status::ready || !result.get()->success) {
      throw std::runtime_error("impedance enable request failed");
    }
  }

  void publishImpedanceTarget(size_t arm, const std::array<double, kJointCount>& positions) {
    const auto topic = "/impedance_controller/arm_" + std::to_string(arm) + "/joint_target";
    auto publisher = client_node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        topic, rclcpp::QoS(1).reliable().durability_volatile());
    for (size_t attempt = 0; attempt < 200 && publisher->get_subscription_count() == 0; ++attempt) {
      std::this_thread::sleep_for(1ms);
    }
    if (publisher->get_subscription_count() == 0) {
      throw std::runtime_error("impedance subscriber discovery timed out");
    }

    trajectory_msgs::msg::JointTrajectory message{};
    message.header.stamp = client_node_->now();
    message.points.resize(1);
    // Reversed names prove the controller reorders the named target before applying joint limits.
    for (size_t reverse_index = 0; reverse_index < kJointCount; ++reverse_index) {
      const size_t joint = kJointCount - reverse_index - 1;
      message.joint_names.push_back("panda" + std::to_string(arm) + "_joint" +
                                    std::to_string(joint + 1));
      message.points.front().positions.push_back(positions[joint]);
    }
    publisher->publish(message);
    std::this_thread::sleep_for(2ms);
  }

  void shutdown() noexcept {
    if (!manager_) {
      return;
    }
    try {
      (void)manager_->shutdown_controllers();
    } catch (...) {
    }
    try {
      auto client =
          client_node_->create_client<controller_manager_msgs::srv::SetHardwareComponentState>(
              "/controller_manager/set_hardware_component_state");
      if (client->wait_for_service(2s)) {
        auto request =
            std::make_shared<controller_manager_msgs::srv::SetHardwareComponentState::Request>();
        request->name = "FrankaMultiHardwareInterface";
        request->target_state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
        request->target_state.label = "inactive";
        auto result = client->async_send_request(request);
        (void)result.wait_for(2s);
      }
    } catch (...) {
    }
    try {
      // A Jazzy read/write error can move the component out of ACTIVE before the lifecycle service
      // can invoke on_deactivate(). Ensure the offline harness still exercises production cleanup
      // and confirms both injected backends stop before ResourceManager destruction.
      if (production_hardware_ && (!backend_store_.backend("panda1")->diagnostics().stopped ||
                                   !backend_store_.backend("panda2")->diagnostics().stopped)) {
        (void)production_hardware_->on_deactivate(rclcpp_lifecycle::State());
      }
    } catch (...) {
    }
    executor_->cancel();
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
    try {
      executor_->remove_node(client_node_);
      executor_->remove_node(manager_);
    } catch (...) {
    }
    manager_.reset();
    production_hardware_ = nullptr;
    client_node_.reset();
    executor_.reset();
  }

 private:
  void loadAndConfigure(const std::string& name, const std::string& type) {
    const auto controller = manager_->load_controller(name, type);
    if (!controller) {
      throw std::runtime_error("plugin load failed: " + type);
    }
    if (controller->get_lifecycle_id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
      throw std::runtime_error("loaded controller was not unconfigured");
    }
    if (manager_->configure_controller(name) != return_type::OK) {
      throw std::runtime_error("controller configure failed: " + name);
    }
    if (controller->get_lifecycle_id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      throw std::runtime_error("configured controller was not inactive");
    }
  }

  BackendStore backend_store_{};
  FrankaMultiHardwareInterface* production_hardware_{nullptr};
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_{};
  std::shared_ptr<controller_manager::ControllerManager> manager_{};
  std::shared_ptr<rclcpp::Node> client_node_{};
  std::thread executor_thread_{};
};

std::vector<std::string> expectedClaims(const std::string& arm_id,
                                        const std::string& interface_name) {
  std::vector<std::string> names{};
  names.reserve(kJointCount);
  for (size_t joint = 0; joint < kJointCount; ++joint) {
    names.push_back(arm_id + "_joint" + std::to_string(joint + 1) + "/" + interface_name);
  }
  std::sort(names.begin(), names.end());
  return names;
}

std::vector<std::string> expectedDualClaims(const std::string& interface_name) {
  auto result = expectedClaims("panda1", interface_name);
  auto panda2 = expectedClaims("panda2", interface_name);
  result.insert(result.end(), panda2.begin(), panda2.end());
  std::sort(result.begin(), result.end());
  return result;
}

std::vector<std::string> expectedDualCommandsInControllerOrder(const std::string& interface_name) {
  std::vector<std::string> result{};
  result.reserve(kDualJointInterfaceCount);
  for (size_t arm = 1; arm <= kArmCount; ++arm) {
    for (size_t joint = 1; joint <= kJointCount; ++joint) {
      result.push_back("panda" + std::to_string(arm) + "_joint" + std::to_string(joint) + "/" +
                       interface_name);
    }
  }
  return result;
}

std::vector<std::string> expectedDualStatesInControllerOrder() {
  std::vector<std::string> result{};
  result.reserve(kArmCount * (2U * kJointCount + 2U));
  for (size_t arm = 1; arm <= kArmCount; ++arm) {
    const auto arm_id = "panda" + std::to_string(arm);
    for (size_t joint = 1; joint <= kJointCount; ++joint) {
      const auto joint_name = arm_id + "_joint" + std::to_string(joint);
      result.push_back(joint_name + "/position");
      result.push_back(joint_name + "/velocity");
    }
    result.push_back(arm_id + "/robot_state");
    result.push_back(arm_id + "/robot_model");
  }
  return result;
}

bool commandVelocityEquals(const MinimalBackend& backend,
                           const std::array<double, kJointCount>& expected) {
  return backend.lastCommand().joint_velocities == expected;
}

bool commandEffortEquals(const MinimalBackend& backend,
                         const std::array<double, kJointCount>& expected,
                         const double tolerance = 1e-12) {
  const auto& actual = backend.lastCommand().efforts;
  for (size_t joint = 0; joint < kJointCount; ++joint) {
    if (std::abs(actual[joint] - expected[joint]) > tolerance) {
      return false;
    }
  }
  return true;
}

bool commandEffortNonzero(const MinimalBackend& backend) {
  const auto& efforts = backend.lastCommand().efforts;
  return std::any_of(efforts.begin(), efforts.end(),
                     [](const double effort) { return effort != 0.0; });
}

void expectOrderedSafeModeTransitions(
    const MinimalBackend& backend,
    const std::vector<std::pair<ControlMode, bool>>& expected_mode_attempts) {
  std::vector<std::pair<ControlMode, bool>> actual_mode_attempts{};
  const auto& events = backend.events();
  for (size_t event_index = 0; event_index < events.size(); ++event_index) {
    const auto& event = events.at(event_index);
    if (event.kind != BackendEventKind::ModeAttempt) {
      continue;
    }
    actual_mode_attempts.emplace_back(event.mode, event.accepted);
    ASSERT_GT(event_index, 0U);
    const auto& preceding_event = events.at(event_index - 1);
    EXPECT_EQ(preceding_event.kind, BackendEventKind::CommandAccepted);
    EXPECT_TRUE(isSafeSnapshot(preceding_event.command, backend.state()));
  }

  ASSERT_EQ(actual_mode_attempts.size(), expected_mode_attempts.size());
  for (size_t index = 0; index < expected_mode_attempts.size(); ++index) {
    EXPECT_EQ(actual_mode_attempts.at(index).first, expected_mode_attempts.at(index).first);
    EXPECT_EQ(actual_mode_attempts.at(index).second, expected_mode_attempts.at(index).second);
  }
}

class InspectableStateBroadcaster final
    : public franka_robot_state_broadcaster::FrankaRobotStateBroadcaster {
 public:
  [[nodiscard]] int64_t lastPublishNanoseconds() const noexcept { return last_pub_.nanoseconds(); }
  [[nodiscard]] int64_t publishIntervalNanoseconds() const noexcept { return publish_interval_ns_; }
  [[nodiscard]] bool cadenceInitialized() const noexcept { return publish_time_initialized_; }
  [[nodiscard]] size_t baseLoanCount() const noexcept { return state_interfaces_.size(); }
  [[nodiscard]] bool semanticReadSucceeds() {
    franka_msgs::msg::FrankaState message{};
    return franka_robot_state && franka_robot_state->get_values_as_message(message);
  }
  [[nodiscard]] bool semanticRead(franka_msgs::msg::FrankaState& message) {
    return franka_robot_state && franka_robot_state->get_values_as_message(message);
  }
  [[nodiscard]] bool lockPublisher() {
    return realtime_franka_state_publisher && realtime_franka_state_publisher->trylock();
  }
  void unlockPublisher() { realtime_franka_state_publisher->unlock(); }
  [[nodiscard]] int64_t lockedMessageStampNanoseconds() {
    for (size_t attempt = 0; attempt < 1000; ++attempt) {
      if (realtime_franka_state_publisher->trylock()) {
        const auto stamp = rclcpp::Time(realtime_franka_state_publisher->msg_.header.stamp);
        realtime_franka_state_publisher->unlock();
        return stamp.nanoseconds();
      }
      std::this_thread::sleep_for(1ms);
    }
    throw std::runtime_error("state realtime publisher did not become available");
  }
  void clearPublisher() { realtime_franka_state_publisher.reset(); }
  [[nodiscard]] bool hasRealtimePublisher() const noexcept {
    return realtime_franka_state_publisher != nullptr;
  }
  [[nodiscard]] bool hasPublisher() const noexcept { return franka_state_publisher != nullptr; }
  [[nodiscard]] bool hasSemanticComponent() const noexcept { return franka_robot_state != nullptr; }
};

class InspectableModelBroadcaster final
    : public franka_robot_state_broadcaster::FrankaRobotModelBroadcaster {
 public:
  [[nodiscard]] int64_t lastPublishNanoseconds() const noexcept { return last_pub_.nanoseconds(); }
  [[nodiscard]] int64_t publishIntervalNanoseconds() const noexcept { return publish_interval_ns_; }
  [[nodiscard]] bool cadenceInitialized() const noexcept { return publish_time_initialized_; }
  [[nodiscard]] size_t baseLoanCount() const noexcept { return state_interfaces_.size(); }
  [[nodiscard]] bool semanticReadSucceeds() {
    franka_msgs::msg::FrankaModel message{};
    return franka_robot_model && franka_robot_model->get_values_as_message(message);
  }
  [[nodiscard]] bool lockPublisher() {
    return realtime_franka_model_publisher && realtime_franka_model_publisher->trylock();
  }
  void unlockPublisher() { realtime_franka_model_publisher->unlock(); }
  [[nodiscard]] int64_t lockedMessageStampNanoseconds() {
    for (size_t attempt = 0; attempt < 1000; ++attempt) {
      if (realtime_franka_model_publisher->trylock()) {
        const auto stamp = rclcpp::Time(realtime_franka_model_publisher->msg_.header.stamp);
        realtime_franka_model_publisher->unlock();
        return stamp.nanoseconds();
      }
      std::this_thread::sleep_for(1ms);
    }
    throw std::runtime_error("model realtime publisher did not become available");
  }
  void clearPublisher() { realtime_franka_model_publisher.reset(); }
  [[nodiscard]] bool modelGetterThrows() {
    try {
      (void)franka_robot_model->getCoriolisForceVector();
    } catch (const std::runtime_error&) {
      return true;
    }
    return false;
  }
  [[nodiscard]] bool hasRealtimePublisher() const noexcept {
    return realtime_franka_model_publisher != nullptr;
  }
  [[nodiscard]] bool hasPublisher() const noexcept { return franka_model_publisher != nullptr; }
  [[nodiscard]] bool hasSemanticComponent() const noexcept { return franka_robot_model != nullptr; }
};

template <typename Broadcaster>
std::unique_ptr<Broadcaster> makeDirectBroadcaster(const std::string& name, int64_t frequency) {
  rclcpp::NodeOptions node_options;
  node_options.enable_rosout(false);
  node_options.start_parameter_event_publisher(false);
  node_options.start_parameter_services(false);
  node_options.parameter_overrides(
      {rclcpp::Parameter("arm_id", "panda1"), rclcpp::Parameter("frequency", frequency)});
  controller_interface::ControllerInterfaceParams parameters{};
  parameters.controller_name = name;
  parameters.update_rate = 1000;
  parameters.controller_manager_update_rate = 1000;
  parameters.node_options = node_options;
  auto broadcaster = std::make_unique<Broadcaster>();
  if (broadcaster->init(parameters) != return_type::OK) {
    throw std::runtime_error("direct broadcaster initialization failed");
  }
  return broadcaster;
}

class DirectBroadcasterInterfaces {
 public:
  DirectBroadcasterInterfaces()
      : state_(franka_hardware::test_support::makeSyntheticRobotState(1, 1000)), model_(1) {
    state_pointer_ = &state_;
    model_pointer_ = &model_;
    state_interface_ = std::make_shared<hardware_interface::StateInterface>(
        "panda1", "robot_state",
        reinterpret_cast<double*>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
            &state_pointer_));
    model_interface_ = std::make_shared<hardware_interface::StateInterface>(
        "panda1", "robot_model",
        reinterpret_cast<double*>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
            &model_pointer_));
  }

  void assign(InspectableStateBroadcaster& broadcaster) const {
    std::vector<hardware_interface::LoanedStateInterface> states;
    states.emplace_back(state_interface_);
    broadcaster.assign_interfaces({}, std::move(states));
  }

  void assign(InspectableModelBroadcaster& broadcaster, bool complete) const {
    std::vector<hardware_interface::LoanedStateInterface> states;
    states.emplace_back(model_interface_);
    if (complete) {
      states.emplace_back(state_interface_);
    }
    broadcaster.assign_interfaces({}, std::move(states));
  }

  void assignModelReordered(InspectableModelBroadcaster& broadcaster) const {
    std::vector<hardware_interface::LoanedStateInterface> states;
    states.emplace_back(state_interface_);
    states.emplace_back(model_interface_);
    broadcaster.assign_interfaces({}, std::move(states));
  }

  void assignDuplicateState(InspectableStateBroadcaster& broadcaster) const {
    std::vector<hardware_interface::LoanedStateInterface> states;
    states.emplace_back(state_interface_);
    states.emplace_back(state_interface_);
    broadcaster.assign_interfaces({}, std::move(states));
  }

  void assignDuplicateModel(InspectableModelBroadcaster& broadcaster) const {
    std::vector<hardware_interface::LoanedStateInterface> states;
    states.emplace_back(model_interface_);
    states.emplace_back(model_interface_);
    broadcaster.assign_interfaces({}, std::move(states));
  }

  void assignWrongTypedState(InspectableStateBroadcaster& broadcaster) const {
    std::vector<hardware_interface::LoanedStateInterface> states;
    states.emplace_back(wrong_typed_state_interface_);
    broadcaster.assign_interfaces({}, std::move(states));
  }

  void assignWrongTypedModel(InspectableModelBroadcaster& broadcaster) const {
    std::vector<hardware_interface::LoanedStateInterface> states;
    states.emplace_back(wrong_typed_model_interface_);
    states.emplace_back(state_interface_);
    broadcaster.assign_interfaces({}, std::move(states));
  }

  [[nodiscard]] std::shared_mutex& stateMutex() const { return state_interface_->get_mutex(); }

  [[nodiscard]] std::shared_mutex& modelMutex() const { return model_interface_->get_mutex(); }

  void setStatePointer(franka::RobotState* pointer) noexcept { state_pointer_ = pointer; }
  void setModelPointer(ModelBase* pointer) noexcept { model_pointer_ = pointer; }
  [[nodiscard]] franka::RobotState& state() noexcept { return state_; }
  [[nodiscard]] ModelBase* model() noexcept { return &model_; }

 private:
  franka::RobotState state_{};
  franka_hardware::test_support::SyntheticModel model_;
  franka::RobotState* state_pointer_{nullptr};
  ModelBase* model_pointer_{nullptr};
  std::shared_ptr<hardware_interface::StateInterface> state_interface_{};
  std::shared_ptr<hardware_interface::StateInterface> model_interface_{};
  std::shared_ptr<hardware_interface::StateInterface> wrong_typed_state_interface_{
      std::make_shared<hardware_interface::StateInterface>("panda1",
                                                           "robot_state",
                                                           "bool",
                                                           "true")};
  std::shared_ptr<hardware_interface::StateInterface> wrong_typed_model_interface_{
      std::make_shared<hardware_interface::StateInterface>("panda1",
                                                           "robot_model",
                                                           "bool",
                                                           "true")};
};

class ThrowingModel final : public ModelBase {
 private:
  std::array<double, 16> poseImpl(franka::Frame,
                                  const std::array<double, 7>&,
                                  const std::array<double, 16>&,
                                  const std::array<double, 16>&) const override {
    return {};
  }
  std::array<double, 42> bodyJacobianImpl(franka::Frame,
                                          const std::array<double, 7>&,
                                          const std::array<double, 16>&,
                                          const std::array<double, 16>&) const override {
    throw std::runtime_error("test-only model failure");
  }
  std::array<double, 42> zeroJacobianImpl(franka::Frame,
                                          const std::array<double, 7>&,
                                          const std::array<double, 16>&,
                                          const std::array<double, 16>&) const override {
    return {};
  }
  std::array<double, 49> massImpl(const std::array<double, 7>&,
                                  const std::array<double, 9>&,
                                  double,
                                  const std::array<double, 3>&) const override {
    return {};
  }
  std::array<double, 7> coriolisImpl(const std::array<double, 7>&,
                                     const std::array<double, 7>&,
                                     const std::array<double, 9>&,
                                     double,
                                     const std::array<double, 3>&) const override {
    return {};
  }
  std::array<double, 7> gravityImpl(const std::array<double, 7>&,
                                    double,
                                    const std::array<double, 3>&,
                                    const std::array<double, 3>&) const override {
    return {};
  }
};

class HeldInterfaceLock {
 public:
  explicit HeldInterfaceLock(std::shared_mutex& mutex) {
    auto acquired_future = acquired_.get_future();
    holder_ = std::thread([this, &mutex]() {
      std::unique_lock<std::shared_mutex> lock(mutex);
      acquired_.set_value();
      release_.get_future().wait();
    });
    if (acquired_future.wait_for(1s) != std::future_status::ready) {
      release_.set_value();
      holder_.join();
      throw std::runtime_error("separate interface lock holder did not start");
    }
  }

  HeldInterfaceLock(const HeldInterfaceLock&) = delete;
  HeldInterfaceLock& operator=(const HeldInterfaceLock&) = delete;
  ~HeldInterfaceLock() {
    release_.set_value();
    holder_.join();
  }

 private:
  std::promise<void> acquired_{};
  std::promise<void> release_{};
  std::thread holder_{};
};

template <typename Broadcaster>
class HeldPublisherLock {
 public:
  explicit HeldPublisherLock(Broadcaster& broadcaster) : broadcaster_(broadcaster) {
    auto acquired_future = acquired_.get_future();
    holder_ = std::thread([this]() {
      while (!broadcaster_.lockPublisher()) {
        std::this_thread::yield();
      }
      acquired_.set_value();
      release_.get_future().wait();
      broadcaster_.unlockPublisher();
    });
    if (acquired_future.wait_for(1s) != std::future_status::ready) {
      release_.set_value();
      holder_.join();
      throw std::runtime_error("separate publisher lock holder did not start");
    }
  }

  HeldPublisherLock(const HeldPublisherLock&) = delete;
  HeldPublisherLock& operator=(const HeldPublisherLock&) = delete;
  ~HeldPublisherLock() {
    release_.set_value();
    holder_.join();
  }

 private:
  Broadcaster& broadcaster_;
  std::promise<void> acquired_{};
  std::promise<void> release_{};
  std::thread holder_{};
};

template <typename Broadcaster>
bool waitForPublisherRoundTrip(Broadcaster& broadcaster) {
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (std::chrono::steady_clock::now() < deadline) {
    if (broadcaster.lockPublisher()) {
      broadcaster.unlockPublisher();
      return true;
    }
    std::this_thread::yield();
  }
  return false;
}

template <typename Broadcaster>
bool waitForPublicationAt(Broadcaster& broadcaster, int64_t stamp_nanoseconds) {
  constexpr size_t kMaximumAttempts = 10'000;
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  for (size_t attempt = 0; attempt < kMaximumAttempts; ++attempt) {
    if (std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
    if (broadcaster.update(rclcpp::Time(stamp_nanoseconds, RCL_SYSTEM_TIME),
                           rclcpp::Duration(kCyclePeriod)) != return_type::OK) {
      return false;
    }
    if (broadcaster.lastPublishNanoseconds() == stamp_nanoseconds &&
        broadcaster.lockedMessageStampNanoseconds() == stamp_nanoseconds) {
      return true;
    }
    std::this_thread::yield();
  }
  return false;
}

template <typename Broadcaster>
bool waitForUpdateResultAt(Broadcaster& broadcaster,
                           int64_t stamp_nanoseconds,
                           return_type expected_result) {
  constexpr size_t kMaximumAttempts = 10'000;
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  for (size_t attempt = 0; attempt < kMaximumAttempts; ++attempt) {
    if (std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
    if (broadcaster.update(rclcpp::Time(stamp_nanoseconds, RCL_SYSTEM_TIME),
                           rclcpp::Duration(kCyclePeriod)) == expected_result) {
      return true;
    }
    std::this_thread::yield();
  }
  return false;
}

std::array<bool, 41> messageErrorFlags(const franka_msgs::msg::Errors& error) {
  return {{
      error.joint_position_limits_violation,
      error.cartesian_position_limits_violation,
      error.self_collision_avoidance_violation,
      error.joint_velocity_violation,
      error.cartesian_velocity_violation,
      error.force_control_safety_violation,
      error.joint_reflex,
      error.cartesian_reflex,
      error.max_goal_pose_deviation_violation,
      error.max_path_pose_deviation_violation,
      error.cartesian_velocity_profile_safety_violation,
      error.joint_position_motion_generator_start_pose_invalid,
      error.joint_motion_generator_position_limits_violation,
      error.joint_motion_generator_velocity_limits_violation,
      error.joint_motion_generator_velocity_discontinuity,
      error.joint_motion_generator_acceleration_discontinuity,
      error.cartesian_position_motion_generator_start_pose_invalid,
      error.cartesian_motion_generator_elbow_limit_violation,
      error.cartesian_motion_generator_velocity_limits_violation,
      error.cartesian_motion_generator_velocity_discontinuity,
      error.cartesian_motion_generator_acceleration_discontinuity,
      error.cartesian_motion_generator_elbow_sign_inconsistent,
      error.cartesian_motion_generator_start_elbow_invalid,
      error.cartesian_motion_generator_joint_position_limits_violation,
      error.cartesian_motion_generator_joint_velocity_limits_violation,
      error.cartesian_motion_generator_joint_velocity_discontinuity,
      error.cartesian_motion_generator_joint_acceleration_discontinuity,
      error.cartesian_position_motion_generator_invalid_frame,
      error.force_controller_desired_force_tolerance_violation,
      error.controller_torque_discontinuity,
      error.start_elbow_sign_inconsistent,
      error.communication_constraints_violation,
      error.power_limit_violation,
      error.joint_p2p_insufficient_torque_for_planning,
      error.tau_j_range_violation,
      error.instability_detected,
      error.joint_move_in_wrong_direction,
      error.cartesian_spline_motion_generator_violation,
      error.joint_via_motion_generator_planning_joint_limit_violation,
      error.base_acceleration_initialization_timeout,
      error.base_acceleration_invalid_reading,
  }};
}

bool commandHasNonzeroMotion(const RobotCommand& command) {
  return !allZero(command.efforts) || !allZero(command.joint_velocities) ||
         !allZero(command.cartesian_velocities);
}

void expectActualBroadcasters(size_t arm_count) {
  test_support::OfflineControllerManagerHarness harness(arm_count);
  bool overrun_warnings = true;
  ASSERT_TRUE(harness.manager().get_parameter("overruns.print_warnings", overrun_warnings));
  EXPECT_FALSE(overrun_warnings);
  std::mutex message_mutex;
  std::vector<franka_msgs::msg::FrankaState::SharedPtr> state_messages(arm_count);
  std::vector<franka_msgs::msg::FrankaModel::SharedPtr> model_messages(arm_count);
  std::vector<size_t> state_message_counts(arm_count, 0);
  std::vector<size_t> model_message_counts(arm_count, 0);
  std::vector<rclcpp::Subscription<franka_msgs::msg::FrankaState>::SharedPtr> state_subscriptions;
  std::vector<rclcpp::Subscription<franka_msgs::msg::FrankaModel>::SharedPtr> model_subscriptions;
  for (size_t arm = 1; arm <= arm_count; ++arm) {
    const auto arm_index = arm - 1;
    const auto prefix = "panda" + std::to_string(arm);
    state_subscriptions.push_back(
        harness.clientNode().create_subscription<franka_msgs::msg::FrankaState>(
            "/" + prefix + "_state_broadcaster/robot_state", rclcpp::SystemDefaultsQoS(),
            [&, arm_index](franka_msgs::msg::FrankaState::SharedPtr message) {
              const std::lock_guard<std::mutex> lock(message_mutex);
              state_messages.at(arm_index) = std::move(message);
              ++state_message_counts.at(arm_index);
            }));
    model_subscriptions.push_back(
        harness.clientNode().create_subscription<franka_msgs::msg::FrankaModel>(
            "/" + prefix + "_model_broadcaster/robot_model", rclcpp::SystemDefaultsQoS(),
            [&, arm_index](franka_msgs::msg::FrankaModel::SharedPtr message) {
              const std::lock_guard<std::mutex> lock(message_mutex);
              model_messages.at(arm_index) = std::move(message);
              ++model_message_counts.at(arm_index);
            }));
  }

  const auto broadcaster_names = harness.loadBroadcasters(arm_count);
  ASSERT_EQ(broadcaster_names.size(), arm_count * 2U);
  const auto listed = harness.controllers();
  for (size_t arm = 1; arm <= arm_count; ++arm) {
    const auto prefix = "panda" + std::to_string(arm);
    for (const auto& expectation : std::array<std::pair<std::string, std::vector<std::string>>, 2>{
             std::pair{prefix + "_state_broadcaster",
                       std::vector<std::string>{prefix + "/robot_state"}},
             std::pair{
                 prefix + "_model_broadcaster",
                 std::vector<std::string>{prefix + "/robot_model", prefix + "/robot_state"}}}) {
      const auto match = std::find_if(
          listed->controller.begin(), listed->controller.end(),
          [&](const auto& controller) { return controller.name == expectation.first; });
      ASSERT_NE(match, listed->controller.end());
      EXPECT_EQ(match->required_state_interfaces, expectation.second);
      EXPECT_TRUE(match->required_command_interfaces.empty());
      EXPECT_TRUE(match->claimed_interfaces.empty());
    }
  }

  const auto state_claimed = [&harness](const std::string& name) {
    const auto response = harness.hardwareInterfaces();
    const auto match =
        std::find_if(response->state_interfaces.begin(), response->state_interfaces.end(),
                     [&name](const auto& interface) { return interface.name == name; });
    if (match == response->state_interfaces.end()) {
      throw std::runtime_error("state interface omitted from ListHardwareInterfaces: " + name);
    }
    return match->is_claimed;
  };
  const auto state_available = [&harness](const std::string& name) {
    const auto response = harness.hardwareInterfaces();
    const auto match =
        std::find_if(response->state_interfaces.begin(), response->state_interfaces.end(),
                     [&name](const auto& interface) { return interface.name == name; });
    if (match == response->state_interfaces.end()) {
      throw std::runtime_error("state interface omitted from ListHardwareInterfaces: " + name);
    }
    return match->is_available;
  };
  for (size_t arm = 1; arm <= arm_count; ++arm) {
    const auto prefix = "panda" + std::to_string(arm);
    EXPECT_TRUE(state_available(prefix + "/robot_state"));
    EXPECT_TRUE(state_available(prefix + "/robot_model"));
    EXPECT_FALSE(state_claimed(prefix + "/robot_state"));
    EXPECT_FALSE(state_claimed(prefix + "/robot_model"));
  }
  ASSERT_EQ(harness.switchControllers(broadcaster_names, {}), return_type::OK);
  for (size_t arm = 1; arm <= arm_count; ++arm) {
    const auto prefix = "panda" + std::to_string(arm);
    // Jazzy state loans are shareable and ResourceManager deliberately has no
    // state-claim query; ListHardwareInterfaces reports them available and
    // never command-style claimed while the active plugins hold real loans.
    EXPECT_TRUE(state_available(prefix + "/robot_state"));
    EXPECT_TRUE(state_available(prefix + "/robot_model"));
    EXPECT_FALSE(state_claimed(prefix + "/robot_state"));
    EXPECT_FALSE(state_claimed(prefix + "/robot_model"));
  }
  ASSERT_TRUE(harness.pumpUntil([&]() {
    const std::lock_guard<std::mutex> lock(message_mutex);
    return std::all_of(state_messages.begin(), state_messages.end(),
                       [](const auto& message) { return message != nullptr; }) &&
           std::all_of(model_messages.begin(), model_messages.end(),
                       [](const auto& message) { return message != nullptr; });
  }));

  for (size_t arm = 1; arm <= arm_count; ++arm) {
    const auto arm_index = arm - 1;
    const auto prefix = "panda" + std::to_string(arm);
    EXPECT_EQ(
        harness.clientNode().count_publishers("/" + prefix + "_state_broadcaster/robot_state"), 1U);
    EXPECT_EQ(
        harness.clientNode().count_publishers("/" + prefix + "_model_broadcaster/robot_model"), 1U);
    const auto backend = harness.backend(prefix);
    const auto expected_state = franka_hardware::test_support::makeSyntheticRobotState(
        static_cast<uint8_t>(arm), static_cast<uint64_t>(arm) * 1000U);
    const auto expected_coriolis = backend->model()->coriolis(expected_state);
    const auto expected_mass = backend->model()->mass(expected_state);
    const auto expected_body =
        backend->model()->bodyJacobian(franka::Frame::kEndEffector, expected_state);
    const auto expected_zero =
        backend->model()->zeroJacobian(franka::Frame::kEndEffector, expected_state);
    const std::lock_guard<std::mutex> lock(message_mutex);
    ASSERT_TRUE(state_messages.at(arm_index));
    ASSERT_TRUE(model_messages.at(arm_index));
    EXPECT_EQ(state_messages.at(arm_index)->q, expected_state.q);
    EXPECT_EQ(state_messages.at(arm_index)->dq, expected_state.dq);
    EXPECT_EQ(model_messages.at(arm_index)->coriolis, expected_coriolis);
    EXPECT_EQ(model_messages.at(arm_index)->mass, expected_mass);
    EXPECT_EQ(model_messages.at(arm_index)->ee_body_jacobian, expected_body);
    EXPECT_EQ(model_messages.at(arm_index)->ee_zero_jacobian, expected_zero);
  }
  if (arm_count == 2U) {
    const std::lock_guard<std::mutex> lock(message_mutex);
    EXPECT_NE(state_messages.at(0)->q, state_messages.at(1)->q);
    EXPECT_NE(model_messages.at(0)->coriolis, model_messages.at(1)->coriolis);
  }

  ASSERT_EQ(harness.switchControllers({}, {"panda1_model_broadcaster"}), return_type::OK);
  for (size_t cycle = 0; cycle < 20; ++cycle) {
    (void)harness.cycle();
    std::this_thread::sleep_for(1ms);
  }
  std::vector<size_t> state_counts_after_model_stop;
  std::vector<size_t> model_counts_after_model_stop;
  {
    const std::lock_guard<std::mutex> lock(message_mutex);
    state_counts_after_model_stop = state_message_counts;
    model_counts_after_model_stop = model_message_counts;
  }
  ASSERT_TRUE(harness.pumpUntil([&]() {
    const std::lock_guard<std::mutex> lock(message_mutex);
    const bool panda1_state_advanced =
        state_message_counts.at(0) > state_counts_after_model_stop.at(0);
    const bool panda2_advanced =
        arm_count == 1U || (state_message_counts.at(1) > state_counts_after_model_stop.at(1) &&
                            model_message_counts.at(1) > model_counts_after_model_stop.at(1));
    return panda1_state_advanced && panda2_advanced;
  }));
  {
    const std::lock_guard<std::mutex> lock(message_mutex);
    EXPECT_EQ(model_message_counts.at(0), model_counts_after_model_stop.at(0));
  }
  EXPECT_FALSE(state_claimed("panda1/robot_model"));
  EXPECT_FALSE(state_claimed("panda1/robot_state"));
  if (arm_count == 2U) {
    EXPECT_FALSE(state_claimed("panda2/robot_model"));
    EXPECT_FALSE(state_claimed("panda2/robot_state"));
  }
  ASSERT_EQ(harness.switchControllers({}, {"panda1_state_broadcaster"}), return_type::OK);
  EXPECT_FALSE(state_claimed("panda1/robot_model"));
  EXPECT_FALSE(state_claimed("panda1/robot_state"));
  if (arm_count == 2U) {
    EXPECT_FALSE(state_claimed("panda2/robot_model"));
    EXPECT_FALSE(state_claimed("panda2/robot_state"));
  }
  for (size_t cycle = 0; cycle < 20; ++cycle) {
    (void)harness.cycle();
    std::this_thread::sleep_for(1ms);
  }
  std::vector<size_t> state_counts_after_state_stop;
  std::vector<size_t> model_counts_after_state_stop;
  {
    const std::lock_guard<std::mutex> lock(message_mutex);
    state_counts_after_state_stop = state_message_counts;
    model_counts_after_state_stop = model_message_counts;
  }
  if (arm_count == 2U) {
    ASSERT_TRUE(harness.pumpUntil([&]() {
      const std::lock_guard<std::mutex> lock(message_mutex);
      return state_message_counts.at(1) > state_counts_after_state_stop.at(1) &&
             model_message_counts.at(1) > model_counts_after_state_stop.at(1);
    }));
  } else {
    for (size_t cycle = 0; cycle < 20; ++cycle) {
      (void)harness.cycle();
      std::this_thread::sleep_for(1ms);
    }
  }
  {
    const std::lock_guard<std::mutex> lock(message_mutex);
    EXPECT_EQ(state_message_counts.at(0), state_counts_after_state_stop.at(0));
    EXPECT_EQ(model_message_counts.at(0), model_counts_after_state_stop.at(0));
  }
  size_t panda1_state_before_reactivation = 0;
  size_t panda1_model_before_reactivation = 0;
  {
    const std::lock_guard<std::mutex> lock(message_mutex);
    panda1_state_before_reactivation = state_message_counts.at(0);
    panda1_model_before_reactivation = model_message_counts.at(0);
  }
  ASSERT_EQ(harness.switchControllers({"panda1_state_broadcaster", "panda1_model_broadcaster"}, {}),
            return_type::OK);
  EXPECT_FALSE(state_claimed("panda1/robot_model"));
  EXPECT_FALSE(state_claimed("panda1/robot_state"));
  ASSERT_TRUE(harness.pumpUntil([&]() {
    const std::lock_guard<std::mutex> lock(message_mutex);
    return state_message_counts.at(0) > panda1_state_before_reactivation &&
           model_message_counts.at(0) > panda1_model_before_reactivation;
  }));

  for (size_t arm = 1; arm <= arm_count; ++arm) {
    const auto backend = harness.backend("panda" + std::to_string(arm));
    const auto measured_state = franka_hardware::test_support::makeSyntheticRobotState(
        static_cast<uint8_t>(arm), static_cast<uint64_t>(arm) * 1000U);
    for (size_t index = 0; index < backend->capturedEventCount(); ++index) {
      const auto event = backend->capturedEvent(index);
      EXPECT_FALSE(event.kind == franka_hardware::test_support::SyntheticEventKind::ModeRequested &&
                   event.mode != ControlMode::None);
    }
    for (size_t index = 0; index < backend->capturedCommandCount(); ++index) {
      const auto command = backend->capturedCommand(index);
      EXPECT_FALSE(commandHasNonzeroMotion(command));
      EXPECT_TRUE(isSafeSnapshot(command, measured_state));
    }
  }

  ASSERT_EQ(harness.switchControllers({}, broadcaster_names), return_type::OK);
  for (const auto& name : broadcaster_names) {
    EXPECT_EQ(harness.lifecycleId(name), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_TRUE(harness.claimedInterfaces(name).empty());
  }
  for (size_t arm = 1; arm <= arm_count; ++arm) {
    const auto prefix = "panda" + std::to_string(arm);
    EXPECT_FALSE(state_claimed(prefix + "/robot_state"));
    EXPECT_FALSE(state_claimed(prefix + "/robot_model"));
  }
  for (size_t cycle = 0; cycle < 20; ++cycle) {
    (void)harness.cycle();
    std::this_thread::sleep_for(1ms);
  }
  std::vector<size_t> inactive_state_counts;
  std::vector<size_t> inactive_model_counts;
  {
    const std::lock_guard<std::mutex> lock(message_mutex);
    inactive_state_counts = state_message_counts;
    inactive_model_counts = model_message_counts;
  }
  for (size_t cycle = 0; cycle < 20; ++cycle) {
    (void)harness.cycle();
    std::this_thread::sleep_for(1ms);
  }
  {
    const std::lock_guard<std::mutex> lock(message_mutex);
    EXPECT_EQ(state_message_counts, inactive_state_counts);
    EXPECT_EQ(model_message_counts, inactive_model_counts);
  }
  harness.deactivateAndUnloadAll();
  EXPECT_TRUE(harness.loadedControllerNames().empty());
  harness.deactivateHardware();
  for (size_t arm = 1; arm <= arm_count; ++arm) {
    const auto backend = harness.backend("panda" + std::to_string(arm));
    EXPECT_EQ(backend->acceptedUnsafeSnapshotCount(), 0U);
    EXPECT_EQ(backend->acceptedSafeSnapshotCount(), backend->acceptedCommandCount());
    EXPECT_EQ(backend->acceptedSafeSnapshotCount() + backend->acceptedUnsafeSnapshotCount(),
              backend->acceptedCommandCount());
    EXPECT_EQ(backend->acceptedNonNoneModeRequestCount(), 0U);
    EXPECT_EQ(backend->rejectedNonNoneModeRequestCount(), 0U);
  }
  ASSERT_TRUE(harness.pumpUntil([&]() {
    for (size_t arm = 1; arm <= arm_count; ++arm) {
      const auto prefix = "panda" + std::to_string(arm);
      if (harness.clientNode().count_publishers("/" + prefix + "_state_broadcaster/robot_state") !=
              0U ||
          harness.clientNode().count_publishers("/" + prefix + "_model_broadcaster/robot_model") !=
              0U) {
        return false;
      }
    }
    return true;
  }));
  for (size_t arm = 1; arm <= arm_count; ++arm) {
    const auto prefix = "panda" + std::to_string(arm);
    EXPECT_EQ(
        harness.clientNode().count_publishers("/" + prefix + "_state_broadcaster/robot_state"), 0U);
    EXPECT_EQ(
        harness.clientNode().count_publishers("/" + prefix + "_model_broadcaster/robot_model"), 0U);
  }
  harness.shutdown();
  const auto cleanup = harness.cleanupSnapshot();
  EXPECT_EQ(cleanup.constructed, arm_count);
  EXPECT_EQ(cleanup.stopped, arm_count);
  EXPECT_EQ(cleanup.destroyed, arm_count);
}

class ProductionControllerManagerIntegrationTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      int argc = 0;
      char** argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(ProductionControllerManagerIntegrationTest,
       ListControllersPreservesExactReviewedInterfaceOrder) {
  ManagerHarness harness;
  harness.loadAndConfigureProductionControllers();
  const auto response = harness.controllers();
  ASSERT_EQ(response->controller.size(), 3U);

  const auto find_controller =
      [&response](const std::string& name) -> const controller_manager_msgs::msg::ControllerState& {
    const auto match =
        std::find_if(response->controller.begin(), response->controller.end(),
                     [&name](const auto& controller) { return controller.name == name; });
    if (match == response->controller.end()) {
      throw std::runtime_error("list_controllers omitted " + name);
    }
    return *match;
  };

  const auto expected_effort = expectedDualCommandsInControllerOrder("effort");
  const auto expected_velocity = expectedDualCommandsInControllerOrder("velocity");
  const auto expected_states = expectedDualStatesInControllerOrder();
  for (const auto* name : {"hold_controller", "impedance_controller"}) {
    const auto& controller = find_controller(name);
    EXPECT_EQ(controller.required_command_interfaces, expected_effort);
    EXPECT_EQ(controller.required_state_interfaces, expected_states);
  }
  const auto& velocity = find_controller("velocity_controller");
  EXPECT_EQ(velocity.required_command_interfaces, expected_velocity);
  EXPECT_TRUE(velocity.required_state_interfaces.empty());
}

TEST_F(ProductionControllerManagerIntegrationTest,
       ImportsProductionSystemActiveAndLoadsRealPluginsWithoutAutostart) {
  PreparedPublicationBarrier prepared_barrier;
  ManagerHarness harness(prepared_barrier.hook());
  const auto panda1 = harness.backend("panda1");
  const auto panda2 = harness.backend("panda2");
  ASSERT_EQ(panda1->startCount(), 1U);
  ASSERT_EQ(panda2->startCount(), 1U);
  ASSERT_GE(panda1->readCount(), 2U);
  ASSERT_GE(panda2->readCount(), 2U);
  ASSERT_FALSE(panda1->commands().empty());
  ASSERT_FALSE(panda2->commands().empty());
  EXPECT_TRUE(isSafeSnapshot(panda1->commands().front(), panda1->state()));
  EXPECT_TRUE(isSafeSnapshot(panda2->commands().front(), panda2->state()));

  harness.loadAndConfigureProductionControllers();
  EXPECT_EQ(harness.lifecycleId("hold_controller"),
            lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(harness.lifecycleId("velocity_controller"),
            lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(harness.lifecycleId("impedance_controller"),
            lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_TRUE(harness.claimedInterfaces("hold_controller").empty());
  EXPECT_TRUE(harness.claimedInterfaces("velocity_controller").empty());
  EXPECT_TRUE(harness.claimedInterfaces("impedance_controller").empty());

  // Bind the production hardware to this update thread, then hold prepare after its immutable
  // payload has been written but before publication completes. An ordinary owner cycle must
  // remain race-free while prepare is blocked -- every explicitly counted cycle below still does
  // exactly one ordinary write() publish, proving the pending off-owner perform() (parked in
  // requestOwnerExecutedEffects()'s bounded poll) never runs its effects on any thread other than
  // this owner thread. Once released, the default activate_asap=false path resolves
  // perform_command_mode_switch() on controller_manager's own service thread (the std::async
  // thread spawned by switch_controller() below) while this thread keeps driving read()/write():
  // the live two-thread topology F-10a was found in. This must now succeed via the bounded
  // owner-thread handoff (applyPreparedTransactionEffects() runs once, on this thread, inside the
  // write() call that observes the pending request) rather than being rejected for arriving off
  // the control-cycle owner thread.
  const auto false_path_controller = harness.addOneArmController("false_path_controller", "panda1");
  ASSERT_EQ(harness.cycle(), return_type::OK);
  const auto panda1_publishes_before_false = panda1->publishAttemptCount();
  const auto panda2_publishes_before_false = panda2->publishAttemptCount();
  const auto panda1_requests_before_false = panda1->modeRequestAttempts().size();
  const auto panda2_requests_before_false = panda2->modeRequestAttempts().size();
  auto false_path_switch = std::async(std::launch::async, [&]() {
    return harness.manager().switch_controller(
        {"false_path_controller"}, {},
        controller_manager_msgs::srv::SwitchController::Request::STRICT, false,
        rclcpp::Duration::from_seconds(1.0));
  });

  const bool reached_prepared_barrier = prepared_barrier.waitUntilEntered();
  if (!reached_prepared_barrier) {
    prepared_barrier.release();
  }
  ASSERT_TRUE(reached_prepared_barrier);
  EXPECT_EQ(harness.cycle(), return_type::OK);
  size_t false_path_cycles = 1;
  prepared_barrier.release();
  for (size_t attempt = 0;
       attempt < 1000 && false_path_switch.wait_for(0ms) != std::future_status::ready; ++attempt) {
    EXPECT_EQ(harness.cycle(), return_type::OK);
    ++false_path_cycles;
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_EQ(false_path_switch.wait_for(0ms), std::future_status::ready);
  EXPECT_EQ(false_path_switch.get(), return_type::OK);
  EXPECT_EQ(false_path_controller->activationCount(), 1U);
  EXPECT_EQ(false_path_controller->deactivationCount(), 0U);
  EXPECT_EQ(harness.lifecycleId("false_path_controller"),
            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_FALSE(harness.claimedInterfaces("false_path_controller").empty());
  // Only panda1 is claimed (see OneArmVelocityClaimController's construction above), so only
  // panda1 sees a mode-request effect; panda2's transaction arm has no request at all.
  EXPECT_EQ(panda1->modeRequestAttempts().size(), panda1_requests_before_false + 1);
  EXPECT_EQ(panda2->modeRequestAttempts().size(), panda2_requests_before_false);
  EXPECT_EQ(panda1->activeControlMode(), ControlMode::JointVelocity);
  EXPECT_EQ(panda2->activeControlMode(), ControlMode::None);
  // F-10c amendment B (F-10d, 2026-08-28): write() publishes every cycle for an arm in a LIVE
  // mode -- the handoff adds work inside one of those write() calls, it never replaces or skips
  // one -- and publishes nothing at all for an arm parked in ControlMode::None, whose command
  // channel would otherwise grow one entry per cycle into a backend with no consumer. panda1 is
  // now in JointVelocity; panda2 was never claimed and has been in None throughout.
  EXPECT_GT(panda1->publishAttemptCount(), panda1_publishes_before_false)
      << "the handoff swallowed panda1's ordinary write() publishes";
  const auto panda1_publishes_after_switch = panda1->publishAttemptCount();
  const auto panda2_publishes_after_switch = panda2->publishAttemptCount();
  constexpr size_t kSteadyCycles = 5;
  for (size_t index = 0; index < kSteadyCycles; ++index) {
    EXPECT_EQ(harness.cycle(), return_type::OK);
  }
  EXPECT_EQ(panda1->publishAttemptCount(), panda1_publishes_after_switch + kSteadyCycles)
      << "a live-mode arm must be published to exactly once per write() cycle";
  EXPECT_EQ(panda2->publishAttemptCount(), panda2_publishes_after_switch)
      << "an arm parked in ControlMode::None must not be published to again (F-10d)";
  EXPECT_FALSE(harness.productionHardware().globalFaultDiagnostic().latched());

  const auto interfaces = harness.hardwareInterfaces();
  const auto claimed_count = static_cast<size_t>(
      std::count_if(interfaces->command_interfaces.begin(), interfaces->command_interfaces.end(),
                    [](const auto& interface) { return interface.is_claimed; }));
  EXPECT_EQ(claimed_count, kJointCount);

  harness.shutdown();
  EXPECT_GE(panda1->stopCount(), 1U);
  EXPECT_GE(panda2->stopCount(), 1U);
  EXPECT_EQ(panda1->activeControlMode(), ControlMode::None);
  EXPECT_EQ(panda2->activeControlMode(), ControlMode::None);
}

TEST_F(ProductionControllerManagerIntegrationTest,
       DefaultFalseImpedanceActivationSucceedsViaOwnerThreadHandoff) {
  // F-10a regression coverage: controller_manager's default activate_asap=false resolves
  // perform_command_mode_switch() on its own service thread (switchControllers()'s std::async
  // thread below) while harness.cycle() keeps driving read()/write() on this test thread --
  // matching production's live two-thread topology (a real ros2_control_node's service thread
  // vs its RT update thread), the exact split a plain `ros2 control switch_controllers
  // --deactivate` (no --switch-asap) hits. This must succeed via the bounded owner-thread
  // handoff (requestOwnerExecutedEffects() / serviceOwnerHandoffIfPending()), not be rejected for
  // arriving off the control-cycle owner thread.
  ManagerHarness harness;
  harness.loadAndConfigureProductionControllers();
  const auto panda1 = harness.backend("panda1");
  const auto panda2 = harness.backend("panda2");
  ASSERT_EQ(harness.cycle(), return_type::OK);
  const auto panda1_requests_before = panda1->modeRequestAttempts().size();
  const auto panda2_requests_before = panda2->modeRequestAttempts().size();

  EXPECT_EQ(harness.switchControllers({"impedance_controller"}, {}, false), return_type::OK);
  EXPECT_EQ(harness.lifecycleId("impedance_controller"),
            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_FALSE(harness.claimedInterfaces("impedance_controller").empty());
  EXPECT_EQ(panda1->modeRequestAttempts().size(), panda1_requests_before + 1);
  EXPECT_EQ(panda2->modeRequestAttempts().size(), panda2_requests_before + 1);
  EXPECT_EQ(panda1->activeControlMode(), ControlMode::JointTorque);
  EXPECT_EQ(panda2->activeControlMode(), ControlMode::JointTorque);
  EXPECT_FALSE(harness.productionHardware().globalFaultDiagnostic().latched());

  // The exact scenario F-10a was found in: releasing an already-active effort-mode controller
  // through the same default (activate_asap=false) path, deterministically, repeatedly.
  for (int cycle_index = 0; cycle_index < 5; ++cycle_index) {
    EXPECT_EQ(harness.switchControllers({}, {"impedance_controller"}, false), return_type::OK)
        << "deactivate cycle " << cycle_index;
    EXPECT_EQ(harness.lifecycleId("impedance_controller"),
              lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_TRUE(harness.claimedInterfaces("impedance_controller").empty());
    EXPECT_EQ(panda1->activeControlMode(), ControlMode::None);
    EXPECT_EQ(panda2->activeControlMode(), ControlMode::None);
    EXPECT_FALSE(harness.productionHardware().globalFaultDiagnostic().latched());

    EXPECT_EQ(harness.switchControllers({"impedance_controller"}, {}, false), return_type::OK)
        << "reactivate cycle " << cycle_index;
    EXPECT_EQ(harness.lifecycleId("impedance_controller"),
              lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    EXPECT_FALSE(harness.claimedInterfaces("impedance_controller").empty());
    EXPECT_EQ(panda1->activeControlMode(), ControlMode::JointTorque);
    EXPECT_EQ(panda2->activeControlMode(), ControlMode::JointTorque);
    EXPECT_FALSE(harness.productionHardware().globalFaultDiagnostic().latched());
  }
}

TEST_F(ProductionControllerManagerIntegrationTest,
       RealImpedanceBehaviorAndTruePathHoldVelocitySwitchesAreBoundedAndIndependent) {
  ManagerHarness harness;
  harness.loadAndConfigureProductionControllers();
  const auto panda1 = harness.backend("panda1");
  const auto panda2 = harness.backend("panda2");
  const std::array<double, kJointCount> zero{};

  ASSERT_EQ(harness.switchControllers({"hold_controller"}, {}), return_type::OK);
  ASSERT_EQ(harness.switchControllers({"impedance_controller"}, {"hold_controller"}),
            return_type::OK);
  EXPECT_EQ(harness.lifecycleId("impedance_controller"),
            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_TRUE(harness.claimedInterfaces("hold_controller").empty());
  EXPECT_EQ(harness.claimedInterfaces("impedance_controller"), expectedDualClaims("effort"));
  EXPECT_EQ(panda1->activeControlMode(), ControlMode::JointTorque);
  EXPECT_EQ(panda2->activeControlMode(), ControlMode::JointTorque);
  ASSERT_EQ(harness.cycle(), return_type::OK);
  EXPECT_TRUE(commandEffortEquals(*panda1, zero));
  EXPECT_TRUE(commandEffortEquals(*panda2, zero));

  const std::array<double, kJointCount> panda1_target{{-0.2, -0.6, 0.1, -1.2, 0.2, 1.1, -0.1}};
  harness.setImpedanceArmEnabled(1, true);
  harness.publishImpedanceTarget(1, panda1_target);
  ASSERT_TRUE(harness.pumpUntil([&]() { return commandEffortNonzero(*panda1); }));
  EXPECT_TRUE(commandEffortEquals(*panda2, zero));
  const auto last_fresh_effort = panda1->lastCommand().efforts;

  // No update cycles run during this wait. Once the target becomes stale, the next cycle must
  // hold the already-applied rate-limited target rather than jump to the unapplied endpoint.
  std::this_thread::sleep_for(120ms);
  ASSERT_EQ(harness.cycle(), return_type::OK);
  EXPECT_TRUE(commandEffortEquals(*panda1, last_fresh_effort));
  EXPECT_TRUE(commandEffortEquals(*panda2, zero));

  ASSERT_EQ(harness.switchControllers({"hold_controller"}, {"impedance_controller"}),
            return_type::OK);
  EXPECT_TRUE(harness.claimedInterfaces("impedance_controller").empty());
  EXPECT_EQ(harness.claimedInterfaces("hold_controller"), expectedDualClaims("effort"));
  ASSERT_EQ(harness.cycle(), return_type::OK);
  EXPECT_TRUE(commandEffortEquals(*panda1, zero));
  EXPECT_TRUE(commandEffortEquals(*panda2, zero));

  ASSERT_EQ(harness.switchControllers({"impedance_controller"}, {"hold_controller"}),
            return_type::OK);
  ASSERT_EQ(harness.switchControllers({"velocity_controller"}, {"impedance_controller"}),
            return_type::OK);
  EXPECT_TRUE(harness.claimedInterfaces("impedance_controller").empty());
  EXPECT_EQ(harness.claimedInterfaces("velocity_controller"), expectedDualClaims("velocity"));
  EXPECT_EQ(panda1->activeControlMode(), ControlMode::JointVelocity);
  EXPECT_EQ(panda2->activeControlMode(), ControlMode::JointVelocity);
  EXPECT_TRUE(commandVelocityEquals(*panda1, zero));
  EXPECT_TRUE(commandVelocityEquals(*panda2, zero));

  ASSERT_EQ(harness.switchControllers({"impedance_controller"}, {"velocity_controller"}),
            return_type::OK);
  EXPECT_TRUE(harness.claimedInterfaces("velocity_controller").empty());
  EXPECT_EQ(harness.claimedInterfaces("impedance_controller"), expectedDualClaims("effort"));
  EXPECT_EQ(panda1->activeControlMode(), ControlMode::JointTorque);
  EXPECT_EQ(panda2->activeControlMode(), ControlMode::JointTorque);
  ASSERT_EQ(harness.cycle(), return_type::OK);
  EXPECT_TRUE(commandEffortEquals(*panda1, zero));
  EXPECT_TRUE(commandEffortEquals(*panda2, zero));

  ASSERT_EQ(harness.switchControllers({}, {"impedance_controller"}), return_type::OK);
  EXPECT_TRUE(harness.claimedInterfaces("impedance_controller").empty());
  EXPECT_EQ(panda1->activeControlMode(), ControlMode::None);
  EXPECT_EQ(panda2->activeControlMode(), ControlMode::None);
}

TEST_F(ProductionControllerManagerIntegrationTest,
       RealHoldVelocitySwitchesDriveRosCommandsWatchdogAndExactRelease) {
  ManagerHarness harness;
  harness.loadAndConfigureProductionControllers();
  const auto panda1 = harness.backend("panda1");
  const auto panda2 = harness.backend("panda2");

  ASSERT_EQ(harness.switchControllers({"hold_controller"}, {}), return_type::OK);
  ASSERT_EQ(harness.lifecycleId("hold_controller"),
            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_EQ(harness.claimedInterfaces("hold_controller"), expectedDualClaims("effort"));
  EXPECT_EQ(harness.claimedInterfaces("hold_controller").size(), kDualJointInterfaceCount);
  EXPECT_EQ(panda1->activeControlMode(), ControlMode::JointTorque);
  EXPECT_EQ(panda2->activeControlMode(), ControlMode::JointTorque);
  ASSERT_EQ(harness.cycle(), return_type::OK);
  EXPECT_TRUE(allZero(panda1->lastCommand().efforts));
  EXPECT_TRUE(allZero(panda2->lastCommand().efforts));
  EXPECT_GT(panda1->modelInstance().coriolisCallCount(), 0U);
  EXPECT_GT(panda2->modelInstance().coriolisCallCount(), 0U);

  ASSERT_EQ(harness.switchControllers({"velocity_controller"}, {"hold_controller"}),
            return_type::OK);
  EXPECT_EQ(harness.lifecycleId("hold_controller"),
            lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(harness.lifecycleId("velocity_controller"),
            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_TRUE(harness.claimedInterfaces("hold_controller").empty());
  EXPECT_EQ(harness.claimedInterfaces("velocity_controller"), expectedDualClaims("velocity"));
  EXPECT_EQ(panda1->activeControlMode(), ControlMode::JointVelocity);
  EXPECT_EQ(panda2->activeControlMode(), ControlMode::JointVelocity);
  EXPECT_TRUE(commandVelocityEquals(*panda1, {}));
  EXPECT_TRUE(commandVelocityEquals(*panda2, {}));

  const std::array<double, kJointCount> panda1_command{0.0004, -0.0003, 0.0002, -0.0001,
                                                       0.0004, -0.0002, 0.0001};
  const std::array<double, kJointCount> panda2_command{-0.0002, 0.0001, -0.0004, 0.0003,
                                                       -0.0001, 0.0002, -0.0003};
  const std::array<double, kJointCount> zero{};

  harness.setVelocityArmEnabled(1, true);
  harness.publishVelocity(1, panda1_command);
  ASSERT_TRUE(harness.pumpUntil([&]() { return commandVelocityEquals(*panda1, panda1_command); }));
  EXPECT_TRUE(commandVelocityEquals(*panda2, zero));

  harness.setVelocityArmEnabled(1, false);
  harness.setVelocityArmEnabled(2, true);
  harness.publishVelocity(2, panda2_command);
  ASSERT_TRUE(harness.pumpUntil([&]() {
    return commandVelocityEquals(*panda1, zero) && commandVelocityEquals(*panda2, panda2_command);
  }));

  harness.setVelocityArmEnabled(1, true);
  harness.publishVelocity(1, panda1_command);
  harness.publishVelocity(2, panda2_command);
  ASSERT_TRUE(harness.pumpUntil([&]() {
    return commandVelocityEquals(*panda1, panda1_command) &&
           commandVelocityEquals(*panda2, panda2_command);
  }));

  ASSERT_TRUE(harness.pumpUntil([&]() {
    return commandVelocityEquals(*panda1, zero) && commandVelocityEquals(*panda2, zero);
  }));

  ASSERT_EQ(harness.switchControllers({"hold_controller"}, {"velocity_controller"}),
            return_type::OK);
  EXPECT_TRUE(harness.claimedInterfaces("velocity_controller").empty());
  EXPECT_EQ(harness.claimedInterfaces("hold_controller"), expectedDualClaims("effort"));
  EXPECT_EQ(panda1->activeControlMode(), ControlMode::JointTorque);
  EXPECT_EQ(panda2->activeControlMode(), ControlMode::JointTorque);
  ASSERT_EQ(harness.cycle(), return_type::OK);
  EXPECT_TRUE(allZero(panda1->lastCommand().efforts));
  EXPECT_TRUE(allZero(panda2->lastCommand().efforts));

  ASSERT_EQ(harness.switchControllers({}, {"hold_controller"}), return_type::OK);
  EXPECT_TRUE(harness.claimedInterfaces("hold_controller").empty());
  EXPECT_EQ(panda1->activeControlMode(), ControlMode::None);
  EXPECT_EQ(panda2->activeControlMode(), ControlMode::None);
  const auto interfaces = harness.hardwareInterfaces();
  EXPECT_EQ(
      std::count_if(interfaces->command_interfaces.begin(), interfaces->command_interfaces.end(),
                    [](const auto& interface) { return interface.is_claimed; }),
      0);

  const std::vector<std::pair<ControlMode, bool>> expected_transitions{
      {ControlMode::JointTorque, true},
      {ControlMode::JointVelocity, true},
      {ControlMode::JointTorque, true},
      {ControlMode::None, true},
  };
  expectOrderedSafeModeTransitions(*panda1, expected_transitions);
  expectOrderedSafeModeTransitions(*panda2, expected_transitions);

  harness.shutdown();
  ASSERT_FALSE(panda1->events().empty());
  ASSERT_FALSE(panda2->events().empty());
  EXPECT_EQ(panda1->events().back().kind, BackendEventKind::Stop);
  EXPECT_EQ(panda2->events().back().kind, BackendEventKind::Stop);
  EXPECT_EQ(panda1->activeControlMode(), ControlMode::None);
  EXPECT_EQ(panda2->activeControlMode(), ControlMode::None);
}

TEST_F(ProductionControllerManagerIntegrationTest,
       TestOnlyOneArmControllersProveArmLocalTransactionsAndNoCrossRequest) {
  ManagerHarness harness;
  harness.addOneArmController("invalid_partial_panda1", "panda1", kJointCount - 1);
  const auto panda1_controller = harness.addOneArmController("panda1_only", "panda1");
  const auto panda2_controller = harness.addOneArmController("panda2_only", "panda2");
  const auto panda1 = harness.backend("panda1");
  const auto panda2 = harness.backend("panda2");
  const std::array<double, kJointCount> panda1_command{0.001, 0.002, 0.003, 0.004,
                                                       0.005, 0.006, 0.007};
  const std::array<double, kJointCount> panda2_command{-0.001, -0.002, -0.003, -0.004,
                                                       -0.005, -0.006, -0.007};
  const std::array<double, kJointCount> zero{};
  panda1_controller->setCommand(panda1_command);
  panda2_controller->setCommand(panda2_command);

  const auto panda1_requests_before_invalid = panda1->modeRequestAttempts().size();
  EXPECT_EQ(harness.switchControllers({"invalid_partial_panda1"}, {}), return_type::ERROR);
  EXPECT_EQ(harness.lifecycleId("invalid_partial_panda1"),
            lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_TRUE(harness.claimedInterfaces("invalid_partial_panda1").empty());
  EXPECT_EQ(panda1->modeRequestAttempts().size(), panda1_requests_before_invalid);
  EXPECT_EQ(panda1->activeControlMode(), ControlMode::None);
  EXPECT_EQ(panda2->activeControlMode(), ControlMode::None);

  ASSERT_EQ(harness.switchControllers({"panda1_only"}, {}), return_type::OK);
  EXPECT_EQ(harness.claimedInterfaces("panda1_only"), expectedClaims("panda1", "velocity"));
  EXPECT_EQ(panda1->activeControlMode(), ControlMode::JointVelocity);
  EXPECT_EQ(panda2->activeControlMode(), ControlMode::None);
  EXPECT_TRUE(panda2->modeRequestAttempts().empty());
  ASSERT_EQ(harness.cycle(), return_type::OK);
  EXPECT_TRUE(commandVelocityEquals(*panda1, panda1_command));
  EXPECT_TRUE(commandVelocityEquals(*panda2, zero));

  const auto panda1_requests_before_stale = panda1->modeRequestAttempts().size();
  EXPECT_EQ(harness.switchControllers({"panda1_only"}, {}), return_type::ERROR);
  EXPECT_EQ(harness.claimedInterfaces("panda1_only"), expectedClaims("panda1", "velocity"));
  EXPECT_EQ(panda1->modeRequestAttempts().size(), panda1_requests_before_stale);
  EXPECT_EQ(panda2->activeControlMode(), ControlMode::None);

  ASSERT_EQ(harness.switchControllers({"panda2_only"}, {"panda1_only"}), return_type::OK);
  EXPECT_TRUE(harness.claimedInterfaces("panda1_only").empty());
  EXPECT_EQ(harness.claimedInterfaces("panda2_only"), expectedClaims("panda2", "velocity"));
  EXPECT_EQ(panda1->activeControlMode(), ControlMode::None);
  EXPECT_EQ(panda2->activeControlMode(), ControlMode::JointVelocity);
  ASSERT_EQ(harness.cycle(), return_type::OK);
  EXPECT_TRUE(commandVelocityEquals(*panda1, zero));
  EXPECT_TRUE(commandVelocityEquals(*panda2, panda2_command));

  ASSERT_EQ(harness.switchControllers({}, {"panda2_only"}), return_type::OK);
  EXPECT_TRUE(harness.claimedInterfaces("panda2_only").empty());
  EXPECT_EQ(panda1->activeControlMode(), ControlMode::None);
  EXPECT_EQ(panda2->activeControlMode(), ControlMode::None);
}

TEST_F(ProductionControllerManagerIntegrationTest,
       HardwareAndControllerActivationFailuresRollbackClaimsAndBothArmsSafe) {
  {
    ManagerHarness harness;
    const auto failing_controller =
        harness.addOneArmController("failing_activation", "panda1", kJointCount, true);
    const auto panda1 = harness.backend("panda1");
    const auto panda2 = harness.backend("panda2");

    EXPECT_EQ(harness.switchControllers({"failing_activation"}, {}), return_type::ERROR);
    EXPECT_EQ(failing_controller->activationCount(), 1U);
    EXPECT_EQ(failing_controller->deactivationCount(), 0U);
    EXPECT_EQ(harness.lifecycleId("failing_activation"),
              lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_TRUE(harness.claimedInterfaces("failing_activation").empty());
    EXPECT_EQ(panda1->activeControlMode(), ControlMode::None);
    EXPECT_EQ(panda2->activeControlMode(), ControlMode::None);
    EXPECT_FALSE(harness.productionHardware().globalFaultDiagnostic().latched());
    EXPECT_TRUE(isSafeSnapshot(panda1->lastCommand(), panda1->state()));
    EXPECT_TRUE(isSafeSnapshot(panda2->lastCommand(), panda2->state()));
    expectOrderedSafeModeTransitions(
        *panda1, {{ControlMode::JointVelocity, true}, {ControlMode::None, true}});
    EXPECT_TRUE(panda2->modeRequestAttempts().empty());
  }

  {
    ManagerHarness harness;
    harness.loadAndConfigureProductionControllers();
    const auto panda1 = harness.backend("panda1");
    const auto panda2 = harness.backend("panda2");
    panda2->rejectNonNoneMode(true);

    EXPECT_EQ(harness.switchControllers({"hold_controller"}, {}), return_type::ERROR);
    EXPECT_EQ(harness.lifecycleId("hold_controller"),
              lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_TRUE(harness.claimedInterfaces("hold_controller").empty());
    EXPECT_EQ(panda1->activeControlMode(), ControlMode::None);
    EXPECT_EQ(panda2->activeControlMode(), ControlMode::None);
    const auto diagnostic = harness.productionHardware().globalFaultDiagnostic();
    EXPECT_EQ(diagnostic.origin_arm_slot, 2U);
    EXPECT_EQ(diagnostic.cause, GlobalFaultCause::ModeRequest);
    EXPECT_EQ(diagnostic.unsafe_safe_publish_mask, 0U);
    EXPECT_EQ(diagnostic.unsafe_none_request_mask, 0U);
    ASSERT_GE(panda1->modeRequestAttempts().size(), 2U);
    EXPECT_EQ(panda1->modeRequestAttempts().front(), ControlMode::JointTorque);
    EXPECT_EQ(panda1->modeRequestAttempts().back(), ControlMode::None);
    ASSERT_GE(panda2->modeRequestAttempts().size(), 2U);
    EXPECT_EQ(panda2->modeRequestAttempts().front(), ControlMode::JointTorque);
    EXPECT_EQ(panda2->modeRequestAttempts().back(), ControlMode::None);
  }
}

TEST_F(ProductionControllerManagerIntegrationTest,
       SafeSnapshotAndActiveReadFailuresRemainObservableAndStopBothArms) {
  {
    ManagerHarness harness;
    harness.loadAndConfigureProductionControllers();
    const auto panda1 = harness.backend("panda1");
    const auto panda2 = harness.backend("panda2");
    const auto panda2_publish_attempts_before = panda2->publishAttemptCount();
    panda2->rejectCommandPublish(true);

    EXPECT_EQ(harness.switchControllers({"velocity_controller"}, {}), return_type::ERROR);
    EXPECT_EQ(harness.lifecycleId("velocity_controller"),
              lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_TRUE(harness.claimedInterfaces("velocity_controller").empty());
    EXPECT_EQ(panda1->activeControlMode(), ControlMode::None);
    EXPECT_EQ(panda2->activeControlMode(), ControlMode::None);
    const auto diagnostic = harness.productionHardware().globalFaultDiagnostic();
    EXPECT_EQ(diagnostic.origin_arm_slot, 2U);
    EXPECT_EQ(diagnostic.cause, GlobalFaultCause::CommandPublish);
    EXPECT_EQ(diagnostic.unsafe_safe_publish_mask, 0b10U);
    EXPECT_EQ(diagnostic.unsafe_none_request_mask, 0U);
    EXPECT_GE(panda2->publishAttemptCount(), panda2_publish_attempts_before + 2U);
    EXPECT_GE(panda2->rejectedCommandCount(), 2U);
    EXPECT_TRUE(isSafeSnapshot(panda1->lastCommand(), panda1->state()));

    const auto interfaces = harness.hardwareInterfaces();
    EXPECT_EQ(
        std::count_if(interfaces->command_interfaces.begin(), interfaces->command_interfaces.end(),
                      [](const auto& interface) { return interface.is_claimed; }),
        0);
    panda2->rejectCommandPublish(false);
  }

  {
    ManagerHarness harness;
    harness.loadAndConfigureProductionControllers();
    const auto panda1 = harness.backend("panda1");
    const auto panda2 = harness.backend("panda2");
    ASSERT_EQ(harness.switchControllers({"hold_controller"}, {}), return_type::OK);
    ASSERT_EQ(harness.lifecycleId("hold_controller"),
              lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    panda1->failNextRead();

    (void)harness.cycle();
    EXPECT_EQ(panda1->activeControlMode(), ControlMode::None);
    EXPECT_EQ(panda2->activeControlMode(), ControlMode::None);
    const auto diagnostic = harness.productionHardware().globalFaultDiagnostic();
    EXPECT_EQ(diagnostic.origin_arm_slot, 1U);
    EXPECT_EQ(diagnostic.cause, GlobalFaultCause::ReadFailure);
    EXPECT_EQ(diagnostic.unsafe_safe_publish_mask, 0U);
    EXPECT_EQ(diagnostic.unsafe_none_request_mask, 0U);
    EXPECT_TRUE(isSafeSnapshot(panda1->lastCommand(), panda1->state()));
    EXPECT_TRUE(isSafeSnapshot(panda2->lastCommand(), panda2->state()));
    EXPECT_EQ(harness.lifecycleId("hold_controller"),
              lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    // Jazzy marks the controller inactive before its automatic stop-mode switch, but the hardware
    // interfaces are already unavailable, so those loans remain attached until unload.
    EXPECT_EQ(harness.claimedInterfaces("hold_controller"), expectedDualClaims("effort"));
    EXPECT_EQ(harness.unloadController("hold_controller"), return_type::OK);
    const auto interfaces = harness.hardwareInterfaces();
    EXPECT_EQ(
        std::count_if(interfaces->command_interfaces.begin(), interfaces->command_interfaces.end(),
                      [](const auto& interface) { return interface.is_claimed; }),
        0);
    harness.shutdown();
    EXPECT_GE(panda1->stopCount(), 1U);
    EXPECT_GE(panda2->stopCount(), 1U);
  }
}

TEST_F(ProductionControllerManagerIntegrationTest,
       ActualFrankaBroadcastersPublishArmLocalOneArmDataWithoutMotion) {
  expectActualBroadcasters(1);
}

TEST_F(ProductionControllerManagerIntegrationTest,
       ActualFrankaBroadcastersPublishUniqueDualArmDataWithoutMotion) {
  expectActualBroadcasters(2);
}

TEST_F(ProductionControllerManagerIntegrationTest,
       ActualManagerRejectsBroadcasterWhoseRequiredArmInterfaceIsUnavailable) {
  test_support::OfflineControllerManagerHarness harness(2);
  harness.loadAndConfigure("missing_arm_state_broadcaster",
                           "franka_robot_state_broadcaster/FrankaRobotStateBroadcaster");
  EXPECT_EQ(harness.lifecycleId("missing_arm_state_broadcaster"),
            lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(harness.switchControllers({"missing_arm_state_broadcaster"}, {}), return_type::ERROR);
  EXPECT_NE(harness.lifecycleId("missing_arm_state_broadcaster"),
            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_TRUE(harness.claimedInterfaces("missing_arm_state_broadcaster").empty());
  const auto interfaces = harness.hardwareInterfaces();
  EXPECT_EQ(
      std::count_if(interfaces->command_interfaces.begin(), interfaces->command_interfaces.end(),
                    [](const auto& interface) { return interface.is_claimed; }),
      0);
  EXPECT_EQ(harness.unloadController("missing_arm_state_broadcaster"), return_type::OK);
  harness.deactivateHardware();
  for (size_t arm = 1; arm <= 2; ++arm) {
    const auto backend = harness.backend("panda" + std::to_string(arm));
    EXPECT_EQ(backend->acceptedNonNoneModeRequestCount(), 0U);
    EXPECT_EQ(backend->rejectedNonNoneModeRequestCount(), 0U);
    EXPECT_EQ(backend->acceptedUnsafeSnapshotCount(), 0U);
    EXPECT_EQ(backend->acceptedSafeSnapshotCount(), backend->acceptedCommandCount());
  }
  harness.shutdown();
}

TEST_F(ProductionControllerManagerIntegrationTest,
       F10aHoldControllerSurvivesRepeatedDefaultDeactivateReactivateCyclesViaOwnerThreadHandoff) {
  // F-10a offline reproduction. This is the exact live-hardware session's controller
  // (dual_arm_joint_hold_controller, loaded here as "hold_controller" by loadReviewedControllers())
  // and the exact broken CLI path: OfflineControllerManagerHarness::switchControllers() runs
  // switch_controller() on its own switch_worker_thread_ while this test thread keeps driving
  // read()/update()/write() through cycle() -- the same live two-thread topology as a real
  // ros2_control_node's service thread vs its RT update thread. `ros2 control switch_controllers
  // --deactivate` (no --switch-asap) resolves to activate_asap=false, reproduced explicitly below
  // on every iteration; SESSION_LOG.md (phase10_session1_2026-08-27) recorded exactly this failing
  // deterministically (2/2) before this fix, with the controller left ACTIVE and holding (safe)
  // both times.
  test_support::OfflineControllerManagerHarness harness(test_support::kArmCount);
  harness.loadReviewedControllers();
  harness.loadBroadcasters(test_support::kArmCount);
  const auto panda1 = harness.backend("panda1");
  const auto panda2 = harness.backend("panda2");

  // Initial activation mirrors the session log's spawner --switch-asap path (activate_asap=true),
  // which already worked before this fix.
  ASSERT_EQ(harness.switchControllers({"hold_controller"}, {}, true), return_type::OK);
  EXPECT_EQ(harness.lifecycleId("hold_controller"),
            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_EQ(panda1->activeControlMode(), ControlMode::JointTorque);
  EXPECT_EQ(panda2->activeControlMode(), ControlMode::JointTorque);

  constexpr int kCycleCount = 20;
  for (int cycle_index = 0; cycle_index < kCycleCount; ++cycle_index) {
    // The exact production CLI path F-10a broke: a plain `ros2 control switch_controllers
    // --deactivate`, activate_asap=false, resolved on the switch-worker thread while cycle() kept
    // the RT loop running on this thread.
    ASSERT_EQ(harness.switchControllers({}, {"hold_controller"}, false), return_type::OK)
        << "deactivate cycle " << cycle_index;
    EXPECT_EQ(harness.lifecycleId("hold_controller"),
              lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_TRUE(harness.claimedInterfaces("hold_controller").empty());
    EXPECT_EQ(panda1->activeControlMode(), ControlMode::None);
    EXPECT_EQ(panda2->activeControlMode(), ControlMode::None);
    EXPECT_FALSE(harness.productionHardware().globalFaultDiagnostic().latched());

    ASSERT_EQ(harness.switchControllers({"hold_controller"}, {}, false), return_type::OK)
        << "reactivate cycle " << cycle_index;
    EXPECT_EQ(harness.lifecycleId("hold_controller"),
              lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    EXPECT_FALSE(harness.claimedInterfaces("hold_controller").empty());
    EXPECT_EQ(panda1->activeControlMode(), ControlMode::JointTorque);
    EXPECT_EQ(panda2->activeControlMode(), ControlMode::JointTorque);
    EXPECT_FALSE(harness.productionHardware().globalFaultDiagnostic().latched());
  }

  EXPECT_FALSE(harness.productionHardware().globalFaultDiagnostic().latched());
  harness.deactivateAndUnloadAll();
  harness.shutdown();
}

TEST_F(ProductionControllerManagerIntegrationTest,
       F10cAdversarialOverlapSoakKeepsImpedanceCommandsFiniteAcrossFiftyOffOwnerCycles) {
  // F-10c regression: the exact live two-thread topology F-10a/b/c are all about (this test
  // thread driving read()/update()/write() via cycle() while switch_controller() resolves
  // off-owner, activate_asap=false, on its own std::async thread -- see switchControllers()
  // below) run 50+ times back to back, with a *third* thread concurrently firing the impedance
  // controller's enable service and joint-target topic throughout every switch, landing
  // requests at arbitrary points relative to activation/deactivation. This is the design's own
  // "activate and deactivate rapidly ... with the owner thread's update() running concurrently"
  // regression (§5) plus concurrent non-owner-thread ROS endpoint traffic layered on top -- the
  // one combination most likely to reproduce a torn arm.internal_target/arm.filtered_velocity
  // read (the F-10c finding) if bindArmInterfaces()/captureActivationState() or the zero-effort
  // write ever again ran off the owner thread. Every command this soak observes must stay
  // finite: a torn read of Arm's non-atomic fields is exactly the kind of defect that produces a
  // NaN/Inf effort, not a merely-wrong-but-finite one, since the fields are freshly overwritten
  // mid-read rather than logically inconsistent.
  ManagerHarness harness;
  harness.loadAndConfigureProductionControllers();
  const auto panda1 = harness.backend("panda1");
  const auto panda2 = harness.backend("panda2");

  ASSERT_EQ(harness.switchControllers({"impedance_controller"}, {}), return_type::OK);

  std::atomic<bool> stop_traffic{false};
  std::atomic<size_t> traffic_iterations{0};
  auto adversarial_traffic = std::async(std::launch::async, [&]() {
    std::mt19937 engine(20260827U);
    std::uniform_real_distribution<double> offset(-0.15, 0.15);
    while (!stop_traffic.load(std::memory_order_acquire)) {
      try {
        harness.setImpedanceArmEnabled(1, true);
        std::array<double, kJointCount> target{};
        for (auto& value : target) {
          value = offset(engine);
        }
        harness.publishImpedanceTarget(1, target);
      } catch (const std::exception&) {
        // The service/topic can be transiently unavailable while impedance_controller is
        // deactivated (this iteration's request simply lands as ControllerInactive on the RT
        // side, or the client/publisher setup itself races a not-yet-reloaded endpoint) -- both
        // are expected, non-fatal outcomes of firing traffic without regard to lifecycle state.
      }
      ++traffic_iterations;
    }
  });

  constexpr int kSoakCycles = 50;
  for (int cycle_index = 0; cycle_index < kSoakCycles; ++cycle_index) {
    ASSERT_EQ(harness.switchControllers({"hold_controller"}, {"impedance_controller"},
                                        /*activate_asap=*/false),
              return_type::OK)
        << "cycle " << cycle_index << " -> hold_controller";
    for (const double value : panda1->lastCommand().efforts) {
      ASSERT_TRUE(std::isfinite(value)) << "cycle " << cycle_index << " panda1 hold effort";
    }
    for (const double value : panda2->lastCommand().efforts) {
      ASSERT_TRUE(std::isfinite(value)) << "cycle " << cycle_index << " panda2 hold effort";
    }
    EXPECT_FALSE(harness.productionHardware().globalFaultDiagnostic().latched())
        << "cycle " << cycle_index << " -> hold_controller";

    ASSERT_EQ(harness.switchControllers({"impedance_controller"}, {"hold_controller"},
                                        /*activate_asap=*/false),
              return_type::OK)
        << "cycle " << cycle_index << " -> impedance_controller";
    // Run a handful of ordinary cycles with the adversarial traffic thread still live, so this
    // iteration's window overlaps captureActivationState()'s first-cycle capture (this cycle)
    // and several steady-state updates, not just the activation instant.
    for (int settle = 0; settle < 5; ++settle) {
      ASSERT_EQ(harness.cycle(), return_type::OK)
          << "cycle " << cycle_index << " settle " << settle;
      for (const double value : panda1->lastCommand().efforts) {
        ASSERT_TRUE(std::isfinite(value))
            << "cycle " << cycle_index << " settle " << settle << " panda1 impedance effort";
      }
      for (const double value : panda2->lastCommand().efforts) {
        ASSERT_TRUE(std::isfinite(value))
            << "cycle " << cycle_index << " settle " << settle << " panda2 impedance effort";
      }
    }
    EXPECT_FALSE(harness.productionHardware().globalFaultDiagnostic().latched())
        << "cycle " << cycle_index << " -> impedance_controller";
  }

  stop_traffic.store(true, std::memory_order_release);
  adversarial_traffic.get();
  EXPECT_GT(traffic_iterations.load(), 0U);

  EXPECT_FALSE(harness.productionHardware().globalFaultDiagnostic().latched());
  ASSERT_EQ(harness.switchControllers({}, {"impedance_controller"}), return_type::OK);
  harness.shutdown();
}

TEST_F(ProductionControllerManagerIntegrationTest,
       BroadcastersRejectUnsafeFrequencyAndIncompleteSemanticLoans) {
  size_t invalid_index = 0;
  for (const int64_t frequency : std::array<int64_t, 5>{
           std::numeric_limits<int64_t>::min(), -1, 0, 1001, std::numeric_limits<int64_t>::max()}) {
    auto state = makeDirectBroadcaster<InspectableStateBroadcaster>(
        "invalid_state_frequency_" + std::to_string(invalid_index), frequency);
    auto model = makeDirectBroadcaster<InspectableModelBroadcaster>(
        "invalid_model_frequency_" + std::to_string(invalid_index), frequency);
    EXPECT_FALSE(controller_interface::configure_succeeds(state));
    EXPECT_FALSE(controller_interface::configure_succeeds(model));
    EXPECT_FALSE(state->hasRealtimePublisher());
    EXPECT_FALSE(state->hasPublisher());
    EXPECT_FALSE(state->hasSemanticComponent());
    EXPECT_FALSE(model->hasRealtimePublisher());
    EXPECT_FALSE(model->hasPublisher());
    EXPECT_FALSE(model->hasSemanticComponent());
    ++invalid_index;
  }

  size_t valid_index = 0;
  for (const auto& [frequency, expected_interval] : std::array<std::pair<int64_t, int64_t>, 3>{
           {{1, 1'000'000'000LL}, {30, 33'333'333LL}, {1000, 1'000'000LL}}}) {
    auto state = makeDirectBroadcaster<InspectableStateBroadcaster>(
        "valid_state_frequency_" + std::to_string(valid_index), frequency);
    auto model = makeDirectBroadcaster<InspectableModelBroadcaster>(
        "valid_model_frequency_" + std::to_string(valid_index), frequency);
    ASSERT_TRUE(controller_interface::configure_succeeds(state));
    ASSERT_TRUE(controller_interface::configure_succeeds(model));
    EXPECT_EQ(state->publishIntervalNanoseconds(), expected_interval);
    EXPECT_EQ(model->publishIntervalNanoseconds(), expected_interval);
    ASSERT_TRUE(controller_interface::cleanup_succeeds(state));
    ASSERT_TRUE(controller_interface::cleanup_succeeds(model));
    EXPECT_FALSE(state->hasRealtimePublisher());
    EXPECT_FALSE(state->hasPublisher());
    EXPECT_FALSE(state->hasSemanticComponent());
    EXPECT_FALSE(model->hasRealtimePublisher());
    EXPECT_FALSE(model->hasPublisher());
    EXPECT_FALSE(model->hasSemanticComponent());
    ++valid_index;
  }
  EXPECT_EQ(1'000'000'000LL % 30LL, 10LL);

  auto incomplete_state =
      makeDirectBroadcaster<InspectableStateBroadcaster>("incomplete_state", 30);
  auto incomplete_model =
      makeDirectBroadcaster<InspectableModelBroadcaster>("incomplete_model", 30);
  ASSERT_TRUE(controller_interface::configure_succeeds(incomplete_state));
  ASSERT_TRUE(controller_interface::configure_succeeds(incomplete_model));
  DirectBroadcasterInterfaces interfaces;
  interfaces.assign(*incomplete_model, false);
  EXPECT_EQ(incomplete_state->get_node()->activate().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_EQ(incomplete_model->get_node()->activate().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_FALSE(incomplete_state->semanticReadSucceeds());
  EXPECT_FALSE(incomplete_model->semanticReadSucceeds());

  auto wrong_state = makeDirectBroadcaster<InspectableStateBroadcaster>("wrong_typed_state", 30);
  auto wrong_model = makeDirectBroadcaster<InspectableModelBroadcaster>("wrong_typed_model", 30);
  ASSERT_TRUE(controller_interface::configure_succeeds(wrong_state));
  ASSERT_TRUE(controller_interface::configure_succeeds(wrong_model));
  interfaces.assignWrongTypedState(*wrong_state);
  interfaces.assignWrongTypedModel(*wrong_model);
  EXPECT_EQ(wrong_state->get_node()->activate().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_EQ(wrong_model->get_node()->activate().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_FALSE(wrong_state->semanticReadSucceeds());
  EXPECT_FALSE(wrong_model->semanticReadSucceeds());
  EXPECT_TRUE(wrong_model->modelGetterThrows());

  auto reordered = makeDirectBroadcaster<InspectableModelBroadcaster>("reordered_model_loans", 30);
  ASSERT_TRUE(controller_interface::configure_succeeds(reordered));
  interfaces.assignModelReordered(*reordered);
  ASSERT_TRUE(controller_interface::activate_succeeds(reordered));
  EXPECT_TRUE(reordered->semanticReadSucceeds());

  auto duplicate_state =
      makeDirectBroadcaster<InspectableStateBroadcaster>("duplicate_state_loans", 30);
  auto duplicate_model =
      makeDirectBroadcaster<InspectableModelBroadcaster>("duplicate_model_loans", 30);
  ASSERT_TRUE(controller_interface::configure_succeeds(duplicate_state));
  ASSERT_TRUE(controller_interface::configure_succeeds(duplicate_model));
  interfaces.assignDuplicateState(*duplicate_state);
  interfaces.assignDuplicateModel(*duplicate_model);
  EXPECT_EQ(duplicate_state->get_node()->activate().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_EQ(duplicate_model->get_node()->activate().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  auto null_state = makeDirectBroadcaster<InspectableStateBroadcaster>("null_state_loan", 30);
  auto null_model = makeDirectBroadcaster<InspectableModelBroadcaster>("null_model_loan", 30);
  ASSERT_TRUE(controller_interface::configure_succeeds(null_state));
  ASSERT_TRUE(controller_interface::configure_succeeds(null_model));
  interfaces.setStatePointer(nullptr);
  interfaces.setModelPointer(nullptr);
  interfaces.assign(*null_state);
  interfaces.assign(*null_model, true);
  EXPECT_EQ(null_state->get_node()->activate().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_EQ(null_model->get_node()->activate().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_FALSE(null_state->semanticReadSucceeds());
  EXPECT_FALSE(null_model->semanticReadSucceeds());
}

TEST_F(ProductionControllerManagerIntegrationTest,
       StateBroadcasterContentionBackwardTimeAndReleaseAreBounded) {
  auto broadcaster =
      makeDirectBroadcaster<InspectableStateBroadcaster>("direct_state_broadcaster", 1000);
  ASSERT_TRUE(controller_interface::configure_succeeds(broadcaster));
  DirectBroadcasterInterfaces interfaces;
  interfaces.assign(*broadcaster);
  ASSERT_TRUE(controller_interface::activate_succeeds(broadcaster));
  ASSERT_EQ(broadcaster->baseLoanCount(), 1U);

  EXPECT_FALSE(broadcaster->cadenceInitialized());
  constexpr int64_t first_update = 5'000'000'000LL;
  EXPECT_EQ(broadcaster->update(rclcpp::Time(first_update, RCL_SYSTEM_TIME),
                                rclcpp::Duration(kCyclePeriod)),
            return_type::OK);
  EXPECT_TRUE(broadcaster->cadenceInitialized());
  EXPECT_EQ(broadcaster->lastPublishNanoseconds(), first_update);
  EXPECT_EQ(broadcaster->lockedMessageStampNanoseconds(), first_update);
  EXPECT_EQ(
      broadcaster->update(rclcpp::Time(first_update + broadcaster->publishIntervalNanoseconds() - 1,
                                       RCL_SYSTEM_TIME),
                          rclcpp::Duration(kCyclePeriod)),
      return_type::OK);
  EXPECT_EQ(broadcaster->lastPublishNanoseconds(), first_update);

  {
    HeldPublisherLock lock(*broadcaster);
    EXPECT_EQ(
        broadcaster->update(
            rclcpp::Time(first_update + broadcaster->publishIntervalNanoseconds(), RCL_SYSTEM_TIME),
            rclcpp::Duration(kCyclePeriod)),
        return_type::OK);
    EXPECT_EQ(broadcaster->lastPublishNanoseconds(), first_update);
  }

  const auto published = first_update + broadcaster->publishIntervalNanoseconds();
  ASSERT_TRUE(waitForPublicationAt(*broadcaster, published))
      << "state broadcaster did not publish after bounded post-contention retries";
  EXPECT_EQ(broadcaster->lastPublishNanoseconds(), published);
  EXPECT_EQ(broadcaster->lockedMessageStampNanoseconds(), published);
  const auto large_forward = published + 20 * broadcaster->publishIntervalNanoseconds() + 17;
  ASSERT_TRUE(waitForPublicationAt(*broadcaster, large_forward))
      << "state broadcaster did not publish after a large forward time jump";
  EXPECT_EQ(broadcaster->lastPublishNanoseconds(), large_forward);
  EXPECT_EQ(broadcaster->lockedMessageStampNanoseconds(), large_forward);
  const auto backward = first_update + 123;
  ASSERT_TRUE(waitForPublicationAt(*broadcaster, backward))
      << "state broadcaster did not publish after a backward time jump";
  EXPECT_EQ(broadcaster->lastPublishNanoseconds(), backward);
  EXPECT_EQ(broadcaster->lockedMessageStampNanoseconds(), backward);
  ASSERT_TRUE(
      waitForPublicationAt(*broadcaster, backward + broadcaster->publishIntervalNanoseconds()))
      << "state broadcaster did not publish one interval after a backward time jump";
  EXPECT_EQ(broadcaster->lastPublishNanoseconds(),
            backward + broadcaster->publishIntervalNanoseconds());

  ASSERT_TRUE(waitForPublisherRoundTrip(*broadcaster));
  const auto before_failed_read = broadcaster->lastPublishNanoseconds();
  {
    HeldInterfaceLock lock(interfaces.stateMutex());
    ASSERT_TRUE(waitForUpdateResultAt(
        *broadcaster, before_failed_read + broadcaster->publishIntervalNanoseconds(),
        return_type::ERROR))
        << "state broadcaster did not reach the expected semantic-read error";
    EXPECT_FALSE(broadcaster->semanticReadSucceeds());
  }
  EXPECT_EQ(broadcaster->lastPublishNanoseconds(), before_failed_read);
  ASSERT_TRUE(waitForPublisherRoundTrip(*broadcaster));
  interfaces.setStatePointer(nullptr);
  EXPECT_FALSE(broadcaster->semanticReadSucceeds());
  interfaces.setStatePointer(&interfaces.state());
  EXPECT_TRUE(broadcaster->semanticReadSucceeds());

  EXPECT_TRUE(broadcaster->semanticReadSucceeds());
  EXPECT_EQ(broadcaster->baseLoanCount(), 1U);
  broadcaster->release_interfaces();
  EXPECT_EQ(broadcaster->baseLoanCount(), 0U);
  EXPECT_FALSE(broadcaster->semanticReadSucceeds());
  EXPECT_FALSE(broadcaster->cadenceInitialized());
  EXPECT_TRUE(waitForUpdateResultAt(*broadcaster, 11'000'000'000LL, return_type::ERROR))
      << "state broadcaster did not reach the expected post-release error";
  EXPECT_FALSE(broadcaster->cadenceInitialized());
  ASSERT_TRUE(controller_interface::deactivate_succeeds(broadcaster));
  interfaces.assign(*broadcaster);
  ASSERT_TRUE(controller_interface::activate_succeeds(broadcaster));
  EXPECT_TRUE(broadcaster->semanticReadSucceeds());
  ASSERT_TRUE(controller_interface::deactivate_succeeds(broadcaster));
  broadcaster->release_interfaces();
}

TEST_F(ProductionControllerManagerIntegrationTest,
       ModelBroadcasterContentionBackwardTimeNullPublisherAndReleaseAreBounded) {
  auto broadcaster =
      makeDirectBroadcaster<InspectableModelBroadcaster>("direct_model_broadcaster", 1000);
  ASSERT_TRUE(controller_interface::configure_succeeds(broadcaster));
  DirectBroadcasterInterfaces interfaces;
  interfaces.assign(*broadcaster, true);
  ASSERT_TRUE(controller_interface::activate_succeeds(broadcaster));
  ASSERT_EQ(broadcaster->baseLoanCount(), 2U);

  EXPECT_FALSE(broadcaster->cadenceInitialized());
  constexpr int64_t first_update = 7'000'000'000LL;
  EXPECT_EQ(broadcaster->update(rclcpp::Time(first_update, RCL_SYSTEM_TIME),
                                rclcpp::Duration(kCyclePeriod)),
            return_type::OK);
  EXPECT_EQ(broadcaster->lockedMessageStampNanoseconds(), first_update);
  EXPECT_EQ(
      broadcaster->update(rclcpp::Time(first_update + broadcaster->publishIntervalNanoseconds() - 1,
                                       RCL_SYSTEM_TIME),
                          rclcpp::Duration(kCyclePeriod)),
      return_type::OK);
  {
    HeldPublisherLock lock(*broadcaster);
    EXPECT_EQ(
        broadcaster->update(
            rclcpp::Time(first_update + broadcaster->publishIntervalNanoseconds(), RCL_SYSTEM_TIME),
            rclcpp::Duration(kCyclePeriod)),
        return_type::OK);
    EXPECT_EQ(broadcaster->lastPublishNanoseconds(), first_update);
  }

  const auto published = first_update + broadcaster->publishIntervalNanoseconds();
  ASSERT_TRUE(waitForPublicationAt(*broadcaster, published))
      << "model broadcaster did not publish after bounded post-contention retries";
  EXPECT_EQ(broadcaster->lastPublishNanoseconds(), published);
  EXPECT_EQ(broadcaster->lockedMessageStampNanoseconds(), published);
  const auto large_forward = published + 50 * broadcaster->publishIntervalNanoseconds() + 3;
  ASSERT_TRUE(waitForPublicationAt(*broadcaster, large_forward))
      << "model broadcaster did not publish after a large forward time jump";
  EXPECT_EQ(broadcaster->lastPublishNanoseconds(), large_forward);
  EXPECT_EQ(broadcaster->lockedMessageStampNanoseconds(), large_forward);
  const auto backward = first_update + 9;
  ASSERT_TRUE(waitForPublicationAt(*broadcaster, backward))
      << "model broadcaster did not publish after a backward time jump";
  EXPECT_EQ(broadcaster->lastPublishNanoseconds(), backward);
  EXPECT_EQ(broadcaster->lockedMessageStampNanoseconds(), backward);
  const auto after_backward = backward + broadcaster->publishIntervalNanoseconds();
  ASSERT_TRUE(waitForPublicationAt(*broadcaster, after_backward))
      << "model broadcaster did not publish one interval after a backward time jump";
  EXPECT_EQ(broadcaster->lastPublishNanoseconds(), after_backward);
  ASSERT_TRUE(waitForPublisherRoundTrip(*broadcaster));
  {
    // Model is decoded first; contention on state proves a second-read failure clears both caches.
    HeldInterfaceLock lock(interfaces.stateMutex());
    ASSERT_TRUE(waitForUpdateResultAt(*broadcaster,
                                      after_backward + broadcaster->publishIntervalNanoseconds(),
                                      return_type::ERROR))
        << "model broadcaster did not reach the expected paired semantic-read error";
    EXPECT_FALSE(broadcaster->semanticReadSucceeds());
    EXPECT_TRUE(broadcaster->modelGetterThrows());
  }
  ASSERT_TRUE(waitForPublisherRoundTrip(*broadcaster));
  EXPECT_TRUE(broadcaster->semanticReadSucceeds());
  interfaces.setModelPointer(nullptr);
  EXPECT_FALSE(broadcaster->semanticReadSucceeds());
  EXPECT_TRUE(broadcaster->modelGetterThrows());
  interfaces.setModelPointer(interfaces.model());
  EXPECT_TRUE(broadcaster->semanticReadSucceeds());

  EXPECT_TRUE(broadcaster->semanticReadSucceeds());
  EXPECT_EQ(broadcaster->baseLoanCount(), 2U);
  broadcaster->release_interfaces();
  EXPECT_EQ(broadcaster->baseLoanCount(), 0U);
  EXPECT_FALSE(broadcaster->semanticReadSucceeds());
  EXPECT_TRUE(broadcaster->modelGetterThrows());
  EXPECT_TRUE(waitForUpdateResultAt(*broadcaster, 13'000'000'000LL, return_type::ERROR))
      << "model broadcaster did not reach the expected post-release error";
  EXPECT_FALSE(broadcaster->cadenceInitialized());
  ASSERT_TRUE(controller_interface::deactivate_succeeds(broadcaster));
  interfaces.assign(*broadcaster, true);
  ASSERT_TRUE(controller_interface::activate_succeeds(broadcaster));
  EXPECT_TRUE(broadcaster->semanticReadSucceeds());
  ASSERT_TRUE(controller_interface::deactivate_succeeds(broadcaster));
  broadcaster->release_interfaces();
  broadcaster->clearPublisher();
  EXPECT_EQ(broadcaster->update(rclcpp::Time(13'000'000'001LL, RCL_SYSTEM_TIME),
                                rclcpp::Duration(kCyclePeriod)),
            return_type::ERROR);
}

TEST_F(ProductionControllerManagerIntegrationTest,
       ModelBroadcasterContainsUnexpectedModelExceptionAndReleasesPublisherLock) {
  auto broadcaster =
      makeDirectBroadcaster<InspectableModelBroadcaster>("throwing_model_broadcaster", 1000);
  ASSERT_TRUE(controller_interface::configure_succeeds(broadcaster));
  DirectBroadcasterInterfaces interfaces;
  ThrowingModel throwing_model;
  interfaces.assign(*broadcaster, true);
  ASSERT_TRUE(controller_interface::activate_succeeds(broadcaster));

  constexpr int64_t first_update = 12'000'000'000LL;
  ASSERT_EQ(broadcaster->update(rclcpp::Time(first_update, RCL_SYSTEM_TIME),
                                rclcpp::Duration(kCyclePeriod)),
            return_type::OK);
  EXPECT_EQ(broadcaster->lockedMessageStampNanoseconds(), first_update);
  const auto throwing_update = first_update + broadcaster->publishIntervalNanoseconds();
  interfaces.setModelPointer(&throwing_model);
  EXPECT_TRUE(waitForUpdateResultAt(*broadcaster, throwing_update, return_type::ERROR))
      << "model broadcaster did not reach the expected throwing-model error";
  EXPECT_EQ(broadcaster->lastPublishNanoseconds(), first_update);
  ASSERT_TRUE(waitForPublisherRoundTrip(*broadcaster));

  interfaces.setModelPointer(interfaces.model());
  ASSERT_TRUE(waitForPublicationAt(*broadcaster, throwing_update + 1))
      << "model broadcaster did not publish after recovery from an unexpected model exception";
  EXPECT_EQ(broadcaster->lastPublishNanoseconds(), throwing_update + 1);
  ASSERT_TRUE(controller_interface::deactivate_succeeds(broadcaster));
  broadcaster->release_interfaces();
}

TEST_F(ProductionControllerManagerIntegrationTest,
       StateSemanticConversionCopiesEveryPinnedFieldWithoutStaleReuse) {
  auto broadcaster =
      makeDirectBroadcaster<InspectableStateBroadcaster>("state_message_contract", 1000);
  ASSERT_TRUE(controller_interface::configure_succeeds(broadcaster));
  DirectBroadcasterInterfaces interfaces;
  interfaces.assign(*broadcaster);
  ASSERT_TRUE(controller_interface::activate_succeeds(broadcaster));

  auto& state = interfaces.state();
  state.O_ddP_O = {{7.1, -8.2, 9.3}};
  franka_msgs::msg::FrankaState message{};
  message.cartesian_collision.fill(-999.0);
  message.cartesian_contact.fill(-999.0);
  message.q.fill(-999.0);
  message.q_d.fill(-999.0);
  message.dq.fill(-999.0);
  message.dq_d.fill(-999.0);
  message.ddq_d.fill(-999.0);
  message.theta.fill(-999.0);
  message.dtheta.fill(-999.0);
  message.tau_j.fill(-999.0);
  message.dtau_j.fill(-999.0);
  message.tau_j_d.fill(-999.0);
  message.k_f_ext_hat_k.fill(-999.0);
  message.elbow.fill(-999.0);
  message.elbow_d.fill(-999.0);
  message.elbow_c.fill(-999.0);
  message.delbow_c.fill(-999.0);
  message.ddelbow_c.fill(-999.0);
  message.joint_collision.fill(-999.0);
  message.joint_contact.fill(-999.0);
  message.o_f_ext_hat_k.fill(-999.0);
  message.o_dp_ee_d.fill(-999.0);
  message.o_ddp_o.fill(-999.0);
  message.o_dp_ee_c.fill(-999.0);
  message.o_ddp_ee_c.fill(-999.0);
  message.tau_ext_hat_filtered.fill(-999.0);
  message.f_x_cee.fill(-999.0);
  message.i_ee.fill(-999.0);
  message.f_x_cload.fill(-999.0);
  message.i_load.fill(-999.0);
  message.f_x_ctotal.fill(-999.0);
  message.i_total.fill(-999.0);
  message.o_t_ee.fill(-999.0);
  message.o_t_ee_d.fill(-999.0);
  message.o_t_ee_c.fill(-999.0);
  message.f_t_ee.fill(-999.0);
  message.f_t_ne.fill(-999.0);
  message.ne_t_ee.fill(-999.0);
  message.ee_t_k.fill(-999.0);
  message.m_ee = -999.0;
  message.m_load = -999.0;
  message.m_total = -999.0;
  message.time = -999.0;
  message.control_command_success_rate = -999.0;
  message.robot_mode = franka_msgs::msg::FrankaState::ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY;
  std::array<bool, 41> all_errors{};
  all_errors.fill(true);
  state.current_errors = franka::Errors(all_errors);
  state.last_motion_errors = franka::Errors(all_errors);
  ASSERT_TRUE(broadcaster->semanticRead(message));
  EXPECT_EQ(message.cartesian_collision, state.cartesian_collision);
  EXPECT_EQ(message.cartesian_contact, state.cartesian_contact);
  EXPECT_EQ(message.q, state.q);
  EXPECT_EQ(message.q_d, state.q_d);
  EXPECT_EQ(message.dq, state.dq);
  EXPECT_EQ(message.dq_d, state.dq_d);
  EXPECT_EQ(message.ddq_d, state.ddq_d);
  EXPECT_EQ(message.theta, state.theta);
  EXPECT_EQ(message.dtheta, state.dtheta);
  EXPECT_EQ(message.tau_j, state.tau_J);
  EXPECT_EQ(message.dtau_j, state.dtau_J);
  EXPECT_EQ(message.tau_j_d, state.tau_J_d);
  EXPECT_EQ(message.k_f_ext_hat_k, state.K_F_ext_hat_K);
  EXPECT_EQ(message.elbow, state.elbow);
  EXPECT_EQ(message.elbow_d, state.elbow_d);
  EXPECT_EQ(message.elbow_c, state.elbow_c);
  EXPECT_EQ(message.delbow_c, state.delbow_c);
  EXPECT_EQ(message.ddelbow_c, state.ddelbow_c);
  EXPECT_EQ(message.joint_collision, state.joint_collision);
  EXPECT_EQ(message.joint_contact, state.joint_contact);
  EXPECT_EQ(message.o_f_ext_hat_k, state.O_F_ext_hat_K);
  EXPECT_EQ(message.o_dp_ee_d, state.O_dP_EE_d);
  EXPECT_EQ(message.o_ddp_o, state.O_ddP_O);
  EXPECT_EQ(message.o_dp_ee_c, state.O_dP_EE_c);
  EXPECT_EQ(message.o_ddp_ee_c, state.O_ddP_EE_c);
  EXPECT_EQ(message.tau_ext_hat_filtered, state.tau_ext_hat_filtered);
  EXPECT_EQ(message.f_x_cee, state.F_x_Cee);
  EXPECT_EQ(message.i_ee, state.I_ee);
  EXPECT_EQ(message.f_x_cload, state.F_x_Cload);
  EXPECT_EQ(message.i_load, state.I_load);
  EXPECT_EQ(message.f_x_ctotal, state.F_x_Ctotal);
  EXPECT_EQ(message.i_total, state.I_total);
  EXPECT_EQ(message.o_t_ee, state.O_T_EE);
  EXPECT_EQ(message.o_t_ee_d, state.O_T_EE_d);
  EXPECT_EQ(message.o_t_ee_c, state.O_T_EE_c);
  EXPECT_EQ(message.f_t_ee, state.F_T_EE);
  EXPECT_EQ(message.f_t_ne, state.F_T_NE);
  EXPECT_EQ(message.ne_t_ee, state.NE_T_EE);
  EXPECT_EQ(message.ee_t_k, state.EE_T_K);
  EXPECT_EQ(message.m_ee, state.m_ee);
  EXPECT_EQ(message.m_load, state.m_load);
  EXPECT_EQ(message.m_total, state.m_total);
  EXPECT_EQ(message.time, state.time.toSec());
  EXPECT_EQ(message.control_command_success_rate, state.control_command_success_rate);
  EXPECT_EQ(messageErrorFlags(message.current_errors), all_errors);
  EXPECT_EQ(messageErrorFlags(message.last_motion_errors), all_errors);
  for (size_t current = 0; current < 41; ++current) {
    std::array<bool, 41> current_flags{};
    std::array<bool, 41> last_flags{};
    current_flags[current] = true;
    last_flags[40U - current] = true;
    constexpr std::array<size_t, 41> named_error_storage_indices{
        {0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
         21, 22, 27, 28, 29, 30, 31, 23, 32, 24, 25, 26, 33, 34, 35, 36, 37, 38, 39, 40}};
    std::array<bool, 41> current_storage{};
    std::array<bool, 41> last_storage{};
    current_storage[named_error_storage_indices[current]] = true;
    last_storage[named_error_storage_indices[40U - current]] = true;
    state.current_errors = franka::Errors(current_storage);
    state.last_motion_errors = franka::Errors(last_storage);
    state.robot_mode = static_cast<franka::RobotMode>(0xFF);
    ASSERT_TRUE(broadcaster->semanticRead(message));
    EXPECT_EQ(messageErrorFlags(message.current_errors), current_flags);
    EXPECT_EQ(messageErrorFlags(message.last_motion_errors), last_flags);
    EXPECT_EQ(message.o_ddp_o, state.O_ddP_O);
    EXPECT_EQ(message.q, state.q);
    EXPECT_EQ(message.robot_mode, franka_msgs::msg::FrankaState::ROBOT_MODE_OTHER);
  }
}

TEST_F(ProductionControllerManagerIntegrationTest,
       BroadcasterLifecycleCleanupIsIdempotentAndOnlyJoinsOnCleanupOrDestruction) {
  // Pay the process-lifetime ROS/RMW publisher initialization cost before measuring the
  // broadcaster-owned worker. That middleware thread is intentionally not owned by cleanup.
  {
    auto warmup =
        makeDirectBroadcaster<InspectableStateBroadcaster>("lifecycle_thread_warmup", 1000);
    ASSERT_TRUE(controller_interface::configure_succeeds(warmup));
    ASSERT_TRUE(controller_interface::cleanup_succeeds(warmup));
  }
  const auto initial_threads = processThreadCount();
  auto broadcaster =
      makeDirectBroadcaster<InspectableStateBroadcaster>("state_lifecycle_cleanup", 1000);
  ASSERT_TRUE(controller_interface::configure_succeeds(broadcaster));
  EXPECT_TRUE(broadcaster->hasRealtimePublisher());
  EXPECT_TRUE(broadcaster->hasPublisher());
  EXPECT_TRUE(broadcaster->hasSemanticComponent());
  EXPECT_EQ(broadcaster->get_node()->count_publishers("/state_lifecycle_cleanup/robot_state"), 1U);
  EXPECT_GE(processThreadCount(), initial_threads + 1U);

  DirectBroadcasterInterfaces interfaces;
  interfaces.assign(*broadcaster);
  ASSERT_TRUE(controller_interface::activate_succeeds(broadcaster));
  ASSERT_TRUE(controller_interface::deactivate_succeeds(broadcaster));
  EXPECT_TRUE(broadcaster->hasRealtimePublisher());
  EXPECT_EQ(broadcaster->get_node()->count_publishers("/state_lifecycle_cleanup/robot_state"), 1U);

  EXPECT_EQ(broadcaster->on_error(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  EXPECT_TRUE(broadcaster->hasRealtimePublisher());
  EXPECT_TRUE(broadcaster->hasPublisher());
  EXPECT_TRUE(broadcaster->hasSemanticComponent());
  EXPECT_FALSE(broadcaster->semanticReadSucceeds());
  EXPECT_EQ(broadcaster->on_shutdown(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  // Error/shutdown can run on the manager update thread: the endpoint and its worker therefore
  // persist until a later non-RT cleanup, unload, or destructor performs the join/destruction.
  EXPECT_TRUE(broadcaster->hasRealtimePublisher());

  ASSERT_TRUE(controller_interface::cleanup_succeeds(broadcaster));
  EXPECT_FALSE(broadcaster->hasRealtimePublisher());
  EXPECT_FALSE(broadcaster->hasPublisher());
  EXPECT_FALSE(broadcaster->hasSemanticComponent());
  EXPECT_EQ(broadcaster->get_node()->count_publishers("/state_lifecycle_cleanup/robot_state"), 0U);
  EXPECT_EQ(broadcaster->on_cleanup(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);

  ASSERT_TRUE(controller_interface::configure_succeeds(broadcaster));
  interfaces.assign(*broadcaster);
  ASSERT_TRUE(controller_interface::activate_succeeds(broadcaster));
  ASSERT_TRUE(controller_interface::deactivate_succeeds(broadcaster));
  ASSERT_TRUE(controller_interface::cleanup_succeeds(broadcaster));
  broadcaster.reset();
  for (size_t attempt = 0; attempt < 100 && processThreadCount() > initial_threads; ++attempt) {
    std::this_thread::sleep_for(1ms);
  }
  EXPECT_LE(processThreadCount(), initial_threads);

  auto model = makeDirectBroadcaster<InspectableModelBroadcaster>("model_lifecycle_cleanup", 1000);
  ASSERT_TRUE(controller_interface::configure_succeeds(model));
  EXPECT_TRUE(model->hasRealtimePublisher());
  EXPECT_EQ(model->get_node()->count_publishers("/model_lifecycle_cleanup/robot_model"), 1U);
  interfaces.assign(*model, true);
  ASSERT_TRUE(controller_interface::activate_succeeds(model));
  EXPECT_EQ(model->on_error(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  EXPECT_TRUE(model->hasRealtimePublisher());
  EXPECT_TRUE(model->hasPublisher());
  EXPECT_TRUE(model->hasSemanticComponent());
  EXPECT_FALSE(model->semanticReadSucceeds());
  EXPECT_EQ(model->on_shutdown(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  // As above, only the later non-RT cleanup owns realtime publisher destruction.
  EXPECT_TRUE(model->hasRealtimePublisher());
  EXPECT_EQ(model->on_cleanup(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  EXPECT_FALSE(model->hasRealtimePublisher());
  EXPECT_FALSE(model->hasPublisher());
  EXPECT_FALSE(model->hasSemanticComponent());
  EXPECT_EQ(model->get_node()->count_publishers("/model_lifecycle_cleanup/robot_model"), 0U);
  model.reset();
  for (size_t attempt = 0; attempt < 100 && processThreadCount() > initial_threads; ++attempt) {
    std::this_thread::sleep_for(1ms);
  }
  EXPECT_LE(processThreadCount(), initial_threads);
}

TEST_F(ProductionControllerManagerIntegrationTest,
       RealtimeSemanticAndLifecycleSourceGatesForbidAllocationLoggingAndJoins) {
  const auto state_read =
      sourceSegment(SEMANTIC_STATE_SOURCE_FILE, "FrankaRobotState::get_robot_state_ptr()",
                    "bool FrankaRobotState::get_values_as_message");
  const auto model_read =
      sourceSegment(SEMANTIC_MODEL_SOURCE_FILE, "bool FrankaRobotModel::update_state_and_model()",
                    "void FrankaRobotModel::initialize()");
  for (const auto& source : {state_read, model_read}) {
    EXPECT_EQ(source.find("std::string"), std::string::npos);
    EXPECT_EQ(source.find(" + \"/\""), std::string::npos);
    EXPECT_EQ(source.find("RCLCPP_"), std::string::npos);
    EXPECT_EQ(source.find("fprintf"), std::string::npos);
    EXPECT_EQ(source.find("get_value("), std::string::npos);
    EXPECT_NE(source.find("get_optional<double>(1)"), std::string::npos);
  }

  for (const auto* path : {STATE_BROADCASTER_SOURCE_FILE, MODEL_BROADCASTER_SOURCE_FILE}) {
    const auto update = sourceSegment(path, "Broadcaster::update(", "}  // namespace");
    EXPECT_EQ(update.find("RCLCPP_"), std::string::npos);
    EXPECT_EQ(update.find("fprintf"), std::string::npos);
    const auto error = sourceSegment(path, "Broadcaster::on_error(", "Broadcaster::on_shutdown(");
    const auto shutdown = sourceSegment(path, "Broadcaster::on_shutdown(",
                                        "Broadcaster::reset_semantic_and_cadence(");
    for (const auto& callback : {error, shutdown}) {
      EXPECT_EQ(callback.find(".reset("), std::string::npos);
      EXPECT_EQ(callback.find("join("), std::string::npos);
      EXPECT_EQ(callback.find("RCLCPP_"), std::string::npos);
      EXPECT_EQ(callback.find("fprintf"), std::string::npos);
    }
  }
}

TEST_F(ProductionControllerManagerIntegrationTest,
       OfflineReleaseStressDryRunHasExactSchemaCountsAndCleanup) {
  test_support::StressOptions options{};
  options.activation_cycles = 1;
  options.mode_transactions = 5;
  options.duration_seconds = 1;
  options.rate_hz = 1000;
  options.seed = 0x5EEDU;
  const auto metrics = test_support::runOfflineReleaseStress(options);
  EXPECT_TRUE(metrics.success);
  EXPECT_EQ(metrics.sanitizer_instrumented, test_support::kStressSanitizerInstrumented);
  EXPECT_EQ(metrics.rss_threshold_applicable, !test_support::kStressSanitizerInstrumented);
  EXPECT_EQ(metrics.rss_threshold_passed, !test_support::kStressSanitizerInstrumented);
  EXPECT_EQ(metrics.activation_successes, options.activation_cycles);
  EXPECT_EQ(metrics.deactivation_successes, options.activation_cycles);
  EXPECT_EQ(metrics.valid_transactions_requested + metrics.invalid_transactions_requested,
            options.mode_transactions);
  EXPECT_EQ(metrics.scheduled_cycles, options.duration_seconds * options.rate_hz);
  EXPECT_EQ(metrics.completed_cycles, metrics.scheduled_cycles);
  EXPECT_GE(metrics.elapsed_ns, 1'000'000'000ULL);
  EXPECT_EQ(metrics.cleanup.constructed, 2U);
  EXPECT_EQ(metrics.cleanup.stopped, 2U);
  EXPECT_EQ(metrics.cleanup.destroyed, 2U);
  const auto json = test_support::stressMetricsJson(metrics);
  const auto csv = test_support::stressCyclesCsv(metrics);
  EXPECT_EQ(static_cast<size_t>(std::count(json.begin(), json.end(), '\n')), 1U);
  (void)parseAndCheckStressJson(json, 1000U, options.mode_transactions);
  EXPECT_EQ(static_cast<size_t>(std::count(csv.begin(), csv.end(), '\n')),
            metrics.scheduled_cycles + 1U);
}

TEST_F(ProductionControllerManagerIntegrationTest,
       OfflineReleaseStressRealCliRejectsMalformedAndWritesVersionedArtifacts) {
  const auto invalid_prefix = std::string("/tmp/franka_offline_release_stress_invalid_") +
                              std::to_string(static_cast<long long>(getpid()));
  const auto establish_absent = [](const std::string& path) {
    if (unlink(path.c_str()) != 0) {
      ASSERT_EQ(errno, ENOENT);
    }
    ASSERT_EQ(access(path.c_str(), F_OK), -1);
    ASSERT_EQ(errno, ENOENT);
  };
  const auto expect_absent = [](const std::string& path) {
    EXPECT_EQ(access(path.c_str(), F_OK), -1);
    EXPECT_EQ(errno, ENOENT);
  };
  const auto zero_json = invalid_prefix + "_zero.json";
  const auto zero_csv = invalid_prefix + "_zero.csv";
  const auto overflow_json = invalid_prefix + "_overflow.json";
  const auto overflow_csv = invalid_prefix + "_overflow.csv";
  const auto malformed_json = invalid_prefix + "_malformed.json";
  const auto malformed_csv = invalid_prefix + "_malformed.csv";
  for (const auto& path :
       {zero_json, zero_csv, overflow_json, overflow_csv, malformed_json, malformed_csv}) {
    establish_absent(path);
  }

  EXPECT_EQ(runStressSubprocess({"--help"}), 0);
  EXPECT_EQ(runStressSubprocess({}), 2);
  EXPECT_EQ(runStressSubprocess({"--activation-cycles", "0", "--mode-transactions", "1",
                                 "--duration-seconds", "1", "--rate-hz", "1000", "--seed", "1",
                                 "--metrics-json", zero_json, "--cycles-csv", zero_csv}),
            2);
  expect_absent(zero_json);
  expect_absent(zero_csv);
  EXPECT_EQ(
      runStressSubprocess({"--activation-cycles", "18446744073709551616", "--mode-transactions",
                           "1", "--duration-seconds", "1", "--rate-hz", "1000", "--seed", "1",
                           "--metrics-json", overflow_json, "--cycles-csv", overflow_csv}),
      2);
  expect_absent(overflow_json);
  expect_absent(overflow_csv);
  EXPECT_EQ(runStressSubprocess({"--activation-cycles", "not-a-number", "--mode-transactions", "1",
                                 "--duration-seconds", "1", "--rate-hz", "1000", "--seed", "1",
                                 "--metrics-json", malformed_json, "--cycles-csv", malformed_csv}),
            2);
  expect_absent(malformed_json);
  expect_absent(malformed_csv);
  EXPECT_EQ(runStressSubprocess({"--unknown", "1"}), 2);

  const auto prefix = std::string("/tmp/franka_offline_release_stress_cli_") +
                      std::to_string(static_cast<long long>(getpid()));
  const auto json_path = prefix + ".json";
  const auto csv_path = prefix + ".csv";
  ASSERT_EQ(runStressSubprocess({"--activation-cycles", "1", "--mode-transactions", "5",
                                 "--duration-seconds", "1", "--rate-hz", "1000", "--seed", "24301",
                                 "--metrics-json", json_path, "--cycles-csv", csv_path}),
            0);
  const auto json = readWholeFile(json_path);
  const auto csv = readWholeFile(csv_path);
  EXPECT_EQ(static_cast<size_t>(std::count(json.begin(), json.end(), '\n')), 1U);
  (void)parseAndCheckStressJson(json, 1000U, 5U);
  EXPECT_EQ(static_cast<size_t>(std::count(csv.begin(), csv.end(), '\n')), 1001U);
  EXPECT_EQ(unlink(json_path.c_str()), 0);
  EXPECT_EQ(unlink(csv_path.c_str()), 0);
}

}  // namespace
}  // namespace franka_example_controllers
