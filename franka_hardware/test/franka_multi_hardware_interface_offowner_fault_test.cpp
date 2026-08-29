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
// F-10l / F10C_LIFECYCLE_RT_DESIGN.md Amendment D, test obligations D.7.1 and D.7.2.
//
// perform_command_mode_switch() has five fault-preflight branches that call enterGlobalFault()
// BEFORE the isControlCycleOwner() dispatch. With controller_manager 4.45.2 and activate_asap
// false -- i.e. a plain `ros2 control switch_controllers` -- perform runs on the service thread
// while the RT loop keeps running, so those branches used to read hw_franka_robot_state_, publish
// through the single-producer command gate and request ControlMode::None on the WRONG thread.
//
// Every test below binds the control-cycle owner to a dedicated thread that runs read()/write()
// exactly the way ros2_control_node.cpp:144-146 does (both, unconditionally, every cycle), drives
// one preflight branch from a second, non-owner thread, and asserts two things:
//
//   1. the off-owner perform() call itself published NOTHING and requested NO mode -- the
//      mutation-killing assertion: revert the latch/settle split and this fails immediately; and
//   2. every backend command publish and every mode request that did happen carries the bound
//      owner thread's token.
//
// The probe backend records into a fixed-capacity, lock-free trace with ATOMIC counters. The
// review's own TracedBackend used std::vector/size_t and was itself racy, which is exactly the
// mistake this file must not repeat.

#include <gtest/gtest.h>

#include <pthread.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "franka_hardware/real/franka_multi_hardware_interface.hpp"
#include "support/synthetic_franka_arm_backend.hpp"

namespace franka_hardware {
namespace {

using test_support::SyntheticFrankaArmBackend;
using test_support::SyntheticFrankaArmBackendConfig;

constexpr auto kSettleTimeout = std::chrono::seconds(5);

hardware_interface::InterfaceInfo makeInterface(const std::string& name) {
  hardware_interface::InterfaceInfo interface{};
  interface.name = name;
  interface.data_type = "double";
  return interface;
}

hardware_interface::HardwareInfo makeHardwareInfo(const std::vector<std::string>& arm_names) {
  hardware_interface::HardwareInfo info{};
  info.name = "FrankaMultiHardwareInterface";
  info.type = "system";
  info.hardware_plugin_name = "franka_hardware/FrankaMultiHardwareInterface";
  info.hardware_parameters["robot_count"] = std::to_string(arm_names.size());
  for (size_t arm_index = 0; arm_index < arm_names.size(); ++arm_index) {
    const auto slot = std::to_string(arm_index + 1);
    const auto& arm_name = arm_names.at(arm_index);
    info.hardware_parameters["ns_" + slot] = arm_name;
    info.hardware_parameters["robot_ip_" + slot] = "offline-placeholder-" + slot;
    for (size_t joint_index = 1; joint_index <= FrankaMultiHardwareInterface::kNumberOfJoints;
         ++joint_index) {
      hardware_interface::ComponentInfo joint{};
      joint.name = arm_name + "_joint" + std::to_string(joint_index);
      joint.type = "joint";
      joint.command_interfaces = {makeInterface("effort"), makeInterface("position"),
                                  makeInterface("velocity")};
      joint.state_interfaces = {makeInterface("position"), makeInterface("velocity"),
                                makeInterface("effort")};
      info.joints.push_back(std::move(joint));
    }
  }
  return info;
}

std::vector<std::string> jointModeInterfaces(const std::string& arm_name,
                                             const std::string& interface_name) {
  std::vector<std::string> interfaces;
  for (size_t joint = 1; joint <= FrankaMultiHardwareInterface::kNumberOfJoints; ++joint) {
    interfaces.push_back(arm_name + "_joint" + std::to_string(joint) + "/" + interface_name);
  }
  return interfaces;
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

enum class ProbeEventKind : uint8_t { Publish, ModeRequest };

struct ProbeEvent {
  ProbeEventKind kind{ProbeEventKind::Publish};
  uint8_t arm_slot{0};
  ControlMode mode{ControlMode::None};
  bool result{false};
  // pthread_self() of the thread that made the backend call, in exactly the representation
  // FrankaMultiHardwareInterface::bindControlCycleOwner() stores as the owner token.
  uintptr_t thread_token{0};
};

// Fixed capacity, no allocation, no mutex: a slot is claimed with a relaxed fetch_add, filled, and
// then published with a release fetch_add on committed_. A reader that acquire-loads committed_
// joins the release sequence of every prior commit, so every field it then reads is visible.
class ProbeTrace {
 public:
  static constexpr size_t kCapacity = 8192;

  void record(ProbeEventKind kind, uint8_t arm_slot, ControlMode mode, bool result) noexcept {
    const size_t slot = claimed_.fetch_add(1, std::memory_order_relaxed);
    if (slot < kCapacity) {
      events_.at(slot) = ProbeEvent{kind, arm_slot, mode, result,
                                    static_cast<uintptr_t>(pthread_self())};
    }
    committed_.fetch_add(1, std::memory_order_release);
  }

  [[nodiscard]] size_t size() const noexcept {
    return std::min(committed_.load(std::memory_order_acquire), kCapacity);
  }

  [[nodiscard]] ProbeEvent at(size_t index) const noexcept { return events_.at(index); }

  [[nodiscard]] bool overflowed() const noexcept {
    return claimed_.load(std::memory_order_acquire) > kCapacity;
  }

 private:
  std::array<ProbeEvent, kCapacity> events_{};
  std::atomic<size_t> claimed_{0};
  std::atomic<size_t> committed_{0};
};

// Everything a test can steer, all atomic: the driving thread writes these while the owner thread
// may be reading them from inside read()/write().
struct ProbeControl {
  std::atomic<bool> no_command_capacity{false};
  // Latch a backend fault as a side effect of a preflight query, so the *next* preflight loop in
  // perform_command_mode_switch() is the one that observes it. This is how the third and fifth
  // branches (:1010 and :1029) are reached without the first and second ones firing.
  std::atomic<bool> fault_on_can_request{false};
  std::atomic<bool> fault_on_can_publish{false};
  std::atomic<uint64_t> publish_calls{0};
  std::atomic<uint64_t> mode_request_calls{0};
};

class ProbeBackend final : public FrankaArmBackend {
 public:
  ProbeBackend(uint8_t arm_slot,
               std::shared_ptr<SyntheticFrankaArmBackend> backend,
               std::shared_ptr<ProbeControl> control,
               ProbeTrace* trace)
      : arm_slot_(arm_slot),
        backend_(std::move(backend)),
        control_(std::move(control)),
        trace_(trace) {}

  bool startStateReading() override { return backend_->startStateReading(); }
  bool stop() override { return backend_->stop(); }
  franka::RobotState readLatestState() override { return backend_->readLatestState(); }
  ModelBase* model() noexcept override { return backend_->model(); }

  bool canPublishCommand() const noexcept override {
    if (control_->fault_on_can_publish.exchange(false, std::memory_order_acq_rel)) {
      backend_->injectFaultForTest();
      return true;
    }
    return !control_->no_command_capacity.load(std::memory_order_acquire) &&
           backend_->canPublishCommand();
  }

  bool publishCommand(const RobotCommand& command) noexcept override {
    control_->publish_calls.fetch_add(1, std::memory_order_relaxed);
    const bool result = backend_->publishCommand(command);
    trace_->record(ProbeEventKind::Publish, arm_slot_, ControlMode::None, result);
    return result;
  }

  bool canRequestControlMode(ControlMode mode) const noexcept override {
    if (control_->fault_on_can_request.exchange(false, std::memory_order_acq_rel)) {
      backend_->injectFaultForTest();
      return true;
    }
    return backend_->canRequestControlMode(mode);
  }

  bool requestControlMode(ControlMode mode) noexcept override {
    control_->mode_request_calls.fetch_add(1, std::memory_order_relaxed);
    const bool result = backend_->requestControlMode(mode);
    trace_->record(ProbeEventKind::ModeRequest, arm_slot_, mode, result);
    return result;
  }

  ControlMode requestedControlMode() const noexcept override {
    return backend_->requestedControlMode();
  }
  ControlMode activeControlMode() const noexcept override { return backend_->activeControlMode(); }
  bool modeEntryInFlight() const noexcept override { return backend_->modeEntryInFlight(); }
  bool hasFault() const noexcept override { return backend_->hasFault(); }
  bool recoverToReading() override { return backend_->recoverToReading(); }
  FrankaArmBackendDiagnostics diagnostics() const noexcept override {
    return backend_->diagnostics();
  }

  void setJointStiffness(
      const franka_msgs::srv::SetJointStiffness::Request::SharedPtr& request) override {
    backend_->setJointStiffness(request);
  }
  void setCartesianStiffness(
      const franka_msgs::srv::SetCartesianStiffness::Request::SharedPtr& request) override {
    backend_->setCartesianStiffness(request);
  }
  void setLoad(const franka_msgs::srv::SetLoad::Request::SharedPtr& request) override {
    backend_->setLoad(request);
  }
  void setTCPFrame(const franka_msgs::srv::SetTCPFrame::Request::SharedPtr& request) override {
    backend_->setTCPFrame(request);
  }
  void setStiffnessFrame(
      const franka_msgs::srv::SetStiffnessFrame::Request::SharedPtr& request) override {
    backend_->setStiffnessFrame(request);
  }
  void setForceTorqueCollisionBehavior(
      const franka_msgs::srv::SetForceTorqueCollisionBehavior::Request::SharedPtr& request)
      override {
    backend_->setForceTorqueCollisionBehavior(request);
  }
  void setFullCollisionBehavior(
      const franka_msgs::srv::SetFullCollisionBehavior::Request::SharedPtr& request) override {
    backend_->setFullCollisionBehavior(request);
  }

 private:
  uint8_t arm_slot_;
  std::shared_ptr<SyntheticFrankaArmBackend> backend_;
  std::shared_ptr<ProbeControl> control_;
  ProbeTrace* trace_;
};

struct ProbeHarness {
  std::vector<std::string> arm_names{"panda1", "panda2"};
  std::map<std::string, SyntheticFrankaArmBackendConfig> configurations;
  std::map<std::string, std::shared_ptr<ProbeControl>> controls;
  std::map<std::string, std::shared_ptr<SyntheticFrankaArmBackend>> backends;
  ProbeTrace trace;

  ProbeHarness() {
    for (size_t arm_index = 0; arm_index < arm_names.size(); ++arm_index) {
      configurations.emplace(arm_names.at(arm_index), SyntheticFrankaArmBackendConfig::forArm(
                                                          static_cast<uint8_t>(arm_index + 1)));
      controls.emplace(arm_names.at(arm_index), std::make_shared<ProbeControl>());
    }
  }

  BackendFactory factory() {
    return [this](const std::string& arm_name, const std::string&,
                  const rclcpp::Logger&) -> std::shared_ptr<FrankaArmBackend> {
      const auto found = std::find(arm_names.begin(), arm_names.end(), arm_name);
      if (found == arm_names.end()) {
        throw std::runtime_error("unknown arm name");
      }
      const auto arm_slot = static_cast<uint8_t>(std::distance(arm_names.begin(), found) + 1);
      auto backend = std::make_shared<SyntheticFrankaArmBackend>(configurations.at(arm_name));
      backends[arm_name] = backend;
      return std::make_shared<ProbeBackend>(arm_slot, std::move(backend), controls.at(arm_name),
                                            &trace);
    };
  }

  std::shared_ptr<SyntheticFrankaArmBackend> backend(const std::string& arm_name) const {
    return backends.at(arm_name);
  }
  std::shared_ptr<ProbeControl> control(const std::string& arm_name) const {
    return controls.at(arm_name);
  }
};

// The control-cycle owner. Runs read() then write() unconditionally, exactly like
// ros2_control_node.cpp:144-146, either one cycle at a time on request or free-running.
class OwnerCycleRunner {
 public:
  explicit OwnerCycleRunner(FrankaMultiHardwareInterface& hardware)
      : hardware_(hardware), thread_([this]() { run(); }) {}

  OwnerCycleRunner(const OwnerCycleRunner&) = delete;
  OwnerCycleRunner& operator=(const OwnerCycleRunner&) = delete;

  ~OwnerCycleRunner() { stop(); }

  // Runs `count` complete read()+write() cycles and returns once they have all finished.
  void runCycles(size_t count) {
    std::unique_lock<std::mutex> lock(mutex_);
    requested_ += count;
    cv_.notify_all();
    (void)cv_.wait_for(lock, kSettleTimeout, [this]() { return completed_ >= requested_; });
  }

  void runFreely() {
    const std::lock_guard<std::mutex> lock(mutex_);
    free_running_ = true;
    cv_.notify_all();
  }

  void stop() {
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      if (stopped_) {
        return;
      }
      stopped_ = true;
      cv_.notify_all();
    }
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  [[nodiscard]] uintptr_t ownerToken() const noexcept {
    return owner_token_.load(std::memory_order_acquire);
  }

  [[nodiscard]] uint64_t completedCycles() const noexcept {
    return cycle_counter_.load(std::memory_order_acquire);
  }

 private:
  void run() {
    owner_token_.store(static_cast<uintptr_t>(pthread_self()), std::memory_order_release);
    for (;;) {
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this]() { return stopped_ || free_running_ || completed_ < requested_; });
        if (stopped_) {
          return;
        }
      }
      (void)hardware_.read(rclcpp::Time(0), rclcpp::Duration(0, 0));
      (void)hardware_.write(rclcpp::Time(0), rclcpp::Duration(0, 0));
      cycle_counter_.fetch_add(1, std::memory_order_release);
      bool pace = false;
      {
        const std::lock_guard<std::mutex> lock(mutex_);
        if (!free_running_) {
          ++completed_;
        }
        cv_.notify_all();
        // Read under the same lock that runFreely() writes it under (V1: the unlocked read
        // here was itself the harness-race mistake this file's header forbids).
        pace = free_running_;
      }
      if (pace) {
        std::this_thread::sleep_for(std::chrono::microseconds(200));
      }
    }
  }

  FrankaMultiHardwareInterface& hardware_;
  std::atomic<uintptr_t> owner_token_{0};
  std::atomic<uint64_t> cycle_counter_{0};
  std::mutex mutex_;
  std::condition_variable cv_;
  size_t requested_{0};
  size_t completed_{0};
  bool free_running_{false};
  bool stopped_{false};
  std::thread thread_;
};

void expectEveryEventOnOwnerThread(const ProbeTrace& trace,
                                   uintptr_t owner_token,
                                   size_t from,
                                   const char* what) {
  ASSERT_FALSE(trace.overflowed());
  ASSERT_NE(owner_token, uintptr_t{0});
  const auto calling_token = static_cast<uintptr_t>(pthread_self());
  ASSERT_NE(owner_token, calling_token) << "the test thread must not be the control-cycle owner";
  for (size_t index = from; index < trace.size(); ++index) {
    const auto event = trace.at(index);
    EXPECT_EQ(event.thread_token, owner_token)
        << what << ": backend "
        << (event.kind == ProbeEventKind::Publish ? "publishCommand" : "requestControlMode")
        << " for arm slot " << static_cast<int>(event.arm_slot) << " (trace index " << index
        << ") ran off the control-cycle owner thread";
  }
}

// Which of perform_command_mode_switch()'s five fault-preflight branches to drive.
enum class PreflightBranch {
  BackendFaultFirstSweep,      // franka_multi_hardware_interface.cpp:988
  BackendFaultServiceGate,     // :1001
  BackendFaultAfterServiceGate,  // :1010
  CommandCapacity,             // :1019
  BackendFaultAfterCapacity,   // :1029
};

const char* branchName(PreflightBranch branch) {
  switch (branch) {
    case PreflightBranch::BackendFaultFirstSweep:
      return "preflight :988 (backend fault, first sweep)";
    case PreflightBranch::BackendFaultServiceGate:
      return "preflight :1001 (service gate held with a faulted backend)";
    case PreflightBranch::BackendFaultAfterServiceGate:
      return "preflight :1010 (fault injected during the service-gate sweep)";
    case PreflightBranch::CommandCapacity:
      return "preflight :1019 (command capacity)";
    case PreflightBranch::BackendFaultAfterCapacity:
      return "preflight :1029 (fault injected during the capacity sweep)";
  }
  return "unknown";
}

// Arms the chosen branch. Called with the owner thread parked, so the owner's own read()/write()
// fault sweep cannot latch the fault first and short-circuit perform's preflight.
void armBranch(PreflightBranch branch, ProbeHarness& harness) {
  switch (branch) {
    case PreflightBranch::BackendFaultFirstSweep:
      harness.backend("panda1")->injectFaultForTest();
      break;
    case PreflightBranch::BackendFaultServiceGate:
      ASSERT_TRUE(harness.backend("panda1")->holdServiceOperationForTest(
          BackendServiceOperation::Parameter));
      harness.backend("panda1")->injectFaultForTest();
      break;
    case PreflightBranch::BackendFaultAfterServiceGate:
      harness.control("panda1")->fault_on_can_request.store(true, std::memory_order_release);
      break;
    case PreflightBranch::CommandCapacity:
      harness.control("panda1")->no_command_capacity.store(true, std::memory_order_release);
      break;
    case PreflightBranch::BackendFaultAfterCapacity:
      harness.control("panda1")->fault_on_can_publish.store(true, std::memory_order_release);
      break;
  }
}

class OffOwnerFaultPreflightTest : public ::testing::TestWithParam<PreflightBranch> {};

TEST_P(OffOwnerFaultPreflightTest, EveryBackendEffectRunsOnTheControlCycleOwnerThread) {
  RclcppScope rclcpp_scope;
  ProbeHarness harness;
  FrankaMultiHardwareInterface hardware(harness.factory());
  ASSERT_EQ(hardware.on_init(makeHardwareInfo(harness.arm_names)), CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  // on_activate() seeds each arm with one safe command on the lifecycle thread, before any owner
  // is bound and before the RT loop exists (A.3.1). Those publishes are outside the owner-handoff
  // rule by construction, so the ownership window this test polices starts after them; pinning
  // their exact count here keeps that boundary explicit rather than implied.
  ASSERT_EQ(harness.trace.size(), harness.arm_names.size());

  OwnerCycleRunner runner(hardware);
  runner.runCycles(2);
  const auto owner_token = runner.ownerToken();
  ASSERT_NE(owner_token, uintptr_t{0});
  ASSERT_NE(owner_token, static_cast<uintptr_t>(pthread_self()));
  ASSERT_FALSE(hardware.globalFaultDiagnostic().latched());
  const size_t after_activation = harness.trace.size();

  const auto effort1 = jointModeInterfaces("panda1", "effort");
  ASSERT_EQ(hardware.prepare_command_mode_switch(effort1, {}), hardware_interface::return_type::OK);

  const auto branch = GetParam();
  ASSERT_NO_FATAL_FAILURE(armBranch(branch, harness));

  const size_t before_perform = harness.trace.size();
  // The off-owner call: this thread is NOT the control-cycle owner, exactly as
  // controller_manager's service thread is not, on the default (no --switch-asap) switch path.
  EXPECT_EQ(hardware.perform_command_mode_switch(effort1, {}),
            hardware_interface::return_type::ERROR)
      << branchName(branch);
  {
    // Pins that each parameter really reaches a DIFFERENT preflight site: the capacity branch is
    // the only one that latches CommandCapacity, and a future refactor that collapsed two of the
    // backend-fault sites into one would still have to keep all five reachable.
    const auto diagnostic = hardware.globalFaultDiagnostic();
    EXPECT_TRUE(diagnostic.latched()) << branchName(branch);
    EXPECT_EQ(diagnostic.cause, branch == PreflightBranch::CommandCapacity
                                    ? GlobalFaultCause::CommandCapacity
                                    : GlobalFaultCause::BackendFault)
        << branchName(branch);
    EXPECT_EQ(diagnostic.origin_arm_slot, 1) << branchName(branch);
  }

  // THE mutation-killing assertion. Before Amendment D this call published both arms' safe
  // commands and requested ControlMode::None for both arms, right here, on this thread.
  EXPECT_EQ(harness.trace.size(), before_perform)
      << branchName(branch)
      << ": an off-owner perform_command_mode_switch() must not touch a backend at all";

  if (branch == PreflightBranch::BackendFaultServiceGate) {
    harness.backend("panda1")->releaseServiceOperationForTest();
  }

  // One owner cycle is all the deferral costs: ros2_control_node calls write() unconditionally
  // every cycle, so the settle step lands at most 1 ms after the latch.
  runner.runCycles(1);

  EXPECT_GT(harness.trace.size(), before_perform)
      << branchName(branch) << ": the owner thread never performed the deferred settle step";
  bool saw_none_request = false;
  for (size_t index = before_perform; index < harness.trace.size(); ++index) {
    const auto event = harness.trace.at(index);
    if (event.kind == ProbeEventKind::ModeRequest && event.mode == ControlMode::None) {
      saw_none_request = true;
    }
    EXPECT_NE(event.kind == ProbeEventKind::ModeRequest && event.mode != ControlMode::None, true)
        << branchName(branch) << ": a latched fault must never request a live mode";
  }
  EXPECT_TRUE(saw_none_request)
      << branchName(branch) << ": the settle step must request ControlMode::None";

  // D.7.1: every backend effect from the moment the owner thread exists -- the warm-up cycles,
  // the off-owner perform, and the deferred settle -- carries the owner token.
  ASSERT_NO_FATAL_FAILURE(expectEveryEventOnOwnerThread(harness.trace, owner_token,
                                                        after_activation, branchName(branch)));

  runner.stop();
  EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

INSTANTIATE_TEST_SUITE_P(AllFivePreflightBranches,
                         OffOwnerFaultPreflightTest,
                         ::testing::Values(PreflightBranch::BackendFaultFirstSweep,
                                           PreflightBranch::BackendFaultServiceGate,
                                           PreflightBranch::BackendFaultAfterServiceGate,
                                           PreflightBranch::CommandCapacity,
                                           PreflightBranch::BackendFaultAfterCapacity));

// The same defect, reproduced with the RT loop genuinely running rather than stepped: the
// CommandCapacity branch is the one preflight that needs no injected backend fault, so the owner
// thread can free-run through the whole window without latching the fault itself first. This is
// the configuration the ThreadSanitizer evidence in
// notes/test_logs/port_review_2026-08-29/rt_concurrency/ was produced from.
TEST(OffOwnerFaultPreflightConcurrentTest, SettleStepStaysOnTheOwnerThreadUnderAConcurrentRtLoop) {
  RclcppScope rclcpp_scope;
  ProbeHarness harness;
  FrankaMultiHardwareInterface hardware(harness.factory());
  ASSERT_EQ(hardware.on_init(makeHardwareInfo(harness.arm_names)), CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  ASSERT_EQ(harness.trace.size(), harness.arm_names.size());  // on_activate()'s seeding, as above
  OwnerCycleRunner runner(hardware);
  runner.runCycles(1);
  const auto owner_token = runner.ownerToken();
  ASSERT_NE(owner_token, uintptr_t{0});
  ASSERT_NE(owner_token, static_cast<uintptr_t>(pthread_self()));
  const size_t after_activation = harness.trace.size();

  const auto effort1 = jointModeInterfaces("panda1", "effort");
  ASSERT_EQ(hardware.prepare_command_mode_switch(effort1, {}), hardware_interface::return_type::OK);
  harness.control("panda1")->no_command_capacity.store(true, std::memory_order_release);

  runner.runFreely();
  // Let the RT loop actually be in flight around the off-owner call.
  const auto cycles_before = runner.completedCycles();
  const auto spin_deadline = std::chrono::steady_clock::now() + kSettleTimeout;
  while (runner.completedCycles() < cycles_before + 5 &&
         std::chrono::steady_clock::now() < spin_deadline) {
    std::this_thread::sleep_for(std::chrono::microseconds(200));
  }

  const size_t before_perform = harness.trace.size();
  EXPECT_EQ(hardware.perform_command_mode_switch(effort1, {}),
            hardware_interface::return_type::ERROR);
  EXPECT_EQ(harness.trace.size(), before_perform)
      << "an off-owner perform_command_mode_switch() must not touch a backend at all";

  // The settle step has run once the provisional all-ones masks have been replaced by the real
  // ones: arm 1 cannot publish (no capacity), arm 2 can.
  const auto deadline = std::chrono::steady_clock::now() + kSettleTimeout;
  GlobalFaultDiagnostic diagnostic{};
  do {
    diagnostic = hardware.globalFaultDiagnostic();
    if (diagnostic.latched() && diagnostic.unsafe_safe_publish_mask == 0x1U) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(200));
  } while (std::chrono::steady_clock::now() < deadline);

  runner.stop();

  EXPECT_TRUE(diagnostic.latched());
  EXPECT_EQ(diagnostic.cause, GlobalFaultCause::CommandCapacity);
  EXPECT_EQ(diagnostic.origin_arm_slot, 1);
  EXPECT_EQ(diagnostic.unsafe_safe_publish_mask, 0x1U);
  EXPECT_EQ(diagnostic.unsafe_none_request_mask, 0x0U);
  EXPECT_GT(harness.trace.size(), before_perform);
  ASSERT_NO_FATAL_FAILURE(expectEveryEventOnOwnerThread(harness.trace, owner_token,
                                                        after_activation, "concurrent RT loop"));
  EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

}  // namespace
}  // namespace franka_hardware
