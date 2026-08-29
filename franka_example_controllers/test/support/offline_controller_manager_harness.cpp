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

#include "support/offline_controller_manager_harness.hpp"

#include <algorithm>
#include <controller_manager_msgs/srv/set_hardware_component_state.hpp>
#include <future>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/types/hardware_component_params.hpp>
#include <hardware_interface/types/resource_manager_params.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <stdexcept>
#include <utility>

namespace franka_example_controllers::test_support
{
namespace
{

using namespace std::chrono_literals;

constexpr char kEmptyRobotDescription[] =
  R"(<?xml version="1.0"?>
<robot name="empty_offline_release_harness">
  <link name="base_link"/>
  <ros2_control name="EmptyBootstrapSystem" type="system">
    <hardware>
      <plugin>mock_components/GenericSystem</plugin>
    </hardware>
  </ros2_control>
</robot>)";

hardware_interface::InterfaceInfo makeInterface(const std::string & name)
{
  hardware_interface::InterfaceInfo interface{};
  interface.name = name;
  interface.data_type = "double";
  return interface;
}

}  // namespace

OfflineControllerManagerHarness::OfflineControllerManagerHarness(size_t arm_count)
: arm_count_(arm_count)
{
  if (arm_count_ == 0 || arm_count_ > kArmCount) {
    throw std::invalid_argument("offline manager arm count must be one or two");
  }

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  hardware_interface::ResourceManagerParams resource_params{};
  resource_params.robot_description = kEmptyRobotDescription;
  resource_params.clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  resource_params.logger = rclcpp::get_logger("offline_release_resource_manager");
  resource_params.node_namespace = "";
  resource_params.executor = executor_;
  resource_params.activate_all = false;
  resource_params.update_rate = 1000;
  auto resource_manager =
    std::make_unique<hardware_interface::ResourceManager>(resource_params, true);
  if (!resource_manager->are_components_initialized()) {
    throw std::runtime_error("empty bootstrap ResourceManager did not initialize");
  }

  auto hardware = std::make_unique<franka_hardware::FrankaMultiHardwareInterface>(
    backendFactory(), franka_hardware::InitializationCheckpointHook{},
    franka_hardware::ModeSwitchCheckpointHook{});
  production_hardware_ = hardware.get();
  hardware_interface::HardwareComponentParams component_params{};
  component_params.hardware_info = makeHardwareInfo();
  component_params.logger = resource_params.logger;
  component_params.clock = resource_params.clock;
  component_params.node_namespace = resource_params.node_namespace;
  component_params.executor = executor_;
  resource_manager->import_component(std::move(hardware), component_params);
  if (resource_manager->system_components_size() != 2U) {
    throw std::runtime_error("production import plus bootstrap registered unexpected components");
  }

  rclcpp_lifecycle::State active_state(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
  if (
    resource_manager->set_component_state("FrankaMultiHardwareInterface", active_state) !=
    hardware_interface::return_type::OK) {
    throw std::runtime_error("injected production hardware did not reach ACTIVE");
  }

  auto manager_options = controller_manager::get_cm_node_options();
  manager_options.arguments({"--ros-args", "--params-file", OFFLINE_CM_TEST_PARAMS_FILE});
  manager_ = std::make_shared<controller_manager::ControllerManager>(
    std::move(resource_manager), executor_, "controller_manager", "", manager_options);
  if (!manager_->is_resource_manager_initialized()) {
    throw std::runtime_error("controller manager rejected initialized ResourceManager");
  }

  client_node_ = std::make_shared<rclcpp::Node>("offline_release_harness_client");
  executor_->add_node(manager_);
  executor_->add_node(client_node_);
  executor_thread_ = std::thread([this]() { executor_->spin(); });
  switch_worker_thread_ = std::thread([this]() { switchWorkerLoop(); });
}

OfflineControllerManagerHarness::~OfflineControllerManagerHarness() { shutdown(); }

std::shared_ptr<franka_hardware::test_support::SyntheticFrankaArmBackend>
OfflineControllerManagerHarness::backend(const std::string & arm_id) const
{
  const auto found = backends_.find(arm_id);
  if (found == backends_.end()) {
    throw std::out_of_range("offline backend does not exist: " + arm_id);
  }
  auto result = found->second.lock();
  if (!result) {
    throw std::runtime_error("offline backend was already destroyed: " + arm_id);
  }
  return result;
}

void OfflineControllerManagerHarness::loadAndConfigure(
  const std::string & name, const std::string & type)
{
  if (
    std::find(loaded_controller_names_.begin(), loaded_controller_names_.end(), name) !=
    loaded_controller_names_.end()) {
    throw std::invalid_argument("controller name already loaded: " + name);
  }
  const auto controller = manager_->load_controller(name, type);
  if (!controller) {
    throw std::runtime_error("plugin load failed: " + type);
  }
  if (controller->get_lifecycle_id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
    throw std::runtime_error("loaded controller was not UNCONFIGURED: " + name);
  }
  if (manager_->configure_controller(name) != controller_interface::return_type::OK) {
    throw std::runtime_error("controller configure failed: " + name);
  }
  if (controller->get_lifecycle_id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    throw std::runtime_error("configured controller was not INACTIVE: " + name);
  }
  loaded_controller_names_.push_back(name);
}

void OfflineControllerManagerHarness::loadReviewedControllers()
{
  if (arm_count_ != kArmCount) {
    throw std::logic_error("reviewed dual-arm controllers require two injected arms");
  }
  loadAndConfigure("hold_controller", "franka_example_controllers/DualArmJointHoldController");
  loadAndConfigure(
    "velocity_controller", "franka_example_controllers/DualArmJointVelocityController");
  loadAndConfigure(
    "impedance_controller", "franka_example_controllers/DualArmJointImpedanceController");
}

std::vector<std::string> OfflineControllerManagerHarness::loadBroadcasters(
  size_t requested_arm_count)
{
  if (requested_arm_count == 0 || requested_arm_count > arm_count_) {
    throw std::invalid_argument("requested broadcaster arm count is unavailable");
  }
  std::vector<std::string> result;
  result.reserve(requested_arm_count * 2U);
  for (size_t arm = 1; arm <= requested_arm_count; ++arm) {
    const auto prefix = "panda" + std::to_string(arm);
    const auto state_name = prefix + "_state_broadcaster";
    const auto model_name = prefix + "_model_broadcaster";
    loadAndConfigure(state_name, "franka_robot_state_broadcaster/FrankaRobotStateBroadcaster");
    loadAndConfigure(model_name, "franka_robot_state_broadcaster/FrankaRobotModelBroadcaster");
    result.push_back(state_name);
    result.push_back(model_name);
  }
  return result;
}

controller_interface::return_type OfflineControllerManagerHarness::switchControllers(
  const std::vector<std::string> & activate, const std::vector<std::string> & deactivate,
  bool activate_asap)
{
  {
    const std::lock_guard<std::mutex> lock(switch_worker_mutex_);
    if (switch_worker_operation_ != WorkerOperation::None || switch_worker_result_ready_) {
      throw std::logic_error("offline switch worker already has a request");
    }
    switch_worker_activate_ = activate;
    switch_worker_deactivate_ = deactivate;
    switch_worker_activate_asap_ = activate_asap;
    switch_worker_operation_ = WorkerOperation::Switch;
  }
  switch_worker_request_.notify_one();
  return awaitSwitchWorker();
}

controller_interface::return_type OfflineControllerManagerHarness::unloadController(
  const std::string & name)
{
  {
    const std::lock_guard<std::mutex> lock(switch_worker_mutex_);
    if (switch_worker_operation_ != WorkerOperation::None || switch_worker_result_ready_) {
      throw std::logic_error("offline switch worker already has a request");
    }
    switch_worker_unload_name_ = name;
    switch_worker_operation_ = WorkerOperation::Unload;
  }
  switch_worker_request_.notify_one();
  const auto status = awaitSwitchWorker();
  if (status == controller_interface::return_type::OK) {
    loaded_controller_names_.erase(
      std::remove(loaded_controller_names_.begin(), loaded_controller_names_.end(), name),
      loaded_controller_names_.end());
  }
  return status;
}

controller_interface::return_type OfflineControllerManagerHarness::awaitSwitchWorker()
{
  for (size_t attempt = 0; attempt < 20000; ++attempt) {
    (void)cycle();
    std::unique_lock<std::mutex> lock(switch_worker_mutex_);
    if (switch_worker_complete_.wait_for(
          lock, 10us, [this]() { return switch_worker_result_ready_; })) {
      const auto result = switch_worker_result_;
      switch_worker_result_ready_ = false;
      lock.unlock();
      settleModeEntries();
      return result;
    }
  }
  throw std::runtime_error("bounded persistent controller worker timed out");
}

void OfflineControllerManagerHarness::settleModeEntries()
{
  // F-10g (2026-08-28), F10C_LIFECYCLE_RT_DESIGN.md amendment C.4. The emulated backends model
  // mode ENTRY asynchronously: after an accepted control-mode request the new control loop's first
  // callback -- the command channel's only consumer -- does not run until a few read cycles have
  // passed, exactly as libfranka's startMotion() blocks on the real robot.
  //
  // In this harness the simulated 1 kHz RT clock only advances when cycle() is called, and
  // awaitSwitchWorker() calls it just often enough to service the switch. Returning the moment the
  // switch lands would therefore issue the NEXT switch a handful of simulated milliseconds later,
  // a cadence no physical mode entry could keep up with: entries would never complete, the command
  // channel would never drain, and the harness would model a machine that cannot exist. Real
  // hardware runs thousands of RT cycles between operator switches. Advance the clock until every
  // arm's entry has landed -- bounded, and a no-op for backends that report nothing in flight
  // (stopped, faulted, or a rejected request).
  constexpr size_t kMaximumSettleCycles = 64;
  for (size_t attempt = 0; attempt < kMaximumSettleCycles; ++attempt) {
    bool entry_in_flight = false;
    for (const auto & entry : backends_) {
      const auto backend = entry.second.lock();
      if (backend && backend->modeEntryInFlight()) {
        entry_in_flight = true;
        break;
      }
    }
    if (!entry_in_flight) {
      return;
    }
    (void)cycle();
  }
}

void OfflineControllerManagerHarness::switchWorkerLoop() noexcept
{
  while (true) {
    WorkerOperation operation = WorkerOperation::None;
    std::vector<std::string> activate;
    std::vector<std::string> deactivate;
    std::string unload_name;
    bool activate_asap = true;
    {
      std::unique_lock<std::mutex> lock(switch_worker_mutex_);
      switch_worker_request_.wait(lock, [this]() {
        return switch_worker_stop_ || switch_worker_operation_ != WorkerOperation::None;
      });
      if (switch_worker_stop_) {
        return;
      }
      operation = switch_worker_operation_;
      activate = std::move(switch_worker_activate_);
      deactivate = std::move(switch_worker_deactivate_);
      unload_name = std::move(switch_worker_unload_name_);
      activate_asap = switch_worker_activate_asap_;
    }

    auto result = controller_interface::return_type::ERROR;
    try {
      if (operation == WorkerOperation::Switch) {
        result = manager_->switch_controller(
          activate, deactivate, controller_manager_msgs::srv::SwitchController::Request::STRICT,
          activate_asap, rclcpp::Duration::from_seconds(2.0));
      } else if (operation == WorkerOperation::Unload) {
        result = manager_->unload_controller(unload_name);
      }
    } catch (...) {
      result = controller_interface::return_type::ERROR;
    }
    {
      const std::lock_guard<std::mutex> lock(switch_worker_mutex_);
      switch_worker_operation_ = WorkerOperation::None;
      switch_worker_result_ = result;
      switch_worker_result_ready_ = true;
    }
    switch_worker_complete_.notify_one();
  }
}

void OfflineControllerManagerHarness::stopSwitchWorker() noexcept
{
  {
    const std::lock_guard<std::mutex> lock(switch_worker_mutex_);
    switch_worker_stop_ = true;
  }
  switch_worker_request_.notify_one();
  if (switch_worker_thread_.joinable()) {
    switch_worker_thread_.join();
  }
}

controller_interface::return_type OfflineControllerManagerHarness::cycle()
{
  const auto time = manager_->get_trigger_clock()->now();
  const auto period = rclcpp::Duration(kCyclePeriod);
  manager_->read(time, period);
  const auto result = manager_->update(time, period);
  manager_->write(time, period);
  return result;
}

std::vector<std::string> OfflineControllerManagerHarness::claimedInterfaces(
  const std::string & name) const
{
  for (const auto & specification : manager_->get_loaded_controllers()) {
    if (specification.info.name == name) {
      auto claims = specification.info.claimed_interfaces;
      std::sort(claims.begin(), claims.end());
      return claims;
    }
  }
  throw std::out_of_range("controller not loaded: " + name);
}

std::vector<std::string> OfflineControllerManagerHarness::requiredStateInterfaces(
  const std::string & name) const
{
  for (const auto & specification : manager_->get_loaded_controllers()) {
    if (specification.info.name == name) {
      auto interfaces = specification.c->state_interface_configuration().names;
      std::sort(interfaces.begin(), interfaces.end());
      return interfaces;
    }
  }
  throw std::out_of_range("controller not loaded: " + name);
}

uint8_t OfflineControllerManagerHarness::lifecycleId(const std::string & name) const
{
  for (const auto & specification : manager_->get_loaded_controllers()) {
    if (specification.info.name == name) {
      return specification.c->get_lifecycle_id();
    }
  }
  throw std::out_of_range("controller not loaded: " + name);
}

controller_manager_msgs::srv::ListHardwareInterfaces::Response::SharedPtr
OfflineControllerManagerHarness::hardwareInterfaces()
{
  auto client = client_node_->create_client<controller_manager_msgs::srv::ListHardwareInterfaces>(
    "/controller_manager/list_hardware_interfaces");
  if (!client->wait_for_service(2s)) {
    throw std::runtime_error("list_hardware_interfaces service unavailable");
  }
  auto request = std::make_shared<controller_manager_msgs::srv::ListHardwareInterfaces::Request>();
  auto result = client->async_send_request(request);
  if (result.wait_for(2s) != std::future_status::ready) {
    throw std::runtime_error("list_hardware_interfaces service timed out");
  }
  return result.get();
}

controller_manager_msgs::srv::ListControllers::Response::SharedPtr
OfflineControllerManagerHarness::controllers()
{
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

void OfflineControllerManagerHarness::deactivateAndUnloadAll()
{
  std::vector<std::string> active;
  for (const auto & name : loaded_controller_names_) {
    if (lifecycleId(name) == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      active.push_back(name);
    }
  }
  if (!active.empty() && switchControllers({}, active) != controller_interface::return_type::OK) {
    throw std::runtime_error("failed to deactivate all offline controllers");
  }
  const auto names = loaded_controller_names_;
  for (auto iterator = names.rbegin(); iterator != names.rend(); ++iterator) {
    if (unloadController(*iterator) != controller_interface::return_type::OK) {
      throw std::runtime_error("failed to unload offline controller: " + *iterator);
    }
  }
}

void OfflineControllerManagerHarness::deactivateHardware() noexcept
{
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
      if (result.wait_for(2s) == std::future_status::ready && result.get()->ok) {
        return;
      }
    }
  } catch (...) {
  }
  try {
    if (production_hardware_) {
      (void)production_hardware_->on_deactivate(rclcpp_lifecycle::State());
    }
  } catch (...) {
  }
}

void OfflineControllerManagerHarness::shutdown() noexcept
{
  if (!manager_) {
    stopSwitchWorker();
    return;
  }
  stopSwitchWorker();
  try {
    (void)manager_->shutdown_controllers();
  } catch (...) {
  }
  deactivateHardware();
  backend_stopped_count_ = 0;
  for (const auto & entry : backends_) {
    const auto backend = entry.second.lock();
    if (backend && backend->diagnostics().stopped) {
      ++backend_stopped_count_;
    }
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

BackendCleanupSnapshot OfflineControllerManagerHarness::cleanupSnapshot() const noexcept
{
  BackendCleanupSnapshot snapshot{};
  snapshot.constructed = backend_constructed_count_;
  snapshot.stopped = backend_stopped_count_;
  snapshot.destroyed = static_cast<size_t>(std::count_if(
    backends_.begin(), backends_.end(), [](const auto & entry) { return entry.second.expired(); }));
  return snapshot;
}

hardware_interface::HardwareInfo OfflineControllerManagerHarness::makeHardwareInfo() const
{
  hardware_interface::HardwareInfo info{};
  info.name = "FrankaMultiHardwareInterface";
  info.type = "system";
  info.rw_rate = 1000;
  info.is_async = false;
  info.thread_priority = 0;
  info.hardware_plugin_name = "franka_hardware/FrankaMultiHardwareInterface";
  info.hardware_parameters["robot_count"] = std::to_string(arm_count_);
  info.original_xml = kEmptyRobotDescription;
  for (size_t arm = 1; arm <= arm_count_; ++arm) {
    const auto slot = std::to_string(arm);
    const auto arm_id = "panda" + slot;
    info.hardware_parameters["ns_" + slot] = arm_id;
    info.hardware_parameters["robot_ip_" + slot] = "offline-test-only-" + slot;
    for (size_t joint = 1; joint <= kJointCount; ++joint) {
      hardware_interface::ComponentInfo component{};
      component.name = arm_id + "_joint" + std::to_string(joint);
      component.type = "joint";
      component.command_interfaces = {
        makeInterface("effort"), makeInterface("position"), makeInterface("velocity")};
      component.state_interfaces = {
        makeInterface("position"), makeInterface("velocity"), makeInterface("effort")};
      info.joints.push_back(std::move(component));
    }
  }
  return info;
}

franka_hardware::BackendFactory OfflineControllerManagerHarness::backendFactory()
{
  return
    [this](
      const std::string & arm_name, const std::string & robot_address,
      const rclcpp::Logger & /*logger*/) -> std::shared_ptr<franka_hardware::FrankaArmBackend> {
      const auto slot = arm_name == "panda1" ? 1U : arm_name == "panda2" ? 2U : 0U;
      if (
        slot == 0U || slot > arm_count_ ||
        robot_address != "offline-test-only-" + std::to_string(slot)) {
        throw std::invalid_argument("unexpected injected backend metadata");
      }
      auto config = franka_hardware::test_support::SyntheticFrankaArmBackendConfig::forArm(
        static_cast<uint8_t>(slot));
      config.command_queue_capacity =
        franka_hardware::test_support::SyntheticFrankaArmBackend::kCommandCaptureCapacity;
      // Keep the actual reviewed effort controllers inside this test fixture's configured
      // limits while preserving deterministic, arm-distinct model values.
      config.model_coriolis_scale = 0.01;
      auto backend =
        std::make_shared<franka_hardware::test_support::SyntheticFrankaArmBackend>(config);
      backends_.emplace(arm_name, backend);
      ++backend_constructed_count_;
      return backend;
    };
}

std::vector<std::string> expectedJointClaims(
  const std::string & arm_id, const std::string & interface_name)
{
  std::vector<std::string> names;
  names.reserve(kJointCount);
  for (size_t joint = 1; joint <= kJointCount; ++joint) {
    names.push_back(arm_id + "_joint" + std::to_string(joint) + "/" + interface_name);
  }
  std::sort(names.begin(), names.end());
  return names;
}

}  // namespace franka_example_controllers::test_support
