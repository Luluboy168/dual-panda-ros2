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

#include <chrono>
#include <condition_variable>
#include <controller_interface/controller_interface.hpp>
#include <controller_manager/controller_manager.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/list_hardware_interfaces.hpp>
#include <cstddef>
#include <cstdint>
#include <franka_hardware/real/franka_multi_hardware_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <map>
#include <memory>
#include <mutex>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <vector>

#include "support/synthetic_franka_arm_backend.hpp"

namespace franka_example_controllers::test_support
{

inline constexpr size_t kArmCount = 2;
inline constexpr size_t kJointCount = 7;
inline constexpr auto kCyclePeriod = std::chrono::milliseconds(1);

struct BackendCleanupSnapshot
{
  size_t constructed{0};
  size_t stopped{0};
  size_t destroyed{0};
};

// Shared by the dynamic broadcaster coverage and the standalone release stress executable. It
// imports an explicitly injected production hardware instance; no plugin, URDF, environment
// variable, or installed selector can construct the synthetic backend.
class OfflineControllerManagerHarness
{
public:
  explicit OfflineControllerManagerHarness(size_t arm_count = kArmCount);
  ~OfflineControllerManagerHarness();

  OfflineControllerManagerHarness(const OfflineControllerManagerHarness &) = delete;
  OfflineControllerManagerHarness & operator=(const OfflineControllerManagerHarness &) = delete;

  [[nodiscard]] size_t armCount() const noexcept { return arm_count_; }
  [[nodiscard]] controller_manager::ControllerManager & manager() { return *manager_; }
  [[nodiscard]] rclcpp::Node & clientNode() { return *client_node_; }
  [[nodiscard]] franka_hardware::FrankaMultiHardwareInterface & productionHardware()
  {
    return *production_hardware_;
  }
  [[nodiscard]] std::shared_ptr<franka_hardware::test_support::SyntheticFrankaArmBackend> backend(
    const std::string & arm_id) const;

  void loadAndConfigure(const std::string & name, const std::string & type);
  void loadReviewedControllers();
  std::vector<std::string> loadBroadcasters(size_t requested_arm_count);

  controller_interface::return_type switchControllers(
    const std::vector<std::string> & activate, const std::vector<std::string> & deactivate,
    bool activate_asap = true);
  controller_interface::return_type unloadController(const std::string & name);
  controller_interface::return_type cycle();

  template <typename Predicate>
  bool pumpUntil(Predicate predicate, size_t maximum_cycles = 1000)
  {
    for (size_t cycle_index = 0; cycle_index < maximum_cycles; ++cycle_index) {
      (void)cycle();
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
  }

  [[nodiscard]] std::vector<std::string> claimedInterfaces(const std::string & name) const;
  [[nodiscard]] std::vector<std::string> requiredStateInterfaces(const std::string & name) const;
  [[nodiscard]] uint8_t lifecycleId(const std::string & name) const;
  [[nodiscard]] controller_manager_msgs::srv::ListHardwareInterfaces::Response::SharedPtr
  hardwareInterfaces();
  [[nodiscard]] controller_manager_msgs::srv::ListControllers::Response::SharedPtr controllers();
  [[nodiscard]] const std::vector<std::string> & loadedControllerNames() const noexcept
  {
    return loaded_controller_names_;
  }

  void deactivateAndUnloadAll();
  void deactivateHardware() noexcept;
  void shutdown() noexcept;
  [[nodiscard]] BackendCleanupSnapshot cleanupSnapshot() const noexcept;

private:
  hardware_interface::HardwareInfo makeHardwareInfo() const;
  franka_hardware::BackendFactory backendFactory();
  void switchWorkerLoop() noexcept;
  controller_interface::return_type awaitSwitchWorker();
  void settleModeEntries();
  void stopSwitchWorker() noexcept;

  enum class WorkerOperation : uint8_t
  {
    None,
    Switch,
    Unload
  };

  size_t arm_count_;
  size_t backend_constructed_count_{0};
  size_t backend_stopped_count_{0};
  std::map<std::string, std::weak_ptr<franka_hardware::test_support::SyntheticFrankaArmBackend>>
    backends_{};
  franka_hardware::FrankaMultiHardwareInterface * production_hardware_{nullptr};
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_{};
  std::shared_ptr<controller_manager::ControllerManager> manager_{};
  std::shared_ptr<rclcpp::Node> client_node_{};
  std::thread executor_thread_{};
  std::thread switch_worker_thread_{};
  std::mutex switch_worker_mutex_{};
  std::condition_variable switch_worker_request_{};
  std::condition_variable switch_worker_complete_{};
  WorkerOperation switch_worker_operation_{WorkerOperation::None};
  std::vector<std::string> switch_worker_activate_{};
  std::vector<std::string> switch_worker_deactivate_{};
  std::string switch_worker_unload_name_{};
  bool switch_worker_activate_asap_{true};
  bool switch_worker_stop_{false};
  bool switch_worker_result_ready_{false};
  controller_interface::return_type switch_worker_result_{controller_interface::return_type::ERROR};
  std::vector<std::string> loaded_controller_names_{};
};

[[nodiscard]] std::vector<std::string> expectedJointClaims(
  const std::string & arm_id, const std::string & interface_name);

}  // namespace franka_example_controllers::test_support
