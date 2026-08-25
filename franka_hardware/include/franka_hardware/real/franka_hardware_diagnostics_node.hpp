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

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>

#include "franka_hardware/real/franka_arm_backend.hpp"
#include "franka_hardware/real/franka_multi_hardware_interface.hpp"

namespace franka_hardware {

struct BuildProvenance {
  std::string source_commit;
  bool source_dirty{false};
  std::string ros_distro;
  std::string rclcpp_version;
  std::string ros2_control_version;
  std::string libfranka_version;
};

struct HardwareLifecycleSnapshot {
  uint8_t id{0};
  std::string label{"unknown"};
};

struct FrankaArmDiagnosticSnapshot {
  std::string arm_id;
  HardwareLifecycleSnapshot hardware_lifecycle;
  FrankaArmBackendDiagnostics backend;
  GlobalFaultDiagnostic global_fault;
  std::string global_fault_origin_arm_id{"none"};
  BuildProvenance provenance;
};

struct FrankaArmDiagnosticSource {
  std::string arm_id;
  std::shared_ptr<FrankaArmBackend> backend;
};

using GlobalFaultDiagnosticProvider = std::function<GlobalFaultDiagnostic()>;
using HardwareLifecycleProvider = std::function<HardwareLifecycleSnapshot()>;

[[nodiscard]] BuildProvenance currentBuildProvenance();
[[nodiscard]] BuildProvenance sanitizeBuildProvenance(const BuildProvenance& provenance);
[[nodiscard]] bool isSanitizedBuildProvenance(const BuildProvenance& provenance) noexcept;

[[nodiscard]] const char* backendFailureReasonLabel(BackendFailureReason reason) noexcept;
[[nodiscard]] const char* backendRecoveryResultLabel(BackendRecoveryResult result) noexcept;

void formatFrankaArmDiagnosticStatus(const FrankaArmDiagnosticSnapshot& snapshot,
                                     uint64_t now_steady_ns,
                                     diagnostic_updater::DiagnosticStatusWrapper& status);

class FrankaHardwareDiagnosticsNode final : public rclcpp::Node {
 public:
  FrankaHardwareDiagnosticsNode(const rclcpp::NodeOptions& options,
                                std::vector<FrankaArmDiagnosticSource> arm_sources,
                                GlobalFaultDiagnosticProvider global_fault_provider,
                                HardwareLifecycleProvider hardware_lifecycle_provider,
                                double update_period_seconds = 1.0);

 private:
  void diagnoseArm(size_t arm_index, diagnostic_updater::DiagnosticStatusWrapper& status);

  std::vector<FrankaArmDiagnosticSource> arm_sources_;
  GlobalFaultDiagnosticProvider global_fault_provider_;
  HardwareLifecycleProvider hardware_lifecycle_provider_;
  BuildProvenance provenance_;
  diagnostic_updater::Updater updater_;
};

}  // namespace franka_hardware
