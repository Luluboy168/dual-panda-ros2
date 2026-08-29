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
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "franka_ik/ik_backend.hpp"
#include "franka_ik/numeric_backend.hpp"
#include "franka_ik/robot_chains.hpp"
#include "franka_ik_interfaces/srv/get_chain_info.hpp"
#include "franka_ik_interfaces/srv/solve_ik.hpp"

namespace franka_ik {

// Pure, exhaustive mapping seam shared by the service callback and contract tests.
std::uint8_t solveStatusToResultCode(SolveStatus status) noexcept;

class ServiceNode final : public rclcpp::Node {
 public:
  explicit ServiceNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  struct Configuration {
    std::string robot_description;
    std::vector<std::string> arm_ids;
    std::uint8_t default_solver{0};
    double position_tolerance{1.0e-4};
    double orientation_tolerance{1.0e-3};
    std::uint16_t numeric_max_iterations{40};
    double numeric_eps{1.0e-6};
    double joint_limit_margin_max{0.1};
  };

  static Configuration loadConfiguration(rclcpp::Node& node);
  static RobotChains buildChains(const Configuration& configuration);

  void handleSolve(const franka_ik_interfaces::srv::SolveIk::Request::SharedPtr request,
                   franka_ik_interfaces::srv::SolveIk::Response::SharedPtr response);
  void handleChainInfo(const franka_ik_interfaces::srv::GetChainInfo::Request::SharedPtr request,
                       franka_ik_interfaces::srv::GetChainInfo::Response::SharedPtr response) const;

  const Configuration configuration_;
  const RobotChains chains_;
  const std::string urdf_sha256_;
  std::vector<std::unique_ptr<NumericBackend>> numeric_backends_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Service<franka_ik_interfaces::srv::SolveIk>::SharedPtr solve_service_;
  rclcpp::Service<franka_ik_interfaces::srv::GetChainInfo>::SharedPtr chain_info_service_;
};

}  // namespace franka_ik
