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

#include <array>
#include <optional>
#include <string>
#include <vector>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>

#include "franka_ik/panda_limits.hpp"

namespace franka_ik {

class ArmChain {
 public:
  const std::string& arm_id() const noexcept;
  const std::string& base_frame() const noexcept;
  const std::string& flange_frame() const noexcept;
  const std::string& hand_tcp_frame() const noexcept;
  const std::array<std::string, kPandaJointCount>& joint_names() const noexcept;
  const std::array<double, kPandaJointCount>& position_lower() const noexcept;
  const std::array<double, kPandaJointCount>& position_upper() const noexcept;
  const std::array<double, kPandaJointCount>& velocity_limits() const noexcept;
  const std::array<double, kPandaJointCount>& effort_limits() const noexcept;
  const KDL::Chain& flange_chain() const noexcept;
  const KDL::Frame& root_to_base() const noexcept;
  const std::optional<KDL::Chain>& hand_tcp_chain() const noexcept;
  const std::optional<KDL::Frame>& flange_to_hand_tcp() const noexcept;

 private:
  friend class RobotChains;

  ArmChain(std::string arm_id,
           std::string base_frame,
           std::string flange_frame,
           std::string hand_tcp_frame,
           std::array<std::string, kPandaJointCount> joint_names,
           std::array<double, kPandaJointCount> position_lower,
           std::array<double, kPandaJointCount> position_upper,
           std::array<double, kPandaJointCount> velocity_limits,
           std::array<double, kPandaJointCount> effort_limits,
           KDL::Chain flange_chain,
           KDL::Frame root_to_base,
           std::optional<KDL::Chain> hand_tcp_chain,
           std::optional<KDL::Frame> flange_to_hand_tcp);

  std::string arm_id_;
  std::string base_frame_;
  std::string flange_frame_;
  std::string hand_tcp_frame_;
  std::array<std::string, kPandaJointCount> joint_names_{};
  std::array<double, kPandaJointCount> position_lower_{};
  std::array<double, kPandaJointCount> position_upper_{};
  std::array<double, kPandaJointCount> velocity_limits_{};
  std::array<double, kPandaJointCount> effort_limits_{};
  KDL::Chain flange_chain_;
  KDL::Frame root_to_base_{KDL::Frame::Identity()};
  std::optional<KDL::Chain> hand_tcp_chain_;
  std::optional<KDL::Frame> flange_to_hand_tcp_;
};

class RobotChains {
 public:
  RobotChains(const std::string& urdf_xml, const std::vector<std::string>& arm_ids);

  const std::string& root_frame() const noexcept;
  const std::vector<ArmChain>& arms() const noexcept;
  bool has_arm(const std::string& arm_id) const noexcept;
  const ArmChain& arm(const std::string& arm_id) const;

 private:
  std::string root_frame_;
  std::vector<ArmChain> arms_;
};

}  // namespace franka_ik
