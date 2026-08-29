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
#include <optional>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>

#include "franka_ik/robot_chains.hpp"

namespace franka_ik {

// This is a conservative necessary-condition classifier, not a complete IK solver. In
// particular, passing its geometric tests does not prove that all pose and joint constraints can
// be satisfied. Reachable is reserved for a composed caller that has an explicit FK-verified joint
// witness; ReachabilityClassifier itself returns Indeterminate when no exclusion is proven.
enum class ReachabilityClass : std::uint8_t {
  Reachable,
  GeometricallyUnreachable,
  JointLimitsExcluded,
  Indeterminate,
};

enum class ReachabilityReferenceFrame : std::uint8_t {
  ArmBase,
  UrdfRoot,
};

enum class ReachabilityTipFrame : std::uint8_t {
  Flange,
  HandTcp,
};

struct ReachabilityQuery {
  KDL::Frame target_pose{KDL::Frame::Identity()};
  ReachabilityReferenceFrame reference_frame{ReachabilityReferenceFrame::ArmBase};
  ReachabilityTipFrame tip_frame{ReachabilityTipFrame::Flange};
  double fixed_q7{0.0};
  double joint_limit_margin{0.0};
  double distance_tolerance{1.0e-9};
};

class ReachabilityClassifier {
 public:
  explicit ReachabilityClassifier(const ArmChain& arm);

  // Returns a proven exclusion or Indeterminate. Invalid/non-finite query fields and a requested
  // hand TCP that is absent from the URDF are also Indeterminate; request validation assigns the
  // corresponding public service result before invoking this classifier.
  ReachabilityClass classify(const ReachabilityQuery& query) const noexcept;

 private:
  KDL::Chain flange_chain_;
  KDL::Frame root_to_base_{KDL::Frame::Identity()};
  std::optional<KDL::Frame> flange_to_hand_tcp_;
  KDL::Vector shoulder_position_;
  double maximum_shoulder_to_flange_distance_{0.0};
  double elbow_distance_squared_constant_{0.0};
  double elbow_distance_squared_cosine_{0.0};
  double elbow_distance_squared_sine_{0.0};
  double q4_lower_{0.0};
  double q4_upper_{0.0};
  double q7_lower_{0.0};
  double q7_upper_{0.0};
};

}  // namespace franka_ik
