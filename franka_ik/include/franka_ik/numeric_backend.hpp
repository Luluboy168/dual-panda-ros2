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

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>

#include "franka_ik/forward_kinematics.hpp"
#include "franka_ik/ik_backend.hpp"
#include "franka_ik/reachability.hpp"
#include "franka_ik/robot_chains.hpp"

namespace franka_ik {

// Constructs the six-joint problem used by NumericBackend.  The canonical joint-7 segment is
// replaced by one fixed segment whose pose is exactly Segment::pose(q7).  This keeps q7 outside
// LMA's optimisation variables instead of hoping that an unconstrained seven-joint solve leaves
// it unchanged.
KDL::Chain makeFixedQ7Chain(const KDL::Chain& flange_chain, double q7);

class NumericBackend final : public IkBackend {
 public:
  explicit NumericBackend(const ArmChain& arm);

  NumericBackend(const NumericBackend&) = delete;
  NumericBackend& operator=(const NumericBackend&) = delete;
  NumericBackend(NumericBackend&&) = delete;
  NumericBackend& operator=(NumericBackend&&) = delete;

  void solve(const SolveInput& input, SolveOutput& output) const noexcept override;

 private:
  KDL::Chain flange_chain_;
  std::array<double, kPandaJointCount> position_lower_{};
  std::array<double, kPandaJointCount> position_upper_{};
  KDL::Frame root_to_base_{KDL::Frame::Identity()};
  std::optional<KDL::Frame> flange_to_hand_tcp_;
  ReachabilityClassifier reachability_;
  // ForwardKinematics owns its own stable chain copy and is mutable only as request-local scratch.
  // The public service serialises calls; direct library callers must likewise not call one backend
  // instance concurrently.
  mutable ForwardKinematics forward_kinematics_;
};

}  // namespace franka_ik
