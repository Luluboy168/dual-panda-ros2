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
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <kdl/frames.hpp>

#include "franka_ik/panda_limits.hpp"
#include "franka_ik/reachability.hpp"

namespace franka_ik {

// Plain-library status values.  The service layer owns the mapping to rosidl result constants.
// NoAcceptableSolution is deliberately distinct from JointLimitsViolated: one rejected numeric
// candidate cannot prove that every IK branch violates a limit.
enum class SolveStatus : std::uint8_t {
  Success,
  BadRequest,
  SeedOutOfLimits,
  GeometricallyUnreachable,
  JointLimitsViolated,
  ToleranceNotMet,
  IterationBudgetExhausted,
  NoAcceptableSolution,
  InternalError,
};

struct SolveInput {
  KDL::Frame target_pose{KDL::Frame::Identity()};
  ReachabilityReferenceFrame reference_frame{ReachabilityReferenceFrame::ArmBase};
  ReachabilityTipFrame tip_frame{ReachabilityTipFrame::Flange};
  std::array<double, kPandaJointCount> seed_positions{};
  double fixed_q7{0.0};
  double joint_limit_margin{0.0};
  double position_tolerance{1.0e-4};
  double orientation_tolerance{1.0e-3};
  double numeric_eps{1.0e-6};
  std::uint16_t numeric_max_iterations{40};
};

struct IkCandidate {
  std::array<double, kPandaJointCount> positions{};
  double redundancy_value{0.0};
  double position_error{0.0};
  double orientation_error{0.0};
  double seed_distance{0.0};
  std::uint8_t branch{255};
};

struct SolveDiagnostics {
  bool solver_invoked{false};
  int solver_return_code{0};
  bool raw_candidate_finite{false};
  bool raw_candidate_within_limits{false};
  bool raw_candidate_within_tolerance{false};
  double raw_position_error{std::numeric_limits<double>::infinity()};
  double raw_orientation_error{std::numeric_limits<double>::infinity()};
  double lma_last_difference{std::numeric_limits<double>::infinity()};
  double lma_last_translational_difference{std::numeric_limits<double>::infinity()};
  double lma_last_rotational_difference{std::numeric_limits<double>::infinity()};
};

struct SolveOutput {
  SolveStatus status{SolveStatus::InternalError};
  std::string message;
  std::vector<IkCandidate> solutions;
  std::uint16_t iterations{0};
  SolveDiagnostics diagnostics;
};

class IkBackend {
 public:
  virtual ~IkBackend() = default;
  virtual void solve(const SolveInput& input, SolveOutput& output) const noexcept = 0;
};

}  // namespace franka_ik
