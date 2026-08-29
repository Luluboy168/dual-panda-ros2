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

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include "franka_ik/panda_limits.hpp"

namespace franka_ik {

struct PoseError {
  double position;
  double orientation;
};

// Returns the position norm and shortest rotation angle from target to actual.
PoseError computePoseError(const KDL::Frame& target, const KDL::Frame& actual) noexcept;

class ForwardKinematics {
 public:
  explicit ForwardKinematics(const KDL::Chain& chain);

  ForwardKinematics(const ForwardKinematics&) = delete;
  ForwardKinematics& operator=(const ForwardKinematics&) = delete;
  ForwardKinematics(ForwardKinematics&&) = delete;
  ForwardKinematics& operator=(ForwardKinematics&&) = delete;

  KDL::Frame compute(const std::array<double, kPandaJointCount>& joint_positions);

 private:
  // ChainFkSolverPos_recursive retains a reference, so chain_ must precede it and this class must
  // not move or copy while the solver exists.
  KDL::Chain chain_;
  KDL::ChainFkSolverPos_recursive solver_;
  KDL::JntArray joint_positions_;
};

}  // namespace franka_ik
