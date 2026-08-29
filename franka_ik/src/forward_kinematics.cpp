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

#include "franka_ik/forward_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace franka_ik {
namespace {

KDL::Chain validateAndCopyChain(const KDL::Chain& chain) {
  if (chain.getNrOfJoints() != kPandaJointCount) {
    std::ostringstream message;
    message << "forward-kinematics chain must contain exactly " << kPandaJointCount
            << " joints; got " << chain.getNrOfJoints();
    throw std::invalid_argument(message.str());
  }
  return chain;
}

}  // namespace

PoseError computePoseError(const KDL::Frame& target, const KDL::Frame& actual) noexcept {
  const double delta_x = actual.p.x() - target.p.x();
  const double delta_y = actual.p.y() - target.p.y();
  const double delta_z = actual.p.z() - target.p.z();

  // Avoid KDL::diff and Vector::Norm here: both apply KDL's default epsilon (1e-6 on the
  // supported host) and silently collapse smaller errors to zero.  The post-solve verifier must
  // remain sensitive at the configured tolerances.
  const KDL::Rotation relative = target.M.Inverse() * actual.M;
  const double cosine =
      std::clamp((relative(0, 0) + relative(1, 1) + relative(2, 2) - 1.0) / 2.0, -1.0, 1.0);
  const double skew_x = (relative(2, 1) - relative(1, 2)) / 2.0;
  const double skew_y = (relative(0, 2) - relative(2, 0)) / 2.0;
  const double skew_z = (relative(1, 0) - relative(0, 1)) / 2.0;
  const double sine_magnitude = std::hypot(skew_x, skew_y, skew_z);

  return PoseError{std::hypot(delta_x, delta_y, delta_z), std::atan2(sine_magnitude, cosine)};
}

ForwardKinematics::ForwardKinematics(const KDL::Chain& chain)
    : chain_(validateAndCopyChain(chain)), solver_(chain_), joint_positions_(kPandaJointCount) {}

KDL::Frame ForwardKinematics::compute(const std::array<double, kPandaJointCount>& joint_positions) {
  for (std::size_t index = 0; index < joint_positions.size(); ++index) {
    joint_positions_(static_cast<unsigned int>(index)) = joint_positions[index];
  }

  KDL::Frame result;
  const int solver_result = solver_.JntToCart(joint_positions_, result);
  if (solver_result < 0) {
    std::ostringstream message;
    message << "forward kinematics failed: " << solver_.strError(solver_result) << " ("
            << solver_result << ')';
    throw std::runtime_error(message.str());
  }
  return result;
}

}  // namespace franka_ik
