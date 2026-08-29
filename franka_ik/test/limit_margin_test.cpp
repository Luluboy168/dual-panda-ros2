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

#include "numeric_test_support.hpp"

#include <array>
#include <cstddef>

#include <gtest/gtest.h>

#include "franka_ik/forward_kinematics.hpp"
#include "franka_ik/numeric_backend.hpp"

namespace franka_ik {
namespace {

constexpr double kMargin = 0.05;

std::array<double, kPandaJointCount> interiorJoints() {
  std::array<double, kPandaJointCount> result{};
  for (std::size_t index = 0; index < result.size(); ++index) {
    result[index] = (kPandaPositionLowerLimits[index] + kPandaPositionUpperLimits[index]) / 2.0;
  }
  return result;
}

TEST(LimitMarginTest, EveryReturnedJointRespectsTheSymmetricMargin) {
  test::Model model;
  ForwardKinematics fk(model.arm().flange_chain());
  NumericBackend backend(model.arm());
  auto joints = interiorJoints();

  for (std::size_t selected_joint = 0; selected_joint < joints.size(); ++selected_joint) {
    for (const bool upper : {false, true}) {
      auto target_joints = joints;
      target_joints[selected_joint] = upper ? kPandaPositionUpperLimits[selected_joint] - kMargin
                                            : kPandaPositionLowerLimits[selected_joint] + kMargin;
      SolveInput input;
      input.target_pose = fk.compute(target_joints);
      input.seed_positions = target_joints;
      input.fixed_q7 = target_joints[6];
      input.joint_limit_margin = kMargin;
      input.position_tolerance = 1.0e-10;
      input.orientation_tolerance = 1.0e-10;
      input.numeric_eps = 1.0e-13;
      SolveOutput output;
      backend.solve(input, output);
      ASSERT_EQ(output.status, SolveStatus::Success) << output.message;
      ASSERT_EQ(output.solutions.size(), 1U);
      for (std::size_t index = 0; index < joints.size(); ++index) {
        EXPECT_GE(output.solutions.front().positions[index],
                  kPandaPositionLowerLimits[index] + kMargin);
        EXPECT_LE(output.solutions.front().positions[index],
                  kPandaPositionUpperLimits[index] - kMargin);
      }
    }
  }
}

TEST(LimitMarginTest, ProvenLimitOnlyTargetsAreRejectedBeforeLma) {
  test::Model model;
  NumericBackend backend(model.arm());
  std::size_t examined = 0;
  for (const auto& witness : test::corpus().unreachable_limits) {
    SolveInput input = test::exactInput(witness);
    input.joint_limit_margin = kMargin;
    for (std::size_t index = 0; index < input.seed_positions.size(); ++index) {
      input.seed_positions[index] =
          std::clamp(input.seed_positions[index], kPandaPositionLowerLimits[index] + kMargin,
                     kPandaPositionUpperLimits[index] - kMargin);
    }
    input.fixed_q7 = input.seed_positions[6];
    SolveOutput output;
    backend.solve(input, output);
    EXPECT_EQ(output.status, SolveStatus::JointLimitsViolated)
        << witness.id << ": " << output.message;
    EXPECT_FALSE(output.diagnostics.solver_invoked) << witness.id;
    ++examined;
  }
  EXPECT_EQ(examined, 200U);
}

TEST(LimitMarginTest, NeverClampsARejectedRawCandidate) {
  test::Model model;
  ForwardKinematics fk(model.arm().flange_chain());
  NumericBackend backend(model.arm());
  auto seed_joints = interiorJoints();
  seed_joints[0] = kPandaPositionLowerLimits[0] + kMargin;
  auto target_joints = seed_joints;
  target_joints[0] -= 0.01;

  SolveInput input;
  input.target_pose = fk.compute(target_joints);
  input.seed_positions = seed_joints;
  input.fixed_q7 = target_joints[6];
  input.joint_limit_margin = kMargin;
  input.numeric_eps = 1.0e-12;
  input.numeric_max_iterations = 1000;
  input.position_tolerance = 1.0e-9;
  input.orientation_tolerance = 1.0e-6;
  SolveOutput output;
  backend.solve(input, output);

  EXPECT_TRUE(output.solutions.empty());
  if (output.diagnostics.raw_candidate_finite && !output.diagnostics.raw_candidate_within_limits) {
    EXPECT_EQ(output.status, SolveStatus::NoAcceptableSolution);
  }
}

}  // namespace
}  // namespace franka_ik
