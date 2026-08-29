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
#include <cstring>
#include <limits>

#include <gtest/gtest.h>

#include "franka_ik/forward_kinematics.hpp"
#include "franka_ik/numeric_backend.hpp"

namespace franka_ik {
namespace {

TEST(SeedValidationTest, RejectsEveryNonFiniteSeedComponentWithoutInvokingLma) {
  test::Model model;
  NumericBackend backend(model.arm());
  const auto& witness = test::corpus().reachable_random.front();
  for (std::size_t joint = 0; joint < kPandaJointCount; ++joint) {
    for (const double invalid :
         {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
          -std::numeric_limits<double>::infinity()}) {
      SolveInput input = test::exactInput(witness);
      input.seed_positions[joint] = invalid;
      SolveOutput output;
      backend.solve(input, output);
      EXPECT_EQ(output.status, SolveStatus::BadRequest);
      EXPECT_FALSE(output.diagnostics.solver_invoked);
    }
  }
}

TEST(SeedValidationTest, DistinguishesFiniteOutOfLimitSeeds) {
  test::Model model;
  NumericBackend backend(model.arm());
  SolveInput input = test::exactInput(test::corpus().reachable_random.front());
  input.seed_positions[3] = 0.0;
  SolveOutput output;
  backend.solve(input, output);
  EXPECT_EQ(output.status, SolveStatus::SeedOutOfLimits);
  EXPECT_FALSE(output.diagnostics.solver_invoked);
}

TEST(SeedValidationTest, PhysicalLimitBoundariesAreInclusiveAtZeroMargin) {
  test::Model model;
  ForwardKinematics fk(model.arm().flange_chain());
  NumericBackend backend(model.arm());
  for (const bool upper : {false, true}) {
    const auto& boundary = upper ? kPandaPositionUpperLimits : kPandaPositionLowerLimits;
    SolveInput input;
    input.target_pose = fk.compute(boundary);
    input.seed_positions = boundary;
    input.fixed_q7 = boundary[6];
    input.position_tolerance = 1.0e-10;
    input.orientation_tolerance = 1.0e-10;
    input.numeric_eps = 1.0e-13;
    SolveOutput output;
    backend.solve(input, output);
    EXPECT_NE(output.status, SolveStatus::SeedOutOfLimits) << output.message;
    EXPECT_NE(output.status, SolveStatus::BadRequest) << output.message;
  }
}

TEST(SeedValidationTest, RejectsMalformedFramesEnumsAndSolverSettings) {
  test::Model model;
  NumericBackend backend(model.arm());
  const SolveInput valid = test::exactInput(test::corpus().reachable_random.front());

  std::array<SolveInput, 13> invalid{};
  invalid.fill(valid);
  invalid[0].target_pose.p[0] = std::numeric_limits<double>::quiet_NaN();
  invalid[1].target_pose.M(1, 1) = std::numeric_limits<double>::infinity();
  invalid[2].reference_frame = static_cast<ReachabilityReferenceFrame>(99);
  invalid[3].tip_frame = static_cast<ReachabilityTipFrame>(99);
  invalid[4].joint_limit_margin = -1.0e-6;
  invalid[5].position_tolerance = 0.0;
  invalid[6].orientation_tolerance = std::numeric_limits<double>::infinity();
  invalid[7].numeric_eps = std::numeric_limits<double>::quiet_NaN();
  invalid[8].numeric_max_iterations = 0;
  invalid[9].fixed_q7 = std::numeric_limits<double>::infinity();
  invalid[10].numeric_eps = 1.0e-2 + 1.0e-12;
  invalid[11].numeric_max_iterations = 9;
  invalid[12].numeric_max_iterations = 2001;
  for (const auto& input : invalid) {
    SolveOutput output;
    backend.solve(input, output);
    EXPECT_EQ(output.status, SolveStatus::BadRequest);
    EXPECT_FALSE(output.diagnostics.solver_invoked);
  }
}

TEST(SeedValidationTest, RejectsAMarginThatEmptiesAnyJointInterval) {
  test::Model model;
  NumericBackend backend(model.arm());
  SolveInput input = test::exactInput(test::corpus().reachable_random.front());
  input.joint_limit_margin = 2.0;
  SolveOutput output;
  backend.solve(input, output);
  EXPECT_EQ(output.status, SolveStatus::BadRequest);
  EXPECT_FALSE(output.diagnostics.solver_invoked);
}

TEST(SeedValidationTest, RejectsFixedQ7BeyondEitherAdjustedBoundary) {
  test::Model model;
  NumericBackend backend(model.arm());
  for (const double fixed_q7 :
       {kPandaPositionLowerLimits[6] - 1.0e-12, kPandaPositionUpperLimits[6] + 1.0e-12}) {
    SolveInput input = test::exactInput(test::corpus().reachable_random.front());
    input.fixed_q7 = fixed_q7;
    SolveOutput output;
    backend.solve(input, output);
    EXPECT_EQ(output.status, SolveStatus::BadRequest);
    EXPECT_FALSE(output.diagnostics.solver_invoked);
  }
}

TEST(SeedValidationTest, FixedQ7IsBitExactAndIsNeverClampedOrOptimised) {
  test::Model model;
  ForwardKinematics fk(model.arm().flange_chain());
  NumericBackend backend(model.arm());
  auto target_joints = test::corpus().reachable_random.front().seed;
  const double fixed_q7 = target_joints[6];
  SolveInput input;
  input.target_pose = fk.compute(target_joints);
  input.seed_positions = target_joints;
  input.seed_positions[6] = std::nextafter(fixed_q7, kPandaPositionUpperLimits[6]);
  input.fixed_q7 = fixed_q7;
  input.position_tolerance = 1.0e-10;
  input.orientation_tolerance = 1.0e-10;
  input.numeric_eps = 1.0e-13;
  SolveOutput output;
  backend.solve(input, output);
  ASSERT_EQ(output.status, SolveStatus::Success) << output.message;
  ASSERT_EQ(output.solutions.size(), 1U);
  EXPECT_EQ(std::memcmp(&output.solutions.front().positions[6], &fixed_q7, sizeof(fixed_q7)), 0);
  EXPECT_EQ(std::memcmp(&output.solutions.front().redundancy_value, &fixed_q7, sizeof(fixed_q7)),
            0);
}

}  // namespace
}  // namespace franka_ik
