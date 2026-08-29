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
#include <iostream>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/segment.hpp>

#include "franka_ik/forward_kinematics.hpp"
#include "franka_ik/numeric_backend.hpp"

namespace franka_ik {
namespace {

TEST(NumericBackendRoundTripTest, ExactSeedSolvesAllReachableCorpusWitnesses) {
  test::Model model;
  NumericBackend backend(model.arm());
  const auto witnesses = test::corpus().allReachable();

  std::size_t examined = 0;
  for (const auto* witness : witnesses) {
    SCOPED_TRACE(witness->id);
    const SolveInput input = test::exactInput(*witness);
    SolveOutput output;
    backend.solve(input, output);
    ASSERT_EQ(output.status, SolveStatus::Success) << output.message;
    ASSERT_EQ(output.solutions.size(), 1U);
    EXPECT_LE(output.solutions.front().position_error, 1.0e-9);
    EXPECT_LE(output.solutions.front().orientation_error, 1.0e-9);
    EXPECT_LE(test::maximumJointDelta(output.solutions.front().positions, witness->seed), 1.0e-9);
    EXPECT_EQ(std::memcmp(&output.solutions.front().positions[6], &input.fixed_q7,
                          sizeof(input.fixed_q7)),
              0);
    ++examined;
  }
  EXPECT_EQ(examined, 6700U);
}

TEST(NumericBackendRoundTripTest, DeterministicSmallPerturbationMeetsDecisionGate) {
  test::Model model;
  NumericBackend backend(model.arm());
  const auto witnesses = test::corpus().allReachable();
  std::size_t successes = 0;
  std::size_t raw_limit_violations = 0;

  for (std::size_t index = 0; index < witnesses.size(); ++index) {
    SolveInput input = test::exactInput(*witnesses[index]);
    input.seed_positions = test::perturbSeed(input.seed_positions, 0.01, index);
    // KDL's internal frame-difference epsilon is 1e-6 rad; the independently verified v1
    // default is comfortably stricter than the public 1e-3 rad default without asserting an
    // accuracy the installed KDL implementation cannot represent after a perturbed solve.
    input.orientation_tolerance = 1.0e-6;
    SolveOutput output;
    backend.solve(input, output);
    raw_limit_violations += output.diagnostics.solver_invoked &&
                            output.diagnostics.raw_candidate_finite &&
                            !output.diagnostics.raw_candidate_within_limits;
    if (output.status == SolveStatus::Success) {
      ++successes;
      EXPECT_LE(output.solutions.front().position_error, input.position_tolerance);
      EXPECT_LE(output.solutions.front().orientation_error, input.orientation_tolerance);
      EXPECT_EQ(std::memcmp(&output.solutions.front().positions[6], &input.fixed_q7,
                            sizeof(input.fixed_q7)),
                0);
    } else if (index == 0) {
      std::cout << "first perturbation failure: " << output.message
                << " rc=" << output.diagnostics.solver_return_code << " iter=" << output.iterations
                << " p=" << output.diagnostics.raw_position_error
                << " r=" << output.diagnostics.raw_orientation_error
                << " lma=" << output.diagnostics.lma_last_difference << '\n';
    }
  }

  const double success_rate = static_cast<double>(successes) / witnesses.size();
  std::cout << "numeric fixed-q7 perturbation: successes=" << successes << '/' << witnesses.size()
            << " rate=" << success_rate << " raw_limit_violations=" << raw_limit_violations << '\n';
  EXPECT_GE(success_rate, 0.99);
}

TEST(NumericBackendRoundTripTest, LargerPerturbationOutcomesAreMeasuredAndPostFiltered) {
  test::Model model;
  NumericBackend backend(model.arm());
  std::vector<const test::Witness*> witnesses;
  witnesses.reserve(2500);
  for (const auto* set : {&test::corpus().reachable_random, &test::corpus().reachable_near_limit}) {
    for (const auto& witness : *set) {
      witnesses.push_back(&witness);
    }
  }
  ASSERT_EQ(witnesses.size(), 2500U);

  for (const double magnitude : {0.1, 0.5}) {
    std::size_t successes = 0;
    std::size_t raw_limit_violations = 0;
    std::array<std::size_t, 9> status_counts{};
    for (std::size_t index = 0; index < witnesses.size(); ++index) {
      SolveInput input = test::exactInput(*witnesses[index]);
      input.seed_positions = test::perturbSeed(input.seed_positions, magnitude, index);
      input.orientation_tolerance = 1.0e-6;
      SolveOutput output;
      backend.solve(input, output);
      const auto status_index = static_cast<std::size_t>(output.status);
      ASSERT_LT(status_index, status_counts.size());
      ++status_counts[status_index];
      raw_limit_violations += output.diagnostics.solver_invoked &&
                              output.diagnostics.raw_candidate_finite &&
                              !output.diagnostics.raw_candidate_within_limits;
      if (output.status == SolveStatus::Success) {
        ++successes;
        ASSERT_EQ(output.solutions.size(), 1U);
        EXPECT_LE(output.solutions.front().position_error, input.position_tolerance);
        EXPECT_LE(output.solutions.front().orientation_error, input.orientation_tolerance);
      } else {
        EXPECT_TRUE(output.solutions.empty());
      }
    }
    std::cout << "numeric fixed-q7 perturbation " << magnitude << " rad: successes=" << successes
              << '/' << witnesses.size()
              << " rate=" << static_cast<double>(successes) / witnesses.size() << " no_acceptable="
              << status_counts[static_cast<std::size_t>(SolveStatus::NoAcceptableSolution)]
              << " tolerance="
              << status_counts[static_cast<std::size_t>(SolveStatus::ToleranceNotMet)]
              << " exhausted="
              << status_counts[static_cast<std::size_t>(SolveStatus::IterationBudgetExhausted)]
              << " raw_limit_violations=" << raw_limit_violations << '\n';
    EXPECT_GT(successes, 0U);
  }
}

TEST(NumericBackendRoundTripTest, ProvenExclusionsNeverInvokeLma) {
  test::Model model;
  NumericBackend backend(model.arm());
  std::size_t examined = 0;
  for (const auto* set : {&test::corpus().unreachable_far, &test::corpus().unreachable_limits}) {
    for (const auto& witness : *set) {
      SCOPED_TRACE(witness.id);
      const SolveInput input = test::exactInput(witness);
      SolveOutput output;
      backend.solve(input, output);
      const SolveStatus expected = witness.category == "unreachable_far"
                                       ? SolveStatus::GeometricallyUnreachable
                                       : SolveStatus::JointLimitsViolated;
      EXPECT_EQ(output.status, expected) << output.message;
      EXPECT_FALSE(output.diagnostics.solver_invoked);
      EXPECT_EQ(output.iterations, 0U);
      EXPECT_TRUE(output.solutions.empty());
      ++examined;
    }
  }
  EXPECT_EQ(examined, 400U);
}

TEST(NumericBackendRoundTripTest, DragTracesRemainOnTheContinuousFixedQ7Branch) {
  test::Model model;
  NumericBackend backend(model.arm());
  for (const auto& trace : test::corpus().drag_traces) {
    ASSERT_EQ(trace.size(), 200U);
    auto previous = trace.front().seed;
    const double fixed_q7 = previous[6];
    for (const auto& witness : trace) {
      SCOPED_TRACE(witness.id);
      SolveInput input = test::exactInput(witness);
      input.seed_positions = previous;
      input.fixed_q7 = fixed_q7;
      input.orientation_tolerance = 1.0e-6;
      SolveOutput output;
      backend.solve(input, output);
      ASSERT_EQ(output.status, SolveStatus::Success) << output.message;
      ASSERT_EQ(output.solutions.size(), 1U);
      const auto& current = output.solutions.front().positions;
      EXPECT_LE(test::maximumJointDelta(current, previous), 0.15);
      EXPECT_EQ(std::memcmp(&current[6], &fixed_q7, sizeof(fixed_q7)), 0);
      previous = current;
    }
  }
}

TEST(NumericBackendRoundTripTest, ReducedChainExactlyMatchesSevenJointForwardKinematics) {
  test::Model model;
  ForwardKinematics full_fk(model.arm().flange_chain());
  auto witnesses = test::corpus().allReachable();
  for (const auto& witness : test::corpus().unreachable_limits) {
    witnesses.push_back(&witness);
  }
  ASSERT_EQ(witnesses.size(), 6900U);
  for (std::size_t index = 0; index < witnesses.size(); ++index) {
    const auto& joints = witnesses[index]->seed;
    const KDL::Chain reduced = makeFixedQ7Chain(model.arm().flange_chain(), joints[6]);
    ASSERT_EQ(reduced.getNrOfJoints(), 6U);
    ASSERT_EQ(reduced.getNrOfSegments(), 8U);
    KDL::ChainFkSolverPos_recursive reduced_fk(reduced);
    KDL::JntArray reduced_joints(6);
    for (std::size_t joint = 0; joint < 6; ++joint) {
      reduced_joints(static_cast<unsigned int>(joint)) = joints[joint];
    }
    KDL::Frame reduced_pose;
    ASSERT_EQ(reduced_fk.JntToCart(reduced_joints, reduced_pose), 0);
    const PoseError error = computePoseError(full_fk.compute(joints), reduced_pose);
    EXPECT_LE(error.position, 1.0e-15) << witnesses[index]->id;
    EXPECT_LE(error.orientation, 1.0e-15) << witnesses[index]->id;
  }
}

TEST(NumericBackendRoundTripTest, FixedQ7RewriteRejectsNoncanonicalSegmentNames) {
  test::Model model;
  const auto& canonical = model.arm().flange_chain();
  KDL::Chain changed;
  for (std::size_t index = 0; index < canonical.getNrOfSegments(); ++index) {
    const auto& segment = canonical.getSegment(static_cast<unsigned int>(index));
    if (index == 6) {
      changed.addSegment(KDL::Segment("not_the_canonical_link7", segment.getJoint(),
                                      segment.getFrameToTip(), segment.getInertia()));
    } else {
      changed.addSegment(segment);
    }
  }
  EXPECT_THROW(makeFixedQ7Chain(changed, 0.0), std::invalid_argument);
}

TEST(NumericBackendRoundTripTest, EightLiteralGoldenPosesAndJointVectorsRoundTrip) {
  struct GoldenPose {
    std::array<double, 3> position;
    std::array<double, 4> orientation_xyzw;
  };
  constexpr std::array<std::array<double, kPandaJointCount>, 8> kGoldenJoints{{
      {{0.0, -0.4, 0.0, -1.8, 0.0, 1.4, 0.0}},
      {{1.2, 0.2, -1.1, -2.4, 0.8, 2.1, -0.9}},
      {{-1.2, -0.8, 1.1, -1.0, -0.8, 0.8, 0.9}},
      {{2.0, 1.0, 2.0, -2.8, -2.0, 3.0, 2.0}},
      {{-2.0, 1.0, -2.0, -2.8, 2.0, 3.0, -2.0}},
      {{2.0, -1.0, -2.0, -0.8, 2.0, 0.3, 1.5}},
      {{-2.0, -1.0, 2.0, -0.8, -2.0, 0.3, -1.5}},
      {{0.3, 0.7, -0.6, -1.5, 1.4, 2.8, -2.3}},
  }};
  // Literal targets were generated once with the Stage-0 independent pure-Python FK.  Keeping
  // both sides in this diff makes a chain-transform regression visible instead of recomputing
  // the expected pose with production ForwardKinematics at test time.
  constexpr std::array<GoldenPose, 8> kGoldenPoses{{
      {{{0.40532174286200329, -4.5875287682245374e-17, 0.69574927294311584}},
       {{1.0, -2.3844996321877859e-17, -6.9388939039072284e-17, 5.6398719906255639e-17}}},
      {{{0.4562262993845646, 0.070705895041835637, 0.25568319799348999}},
       {{-0.94256757846013195, -0.20798448405183795, 0.062547071546381075, 0.25376500600056295}}},
      {{{0.23022550750242551, 0.27602503192983874, 0.87996934444360564}},
       {{0.51410519939348942, -0.63267847385035891, 0.36509227389120219, 0.44957916347169924}}},
      {{{-0.1330793386437753, -0.2998416877159259, 0.41276161227075209}},
       {{0.25324670123593013, -0.81479132983859304, 0.49079828874773845, 0.17634692198977642}}},
      {{{-0.13307933864377533, 0.2998416877159259, 0.41276161227075214}},
       {{-0.25324670123593024, -0.81479132983859304, -0.49079828874773862, 0.17634692198977642}}},
      {{{0.43223432584725702, -0.37543257074213759, 0.67639103016683788}},
       {{-0.58538271465931091, -0.27478760038305827, 0.44965462312737953, 0.6161408702188087}}},
      {{{0.43223432584725691, 0.37543257074213765, 0.67639103016683799}},
       {{0.58538271465931091, -0.27478760038305833, -0.44965462312737969, 0.6161408702188087}}},
      {{{0.71817035842527932, -0.16610437769054123, 0.37520818475872292}},
       {{0.76791991483588617, 0.46636994575354662, 0.36673324638010363, 0.24146387741397202}}},
  }};
  test::Model model;
  NumericBackend backend(model.arm());
  for (std::size_t index = 0; index < kGoldenJoints.size(); ++index) {
    const auto& joints = kGoldenJoints[index];
    const auto& pose = kGoldenPoses[index];
    SolveInput input;
    input.target_pose =
        KDL::Frame(KDL::Rotation::Quaternion(pose.orientation_xyzw[0], pose.orientation_xyzw[1],
                                             pose.orientation_xyzw[2], pose.orientation_xyzw[3]),
                   KDL::Vector(pose.position[0], pose.position[1], pose.position[2]));
    input.seed_positions = joints;
    input.fixed_q7 = joints[6];
    input.position_tolerance = 1.0e-12;
    input.orientation_tolerance = 1.0e-12;
    input.numeric_eps = 1.0e-14;
    SolveOutput output;
    backend.solve(input, output);
    ASSERT_EQ(output.status, SolveStatus::Success) << output.message;
    ASSERT_EQ(output.solutions.size(), 1U);
    EXPECT_LE(test::maximumJointDelta(output.solutions.front().positions, joints), 1.0e-12);
  }
}

}  // namespace
}  // namespace franka_ik
