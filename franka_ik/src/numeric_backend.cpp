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

#include "franka_ik/numeric_backend.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <exception>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <Eigen/Core>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/joint.hpp>
#include <kdl/segment.hpp>

namespace franka_ik {
namespace {

constexpr std::size_t kJoint7SegmentIndex = 6;
constexpr std::size_t kFlangeSegmentIndex = 7;
constexpr std::uint8_t kNumericBranch = 255;
constexpr std::uint16_t kMinimumIterations = 10;
constexpr std::uint16_t kMaximumIterations = 2000;
constexpr double kMaximumNumericEps = 1.0e-2;

bool isFinite(const KDL::Frame& frame) noexcept {
  for (unsigned int row = 0; row < 3; ++row) {
    if (!std::isfinite(frame.p[row])) {
      return false;
    }
    for (unsigned int column = 0; column < 3; ++column) {
      if (!std::isfinite(frame.M(row, column))) {
        return false;
      }
    }
  }
  return true;
}

bool validReferenceFrame(const ReachabilityReferenceFrame value) noexcept {
  return value == ReachabilityReferenceFrame::ArmBase ||
         value == ReachabilityReferenceFrame::UrdfRoot;
}

bool validTipFrame(const ReachabilityTipFrame value) noexcept {
  return value == ReachabilityTipFrame::Flange || value == ReachabilityTipFrame::HandTcp;
}

bool finiteJoints(const std::array<double, kPandaJointCount>& joints) noexcept {
  return std::all_of(joints.begin(), joints.end(),
                     [](const double value) { return std::isfinite(value); });
}

bool validMargins(const std::array<double, kPandaJointCount>& lower,
                  const std::array<double, kPandaJointCount>& upper,
                  const double margin) noexcept {
  if (!std::isfinite(margin) || margin < 0.0) {
    return false;
  }
  for (std::size_t index = 0; index < lower.size(); ++index) {
    if (lower[index] + margin > upper[index] - margin) {
      return false;
    }
  }
  return true;
}

bool withinLimits(const std::array<double, kPandaJointCount>& joints,
                  const std::array<double, kPandaJointCount>& lower,
                  const std::array<double, kPandaJointCount>& upper,
                  const double margin) noexcept {
  for (std::size_t index = 0; index < joints.size(); ++index) {
    if (joints[index] < lower[index] + margin || joints[index] > upper[index] - margin) {
      return false;
    }
  }
  return true;
}

double seedDistance(const std::array<double, kPandaJointCount>& candidate,
                    const std::array<double, kPandaJointCount>& seed) noexcept {
  double result = 0.0;
  for (std::size_t index = 0; index < candidate.size(); ++index) {
    result = std::max(result, std::abs(candidate[index] - seed[index]));
  }
  return result;
}

KDL::Frame targetInArmBase(const SolveInput& input,
                           const KDL::Frame& root_to_base,
                           const std::optional<KDL::Frame>& flange_to_hand_tcp) {
  KDL::Frame base_to_target = input.target_pose;
  if (input.reference_frame == ReachabilityReferenceFrame::UrdfRoot) {
    base_to_target = root_to_base.Inverse() * base_to_target;
  }
  if (input.tip_frame == ReachabilityTipFrame::HandTcp) {
    if (!flange_to_hand_tcp.has_value()) {
      throw std::invalid_argument("hand TCP requested but this chain has no hand TCP");
    }
    base_to_target = base_to_target * flange_to_hand_tcp->Inverse();
  }
  return base_to_target;
}

void resetOutput(SolveOutput& output) {
  output = SolveOutput{};
  output.solutions.clear();
}

}  // namespace

KDL::Chain makeFixedQ7Chain(const KDL::Chain& flange_chain, const double q7) {
  if (!std::isfinite(q7)) {
    throw std::invalid_argument("fixed q7 must be finite");
  }
  if (flange_chain.getNrOfSegments() != kFlangeSegmentIndex + 1 ||
      flange_chain.getNrOfJoints() != kPandaJointCount) {
    throw std::invalid_argument("numeric IK requires the canonical eight-segment Panda chain");
  }
  const auto& first_joint = flange_chain.getSegment(0).getJoint();
  const std::string first_suffix = "_joint1";
  if (first_joint.getName().size() <= first_suffix.size() ||
      first_joint.getName().compare(first_joint.getName().size() - first_suffix.size(),
                                    first_suffix.size(), first_suffix) != 0) {
    throw std::invalid_argument("numeric IK chain does not start with a canonical Panda joint1");
  }
  const std::string arm_id =
      first_joint.getName().substr(0, first_joint.getName().size() - first_suffix.size());
  for (std::size_t index = 0; index < kPandaJointCount; ++index) {
    const auto& segment = flange_chain.getSegment(static_cast<unsigned int>(index));
    if (segment.getJoint().getType() == KDL::Joint::Fixed ||
        segment.getJoint().getName() != arm_id + "_joint" + std::to_string(index + 1) ||
        segment.getName() != arm_id + "_link" + std::to_string(index + 1)) {
      throw std::invalid_argument("numeric IK requires seven moving Panda joint segments");
    }
  }
  const auto& flange_segment = flange_chain.getSegment(kFlangeSegmentIndex);
  if (flange_segment.getJoint().getType() != KDL::Joint::Fixed ||
      flange_segment.getJoint().getName() != arm_id + "_joint8" ||
      flange_segment.getName() != arm_id + "_link8") {
    throw std::invalid_argument("numeric IK requires a fixed flange segment");
  }

  KDL::Chain result;
  for (std::size_t index = 0; index < flange_chain.getNrOfSegments(); ++index) {
    const auto& segment = flange_chain.getSegment(static_cast<unsigned int>(index));
    if (index == kJoint7SegmentIndex) {
      const auto fixed_joint_name = segment.getJoint().getName() + "_fixed_redundancy";
      result.addSegment(KDL::Segment(segment.getName(),
                                     KDL::Joint(fixed_joint_name, KDL::Joint::Fixed),
                                     segment.pose(q7), segment.getInertia()));
    } else {
      result.addSegment(segment);
    }
  }
  if (result.getNrOfSegments() != flange_chain.getNrOfSegments() ||
      result.getNrOfJoints() != kPandaJointCount - 1) {
    throw std::logic_error("failed to construct the fixed-q7 Panda chain");
  }
  return result;
}

NumericBackend::NumericBackend(const ArmChain& arm)
    : flange_chain_(arm.flange_chain()),
      position_lower_(arm.position_lower()),
      position_upper_(arm.position_upper()),
      root_to_base_(arm.root_to_base()),
      flange_to_hand_tcp_(arm.flange_to_hand_tcp()),
      reachability_(arm),
      forward_kinematics_(flange_chain_) {
  // Validate the exact topology once at construction, including the request-local rewrite.
  (void)makeFixedQ7Chain(flange_chain_, 0.0);
}

void NumericBackend::solve(const SolveInput& input, SolveOutput& output) const noexcept {
  resetOutput(output);
  try {
    if (!isFinite(input.target_pose) || !validReferenceFrame(input.reference_frame) ||
        !validTipFrame(input.tip_frame) || !finiteJoints(input.seed_positions) ||
        !std::isfinite(input.fixed_q7) ||
        !validMargins(position_lower_, position_upper_, input.joint_limit_margin) ||
        !std::isfinite(input.position_tolerance) || input.position_tolerance <= 0.0 ||
        !std::isfinite(input.orientation_tolerance) || input.orientation_tolerance <= 0.0 ||
        !std::isfinite(input.numeric_eps) || input.numeric_eps <= 0.0 ||
        input.numeric_eps > kMaximumNumericEps ||
        input.numeric_max_iterations < kMinimumIterations ||
        input.numeric_max_iterations > kMaximumIterations) {
      output.status = SolveStatus::BadRequest;
      output.message = "numeric IK input is malformed or has a non-positive solver setting";
      return;
    }
    if (input.tip_frame == ReachabilityTipFrame::HandTcp && !flange_to_hand_tcp_.has_value()) {
      output.status = SolveStatus::BadRequest;
      output.message = "hand TCP requested but this chain has no hand TCP";
      return;
    }
    if (!withinLimits(input.seed_positions, position_lower_, position_upper_,
                      input.joint_limit_margin)) {
      output.status = SolveStatus::SeedOutOfLimits;
      output.message = "seed is outside the margin-adjusted joint limits";
      return;
    }
    if (input.fixed_q7 < position_lower_[6] + input.joint_limit_margin ||
        input.fixed_q7 > position_upper_[6] - input.joint_limit_margin) {
      output.status = SolveStatus::BadRequest;
      output.message = "fixed q7 is outside its margin-adjusted joint limit";
      return;
    }

    ReachabilityQuery query;
    query.target_pose = input.target_pose;
    query.reference_frame = input.reference_frame;
    query.tip_frame = input.tip_frame;
    query.fixed_q7 = input.fixed_q7;
    query.joint_limit_margin = input.joint_limit_margin;
    const ReachabilityClass reachability = reachability_.classify(query);
    if (reachability == ReachabilityClass::GeometricallyUnreachable) {
      output.status = SolveStatus::GeometricallyUnreachable;
      output.message = "target is provably outside the geometric workspace";
      return;
    }
    if (reachability == ReachabilityClass::JointLimitsExcluded) {
      output.status = SolveStatus::JointLimitsViolated;
      output.message = "target is provably excluded by the margin-adjusted joint limits";
      return;
    }

    const KDL::Frame base_to_flange = targetInArmBase(input, root_to_base_, flange_to_hand_tcp_);
    KDL::Chain reduced_chain = makeFixedQ7Chain(flange_chain_, input.fixed_q7);
    KDL::JntArray reduced_seed(kPandaJointCount - 1);
    KDL::JntArray reduced_result(kPandaJointCount - 1);
    for (std::size_t index = 0; index < kPandaJointCount - 1; ++index) {
      reduced_seed(static_cast<unsigned int>(index)) = input.seed_positions[index];
    }

    // These are the explicit, reviewed KDL weights: orientation error contributes at 1/100 the
    // translation scale.  Tighten epsilon to the strictest acceptance threshold in weighted
    // task space so LMA cannot report convergence looser than the independent FK verifier.
    Eigen::Matrix<double, 6, 1> task_weights;
    task_weights << 1.0, 1.0, 1.0, 0.01, 0.01, 0.01;
    const double effective_eps =
        std::min({input.numeric_eps, input.position_tolerance, 0.01 * input.orientation_tolerance});
    KDL::ChainIkSolverPos_LMA solver(reduced_chain, task_weights, effective_eps,
                                     static_cast<int>(input.numeric_max_iterations), 1.0e-15);
    output.diagnostics.solver_invoked = true;
    const int return_code = solver.CartToJnt(reduced_seed, base_to_flange, reduced_result);
    output.diagnostics.solver_return_code = return_code;
    output.diagnostics.lma_last_difference = solver.lastDifference;
    output.diagnostics.lma_last_translational_difference = solver.lastTransDiff;
    output.diagnostics.lma_last_rotational_difference = solver.lastRotDiff;
    if (solver.lastNrOfIter < 0 ||
        solver.lastNrOfIter > static_cast<int>(input.numeric_max_iterations)) {
      output.status = SolveStatus::InternalError;
      output.message = "numeric solver reported an impossible iteration count";
      return;
    }
    output.iterations = static_cast<std::uint16_t>(solver.lastNrOfIter);

    const bool expected_return_code =
        return_code == KDL::SolverI::E_NOERROR ||
        return_code == KDL::SolverI::E_MAX_ITERATIONS_EXCEEDED ||
        return_code == KDL::ChainIkSolverPos_LMA::E_GRADIENT_JOINTS_TOO_SMALL ||
        return_code == KDL::ChainIkSolverPos_LMA::E_INCREMENT_JOINTS_TOO_SMALL;
    if (!expected_return_code) {
      output.status = SolveStatus::InternalError;
      output.message = "numeric solver returned an unexpected internal KDL error";
      return;
    }

    std::array<double, kPandaJointCount> candidate{};
    for (std::size_t index = 0; index < kPandaJointCount - 1; ++index) {
      candidate[index] = reduced_result(static_cast<unsigned int>(index));
    }
    // Assignment from the input preserves every bit of the requested redundancy value.
    candidate[6] = input.fixed_q7;
    output.diagnostics.raw_candidate_finite = finiteJoints(candidate);
    if (!output.diagnostics.raw_candidate_finite) {
      output.status = SolveStatus::InternalError;
      output.message = "numeric solver produced a non-finite candidate";
      return;
    }

    output.diagnostics.raw_candidate_within_limits =
        withinLimits(candidate, position_lower_, position_upper_, input.joint_limit_margin);
    const KDL::Frame verified_pose = forward_kinematics_.compute(candidate);
    const PoseError error = computePoseError(base_to_flange, verified_pose);
    output.diagnostics.raw_position_error = error.position;
    output.diagnostics.raw_orientation_error = error.orientation;
    output.diagnostics.raw_candidate_within_tolerance =
        error.position <= input.position_tolerance &&
        error.orientation <= input.orientation_tolerance;

    if (output.diagnostics.raw_candidate_within_limits &&
        output.diagnostics.raw_candidate_within_tolerance) {
      IkCandidate accepted;
      accepted.positions = candidate;
      accepted.redundancy_value = candidate[6];
      accepted.position_error = error.position;
      accepted.orientation_error = error.orientation;
      accepted.seed_distance = seedDistance(candidate, input.seed_positions);
      accepted.branch = kNumericBranch;
      output.solutions.push_back(accepted);
      output.status = SolveStatus::Success;
      output.message = "numeric IK found one FK-verified solution";
      return;
    }

    if (!output.diagnostics.raw_candidate_within_limits) {
      output.status = SolveStatus::NoAcceptableSolution;
      output.message = "numeric solver's sole candidate violates a margin-adjusted joint limit";
      return;
    }
    if (return_code == KDL::SolverI::E_MAX_ITERATIONS_EXCEEDED) {
      output.status = SolveStatus::IterationBudgetExhausted;
      output.message = "numeric IK exhausted its iteration budget";
      return;
    }
    output.status = SolveStatus::ToleranceNotMet;
    std::ostringstream message;
    message << "numeric solver stopped without meeting FK tolerances (KDL code " << return_code
            << ')';
    output.message = message.str();
  } catch (const std::exception& error) {
    output.status = SolveStatus::InternalError;
    output.message = std::string("numeric IK internal error: ") + error.what();
    output.solutions.clear();
  } catch (...) {
    output.status = SolveStatus::InternalError;
    output.message = "numeric IK internal error: unknown exception";
    output.solutions.clear();
  }
}

}  // namespace franka_ik
