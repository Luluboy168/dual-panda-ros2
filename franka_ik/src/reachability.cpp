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

#include "franka_ik/reachability.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <kdl/joint.hpp>

namespace franka_ik {
namespace {

constexpr std::size_t kShoulderSegmentCount = 2;
constexpr std::size_t kWristSegmentCount = 6;
constexpr std::size_t kJoint7SegmentIndex = 6;
constexpr std::size_t kFlangeSegmentIndex = 7;
constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kGeometryRoundoffTolerance = 1.0e-12;

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

double distance(const KDL::Vector& left, const KDL::Vector& right) noexcept {
  return (left - right).Norm();
}

KDL::Frame frameAtSegmentEnd(const KDL::Chain& chain,
                             const std::array<double, kPandaJointCount>& joint_positions,
                             const std::size_t segment_count) {
  if (segment_count > chain.getNrOfSegments()) {
    throw std::invalid_argument("requested segment is outside the Panda chain");
  }

  KDL::Frame result = KDL::Frame::Identity();
  std::size_t joint_index = 0;
  for (std::size_t segment_index = 0; segment_index < segment_count; ++segment_index) {
    const auto& segment = chain.getSegment(static_cast<unsigned int>(segment_index));
    double position = 0.0;
    if (segment.getJoint().getType() != KDL::Joint::Fixed) {
      if (joint_index >= joint_positions.size()) {
        throw std::invalid_argument("Panda chain has more than seven moving joints");
      }
      position = joint_positions[joint_index++];
    }
    result = result * segment.pose(position);
  }
  return result;
}

double squaredElbowDistance(const double constant,
                            const double cosine_coefficient,
                            const double sine_coefficient,
                            const double q4) noexcept {
  return constant + cosine_coefficient * std::cos(q4) + sine_coefficient * std::sin(q4);
}

double nonnegativeSquareRoot(const double value) noexcept {
  return std::sqrt(std::max(0.0, value));
}

std::pair<double, double> elbowDistanceRange(const double constant,
                                             const double cosine_coefficient,
                                             const double sine_coefficient,
                                             const double lower,
                                             const double upper) noexcept {
  double minimum = squaredElbowDistance(constant, cosine_coefficient, sine_coefficient, lower);
  double maximum = minimum;
  const auto include = [&](const double angle) {
    const double value =
        squaredElbowDistance(constant, cosine_coefficient, sine_coefficient, angle);
    minimum = std::min(minimum, value);
    maximum = std::max(maximum, value);
  };
  include(upper);

  const double maximum_angle = std::atan2(sine_coefficient, cosine_coefficient);
  for (const double critical_angle : {maximum_angle, maximum_angle + kPi}) {
    const double turns = std::ceil((lower - critical_angle) / kTwoPi);
    const double candidate = critical_angle + turns * kTwoPi;
    if (candidate >= lower && candidate <= upper) {
      include(candidate);
    }
  }

  return {nonnegativeSquareRoot(minimum), nonnegativeSquareRoot(maximum)};
}

bool outsideWithTolerance(const double value,
                          const double lower,
                          const double upper,
                          const double tolerance) noexcept {
  return value < lower - tolerance || value > upper + tolerance;
}

}  // namespace

ReachabilityClassifier::ReachabilityClassifier(const ArmChain& arm)
    : flange_chain_(arm.flange_chain()),
      root_to_base_(arm.root_to_base()),
      flange_to_hand_tcp_(arm.flange_to_hand_tcp()),
      q4_lower_(arm.position_lower()[3]),
      q4_upper_(arm.position_upper()[3]),
      q7_lower_(arm.position_lower()[6]),
      q7_upper_(arm.position_upper()[6]) {
  if (flange_chain_.getNrOfSegments() != kFlangeSegmentIndex + 1 ||
      flange_chain_.getNrOfJoints() != kPandaJointCount) {
    throw std::invalid_argument("reachability requires the canonical eight-segment Panda chain");
  }
  for (std::size_t index = 0; index < kPandaJointCount; ++index) {
    if (flange_chain_.getSegment(static_cast<unsigned int>(index)).getJoint().getType() ==
        KDL::Joint::Fixed) {
      throw std::invalid_argument("reachability requires seven moving Panda joint segments");
    }
  }
  if (flange_chain_.getSegment(kFlangeSegmentIndex).getJoint().getType() != KDL::Joint::Fixed) {
    throw std::invalid_argument("reachability requires a fixed flange segment");
  }

  const std::array<double, kPandaJointCount> zero_positions{};
  shoulder_position_ = frameAtSegmentEnd(flange_chain_, zero_positions, kShoulderSegmentCount).p;

  // Triangle inequality over the literal URDF segment offsets. Starting at link2 deliberately
  // removes the base-to-shoulder offset, leaving an orientation-independent outward bound.
  for (std::size_t segment_index = kShoulderSegmentCount;
       segment_index < flange_chain_.getNrOfSegments(); ++segment_index) {
    maximum_shoulder_to_flange_distance_ +=
        flange_chain_.getSegment(static_cast<unsigned int>(segment_index)).pose(0.0).p.Norm();
  }

  const auto distance_squared_at_q4 = [&](const double q4) {
    auto joint_positions = zero_positions;
    joint_positions[3] = q4;
    const auto wrist = frameAtSegmentEnd(flange_chain_, joint_positions, kWristSegmentCount).p;
    const double wrist_distance = distance(shoulder_position_, wrist);
    return wrist_distance * wrist_distance;
  };
  const double at_zero = distance_squared_at_q4(0.0);
  const double at_half_pi = distance_squared_at_q4(kPi / 2.0);
  const double at_pi = distance_squared_at_q4(kPi);
  elbow_distance_squared_constant_ = (at_zero + at_pi) / 2.0;
  elbow_distance_squared_cosine_ = (at_zero - at_pi) / 2.0;
  elbow_distance_squared_sine_ = at_half_pi - elbow_distance_squared_constant_;

  const double amplitude = std::hypot(elbow_distance_squared_cosine_, elbow_distance_squared_sine_);
  if (!std::isfinite(maximum_shoulder_to_flange_distance_) ||
      maximum_shoulder_to_flange_distance_ <= 0.0 ||
      !std::isfinite(elbow_distance_squared_constant_) || !std::isfinite(amplitude) ||
      elbow_distance_squared_constant_ + kGeometryRoundoffTolerance < amplitude) {
    throw std::invalid_argument("Panda URDF produced invalid reachability geometry");
  }
}

ReachabilityClass ReachabilityClassifier::classify(const ReachabilityQuery& query) const noexcept {
  if (!isFinite(query.target_pose) || !std::isfinite(query.fixed_q7) ||
      !std::isfinite(query.joint_limit_margin) || query.joint_limit_margin < 0.0 ||
      !std::isfinite(query.distance_tolerance) || query.distance_tolerance < 0.0) {
    return ReachabilityClass::Indeterminate;
  }

  const double q4_lower = q4_lower_ + query.joint_limit_margin;
  const double q4_upper = q4_upper_ - query.joint_limit_margin;
  const double q7_lower = q7_lower_ + query.joint_limit_margin;
  const double q7_upper = q7_upper_ - query.joint_limit_margin;
  if (q4_lower > q4_upper || q7_lower > q7_upper || query.fixed_q7 < q7_lower ||
      query.fixed_q7 > q7_upper) {
    return ReachabilityClass::Indeterminate;
  }

  KDL::Frame base_to_target;
  switch (query.reference_frame) {
    case ReachabilityReferenceFrame::ArmBase:
      base_to_target = query.target_pose;
      break;
    case ReachabilityReferenceFrame::UrdfRoot:
      // root_T_target = root_T_base * base_T_target
      base_to_target = root_to_base_.Inverse() * query.target_pose;
      break;
    default:
      return ReachabilityClass::Indeterminate;
  }

  KDL::Frame base_to_flange;
  switch (query.tip_frame) {
    case ReachabilityTipFrame::Flange:
      base_to_flange = base_to_target;
      break;
    case ReachabilityTipFrame::HandTcp:
      if (!flange_to_hand_tcp_.has_value()) {
        return ReachabilityClass::Indeterminate;
      }
      // base_T_hand = base_T_flange * flange_T_hand
      base_to_flange = base_to_target * flange_to_hand_tcp_->Inverse();
      break;
    default:
      return ReachabilityClass::Indeterminate;
  }

  const double shoulder_to_flange = distance(shoulder_position_, base_to_flange.p);
  if (shoulder_to_flange > maximum_shoulder_to_flange_distance_ + query.distance_tolerance) {
    return ReachabilityClass::GeometricallyUnreachable;
  }

  const KDL::Frame link6_to_flange =
      flange_chain_.getSegment(kJoint7SegmentIndex).pose(query.fixed_q7) *
      flange_chain_.getSegment(kFlangeSegmentIndex).pose(0.0);
  const KDL::Frame base_to_link6 = base_to_flange * link6_to_flange.Inverse();
  const double shoulder_to_wrist = distance(shoulder_position_, base_to_link6.p);

  const double amplitude = std::hypot(elbow_distance_squared_cosine_, elbow_distance_squared_sine_);
  const double geometric_minimum =
      nonnegativeSquareRoot(elbow_distance_squared_constant_ - amplitude);
  const double geometric_maximum =
      nonnegativeSquareRoot(elbow_distance_squared_constant_ + amplitude);
  if (outsideWithTolerance(shoulder_to_wrist, geometric_minimum, geometric_maximum,
                           query.distance_tolerance)) {
    return ReachabilityClass::GeometricallyUnreachable;
  }

  const auto limited_range =
      elbowDistanceRange(elbow_distance_squared_constant_, elbow_distance_squared_cosine_,
                         elbow_distance_squared_sine_, q4_lower, q4_upper);
  if (outsideWithTolerance(shoulder_to_wrist, limited_range.first, limited_range.second,
                           query.distance_tolerance)) {
    return ReachabilityClass::JointLimitsExcluded;
  }

  return ReachabilityClass::Indeterminate;
}

}  // namespace franka_ik
