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

#include <array>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <kdl/frames.hpp>
#include <kdl/joint.hpp>

#ifndef FRANKA_IK_CORPUS_FILE
#error "FRANKA_IK_CORPUS_FILE must name the checked-in Stage-0 corpus"
#endif

#ifndef PANDA_IK_SINGLE_TEST_URDF
#error "PANDA_IK_SINGLE_TEST_URDF must name the rendered single-arm wrapper"
#endif

#ifndef PANDA_IK_DUAL_TEST_URDF
#error "PANDA_IK_DUAL_TEST_URDF must name the rendered dual-arm wrapper"
#endif

namespace franka_ik {
namespace {

constexpr std::size_t kShoulderSegmentCount = 2;
constexpr std::size_t kWristSegmentCount = 6;
constexpr std::size_t kJoint7SegmentIndex = 6;
constexpr std::size_t kFlangeSegmentIndex = 7;
constexpr double kPi = 3.141592653589793238462643383279502884;

struct CorpusWitness {
  std::string id;
  KDL::Frame target_pose{KDL::Frame::Identity()};
  std::array<double, kPandaJointCount> seed_positions{};
  double shoulder_to_wrist_distance{0.0};
  double minimum_distance_with_valid_q4{0.0};
  double construction_q4{0.0};
};

struct CorpusData {
  std::vector<CorpusWitness> reachable;
  std::vector<CorpusWitness> unreachable_far;
  std::vector<CorpusWitness> unreachable_limits;
};

void require(const bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error("invalid Stage-0 corpus: " + message);
  }
}

std::string readFile(const std::string& path) {
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("cannot read '" + path + "'");
  }
  return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

std::array<double, kPandaJointCount> loadJointPositions(const YAML::Node& node,
                                                        const std::string& field) {
  require(node.IsSequence() && node.size() == kPandaJointCount,
          field + " must contain seven values");
  std::array<double, kPandaJointCount> result{};
  for (std::size_t index = 0; index < result.size(); ++index) {
    result[index] = node[index].as<double>();
    require(std::isfinite(result[index]), field + " contains a non-finite value");
  }
  return result;
}

KDL::Frame loadPose(const YAML::Node& node) {
  const auto position = node["position"];
  const auto orientation = node["orientation_xyzw"];
  require(position.IsSequence() && position.size() == 3, "pose position must have three values");
  require(orientation.IsSequence() && orientation.size() == 4,
          "pose orientation must have four values");

  std::array<double, 3> translation{};
  std::array<double, 4> quaternion{};
  for (std::size_t index = 0; index < translation.size(); ++index) {
    translation[index] = position[index].as<double>();
    require(std::isfinite(translation[index]), "pose position contains a non-finite value");
  }
  double quaternion_norm_squared = 0.0;
  for (std::size_t index = 0; index < quaternion.size(); ++index) {
    quaternion[index] = orientation[index].as<double>();
    require(std::isfinite(quaternion[index]), "pose orientation contains a non-finite value");
    quaternion_norm_squared += quaternion[index] * quaternion[index];
  }
  require(std::abs(std::sqrt(quaternion_norm_squared) - 1.0) <= 2.0e-15,
          "pose quaternion is not unit length");

  return {KDL::Rotation::Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]),
          KDL::Vector(translation[0], translation[1], translation[2])};
}

CorpusData loadCorpus() {
  const auto root = YAML::LoadFile(FRANKA_IK_CORPUS_FILE);
  require(root["schema_version"].as<int>() == 1, "schema_version must be one");
  require(root["generator"]["base_frame"].as<std::string>() == "panda_link0",
          "generator base frame changed");
  require(root["generator"]["tip_frame"].as<std::string>() == "panda_link8",
          "generator tip frame changed");
  require(root["counts"]["single_poses"].as<std::size_t>() == 3100, "single-pose count changed");
  require(root["counts"]["unreachable_far"].as<std::size_t>() == 200, "far-witness count changed");
  require(root["counts"]["unreachable_limits"].as<std::size_t>() == 200,
          "limit-witness count changed");

  const auto corpus_lower =
      loadJointPositions(root["joint_limits"]["position_lower"], "joint_limits.position_lower");
  const auto corpus_upper =
      loadJointPositions(root["joint_limits"]["position_upper"], "joint_limits.position_upper");
  require(corpus_lower == kPandaPositionLowerLimits, "lower limits differ from Panda constants");
  require(corpus_upper == kPandaPositionUpperLimits, "upper limits differ from Panda constants");

  CorpusData result;
  const auto single_poses = root["single_poses"];
  require(single_poses.IsSequence() && single_poses.size() == 3100,
          "single_poses must contain 3100 entries");
  for (const auto& entry : single_poses) {
    const auto category = entry["category"].as<std::string>();
    const bool keep_reachable = category == "reachable_random" && result.reachable.size() < 32;
    if (!keep_reachable && category != "unreachable_far" && category != "unreachable_limits") {
      continue;
    }

    CorpusWitness witness;
    witness.id = entry["id"].as<std::string>();
    witness.target_pose = loadPose(entry["target_pose"]);
    witness.seed_positions = loadJointPositions(entry["seed_positions"], "seed_positions");
    for (std::size_t index = 0; index < witness.seed_positions.size(); ++index) {
      require(witness.seed_positions[index] >= kPandaPositionLowerLimits[index] &&
                  witness.seed_positions[index] <= kPandaPositionUpperLimits[index],
              witness.id + " seed is outside the Panda limits");
    }

    if (category == "reachable_random") {
      require(entry["expected_result"].as<std::string>() == "RESULT_SUCCESS",
              witness.id + " expected result changed");
      result.reachable.push_back(std::move(witness));
      continue;
    }
    if (category == "unreachable_far") {
      require(entry["expected_result"].as<std::string>() == "RESULT_UNREACHABLE",
              witness.id + " expected result changed");
      require(entry["translation_distance"].as<double>() == 1.5,
              witness.id + " translation distance changed");
      result.unreachable_far.push_back(std::move(witness));
      continue;
    }

    require(entry["expected_result"].as<std::string>() == "RESULT_LIMITS_VIOLATED",
            witness.id + " expected result changed");
    const auto construction =
        loadJointPositions(entry["construction_joint_positions"], "construction joints");
    witness.construction_q4 = construction[3];
    witness.shoulder_to_wrist_distance = entry["shoulder_to_wrist_distance"].as<double>();
    witness.minimum_distance_with_valid_q4 = entry["minimum_distance_with_valid_q4"].as<double>();
    require(std::isfinite(witness.shoulder_to_wrist_distance) &&
                std::isfinite(witness.minimum_distance_with_valid_q4),
            witness.id + " folded-distance metadata is non-finite");
    require(witness.shoulder_to_wrist_distance < witness.minimum_distance_with_valid_q4 - 0.10,
            witness.id + " lost the q4 folded-distance proof margin");
    require(witness.construction_q4 < kPandaPositionLowerLimits[3] ||
                witness.construction_q4 > kPandaPositionUpperLimits[3],
            witness.id + " construction q4 is no longer outside the Panda limit");
    require(entry["fixed_redundancy_value"].as<double>() == witness.seed_positions[6],
            witness.id + " fixed q7 differs from its request seed");
    result.unreachable_limits.push_back(std::move(witness));
  }

  require(result.reachable.size() == 32, "did not load 32 reachable audit witnesses");
  require(result.unreachable_far.size() == 200, "did not load all far witnesses");
  require(result.unreachable_limits.size() == 200, "did not load all limit witnesses");
  return result;
}

const CorpusData& corpus() {
  static const CorpusData data = loadCorpus();
  return data;
}

KDL::Frame frameAtSegmentEnd(const KDL::Chain& chain,
                             const std::array<double, kPandaJointCount>& joint_positions,
                             const std::size_t segment_count) {
  KDL::Frame result = KDL::Frame::Identity();
  std::size_t joint_index = 0;
  for (std::size_t segment_index = 0; segment_index < segment_count; ++segment_index) {
    const auto& segment = chain.getSegment(static_cast<unsigned int>(segment_index));
    const bool fixed = segment.getJoint().getType() == KDL::Joint::Fixed;
    result = result * segment.pose(fixed ? 0.0 : joint_positions.at(joint_index++));
  }
  return result;
}

double maximumShoulderToFlangePath(const ArmChain& arm) {
  double result = 0.0;
  for (std::size_t index = kShoulderSegmentCount; index < arm.flange_chain().getNrOfSegments();
       ++index) {
    result += arm.flange_chain().getSegment(static_cast<unsigned int>(index)).pose(0.0).p.Norm();
  }
  return result;
}

double maximumShoulderToWristDistance(const ArmChain& arm) {
  const auto squared_distance_at_q4 = [&](const double q4) {
    std::array<double, kPandaJointCount> joints{};
    joints[3] = q4;
    const auto shoulder = frameAtSegmentEnd(arm.flange_chain(), joints, kShoulderSegmentCount).p;
    const auto wrist = frameAtSegmentEnd(arm.flange_chain(), joints, kWristSegmentCount).p;
    const double distance = (shoulder - wrist).Norm();
    return distance * distance;
  };
  const double at_zero = squared_distance_at_q4(0.0);
  const double at_half_pi = squared_distance_at_q4(kPi / 2.0);
  const double at_pi = squared_distance_at_q4(kPi);
  const double constant = (at_zero + at_pi) / 2.0;
  const double cosine = (at_zero - at_pi) / 2.0;
  const double sine = at_half_pi - constant;
  return std::sqrt(constant + std::hypot(cosine, sine));
}

ReachabilityQuery baseFlangeQuery(const CorpusWitness& witness) {
  ReachabilityQuery query;
  query.target_pose = witness.target_pose;
  query.reference_frame = ReachabilityReferenceFrame::ArmBase;
  query.tip_frame = ReachabilityTipFrame::Flange;
  query.fixed_q7 = witness.seed_positions[6];
  return query;
}

KDL::Frame targetAtShoulderToWristDistance(const ArmChain& arm,
                                           const double fixed_q7,
                                           const double wrist_distance) {
  const std::array<double, kPandaJointCount> zero_positions{};
  const auto shoulder =
      frameAtSegmentEnd(arm.flange_chain(), zero_positions, kShoulderSegmentCount).p;
  const KDL::Frame base_to_link6(KDL::Rotation::Identity(),
                                 shoulder + KDL::Vector(wrist_distance, 0.0, 0.0));
  const KDL::Frame link6_to_flange =
      arm.flange_chain().getSegment(kJoint7SegmentIndex).pose(fixed_q7) *
      arm.flange_chain().getSegment(kFlangeSegmentIndex).pose(0.0);
  return base_to_link6 * link6_to_flange;
}

std::string addFixedHandTcp(std::string urdf) {
  const auto robot_end = urdf.rfind("</robot>");
  if (robot_end == std::string::npos) {
    throw std::runtime_error("rendered single-arm URDF has no closing robot tag");
  }
  const std::string hand = R"(
  <link name="panda_hand"/>
  <joint name="panda_hand_joint" type="fixed">
    <parent link="panda_link8"/><child link="panda_hand"/>
    <origin xyz="0 0 0" rpy="0 0 -0.7853981633974483"/>
  </joint>
  <link name="panda_hand_tcp"/>
  <joint name="panda_hand_tcp_joint" type="fixed">
    <parent link="panda_hand"/><child link="panda_hand_tcp"/>
    <origin xyz="0 0 0.1034" rpy="0 0 0"/>
  </joint>
)";
  urdf.insert(robot_end, hand);
  return urdf;
}

std::string replaceOnce(std::string text,
                        const std::string& expected,
                        const std::string& replacement) {
  const auto position = text.find(expected);
  if (position == std::string::npos) {
    throw std::runtime_error("rendered URDF does not contain expected mount origin");
  }
  text.replace(position, expected.size(), replacement);
  return text;
}

TEST(UnreachableClassificationTest, FarCorpusIsOutsideUrdfPathLengthBound) {
  const RobotChains chains(readFile(PANDA_IK_SINGLE_TEST_URDF), {"panda"});
  const auto& arm = chains.arm("panda");
  const ReachabilityClassifier classifier(arm);
  const std::array<double, kPandaJointCount> zero_positions{};
  const auto shoulder =
      frameAtSegmentEnd(arm.flange_chain(), zero_positions, kShoulderSegmentCount).p;
  const double outward_bound = maximumShoulderToFlangePath(arm);

  for (const auto& witness : corpus().unreachable_far) {
    EXPECT_GT((witness.target_pose.p - shoulder).Norm(), outward_bound + 0.05) << witness.id;
    EXPECT_EQ(classifier.classify(baseFlangeQuery(witness)),
              ReachabilityClass::GeometricallyUnreachable)
        << witness.id;
  }
}

TEST(UnreachableClassificationTest, FoldedCorpusIsExcludedByJoint4DistanceRange) {
  const RobotChains chains(readFile(PANDA_IK_SINGLE_TEST_URDF), {"panda"});
  const auto& arm = chains.arm("panda");
  const ReachabilityClassifier classifier(arm);
  const std::array<double, kPandaJointCount> zero_positions{};
  const auto shoulder =
      frameAtSegmentEnd(arm.flange_chain(), zero_positions, kShoulderSegmentCount).p;

  for (const auto& witness : corpus().unreachable_limits) {
    const KDL::Frame link6_to_flange =
        arm.flange_chain().getSegment(kJoint7SegmentIndex).pose(witness.seed_positions[6]) *
        arm.flange_chain().getSegment(kFlangeSegmentIndex).pose(0.0);
    const auto target_link6 = witness.target_pose * link6_to_flange.Inverse();
    EXPECT_NEAR((target_link6.p - shoulder).Norm(), witness.shoulder_to_wrist_distance, 1.0e-12)
        << witness.id;
    EXPECT_LT(witness.shoulder_to_wrist_distance, witness.minimum_distance_with_valid_q4 - 0.10)
        << witness.id;
    EXPECT_TRUE(witness.construction_q4 < kPandaPositionLowerLimits[3] ||
                witness.construction_q4 > kPandaPositionUpperLimits[3])
        << witness.id;
    EXPECT_EQ(classifier.classify(baseFlangeQuery(witness)), ReachabilityClass::JointLimitsExcluded)
        << witness.id;
  }
}

TEST(UnreachableClassificationTest, DistanceToleranceMakesExclusionBoundariesConservative) {
  const RobotChains chains(readFile(PANDA_IK_SINGLE_TEST_URDF), {"panda"});
  const auto& arm = chains.arm("panda");
  const ReachabilityClassifier classifier(arm);
  constexpr double tolerance = 1.0e-6;
  constexpr double fixed_q7 = 0.0;

  ReachabilityQuery query;
  query.fixed_q7 = fixed_q7;
  query.distance_tolerance = tolerance;

  const double geometric_maximum = maximumShoulderToWristDistance(arm);
  query.target_pose =
      targetAtShoulderToWristDistance(arm, fixed_q7, geometric_maximum + 0.5 * tolerance);
  EXPECT_EQ(classifier.classify(query), ReachabilityClass::Indeterminate);
  query.target_pose =
      targetAtShoulderToWristDistance(arm, fixed_q7, geometric_maximum + 2.0 * tolerance);
  EXPECT_EQ(classifier.classify(query), ReachabilityClass::GeometricallyUnreachable);

  const double limited_minimum = corpus().unreachable_limits.front().minimum_distance_with_valid_q4;
  query.target_pose =
      targetAtShoulderToWristDistance(arm, fixed_q7, limited_minimum - 0.5 * tolerance);
  EXPECT_EQ(classifier.classify(query), ReachabilityClass::Indeterminate);
  query.target_pose =
      targetAtShoulderToWristDistance(arm, fixed_q7, limited_minimum - 2.0 * tolerance);
  EXPECT_EQ(classifier.classify(query), ReachabilityClass::JointLimitsExcluded);
}

TEST(UnreachableClassificationTest, FlangeAndOptionalHandTipUseInverseFixedTransform) {
  const auto handless_urdf = readFile(PANDA_IK_SINGLE_TEST_URDF);
  const RobotChains handless_chains(handless_urdf, {"panda"});
  const ReachabilityClassifier handless_classifier(handless_chains.arm("panda"));

  auto unsupported = baseFlangeQuery(corpus().unreachable_limits.front());
  unsupported.tip_frame = ReachabilityTipFrame::HandTcp;
  EXPECT_EQ(handless_classifier.classify(unsupported), ReachabilityClass::Indeterminate);

  const RobotChains hand_chains(addFixedHandTcp(handless_urdf), {"panda"});
  const auto& hand_arm = hand_chains.arm("panda");
  ASSERT_TRUE(hand_arm.flange_to_hand_tcp().has_value());
  const ReachabilityClassifier hand_classifier(hand_arm);

  for (const auto& witness : corpus().unreachable_limits) {
    const auto flange_query = baseFlangeQuery(witness);
    auto hand_query = flange_query;
    hand_query.tip_frame = ReachabilityTipFrame::HandTcp;
    hand_query.target_pose = witness.target_pose * *hand_arm.flange_to_hand_tcp();
    EXPECT_EQ(hand_classifier.classify(flange_query), ReachabilityClass::JointLimitsExcluded)
        << witness.id;
    EXPECT_EQ(hand_classifier.classify(hand_query), ReachabilityClass::JointLimitsExcluded)
        << witness.id;
  }
}

TEST(UnreachableClassificationTest, RootTargetsUseInverseRootToBaseTransform) {
  const RobotChains dual_chains(readFile(PANDA_IK_DUAL_TEST_URDF), {"panda1", "panda2"});
  for (const auto& arm_id : {"panda1", "panda2"}) {
    const auto& arm = dual_chains.arm(arm_id);
    const ReachabilityClassifier classifier(arm);
    for (const auto& witness : corpus().unreachable_limits) {
      auto query = baseFlangeQuery(witness);
      query.reference_frame = ReachabilityReferenceFrame::UrdfRoot;
      query.target_pose = arm.root_to_base() * witness.target_pose;
      EXPECT_EQ(classifier.classify(query), ReachabilityClass::JointLimitsExcluded)
          << arm_id << " " << witness.id;
    }
  }

  const std::string original_origin = "<origin rpy=\"0 0 0\" xyz=\"0 +0.50 0\"/>";
  const std::string noncommuting_origin = "<origin rpy=\"0.3 -0.4 0.7\" xyz=\"0.4 -0.2 0.1\"/>";
  const RobotChains transformed_chains(
      replaceOnce(readFile(PANDA_IK_DUAL_TEST_URDF), original_origin, noncommuting_origin),
      {"panda1"});
  const auto& transformed_arm = transformed_chains.arm("panda1");
  const ReachabilityClassifier transformed_classifier(transformed_arm);
  for (const auto& witness : corpus().unreachable_limits) {
    auto query = baseFlangeQuery(witness);
    query.reference_frame = ReachabilityReferenceFrame::UrdfRoot;
    query.target_pose = transformed_arm.root_to_base() * witness.target_pose;
    EXPECT_EQ(transformed_classifier.classify(query), ReachabilityClass::JointLimitsExcluded)
        << witness.id;
  }
}

TEST(UnreachableClassificationTest, GeometryOnlyDoesNotClaimGlobalReachability) {
  const RobotChains chains(readFile(PANDA_IK_SINGLE_TEST_URDF), {"panda"});
  const ReachabilityClassifier classifier(chains.arm("panda"));
  for (const auto& witness : corpus().reachable) {
    EXPECT_EQ(classifier.classify(baseFlangeQuery(witness)), ReachabilityClass::Indeterminate)
        << witness.id;
  }
}

TEST(UnreachableClassificationTest, InvalidGeometryInputsRemainIndeterminate) {
  const RobotChains chains(readFile(PANDA_IK_SINGLE_TEST_URDF), {"panda"});
  const ReachabilityClassifier classifier(chains.arm("panda"));
  auto query = baseFlangeQuery(corpus().unreachable_limits.front());

  query.fixed_q7 = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(classifier.classify(query), ReachabilityClass::Indeterminate);
  query = baseFlangeQuery(corpus().unreachable_limits.front());
  query.distance_tolerance = -1.0;
  EXPECT_EQ(classifier.classify(query), ReachabilityClass::Indeterminate);
  query = baseFlangeQuery(corpus().unreachable_limits.front());
  query.joint_limit_margin = 2.0;
  EXPECT_EQ(classifier.classify(query), ReachabilityClass::Indeterminate);
}

}  // namespace
}  // namespace franka_ik
