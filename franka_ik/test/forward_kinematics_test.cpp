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

#include <array>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <kdl/joint.hpp>
#include <kdl/segment.hpp>

#include "franka_ik/robot_chains.hpp"

#ifndef FRANKA_IK_CORPUS_FILE
#error "FRANKA_IK_CORPUS_FILE must name the checked-in Stage-0 corpus"
#endif

#ifndef PANDA_IK_SINGLE_TEST_URDF
#error "PANDA_IK_SINGLE_TEST_URDF must name the generated single-arm test URDF"
#endif

namespace franka_ik {
namespace {

constexpr double kCorpusTolerance = 1.0e-12;
constexpr std::size_t kExpectedCorpusWitnesses = 6900;

struct CorpusWitness {
  std::string id;
  std::string category;
  std::array<double, kPandaJointCount> joint_positions{};
  KDL::Frame target;
};

std::string trim(const std::string& input) {
  const auto first = input.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return {};
  }
  const auto last = input.find_last_not_of(" \t\r\n");
  return input.substr(first, last - first + 1);
}

std::string parseStringField(const std::string& line, const std::string& field) {
  const std::string prefix = '"' + field + "\": \"";
  const auto begin = line.find(prefix);
  if (begin == std::string::npos) {
    throw std::runtime_error("missing JSON string field '" + field + "'");
  }
  const auto value_begin = begin + prefix.size();
  const auto value_end = line.find('"', value_begin);
  if (value_end == std::string::npos) {
    throw std::runtime_error("unterminated JSON string field '" + field + "'");
  }
  return line.substr(value_begin, value_end - value_begin);
}

double parseNumberLine(const std::string& line, const std::size_t line_number) {
  std::string token = trim(line);
  if (!token.empty() && token.back() == ',') {
    token.pop_back();
  }
  std::size_t parsed = 0;
  try {
    const double value = std::stod(token, &parsed);
    if (parsed != token.size()) {
      throw std::runtime_error("trailing characters");
    }
    return value;
  } catch (const std::exception& error) {
    throw std::runtime_error("invalid JSON number at line " + std::to_string(line_number) + ": " +
                             error.what());
  }
}

template <std::size_t Size>
std::array<double, Size> readNumberArray(std::ifstream& input, std::size_t& line_number) {
  std::array<double, Size> result{};
  std::string line;
  for (double& value : result) {
    if (!std::getline(input, line)) {
      throw std::runtime_error("unexpected end of corpus while reading numeric array");
    }
    ++line_number;
    value = parseNumberLine(line, line_number);
  }
  if (!std::getline(input, line)) {
    throw std::runtime_error("unexpected end of corpus after numeric array");
  }
  ++line_number;
  const std::string closing = trim(line);
  if (closing != "]" && closing != "],") {
    throw std::runtime_error("invalid numeric-array terminator at line " +
                             std::to_string(line_number));
  }
  return result;
}

bool isFkWitnessCategory(const std::string& category) {
  return category == "reachable_random" || category == "reachable_near_limit" ||
         category == "singular" || category == "drag_traces" || category == "unreachable_limits";
}

std::vector<CorpusWitness> loadCorpusWitnesses(const std::string& path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("cannot open Stage-0 corpus '" + path + "'");
  }

  std::vector<CorpusWitness> result;
  result.reserve(kExpectedCorpusWitnesses);
  CorpusWitness current;
  std::array<double, 3> position{};
  std::array<double, 4> orientation{};
  bool selected = false;
  bool have_joint_positions = false;
  bool have_position = false;
  bool have_orientation = false;
  std::size_t line_number = 0;
  std::string line;

  while (std::getline(input, line)) {
    ++line_number;
    if (line.find("\"category\": \"") != std::string::npos) {
      if (selected) {
        throw std::runtime_error("incomplete corpus witness before line " +
                                 std::to_string(line_number));
      }
      const std::string category = parseStringField(line, "category");
      selected = isFkWitnessCategory(category);
      if (selected) {
        current = CorpusWitness{};
        current.category = category;
        have_joint_positions = false;
        have_position = false;
        have_orientation = false;
      }
      continue;
    }
    if (!selected) {
      continue;
    }

    if (line.find("\"id\": \"") != std::string::npos) {
      current.id = parseStringField(line, "id");
    } else if (current.category == "unreachable_limits" &&
               line.find("\"construction_joint_positions\": [") != std::string::npos) {
      current.joint_positions = readNumberArray<kPandaJointCount>(input, line_number);
      have_joint_positions = true;
    } else if (current.category != "unreachable_limits" &&
               line.find("\"seed_positions\": [") != std::string::npos) {
      current.joint_positions = readNumberArray<kPandaJointCount>(input, line_number);
      have_joint_positions = true;
    } else if (line.find("\"orientation_xyzw\": [") != std::string::npos) {
      orientation = readNumberArray<4>(input, line_number);
      have_orientation = true;
    } else if (line.find("\"position\": [") != std::string::npos) {
      position = readNumberArray<3>(input, line_number);
      have_position = true;
    }

    if (have_joint_positions && have_position && have_orientation) {
      if (current.id.empty()) {
        throw std::runtime_error("corpus witness ending at line " + std::to_string(line_number) +
                                 " has no id");
      }
      current.target = KDL::Frame(
          KDL::Rotation::Quaternion(orientation[0], orientation[1], orientation[2], orientation[3]),
          KDL::Vector(position[0], position[1], position[2]));
      result.push_back(current);
      selected = false;
    }
  }

  if (selected) {
    throw std::runtime_error("incomplete corpus witness at end of file");
  }
  if (result.size() != kExpectedCorpusWitnesses) {
    throw std::runtime_error("expected " + std::to_string(kExpectedCorpusWitnesses) +
                             " FK witnesses, loaded " + std::to_string(result.size()));
  }
  return result;
}

std::string readTextFile(const std::string& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("cannot open file '" + path + "'");
  }
  std::ostringstream contents;
  contents << input.rdbuf();
  return contents.str();
}

KDL::Chain makeMovingJointChain(const std::size_t joint_count) {
  KDL::Chain chain;
  for (std::size_t index = 0; index < joint_count; ++index) {
    chain.addSegment(
        KDL::Segment("link" + std::to_string(index + 1),
                     KDL::Joint("joint" + std::to_string(index + 1), KDL::Joint::RotZ)));
  }
  return chain;
}

static_assert(!std::is_copy_constructible_v<ForwardKinematics>);
static_assert(!std::is_copy_assignable_v<ForwardKinematics>);
static_assert(!std::is_move_constructible_v<ForwardKinematics>);
static_assert(!std::is_move_assignable_v<ForwardKinematics>);

TEST(ForwardKinematicsTest, RejectsChainsThatDoNotHaveSevenJoints) {
  EXPECT_THROW(ForwardKinematics(KDL::Chain{}), std::invalid_argument);
  EXPECT_THROW(ForwardKinematics(makeMovingJointChain(kPandaJointCount - 1)),
               std::invalid_argument);
  EXPECT_THROW(ForwardKinematics(makeMovingJointChain(kPandaJointCount + 1)),
               std::invalid_argument);
}

TEST(ForwardKinematicsTest, OwnsAStableCopyOfTheInputChain) {
  const RobotChains robot_chains(readTextFile(PANDA_IK_SINGLE_TEST_URDF), {"panda"});
  KDL::Chain source_chain = robot_chains.arm("panda").flange_chain();
  ForwardKinematics forward_kinematics(source_chain);
  source_chain = KDL::Chain{};

  const std::array<double, kPandaJointCount> joint_positions{0.0,
                                                             -0.7853981633974483,
                                                             0.0,
                                                             -2.356194490192345,
                                                             0.0,
                                                             1.5707963267948966,
                                                             0.7853981633974483};
  const KDL::Frame actual = forward_kinematics.compute(joint_positions);

  EXPECT_TRUE(std::isfinite(actual.p.x()));
  EXPECT_TRUE(std::isfinite(actual.p.y()));
  EXPECT_TRUE(std::isfinite(actual.p.z()));
}

TEST(ForwardKinematicsTest, MatchesEveryApplicableStageZeroCorpusWitness) {
  const RobotChains robot_chains(readTextFile(PANDA_IK_SINGLE_TEST_URDF), {"panda"});
  ForwardKinematics forward_kinematics(robot_chains.arm("panda").flange_chain());
  const auto witnesses = loadCorpusWitnesses(FRANKA_IK_CORPUS_FILE);

  std::size_t reachable_random_count = 0;
  std::size_t reachable_near_limit_count = 0;
  std::size_t singular_count = 0;
  std::size_t drag_trace_count = 0;
  std::size_t invalid_construction_count = 0;
  for (const auto& witness : witnesses) {
    SCOPED_TRACE(witness.id);
    const KDL::Frame actual = forward_kinematics.compute(witness.joint_positions);
    for (int index = 0; index < 3; ++index) {
      EXPECT_NEAR(actual.p(index), witness.target.p(index), kCorpusTolerance);
      for (int column = 0; column < 3; ++column) {
        EXPECT_NEAR(actual.M(index, column), witness.target.M(index, column), kCorpusTolerance);
      }
    }
    const PoseError error = computePoseError(witness.target, actual);
    EXPECT_LE(error.position, kCorpusTolerance);
    EXPECT_LE(error.orientation, kCorpusTolerance);

    if (witness.category == "reachable_random") {
      ++reachable_random_count;
    } else if (witness.category == "reachable_near_limit") {
      ++reachable_near_limit_count;
    } else if (witness.category == "singular") {
      ++singular_count;
    } else if (witness.category == "drag_traces") {
      ++drag_trace_count;
    } else if (witness.category == "unreachable_limits") {
      ++invalid_construction_count;
    }
  }

  EXPECT_EQ(reachable_random_count, 2000U);
  EXPECT_EQ(reachable_near_limit_count, 500U);
  EXPECT_EQ(singular_count, 200U);
  EXPECT_EQ(drag_trace_count, 4000U);
  EXPECT_EQ(invalid_construction_count, 200U);
}

TEST(PoseErrorTest, ReturnsPositionNormAndShortestRotationAngle) {
  const KDL::Frame target = KDL::Frame::Identity();
  const KDL::Frame translated(KDL::Rotation::Identity(), KDL::Vector(3.0, 4.0, 12.0));
  const PoseError translation_error = computePoseError(target, translated);
  EXPECT_DOUBLE_EQ(translation_error.position, 13.0);
  EXPECT_DOUBLE_EQ(translation_error.orientation, 0.0);

  constexpr double kPi = 3.14159265358979323846;
  const KDL::Frame wrapped_rotation(KDL::Rotation::RotZ(1.5 * kPi));
  const PoseError rotation_error = computePoseError(target, wrapped_rotation);
  EXPECT_DOUBLE_EQ(rotation_error.position, 0.0);
  EXPECT_NEAR(rotation_error.orientation, 0.5 * kPi, 1.0e-15);

  const PoseError reverse_error = computePoseError(wrapped_rotation, target);
  EXPECT_NEAR(reverse_error.orientation, rotation_error.orientation, 1.0e-15);
}

TEST(PoseErrorTest, TreatsEquivalentQuaternionSignsAsTheSameRotation) {
  constexpr double kInverseSqrtTwo = 0.70710678118654752440;
  const KDL::Frame positive(KDL::Rotation::Quaternion(0.0, 0.0, kInverseSqrtTwo, kInverseSqrtTwo));
  const KDL::Frame negative(
      KDL::Rotation::Quaternion(-0.0, -0.0, -kInverseSqrtTwo, -kInverseSqrtTwo));

  const PoseError error = computePoseError(positive, negative);
  EXPECT_DOUBLE_EQ(error.position, 0.0);
  EXPECT_NEAR(error.orientation, 0.0, 1.0e-15);
}

TEST(PoseErrorTest, PreservesErrorsBelowTheKdlDefaultEpsilon) {
  const KDL::Frame target = KDL::Frame::Identity();
  for (const double magnitude : {1.0e-7, 1.0e-9}) {
    SCOPED_TRACE(magnitude);
    const KDL::Frame actual(KDL::Rotation::RotZ(magnitude), KDL::Vector(magnitude, 0.0, 0.0));
    const PoseError error = computePoseError(target, actual);
    EXPECT_DOUBLE_EQ(error.position, magnitude);
    EXPECT_NEAR(error.orientation, magnitude, magnitude * 1.0e-12);
  }
}

}  // namespace
}  // namespace franka_ik
