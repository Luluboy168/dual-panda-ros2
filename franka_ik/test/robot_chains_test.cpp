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

#include "franka_ik/robot_chains.hpp"

#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#include <gtest/gtest.h>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>

namespace franka_ik {
namespace {

constexpr double kPi = 3.14159265358979323846;

struct ArmFixture {
  std::string arm_id;
  std::string mount_xyz{"0 0 0"};
  std::string mount_rpy{"0 0 0"};
  bool include_hand{false};
  bool moving_mount{false};
  bool moving_hand{false};
  bool extra_moving_joint{false};
  int non_revolute_joint{0};
};

std::string jointLimitXml(const std::size_t index) {
  std::ostringstream xml;
  xml << "<limit lower=\"" << kPandaPositionLowerLimits[index] << "\" upper=\""
      << kPandaPositionUpperLimits[index] << "\" velocity=\"" << kPandaVelocityLimits[index]
      << "\" effort=\"" << kPandaEffortLimits[index] << "\"/>";
  return xml.str();
}

std::string armXml(const ArmFixture& fixture, const bool needs_mount) {
  std::ostringstream xml;
  if (needs_mount) {
    xml << "<link name=\"" << fixture.arm_id << "_link0\"/>" << "<joint name=\"" << fixture.arm_id
        << "_mount_joint\" type=\"" << (fixture.moving_mount ? "revolute" : "fixed") << "\">"
        << "<parent link=\"base_link\"/><child link=\"" << fixture.arm_id << "_link0\"/>"
        << "<origin xyz=\"" << fixture.mount_xyz << "\" rpy=\"" << fixture.mount_rpy << "\"/>";
    if (fixture.moving_mount) {
      xml << "<axis xyz=\"0 0 1\"/><limit lower=\"-1\" upper=\"1\" velocity=\"1\" "
             "effort=\"1\"/>";
    }
    xml << "</joint>";
  }

  for (std::size_t index = 1; index <= 8; ++index) {
    xml << "<link name=\"" << fixture.arm_id << "_link" << index << "\"/>";
  }
  if (fixture.extra_moving_joint) {
    xml << "<link name=\"" << fixture.arm_id << "_extra_link\"/>";
  }

  for (std::size_t index = 0; index < kPandaJointCount; ++index) {
    const bool is_non_revolute = fixture.non_revolute_joint == static_cast<int>(index + 1);
    xml << "<joint name=\"" << fixture.arm_id << "_joint" << index + 1 << "\" type=\""
        << (is_non_revolute ? "prismatic" : "revolute") << "\">" << "<parent link=\""
        << fixture.arm_id << "_link" << index << "\"/>" << "<child link=\"" << fixture.arm_id
        << "_link" << index + 1 << "\"/>"
        << "<origin xyz=\"0 0 0.1\" rpy=\"0 0 0\"/><axis xyz=\"0 0 1\"/>" << jointLimitXml(index)
        << "</joint>";
  }

  std::string flange_parent = fixture.arm_id + "_link7";
  if (fixture.extra_moving_joint) {
    xml << "<joint name=\"" << fixture.arm_id << "_extra_joint\" type=\"revolute\">"
        << "<parent link=\"" << fixture.arm_id << "_link7\"/><child link=\"" << fixture.arm_id
        << "_extra_link\"/><axis xyz=\"0 0 1\"/>"
        << "<limit lower=\"-1\" upper=\"1\" velocity=\"1\" effort=\"1\"/></joint>";
    flange_parent = fixture.arm_id + "_extra_link";
  }
  xml << "<joint name=\"" << fixture.arm_id << "_joint8\" type=\"fixed\">" << "<parent link=\""
      << flange_parent << "\"/><child link=\"" << fixture.arm_id
      << "_link8\"/><origin xyz=\"0 0 0.107\" rpy=\"0 0 0\"/></joint>";

  if (fixture.include_hand) {
    xml << "<link name=\"" << fixture.arm_id << "_hand\"/><link name=\"" << fixture.arm_id
        << "_hand_tcp\"/>" << "<joint name=\"" << fixture.arm_id << "_hand_joint\" type=\""
        << (fixture.moving_hand ? "revolute" : "fixed") << "\">" << "<parent link=\""
        << fixture.arm_id << "_link8\"/><child link=\"" << fixture.arm_id
        << "_hand\"/><origin xyz=\"0 0 0\" rpy=\"0 0 -0.7853981633974483\"/>";
    if (fixture.moving_hand) {
      xml << "<axis xyz=\"0 0 1\"/><limit lower=\"-1\" upper=\"1\" velocity=\"1\" "
             "effort=\"1\"/>";
    }
    xml << "</joint><joint name=\"" << fixture.arm_id << "_hand_tcp_joint\" type=\"fixed\">"
        << "<parent link=\"" << fixture.arm_id << "_hand\"/><child link=\"" << fixture.arm_id
        << "_hand_tcp\"/><origin xyz=\"0 0 0.1034\" rpy=\"0 0 0\"/></joint>";
  }
  return xml.str();
}

std::string robotXml(const std::vector<ArmFixture>& fixtures, const bool single_arm_root = false) {
  std::ostringstream xml;
  xml << "<robot name=\"robot_chains_fixture\">";
  if (!single_arm_root) {
    xml << "<link name=\"base_link\"/>";
  } else {
    xml << "<link name=\"" << fixtures.front().arm_id << "_link0\"/>";
  }
  for (const auto& fixture : fixtures) {
    xml << armXml(fixture, !single_arm_root);
  }
  xml << "</robot>";
  return xml.str();
}

std::string robotWithTwoSegmentMount() {
  const ArmFixture fixture{"panda"};
  std::ostringstream xml;
  xml << std::setprecision(17);
  xml << "<robot name=\"two_segment_mount\"><link name=\"world\"/><link name=\"pedestal\"/>"
      << "<joint name=\"world_to_pedestal\" type=\"fixed\"><parent link=\"world\"/>"
      << "<child link=\"pedestal\"/><origin xyz=\"1 0 0\" rpy=\"0 0 " << kPi / 2.0
      << "\"/></joint><link name=\"panda_link0\"/>"
      << "<joint name=\"pedestal_to_panda\" type=\"fixed\"><parent link=\"pedestal\"/>"
      << "<child link=\"panda_link0\"/><origin xyz=\"1 0 0\" rpy=\"0 0 0\"/></joint>"
      << armXml(fixture, false) << "</robot>";
  return xml.str();
}

void expectFrame(const KDL::Frame& frame,
                 const std::array<double, 3>& xyz,
                 const std::array<double, 3>& rpy) {
  EXPECT_NEAR(frame.p.x(), xyz[0], 1.0e-12);
  EXPECT_NEAR(frame.p.y(), xyz[1], 1.0e-12);
  EXPECT_NEAR(frame.p.z(), xyz[2], 1.0e-12);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  frame.M.GetRPY(roll, pitch, yaw);
  EXPECT_NEAR(roll, rpy[0], 1.0e-12);
  EXPECT_NEAR(pitch, rpy[1], 1.0e-12);
  EXPECT_NEAR(yaw, rpy[2], 1.0e-12);
}

TEST(RobotChainsTest, ExtractsAllSupportedPandaPrefixes) {
  const std::vector<ArmFixture> fixtures{
      {"panda", "0 0 0"}, {"panda1", "0 0.26 0"}, {"panda2", "0 -0.26 0"}};
  const RobotChains chains(robotXml(fixtures), {"panda", "panda1", "panda2"});

  EXPECT_EQ(chains.root_frame(), "base_link");
  ASSERT_EQ(chains.arms().size(), 3U);
  for (const auto& fixture : fixtures) {
    ASSERT_TRUE(chains.has_arm(fixture.arm_id));
    const auto& arm = chains.arm(fixture.arm_id);
    EXPECT_EQ(arm.arm_id(), fixture.arm_id);
    EXPECT_EQ(arm.base_frame(), fixture.arm_id + "_link0");
    EXPECT_EQ(arm.flange_frame(), fixture.arm_id + "_link8");
    EXPECT_EQ(arm.flange_chain().getNrOfJoints(), kPandaJointCount);
    EXPECT_EQ(arm.flange_chain().getNrOfSegments(), 8U);
    EXPECT_TRUE(arm.hand_tcp_frame().empty());
    EXPECT_FALSE(arm.hand_tcp_chain().has_value());
    EXPECT_FALSE(arm.flange_to_hand_tcp().has_value());
    for (std::size_t index = 0; index < kPandaJointCount; ++index) {
      EXPECT_EQ(arm.joint_names()[index], fixture.arm_id + "_joint" + std::to_string(index + 1));
    }
  }
  EXPECT_THROW(chains.arm("missing"), std::out_of_range);
}

TEST(RobotChainsTest, SupportsOneToFourArbitraryValidArmIds) {
  const std::vector<ArmFixture> fixtures{{"alpha"}, {"B2"}, {"arm_3"}, {"Z_4"}};
  const RobotChains chains(robotXml(fixtures), {"alpha", "B2", "arm_3", "Z_4"});
  EXPECT_EQ(chains.arms().size(), 4U);
  EXPECT_TRUE(chains.has_arm("alpha"));
  EXPECT_TRUE(chains.has_arm("B2"));
  EXPECT_TRUE(chains.has_arm("arm_3"));
  EXPECT_TRUE(chains.has_arm("Z_4"));
}

TEST(RobotChainsTest, ExposesSingleAndDualRootTransformsFromUrdf) {
  const RobotChains single(robotXml({{"panda"}}, true), {"panda"});
  EXPECT_EQ(single.root_frame(), "panda_link0");
  expectFrame(single.arm("panda").root_to_base(), {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});

  const std::vector<ArmFixture> dual{{"panda1", "0 0.26 0", "0 0 0"},
                                     {"panda2", "0 -0.26 0", "0 0 0.5"}};
  const RobotChains chains(robotXml(dual), {"panda1", "panda2"});
  expectFrame(chains.arm("panda1").root_to_base(), {0.0, 0.26, 0.0}, {0.0, 0.0, 0.0});
  expectFrame(chains.arm("panda2").root_to_base(), {0.0, -0.26, 0.0}, {0.0, 0.0, 0.5});
}

TEST(RobotChainsTest, ComposesNoncommutingTwoSegmentRootTransformInOrder) {
  const RobotChains chains(robotWithTwoSegmentMount(), {"panda"});
  expectFrame(chains.arm("panda").root_to_base(), {1.0, 1.0, 0.0}, {0.0, 0.0, kPi / 2.0});
}

TEST(RobotChainsTest, ExtractsOptionalFixedHandTcpChainAndTransform) {
  const RobotChains chains(robotXml({{"panda", "0 0 0", "0 0 0", true}}, true), {"panda"});
  const auto& arm = chains.arm("panda");
  EXPECT_EQ(arm.hand_tcp_frame(), "panda_hand_tcp");
  ASSERT_TRUE(arm.hand_tcp_chain().has_value());
  EXPECT_EQ(arm.hand_tcp_chain()->getNrOfJoints(), 0U);
  EXPECT_EQ(arm.hand_tcp_chain()->getNrOfSegments(), 2U);
  ASSERT_TRUE(arm.flange_to_hand_tcp().has_value());
  expectFrame(*arm.flange_to_hand_tcp(), {0.0, 0.0, 0.1034}, {0.0, 0.0, -kPi / 4.0});
}

TEST(RobotChainsTest, RejectsMalformedAndMissingDescriptions) {
  EXPECT_THROW(RobotChains("<robot>", {"panda"}), std::invalid_argument);
  EXPECT_THROW(RobotChains(robotXml({{"panda"}}, true), {"panda1"}), std::invalid_argument);
}

TEST(RobotChainsTest, RejectsInvalidArmIdLists) {
  const auto urdf = robotXml({{"panda"}}, true);
  EXPECT_THROW(RobotChains(urdf, {}), std::invalid_argument);
  EXPECT_THROW(
      RobotChains(robotXml({{"a"}, {"b"}, {"c"}, {"d"}, {"e"}}), {"a", "b", "c", "d", "e"}),
      std::invalid_argument);
  EXPECT_THROW(RobotChains(urdf, {"1panda"}), std::invalid_argument);
  EXPECT_THROW(RobotChains(urdf, {"panda-dash"}), std::invalid_argument);
  EXPECT_THROW(RobotChains(urdf, {"panda", "panda"}), std::invalid_argument);
}

TEST(RobotChainsTest, RejectsNonRevoluteAndWrongJointCounts) {
  EXPECT_THROW(
      RobotChains(robotXml({{"panda", "0 0 0", "0 0 0", false, false, false, false, 4}}, true),
                  {"panda"}),
      std::invalid_argument);
  EXPECT_THROW(RobotChains(robotXml({{"panda", "0 0 0", "0 0 0", false, false, false, true}}, true),
                           {"panda"}),
               std::invalid_argument);
}

TEST(RobotChainsTest, RejectsNonFixedMountAndHandTcpChains) {
  EXPECT_THROW(RobotChains(robotXml({{"panda", "0 0 0", "0 0 0", false, true}}), {"panda"}),
               std::invalid_argument);
  EXPECT_THROW(
      RobotChains(robotXml({{"panda", "0 0 0", "0 0 0", true, false, true}}, true), {"panda"}),
      std::invalid_argument);
}

TEST(RobotChainsTest, RejectsDescriptionLimitDrift) {
  auto urdf = robotXml({{"panda"}}, true);
  const std::string expected = "lower=\"-3.0718\"";
  const auto position = urdf.find(expected);
  ASSERT_NE(position, std::string::npos);
  urdf.replace(position, expected.size(), "lower=\"-3.0\"");
  EXPECT_THROW(RobotChains(urdf, {"panda"}), std::invalid_argument);
}

TEST(RobotChainsTest, AccessorsExposeOnlyConstModelReferences) {
  using ArmsAccessor = decltype(std::declval<const RobotChains&>().arms());
  using ChainAccessor = decltype(std::declval<const ArmChain&>().flange_chain());
  using LimitsAccessor = decltype(std::declval<const ArmChain&>().position_lower());
  static_assert(std::is_same_v<ArmsAccessor, const std::vector<ArmChain>&>);
  static_assert(std::is_same_v<ChainAccessor, const KDL::Chain&>);
  static_assert(std::is_same_v<LimitsAccessor, const std::array<double, kPandaJointCount>&>);
  SUCCEED();
}

}  // namespace
}  // namespace franka_ik
