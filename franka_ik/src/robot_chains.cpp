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

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <urdf/model.h>
#include <kdl/joint.hpp>
#include <kdl/segment.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace franka_ik {
namespace {

constexpr std::size_t kMinimumArmCount = 1;
constexpr std::size_t kMaximumArmCount = 4;
constexpr double kLimitTolerance = 1.0e-12;

bool isAsciiLetter(const char character) {
  return (character >= 'A' && character <= 'Z') || (character >= 'a' && character <= 'z');
}

bool isAsciiDigit(const char character) {
  return character >= '0' && character <= '9';
}

bool isValidArmId(const std::string& arm_id) {
  if (arm_id.empty() || arm_id.size() > kPandaArmIdMaxLength || !isAsciiLetter(arm_id.front())) {
    return false;
  }
  return std::all_of(arm_id.begin() + 1, arm_id.end(), [](const char character) {
    return isAsciiLetter(character) || isAsciiDigit(character) || character == '_';
  });
}

std::string canonicalJointName(const std::string& arm_id, const std::size_t index) {
  return arm_id + "_joint" + std::to_string(index + 1);
}

std::string canonicalLinkName(const std::string& arm_id, const std::size_t index) {
  return arm_id + "_link" + std::to_string(index);
}

void requireClose(const double actual,
                  const double expected,
                  const std::string& field,
                  const std::string& joint_name) {
  if (!std::isfinite(actual) || std::abs(actual - expected) > kLimitTolerance) {
    std::ostringstream message;
    message << "joint '" << joint_name << "' has non-canonical " << field << " " << actual
            << "; expected " << expected;
    throw std::invalid_argument(message.str());
  }
}

KDL::Frame composeFixedChain(const KDL::Chain& chain, const std::string& description) {
  if (chain.getNrOfJoints() != 0U) {
    throw std::invalid_argument(description + " must contain only fixed joints");
  }

  KDL::Frame transform = KDL::Frame::Identity();
  for (unsigned int index = 0; index < chain.getNrOfSegments(); ++index) {
    const auto& segment = chain.getSegment(index);
    if (segment.getJoint().getType() != KDL::Joint::Fixed) {
      throw std::invalid_argument(description + " must contain only fixed joints");
    }
    transform = transform * segment.pose(0.0);
  }
  return transform;
}

struct ArmChainData {
  std::string arm_id;
  std::string base_frame;
  std::string flange_frame;
  std::string hand_tcp_frame;
  std::array<std::string, kPandaJointCount> joint_names{};
  std::array<double, kPandaJointCount> position_lower{};
  std::array<double, kPandaJointCount> position_upper{};
  std::array<double, kPandaJointCount> velocity_limits{};
  std::array<double, kPandaJointCount> effort_limits{};
  KDL::Chain flange_chain;
  KDL::Frame root_to_base{KDL::Frame::Identity()};
  std::optional<KDL::Chain> hand_tcp_chain;
  std::optional<KDL::Frame> flange_to_hand_tcp;
};

ArmChainData buildArmChainData(const urdf::Model& model,
                               const KDL::Tree& tree,
                               const std::string& root_frame,
                               const std::string& arm_id) {
  ArmChainData result;
  result.arm_id = arm_id;
  result.base_frame = canonicalLinkName(arm_id, 0);
  result.flange_frame = canonicalLinkName(arm_id, 8);

  for (std::size_t index = 0; index <= kPandaJointCount + 1; ++index) {
    const auto link_name = canonicalLinkName(arm_id, index);
    if (!model.getLink(link_name)) {
      throw std::invalid_argument("configured arm '" + arm_id + "' is missing link '" + link_name +
                                  "'");
    }
  }

  if (!tree.getChain(result.base_frame, result.flange_frame, result.flange_chain)) {
    throw std::invalid_argument("cannot extract chain '" + result.base_frame + "' -> '" +
                                result.flange_frame + "'");
  }
  if (result.flange_chain.getNrOfJoints() != kPandaJointCount) {
    throw std::invalid_argument("chain for arm '" + arm_id + "' must contain exactly seven joints");
  }

  std::vector<std::string> moving_joint_names;
  moving_joint_names.reserve(kPandaJointCount);
  for (unsigned int index = 0; index < result.flange_chain.getNrOfSegments(); ++index) {
    const auto& joint = result.flange_chain.getSegment(index).getJoint();
    if (joint.getType() != KDL::Joint::Fixed) {
      moving_joint_names.push_back(joint.getName());
    }
  }
  if (moving_joint_names.size() != kPandaJointCount) {
    throw std::invalid_argument("chain for arm '" + arm_id + "' must contain exactly seven joints");
  }

  for (std::size_t index = 0; index < kPandaJointCount; ++index) {
    const auto joint_name = canonicalJointName(arm_id, index);
    result.joint_names[index] = joint_name;
    if (moving_joint_names[index] != joint_name) {
      throw std::invalid_argument("chain for arm '" + arm_id + "' has non-canonical joint order");
    }

    const auto joint = model.getJoint(joint_name);
    if (!joint) {
      throw std::invalid_argument("configured arm '" + arm_id + "' is missing joint '" +
                                  joint_name + "'");
    }
    if (joint->type != urdf::Joint::REVOLUTE) {
      throw std::invalid_argument("joint '" + joint_name + "' must be revolute");
    }
    if (!joint->limits) {
      throw std::invalid_argument("joint '" + joint_name + "' has no limits");
    }
    if (joint->parent_link_name != canonicalLinkName(arm_id, index) ||
        joint->child_link_name != canonicalLinkName(arm_id, index + 1)) {
      throw std::invalid_argument("joint '" + joint_name + "' has non-canonical parent or child");
    }

    requireClose(joint->limits->lower, kPandaPositionLowerLimits[index], "lower limit", joint_name);
    requireClose(joint->limits->upper, kPandaPositionUpperLimits[index], "upper limit", joint_name);
    requireClose(joint->limits->velocity, kPandaVelocityLimits[index], "velocity limit",
                 joint_name);
    requireClose(joint->limits->effort, kPandaEffortLimits[index], "effort limit", joint_name);

    result.position_lower[index] = joint->limits->lower;
    result.position_upper[index] = joint->limits->upper;
    result.velocity_limits[index] = joint->limits->velocity;
    result.effort_limits[index] = joint->limits->effort;
  }

  KDL::Chain root_to_base_chain;
  if (!tree.getChain(root_frame, result.base_frame, root_to_base_chain)) {
    throw std::invalid_argument("cannot extract root-to-base chain for arm '" + arm_id + "'");
  }
  result.root_to_base =
      composeFixedChain(root_to_base_chain, "root-to-base chain for arm '" + arm_id + "'");

  const std::string hand_tcp_frame = arm_id + "_hand_tcp";
  if (model.getLink(hand_tcp_frame)) {
    KDL::Chain hand_tcp_chain;
    if (!tree.getChain(result.flange_frame, hand_tcp_frame, hand_tcp_chain)) {
      throw std::invalid_argument("cannot extract flange-to-hand-TCP chain for arm '" + arm_id +
                                  "'");
    }
    result.flange_to_hand_tcp =
        composeFixedChain(hand_tcp_chain, "flange-to-hand-TCP chain for arm '" + arm_id + "'");
    result.hand_tcp_frame = hand_tcp_frame;
    result.hand_tcp_chain = std::move(hand_tcp_chain);
  }

  return result;
}

}  // namespace

ArmChain::ArmChain(std::string arm_id,
                   std::string base_frame,
                   std::string flange_frame,
                   std::string hand_tcp_frame,
                   std::array<std::string, kPandaJointCount> joint_names,
                   std::array<double, kPandaJointCount> position_lower,
                   std::array<double, kPandaJointCount> position_upper,
                   std::array<double, kPandaJointCount> velocity_limits,
                   std::array<double, kPandaJointCount> effort_limits,
                   KDL::Chain flange_chain,
                   KDL::Frame root_to_base,
                   std::optional<KDL::Chain> hand_tcp_chain,
                   std::optional<KDL::Frame> flange_to_hand_tcp)
    : arm_id_(std::move(arm_id)),
      base_frame_(std::move(base_frame)),
      flange_frame_(std::move(flange_frame)),
      hand_tcp_frame_(std::move(hand_tcp_frame)),
      joint_names_(std::move(joint_names)),
      position_lower_(std::move(position_lower)),
      position_upper_(std::move(position_upper)),
      velocity_limits_(std::move(velocity_limits)),
      effort_limits_(std::move(effort_limits)),
      flange_chain_(std::move(flange_chain)),
      root_to_base_(std::move(root_to_base)),
      hand_tcp_chain_(std::move(hand_tcp_chain)),
      flange_to_hand_tcp_(std::move(flange_to_hand_tcp)) {}

const std::string& ArmChain::arm_id() const noexcept {
  return arm_id_;
}

const std::string& ArmChain::base_frame() const noexcept {
  return base_frame_;
}

const std::string& ArmChain::flange_frame() const noexcept {
  return flange_frame_;
}

const std::string& ArmChain::hand_tcp_frame() const noexcept {
  return hand_tcp_frame_;
}

const std::array<std::string, kPandaJointCount>& ArmChain::joint_names() const noexcept {
  return joint_names_;
}

const std::array<double, kPandaJointCount>& ArmChain::position_lower() const noexcept {
  return position_lower_;
}

const std::array<double, kPandaJointCount>& ArmChain::position_upper() const noexcept {
  return position_upper_;
}

const std::array<double, kPandaJointCount>& ArmChain::velocity_limits() const noexcept {
  return velocity_limits_;
}

const std::array<double, kPandaJointCount>& ArmChain::effort_limits() const noexcept {
  return effort_limits_;
}

const KDL::Chain& ArmChain::flange_chain() const noexcept {
  return flange_chain_;
}

const KDL::Frame& ArmChain::root_to_base() const noexcept {
  return root_to_base_;
}

const std::optional<KDL::Chain>& ArmChain::hand_tcp_chain() const noexcept {
  return hand_tcp_chain_;
}

const std::optional<KDL::Frame>& ArmChain::flange_to_hand_tcp() const noexcept {
  return flange_to_hand_tcp_;
}

RobotChains::RobotChains(const std::string& urdf_xml, const std::vector<std::string>& arm_ids) {
  if (arm_ids.size() < kMinimumArmCount || arm_ids.size() > kMaximumArmCount) {
    throw std::invalid_argument("arm_ids must contain between one and four entries");
  }
  for (const auto& arm_id : arm_ids) {
    if (!isValidArmId(arm_id)) {
      throw std::invalid_argument("invalid arm id '" + arm_id + "'");
    }
  }
  for (std::size_t index = 0; index < arm_ids.size(); ++index) {
    if (std::find(arm_ids.begin(), arm_ids.begin() + static_cast<std::ptrdiff_t>(index),
                  arm_ids[index]) != arm_ids.begin() + static_cast<std::ptrdiff_t>(index)) {
      throw std::invalid_argument("duplicate arm id '" + arm_ids[index] + "'");
    }
  }

  urdf::Model model;
  if (!model.initString(urdf_xml)) {
    throw std::invalid_argument("robot_description is not valid URDF");
  }
  const auto root = model.getRoot();
  if (!root || root->name.empty()) {
    throw std::invalid_argument("robot_description has no root link");
  }
  root_frame_ = root->name;

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree)) {
    throw std::invalid_argument("robot_description cannot be converted to a KDL tree");
  }

  arms_.reserve(arm_ids.size());
  for (const auto& arm_id : arm_ids) {
    auto data = buildArmChainData(model, tree, root_frame_, arm_id);
    arms_.emplace_back(ArmChain(
        std::move(data.arm_id), std::move(data.base_frame), std::move(data.flange_frame),
        std::move(data.hand_tcp_frame), std::move(data.joint_names), std::move(data.position_lower),
        std::move(data.position_upper), std::move(data.velocity_limits),
        std::move(data.effort_limits), std::move(data.flange_chain), std::move(data.root_to_base),
        std::move(data.hand_tcp_chain), std::move(data.flange_to_hand_tcp)));
  }
}

const std::string& RobotChains::root_frame() const noexcept {
  return root_frame_;
}

const std::vector<ArmChain>& RobotChains::arms() const noexcept {
  return arms_;
}

bool RobotChains::has_arm(const std::string& arm_id) const noexcept {
  return std::any_of(arms_.begin(), arms_.end(),
                     [&arm_id](const ArmChain& arm_chain) { return arm_chain.arm_id() == arm_id; });
}

const ArmChain& RobotChains::arm(const std::string& arm_id) const {
  const auto found = std::find_if(arms_.begin(), arms_.end(), [&arm_id](const ArmChain& arm_chain) {
    return arm_chain.arm_id() == arm_id;
  });
  if (found == arms_.end()) {
    throw std::out_of_range("unknown arm id '" + arm_id + "'");
  }
  return *found;
}

}  // namespace franka_ik
