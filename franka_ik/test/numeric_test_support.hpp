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

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>
#include <kdl/frames.hpp>

#include "franka_ik/numeric_backend.hpp"
#include "franka_ik/robot_chains.hpp"

#ifndef FRANKA_IK_CORPUS_FILE
#error "FRANKA_IK_CORPUS_FILE must name the checked-in Stage-0 corpus"
#endif

#ifndef PANDA_IK_SINGLE_TEST_URDF
#error "PANDA_IK_SINGLE_TEST_URDF must name the rendered single-arm wrapper"
#endif

namespace franka_ik::test {

struct Witness {
  std::string id;
  std::string category;
  std::array<double, kPandaJointCount> seed{};
  KDL::Frame target{KDL::Frame::Identity()};
};

struct Corpus {
  std::vector<Witness> reachable_random;
  std::vector<Witness> reachable_near_limit;
  std::vector<Witness> singular;
  std::vector<Witness> unreachable_far;
  std::vector<Witness> unreachable_limits;
  std::vector<std::vector<Witness>> drag_traces;

  std::vector<const Witness*> allReachable() const {
    std::vector<const Witness*> result;
    result.reserve(reachable_random.size() + reachable_near_limit.size() + singular.size() + 4000);
    for (const auto* set : {&reachable_random, &reachable_near_limit, &singular}) {
      for (const auto& witness : *set) {
        result.push_back(&witness);
      }
    }
    for (const auto& trace : drag_traces) {
      for (const auto& witness : trace) {
        result.push_back(&witness);
      }
    }
    return result;
  }
};

inline void require(const bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error("invalid Stage-0 corpus: " + message);
  }
}

inline std::string readFile(const std::string& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("cannot read '" + path + "'");
  }
  return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

inline std::array<double, kPandaJointCount> loadJoints(const YAML::Node& node) {
  require(node.IsSequence() && node.size() == kPandaJointCount,
          "joint vector must contain seven values");
  std::array<double, kPandaJointCount> result{};
  for (std::size_t index = 0; index < result.size(); ++index) {
    result[index] = node[index].as<double>();
    require(std::isfinite(result[index]), "joint vector contains non-finite value");
  }
  return result;
}

inline KDL::Frame loadPose(const YAML::Node& node) {
  const auto position = node["position"];
  const auto orientation = node["orientation_xyzw"];
  require(position.IsSequence() && position.size() == 3, "position must have three values");
  require(orientation.IsSequence() && orientation.size() == 4, "quaternion must have four values");
  return {
      KDL::Rotation::Quaternion(orientation[0].as<double>(), orientation[1].as<double>(),
                                orientation[2].as<double>(), orientation[3].as<double>()),
      KDL::Vector(position[0].as<double>(), position[1].as<double>(), position[2].as<double>())};
}

inline Witness loadWitness(const YAML::Node& node) {
  Witness result;
  result.id = node["id"].as<std::string>();
  result.category = node["category"].as<std::string>();
  result.seed = loadJoints(node["seed_positions"]);
  result.target = loadPose(node["target_pose"]);
  return result;
}

inline Corpus loadCorpus() {
  const auto root = YAML::LoadFile(FRANKA_IK_CORPUS_FILE);
  require(root["schema_version"].as<int>() == 1, "schema version changed");
  Corpus result;
  for (const auto& node : root["single_poses"]) {
    Witness witness = loadWitness(node);
    if (witness.category == "reachable_random") {
      result.reachable_random.push_back(std::move(witness));
    } else if (witness.category == "reachable_near_limit") {
      result.reachable_near_limit.push_back(std::move(witness));
    } else if (witness.category == "singular") {
      result.singular.push_back(std::move(witness));
    } else if (witness.category == "unreachable_far") {
      result.unreachable_far.push_back(std::move(witness));
    } else if (witness.category == "unreachable_limits") {
      result.unreachable_limits.push_back(std::move(witness));
    } else {
      throw std::runtime_error("unknown corpus category '" + witness.category + "'");
    }
  }
  for (const auto& trace_node : root["drag_traces"]) {
    std::vector<Witness> trace;
    for (const auto& step : trace_node["steps"]) {
      trace.push_back(loadWitness(step));
    }
    result.drag_traces.push_back(std::move(trace));
  }
  require(result.reachable_random.size() == 2000, "random count changed");
  require(result.reachable_near_limit.size() == 500, "near-limit count changed");
  require(result.singular.size() == 200, "singular count changed");
  require(result.unreachable_far.size() == 200, "far count changed");
  require(result.unreachable_limits.size() == 200, "limits count changed");
  require(result.drag_traces.size() == 20, "trace count changed");
  for (const auto& trace : result.drag_traces) {
    require(trace.size() == 200, "trace length changed");
  }
  require(result.allReachable().size() == 6700, "reachable witness count changed");
  return result;
}

inline const Corpus& corpus() {
  static const Corpus value = loadCorpus();
  return value;
}

class Model {
 public:
  Model() : chains_(readFile(PANDA_IK_SINGLE_TEST_URDF), {"panda"}) {}

  const ArmChain& arm() const { return chains_.arm("panda"); }

 private:
  RobotChains chains_;
};

inline SolveInput exactInput(const Witness& witness) {
  SolveInput result;
  result.target_pose = witness.target;
  result.seed_positions = witness.seed;
  result.fixed_q7 = witness.seed[6];
  result.position_tolerance = 1.0e-9;
  result.orientation_tolerance = 1.0e-9;
  result.numeric_eps = 1.0e-12;
  result.numeric_max_iterations = 500;
  return result;
}

inline std::array<double, kPandaJointCount> perturbSeed(
    const std::array<double, kPandaJointCount>& seed,
    const double magnitude,
    const std::size_t witness_index) {
  std::array<double, kPandaJointCount> result = seed;
  for (std::size_t joint = 0; joint < kPandaJointCount - 1; ++joint) {
    const double direction = ((witness_index + joint) % 2 == 0) ? 1.0 : -1.0;
    double proposed = result[joint] + direction * magnitude;
    const double lower = kPandaPositionLowerLimits[joint] + 1.0e-12;
    const double upper = kPandaPositionUpperLimits[joint] - 1.0e-12;
    if (proposed < lower || proposed > upper) {
      proposed = result[joint] - direction * magnitude;
    }
    result[joint] = std::clamp(proposed, lower, upper);
  }
  // The fixed-redundancy solve intentionally does not perturb q7.
  result[6] = seed[6];
  return result;
}

inline double maximumJointDelta(const std::array<double, kPandaJointCount>& left,
                                const std::array<double, kPandaJointCount>& right) {
  double result = 0.0;
  for (std::size_t index = 0; index < left.size(); ++index) {
    result = std::max(result, std::abs(left[index] - right[index]));
  }
  return result;
}

}  // namespace franka_ik::test
