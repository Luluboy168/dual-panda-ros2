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

#include "franka_ik/service_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rcutils/sha256.h>
#include <geometry_msgs/msg/transform.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include "franka_ik_interfaces/msg/chain_info.hpp"
#include "franka_ik_interfaces/msg/ik_request.hpp"
#include "franka_ik_interfaces/msg/ik_result.hpp"
#include "franka_ik_interfaces/msg/ik_solution.hpp"

namespace franka_ik {
namespace {

using IkRequest = franka_ik_interfaces::msg::IkRequest;
using IkResult = franka_ik_interfaces::msg::IkResult;
using IkSolution = franka_ik_interfaces::msg::IkSolution;

constexpr std::size_t kMaximumResultMessageLength = 256;
constexpr std::size_t kMaximumRequestArmIdLength = 64;
constexpr std::size_t kMaximumRequestFrameIdLength = 128;
constexpr double kMinimumQuaternionNorm = 1.0e-6;
constexpr double kMaximumQuaternionNorm = 1.0e6;
constexpr double kMaximumPositionTolerance = 1.0e-2;
constexpr double kMaximumOrientationTolerance = 1.0e-1;
constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;
constexpr char kPackageVersion[] = "0.1.0";

rcl_interfaces::msg::ParameterDescriptor readOnlyDescriptor(const std::string& description) {
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  descriptor.read_only = true;
  return descriptor;
}

template <typename Value>
Value declareReadOnlyParameter(rclcpp::Node& node,
                               const std::string& name,
                               const Value& default_value,
                               const std::string& description) {
  return node.declare_parameter<Value>(name, default_value, readOnlyDescriptor(description));
}

[[noreturn]] void throwParameterError(const std::string& name, const std::string& requirement) {
  throw std::invalid_argument("parameter '" + name + "' " + requirement);
}

bool validArmId(const std::string& arm_id) noexcept {
  if (arm_id.empty() || arm_id.size() > kPandaArmIdMaxLength ||
      !((arm_id.front() >= 'A' && arm_id.front() <= 'Z') ||
        (arm_id.front() >= 'a' && arm_id.front() <= 'z'))) {
    return false;
  }
  return std::all_of(arm_id.begin() + 1, arm_id.end(), [](const char character) {
    return (character >= 'A' && character <= 'Z') || (character >= 'a' && character <= 'z') ||
           (character >= '0' && character <= '9') || character == '_';
  });
}

std::string sha256Hex(const std::string& data) {
  rcutils_sha256_ctx_t context;
  rcutils_sha256_init(&context);
  rcutils_sha256_update(&context, reinterpret_cast<const std::uint8_t*>(data.data()), data.size());
  std::array<std::uint8_t, RCUTILS_SHA256_BLOCK_SIZE> digest{};
  rcutils_sha256_final(&context, digest.data());

  constexpr char kHexDigits[] = "0123456789abcdef";
  std::string result(digest.size() * 2, '0');
  for (std::size_t index = 0; index < digest.size(); ++index) {
    result[2 * index] = kHexDigits[digest[index] >> 4U];
    result[2 * index + 1] = kHexDigits[digest[index] & 0x0fU];
  }
  return result;
}

std::string boundedMessage(const std::string& message) {
  return message.substr(0, kMaximumResultMessageLength);
}

void initializeResult(IkResult& result, const std::uint8_t solver_used) {
  result.result = IkResult::RESULT_INTERNAL_ERROR;
  result.message.clear();
  result.solutions.clear();
  result.solver_used = solver_used;
  result.iterations = 0;
  result.solve_time.sec = 0;
  result.solve_time.nanosec = 0;
}

void setFailure(IkResult& result, const std::uint8_t code, const std::string& message) {
  result.result = code;
  result.message = boundedMessage(message);
  result.solutions.clear();
}

void setSolveDuration(IkResult& result, const std::chrono::steady_clock::duration elapsed) {
  const auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count();
  const auto seconds = nanoseconds / kNanosecondsPerSecond;
  if (nanoseconds < 0 || seconds > std::numeric_limits<std::int32_t>::max()) {
    result.solve_time.sec = 0;
    result.solve_time.nanosec = 0;
    return;
  }
  result.solve_time.sec = static_cast<std::int32_t>(seconds);
  result.solve_time.nanosec = static_cast<std::uint32_t>(nanoseconds % kNanosecondsPerSecond);
}

void copyCandidate(const IkCandidate& source, IkSolution& destination) {
  destination.positions = source.positions;
  destination.redundancy_value = source.redundancy_value;
  destination.position_error = source.position_error;
  destination.orientation_error = source.orientation_error;
  destination.seed_distance = source.seed_distance;
  destination.branch = source.branch;
}

void copyTransform(const KDL::Frame& source, geometry_msgs::msg::Transform& destination) {
  destination.translation.x = source.p.x();
  destination.translation.y = source.p.y();
  destination.translation.z = source.p.z();
  source.M.GetQuaternion(destination.rotation.x, destination.rotation.y, destination.rotation.z,
                         destination.rotation.w);
}

bool finitePosePosition(const geometry_msgs::msg::Point& position) noexcept {
  return std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(position.z);
}

bool normalizeQuaternion(const geometry_msgs::msg::Quaternion& quaternion,
                         KDL::Rotation& rotation) noexcept {
  if (!std::isfinite(quaternion.x) || !std::isfinite(quaternion.y) ||
      !std::isfinite(quaternion.z) || !std::isfinite(quaternion.w)) {
    return false;
  }
  const double norm =
      std::hypot(std::hypot(quaternion.x, quaternion.y), std::hypot(quaternion.z, quaternion.w));
  if (!std::isfinite(norm) || norm < kMinimumQuaternionNorm || norm > kMaximumQuaternionNorm) {
    return false;
  }
  rotation = KDL::Rotation::Quaternion(quaternion.x / norm, quaternion.y / norm,
                                       quaternion.z / norm, quaternion.w / norm);
  return true;
}

}  // namespace

std::uint8_t solveStatusToResultCode(const SolveStatus status) noexcept {
  switch (status) {
    case SolveStatus::Success:
      return IkResult::RESULT_SUCCESS;
    case SolveStatus::BadRequest:
      return IkResult::RESULT_BAD_REQUEST;
    case SolveStatus::SeedOutOfLimits:
      return IkResult::RESULT_SEED_OUT_OF_LIMITS;
    case SolveStatus::GeometricallyUnreachable:
      return IkResult::RESULT_UNREACHABLE;
    case SolveStatus::JointLimitsViolated:
      return IkResult::RESULT_LIMITS_VIOLATED;
    case SolveStatus::ToleranceNotMet:
      return IkResult::RESULT_TOLERANCE_NOT_MET;
    case SolveStatus::IterationBudgetExhausted:
      return IkResult::RESULT_ITERATION_BUDGET_EXHAUSTED;
    case SolveStatus::NoAcceptableSolution:
      return IkResult::RESULT_NO_ACCEPTABLE_SOLUTION;
    case SolveStatus::InternalError:
      return IkResult::RESULT_INTERNAL_ERROR;
  }
  return IkResult::RESULT_INTERNAL_ERROR;
}

ServiceNode::Configuration ServiceNode::loadConfiguration(rclcpp::Node& node) {
  Configuration configuration;
  configuration.robot_description = declareReadOnlyParameter<std::string>(
      node, "robot_description", "", "Complete URDF XML parsed by the IK service");
  configuration.arm_ids = declareReadOnlyParameter<std::vector<std::string>>(
      node, "arm_ids", {"panda1", "panda2"}, "Configured Panda arm identifiers");
  const std::string default_solver = declareReadOnlyParameter<std::string>(
      node, "default_solver", "numeric", "Default IK backend: numeric in this build");
  configuration.position_tolerance = declareReadOnlyParameter<double>(
      node, "position_tolerance", 1.0e-4, "Default FK position acceptance tolerance in metres");
  configuration.orientation_tolerance =
      declareReadOnlyParameter<double>(node, "orientation_tolerance", 1.0e-3,
                                       "Default FK orientation acceptance tolerance in radians");
  const std::int64_t numeric_max_iterations = declareReadOnlyParameter<std::int64_t>(
      node, "numeric_max_iterations", 40, "KDL LMA iteration budget");
  configuration.numeric_eps = declareReadOnlyParameter<double>(
      node, "numeric_eps", 1.0e-6, "KDL LMA weighted task-space epsilon");
  configuration.joint_limit_margin_max = declareReadOnlyParameter<double>(
      node, "joint_limit_margin_max", 0.100, "Maximum accepted per-request joint margin");

  if (configuration.robot_description.empty()) {
    throwParameterError("robot_description", "must contain non-empty URDF XML");
  }
  if (configuration.arm_ids.empty() || configuration.arm_ids.size() > 4) {
    throwParameterError("arm_ids", "must contain between one and four entries");
  }
  std::vector<std::string> unique_arm_ids;
  for (const auto& arm_id : configuration.arm_ids) {
    if (!validArmId(arm_id)) {
      throwParameterError("arm_ids", "contains an entry outside [A-Za-z][A-Za-z0-9_]*");
    }
    if (std::find(unique_arm_ids.begin(), unique_arm_ids.end(), arm_id) != unique_arm_ids.end()) {
      throwParameterError("arm_ids", "must not contain duplicate entries");
    }
    unique_arm_ids.push_back(arm_id);
  }
  if (default_solver == "numeric") {
    configuration.default_solver = IkRequest::SOLVER_NUMERIC;
  } else if (default_solver == "analytic") {
    throwParameterError("default_solver", "requests unavailable analytic support in this build");
  } else {
    throwParameterError("default_solver", "must be either 'numeric' or 'analytic'");
  }
  if (!std::isfinite(configuration.position_tolerance) || configuration.position_tolerance <= 0.0 ||
      configuration.position_tolerance > kMaximumPositionTolerance) {
    throwParameterError("position_tolerance", "must be finite and in (0, 0.01]");
  }
  if (!std::isfinite(configuration.orientation_tolerance) ||
      configuration.orientation_tolerance <= 0.0 ||
      configuration.orientation_tolerance > kMaximumOrientationTolerance) {
    throwParameterError("orientation_tolerance", "must be finite and in (0, 0.1]");
  }
  if (numeric_max_iterations < 10 || numeric_max_iterations > 2000) {
    throwParameterError("numeric_max_iterations", "must be in [10, 2000]");
  }
  configuration.numeric_max_iterations = static_cast<std::uint16_t>(numeric_max_iterations);
  if (!std::isfinite(configuration.numeric_eps) || configuration.numeric_eps <= 0.0 ||
      configuration.numeric_eps > 1.0e-2) {
    throwParameterError("numeric_eps", "must be finite and in (0, 0.01]");
  }
  if (!std::isfinite(configuration.joint_limit_margin_max) ||
      configuration.joint_limit_margin_max < 0.0 || configuration.joint_limit_margin_max > 0.5) {
    throwParameterError("joint_limit_margin_max", "must be finite and in [0, 0.5]");
  }
  return configuration;
}

RobotChains ServiceNode::buildChains(const Configuration& configuration) {
  try {
    return RobotChains(configuration.robot_description, configuration.arm_ids);
  } catch (const std::exception& error) {
    throw std::invalid_argument(
        std::string("parameters 'robot_description' and 'arm_ids' are inconsistent: ") +
        error.what());
  }
}

ServiceNode::ServiceNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("franka_ik_service", options),
      configuration_(loadConfiguration(*this)),
      chains_(buildChains(configuration_)),
      urdf_sha256_(sha256Hex(configuration_.robot_description)) {
  numeric_backends_.reserve(chains_.arms().size());
  for (const auto& arm : chains_.arms()) {
    numeric_backends_.push_back(std::make_unique<NumericBackend>(arm));
  }

  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  solve_service_ = create_service<franka_ik_interfaces::srv::SolveIk>(
      "~/solve_ik",
      [this](const franka_ik_interfaces::srv::SolveIk::Request::SharedPtr request,
             franka_ik_interfaces::srv::SolveIk::Response::SharedPtr response) {
        handleSolve(request, response);
      },
      rclcpp::ServicesQoS(), callback_group_);
  chain_info_service_ = create_service<franka_ik_interfaces::srv::GetChainInfo>(
      "~/chain_info",
      [this](const franka_ik_interfaces::srv::GetChainInfo::Request::SharedPtr request,
             franka_ik_interfaces::srv::GetChainInfo::Response::SharedPtr response) {
        handleChainInfo(request, response);
      },
      rclcpp::ServicesQoS(), callback_group_);
}

void ServiceNode::handleSolve(const franka_ik_interfaces::srv::SolveIk::Request::SharedPtr request,
                              franka_ik_interfaces::srv::SolveIk::Response::SharedPtr response) {
  std::uint8_t resolved_solver = configuration_.default_solver;
  if (request->request.solver == IkRequest::SOLVER_ANALYTIC) {
    resolved_solver = IkRequest::SOLVER_ANALYTIC;
  }
  initializeResult(response->result, resolved_solver);

  if (request->request.arm_id.size() > kMaximumRequestArmIdLength ||
      request->request.frame_id.size() > kMaximumRequestFrameIdLength) {
    setFailure(response->result, IkResult::RESULT_BAD_REQUEST,
               "arm_id or frame_id exceeds its bounded request length");
    return;
  }
  if (request->request.max_solutions > 4) {
    setFailure(response->result, IkResult::RESULT_BAD_REQUEST,
               "max_solutions must be between zero and four");
    return;
  }
  if (request->request.solver == IkRequest::SOLVER_ANALYTIC) {
    setFailure(response->result, IkResult::RESULT_BAD_REQUEST,
               "analytic solver is unavailable in this numeric-only build");
    return;
  }
  if (request->request.solver != IkRequest::SOLVER_DEFAULT &&
      request->request.solver != IkRequest::SOLVER_NUMERIC) {
    setFailure(response->result, IkResult::RESULT_BAD_REQUEST, "solver enum is invalid");
    return;
  }

  std::size_t arm_index = 0;
  while (arm_index < chains_.arms().size() &&
         chains_.arms()[arm_index].arm_id() != request->request.arm_id) {
    ++arm_index;
  }
  if (arm_index == chains_.arms().size()) {
    setFailure(response->result, IkResult::RESULT_UNKNOWN_ARM,
               "arm_id is not one of the configured chains");
    return;
  }
  const ArmChain& arm = chains_.arms()[arm_index];

  SolveInput input;
  if (request->request.frame_id.empty() || request->request.frame_id == arm.base_frame()) {
    input.reference_frame = ReachabilityReferenceFrame::ArmBase;
  } else if (request->request.frame_id == chains_.root_frame()) {
    input.reference_frame = ReachabilityReferenceFrame::UrdfRoot;
  } else {
    setFailure(response->result, IkResult::RESULT_UNKNOWN_FRAME,
               "frame_id is neither the arm base nor the URDF root");
    return;
  }

  if (request->request.tip_frame == IkRequest::TIP_FLANGE) {
    input.tip_frame = ReachabilityTipFrame::Flange;
  } else if (request->request.tip_frame == IkRequest::TIP_HAND_TCP) {
    if (!arm.flange_to_hand_tcp().has_value()) {
      setFailure(response->result, IkResult::RESULT_UNSUPPORTED_TIP,
                 "hand TCP requested but this arm has no hand TCP");
      return;
    }
    input.tip_frame = ReachabilityTipFrame::HandTcp;
  } else {
    setFailure(response->result, IkResult::RESULT_BAD_REQUEST, "tip_frame enum is invalid");
    return;
  }

  KDL::Rotation normalized_rotation;
  if (!finitePosePosition(request->request.target_pose.position) ||
      !normalizeQuaternion(request->request.target_pose.orientation, normalized_rotation)) {
    setFailure(response->result, IkResult::RESULT_BAD_REQUEST,
               "target pose must be finite with quaternion norm in [1e-6, 1e6]");
    return;
  }
  input.target_pose =
      KDL::Frame(normalized_rotation, KDL::Vector(request->request.target_pose.position.x,
                                                  request->request.target_pose.position.y,
                                                  request->request.target_pose.position.z));
  input.seed_positions = request->request.seed_positions;

  if (!std::isfinite(request->request.redundancy_value)) {
    setFailure(response->result, IkResult::RESULT_BAD_REQUEST,
               "redundancy_value must be finite even when seed redundancy is selected");
    return;
  }

  if (request->request.redundancy_mode == IkRequest::REDUNDANCY_FROM_SEED) {
    input.fixed_q7 = input.seed_positions[6];
  } else if (request->request.redundancy_mode == IkRequest::REDUNDANCY_FIXED) {
    input.fixed_q7 = request->request.redundancy_value;
  } else {
    setFailure(response->result, IkResult::RESULT_BAD_REQUEST, "redundancy_mode enum is invalid");
    return;
  }

  if (!std::isfinite(request->request.position_tolerance) ||
      request->request.position_tolerance < 0.0 ||
      request->request.position_tolerance > kMaximumPositionTolerance) {
    setFailure(response->result, IkResult::RESULT_BAD_REQUEST,
               "position_tolerance must be zero or in (0, 0.01]");
    return;
  }
  if (!std::isfinite(request->request.orientation_tolerance) ||
      request->request.orientation_tolerance < 0.0 ||
      request->request.orientation_tolerance > kMaximumOrientationTolerance) {
    setFailure(response->result, IkResult::RESULT_BAD_REQUEST,
               "orientation_tolerance must be zero or in (0, 0.1]");
    return;
  }
  if (!std::isfinite(request->request.joint_limit_margin) ||
      request->request.joint_limit_margin < 0.0 ||
      request->request.joint_limit_margin > configuration_.joint_limit_margin_max) {
    setFailure(response->result, IkResult::RESULT_BAD_REQUEST,
               "joint_limit_margin is outside the configured range");
    return;
  }
  input.position_tolerance = request->request.position_tolerance == 0.0
                                 ? configuration_.position_tolerance
                                 : request->request.position_tolerance;
  input.orientation_tolerance = request->request.orientation_tolerance == 0.0
                                    ? configuration_.orientation_tolerance
                                    : request->request.orientation_tolerance;
  input.joint_limit_margin = request->request.joint_limit_margin;
  input.numeric_max_iterations = configuration_.numeric_max_iterations;
  input.numeric_eps = configuration_.numeric_eps;

  SolveOutput backend_output;
  const auto solve_begin = std::chrono::steady_clock::now();
  numeric_backends_[arm_index]->solve(input, backend_output);
  const auto solve_end = std::chrono::steady_clock::now();
  setSolveDuration(response->result, solve_end - solve_begin);
  response->result.solver_used = IkRequest::SOLVER_NUMERIC;
  response->result.iterations = backend_output.iterations;
  response->result.result = solveStatusToResultCode(backend_output.status);
  response->result.message = boundedMessage(backend_output.message);
  response->result.solutions.clear();

  if (backend_output.status != SolveStatus::Success) {
    return;
  }
  if (backend_output.solutions.empty()) {
    setFailure(response->result, IkResult::RESULT_INTERNAL_ERROR,
               "backend reported success without a valid witness");
    return;
  }
  if (request->request.max_solutions == 0) {
    return;
  }
  const std::size_t solution_count =
      std::min<std::size_t>(request->request.max_solutions, backend_output.solutions.size());
  response->result.solutions.reserve(solution_count);
  for (std::size_t index = 0; index < solution_count; ++index) {
    IkSolution solution;
    copyCandidate(backend_output.solutions[index], solution);
    response->result.solutions.push_back(std::move(solution));
  }
}

void ServiceNode::handleChainInfo(
    const franka_ik_interfaces::srv::GetChainInfo::Request::SharedPtr request,
    franka_ik_interfaces::srv::GetChainInfo::Response::SharedPtr response) const {
  (void)request;
  response->urdf_root_frame = chains_.root_frame();
  response->urdf_sha256 = urdf_sha256_;
  response->chains.clear();
  response->chains.reserve(chains_.arms().size());
  for (const auto& arm : chains_.arms()) {
    franka_ik_interfaces::msg::ChainInfo info;
    info.arm_id = arm.arm_id();
    info.base_frame = arm.base_frame();
    info.flange_frame = arm.flange_frame();
    info.hand_tcp_frame = arm.flange_to_hand_tcp().has_value() ? arm.hand_tcp_frame() : "";
    info.joint_names = arm.joint_names();
    info.position_lower = arm.position_lower();
    info.position_upper = arm.position_upper();
    info.velocity_limit = arm.velocity_limits();
    copyTransform(arm.root_to_base(), info.root_to_base);
    response->chains.push_back(std::move(info));
  }
  response->default_solver = configuration_.default_solver;
  response->analytic_backend_available = false;
  response->package_version = kPackageVersion;
}

}  // namespace franka_ik
