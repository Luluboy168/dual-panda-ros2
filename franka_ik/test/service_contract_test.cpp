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

#include "service_test_support.hpp"

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

#include <gtest/gtest.h>
#include <geometry_msgs/msg/pose.hpp>
#include <kdl/frames.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "franka_ik/forward_kinematics.hpp"
#include "franka_ik/panda_limits.hpp"
#include "franka_ik/robot_chains.hpp"
#include "franka_ik/service_node.hpp"
#include "franka_ik_interfaces/msg/ik_request.hpp"
#include "franka_ik_interfaces/msg/ik_result.hpp"
#include "franka_ik_interfaces/msg/ik_solution.hpp"
#include "franka_ik_interfaces/srv/get_chain_info.hpp"
#include "franka_ik_interfaces/srv/solve_ik.hpp"

#ifndef PANDA_IK_DUAL_TEST_URDF
#error "PANDA_IK_DUAL_TEST_URDF must name the rendered dual-arm IK wrapper"
#endif

#ifndef PANDA_IK_DUAL_HAND_TEST_URDF
#error "PANDA_IK_DUAL_HAND_TEST_URDF must name the rendered hand-equipped dual-arm IK wrapper"
#endif

namespace franka_ik {
namespace {

using namespace std::chrono_literals;
using IkRequest = franka_ik_interfaces::msg::IkRequest;
using IkResult = franka_ik_interfaces::msg::IkResult;
using IkSolution = franka_ik_interfaces::msg::IkSolution;
using GetChainInfo = franka_ik_interfaces::srv::GetChainInfo;
using SolveIk = franka_ik_interfaces::srv::SolveIk;

constexpr std::array<double, kPandaJointCount> kNominalJoints{
    {0.0, -0.4, 0.0, -1.8, 0.0, 1.4, 0.0}};

geometry_msgs::msg::Pose toPose(const KDL::Frame& frame) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = frame.p.x();
  pose.position.y = frame.p.y();
  pose.position.z = frame.p.z();
  frame.M.GetQuaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z,
                        pose.orientation.w);
  return pose;
}

void expectSameSolution(const IkSolution& left, const IkSolution& right) {
  EXPECT_EQ(left.positions, right.positions);
  EXPECT_EQ(left.redundancy_value, right.redundancy_value);
  EXPECT_EQ(left.position_error, right.position_error);
  EXPECT_EQ(left.orientation_error, right.orientation_error);
  EXPECT_EQ(left.seed_distance, right.seed_distance);
  EXPECT_EQ(left.branch, right.branch);
}

class ServiceContractTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  // Overridden by the hand-equipped fixture below; every other test keeps the hand-less wrapper.
  virtual const char* urdfPath() const { return PANDA_IK_DUAL_TEST_URDF; }

  void SetUp() override {
    urdf_ = test::readTextFile(urdfPath());
    chains_ = std::make_unique<RobotChains>(urdf_, std::vector<std::string>{"panda1", "panda2"});

    rclcpp::NodeOptions options;
    options.use_global_arguments(false);
    options.parameter_overrides({
        rclcpp::Parameter("robot_description", urdf_),
        rclcpp::Parameter("arm_ids", std::vector<std::string>{"panda1", "panda2"}),
        rclcpp::Parameter("default_solver", "numeric"),
    });
    service_node_ = std::make_shared<ServiceNode>(options);
    rclcpp::NodeOptions client_options;
    client_options.use_global_arguments(false);
    client_node_ = std::make_shared<rclcpp::Node>("franka_ik_contract_client", client_options);
    solve_client_ = client_node_->create_client<SolveIk>("/franka_ik_service/solve_ik");
    chain_info_client_ = client_node_->create_client<GetChainInfo>("/franka_ik_service/chain_info");
    executor_.add_node(service_node_);
    executor_.add_node(client_node_);
    ASSERT_TRUE(solve_client_->wait_for_service(3s));
    ASSERT_TRUE(chain_info_client_->wait_for_service(3s));
  }

  void TearDown() override {
    if (client_node_) {
      executor_.remove_node(client_node_);
    }
    if (service_node_) {
      executor_.remove_node(service_node_);
    }
    solve_client_.reset();
    chain_info_client_.reset();
    client_node_.reset();
    service_node_.reset();
    chains_.reset();
  }

  IkRequest validRequest(const std::array<double, kPandaJointCount>& target_joints = kNominalJoints,
                         const std::string& arm_id = "panda1") const {
    const ArmChain& arm = chains_->arm(arm_id);
    ForwardKinematics forward_kinematics(arm.flange_chain());
    IkRequest request;
    request.frame_id = arm.base_frame();
    request.arm_id = arm_id;
    request.tip_frame = IkRequest::TIP_FLANGE;
    request.target_pose = toPose(forward_kinematics.compute(target_joints));
    request.seed_positions = target_joints;
    request.redundancy_mode = IkRequest::REDUNDANCY_FROM_SEED;
    request.redundancy_value = 0.0;
    request.max_solutions = 1;
    request.solver = IkRequest::SOLVER_DEFAULT;
    request.position_tolerance = 0.0;
    request.orientation_tolerance = 0.0;
    request.joint_limit_margin = 0.0;
    return request;
  }

  std::shared_ptr<SolveIk::Response> callSolve(const IkRequest& request) {
    auto service_request = std::make_shared<SolveIk::Request>();
    service_request->request = request;
    auto future = solve_client_->async_send_request(service_request);
    if (executor_.spin_until_future_complete(future, 5s) != rclcpp::FutureReturnCode::SUCCESS) {
      throw std::runtime_error("SolveIk request did not complete");
    }
    return future.get();
  }

  std::shared_ptr<GetChainInfo::Response> callChainInfo() {
    auto future = chain_info_client_->async_send_request(std::make_shared<GetChainInfo::Request>());
    if (executor_.spin_until_future_complete(future, 5s) != rclcpp::FutureReturnCode::SUCCESS) {
      throw std::runtime_error("GetChainInfo request did not complete");
    }
    return future.get();
  }

  std::string urdf_;
  std::unique_ptr<RobotChains> chains_;
  std::shared_ptr<ServiceNode> service_node_;
  std::shared_ptr<rclcpp::Node> client_node_;
  rclcpp::Client<SolveIk>::SharedPtr solve_client_;
  rclcpp::Client<GetChainInfo>::SharedPtr chain_info_client_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

TEST_F(ServiceContractTest, ChainInfoReportsExactModelAndIndependentUrdfHash) {
  ASSERT_EQ(test::sha256Hex("abc"),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  const auto response = callChainInfo();
  EXPECT_EQ(response->urdf_root_frame, "base_link");
  EXPECT_EQ(response->urdf_sha256, test::sha256Hex(urdf_));
  EXPECT_EQ(response->urdf_sha256.size(), 64U);
  EXPECT_EQ(response->default_solver, IkRequest::SOLVER_NUMERIC);
  EXPECT_FALSE(response->analytic_backend_available);
  EXPECT_EQ(response->package_version, "0.1.0");
  ASSERT_EQ(response->chains.size(), 2U);

  for (std::size_t arm_index = 0; arm_index < response->chains.size(); ++arm_index) {
    const std::string arm_id = arm_index == 0 ? "panda1" : "panda2";
    const auto& info = response->chains[arm_index];
    EXPECT_EQ(info.arm_id, arm_id);
    EXPECT_EQ(info.base_frame, arm_id + "_link0");
    EXPECT_EQ(info.flange_frame, arm_id + "_link8");
    EXPECT_TRUE(info.hand_tcp_frame.empty());
    EXPECT_EQ(info.position_lower, kPandaPositionLowerLimits);
    EXPECT_EQ(info.position_upper, kPandaPositionUpperLimits);
    EXPECT_EQ(info.velocity_limit, kPandaVelocityLimits);
    for (std::size_t joint = 0; joint < kPandaJointCount; ++joint) {
      EXPECT_EQ(info.joint_names[joint], arm_id + "_joint" + std::to_string(joint + 1U));
    }
    EXPECT_DOUBLE_EQ(info.root_to_base.translation.x, 0.0);
    EXPECT_DOUBLE_EQ(info.root_to_base.translation.y, arm_index == 0 ? 0.50 : -0.50);
    EXPECT_DOUBLE_EQ(info.root_to_base.translation.z, 0.0);
    EXPECT_DOUBLE_EQ(info.root_to_base.rotation.x, 0.0);
    EXPECT_DOUBLE_EQ(info.root_to_base.rotation.y, 0.0);
    EXPECT_DOUBLE_EQ(info.root_to_base.rotation.z, 0.0);
    EXPECT_DOUBLE_EQ(info.root_to_base.rotation.w, 1.0);
  }
}

TEST_F(ServiceContractTest, DefaultAndExplicitNumericSolvesAreDeterministic) {
  IkRequest request = validRequest();
  const auto first = callSolve(request)->result;
  ASSERT_EQ(first.result, IkResult::RESULT_SUCCESS) << first.message;
  ASSERT_EQ(first.solutions.size(), 1U);
  EXPECT_EQ(first.solver_used, IkRequest::SOLVER_NUMERIC);
  EXPECT_EQ(first.solutions.front().positions, kNominalJoints);
  EXPECT_EQ(first.solutions.front().redundancy_value, first.solutions.front().positions[6]);
  EXPECT_LE(first.solutions.front().position_error, 1.0e-4);
  EXPECT_LE(first.solutions.front().orientation_error, 1.0e-3);
  EXPECT_EQ(first.solutions.front().branch, IkSolution::BRANCH_NUMERIC);
  EXPECT_GE(first.solve_time.sec, 0);
  EXPECT_LT(first.solve_time.nanosec, 1000000000U);

  request.solver = IkRequest::SOLVER_NUMERIC;
  const auto second = callSolve(request)->result;
  ASSERT_EQ(second.result, IkResult::RESULT_SUCCESS) << second.message;
  ASSERT_EQ(second.solutions.size(), 1U);
  EXPECT_EQ(second.solver_used, IkRequest::SOLVER_NUMERIC);
  EXPECT_EQ(second.result, first.result);
  EXPECT_EQ(second.message, first.message);
  EXPECT_EQ(second.iterations, first.iterations);
  expectSameSolution(second.solutions.front(), first.solutions.front());
}

TEST_F(ServiceContractTest, ReachabilityProbeSolvesButSuppressesItsWitness) {
  IkRequest request = validRequest();
  request.max_solutions = 0;
  const auto result = callSolve(request)->result;
  EXPECT_EQ(result.result, IkResult::RESULT_SUCCESS) << result.message;
  EXPECT_TRUE(result.solutions.empty());
  EXPECT_EQ(result.solver_used, IkRequest::SOLVER_NUMERIC);
}

TEST_F(ServiceContractTest, MaximumSolutionBoundIsEnforced) {
  IkRequest request = validRequest();
  request.max_solutions = 4;
  const auto bounded = callSolve(request)->result;
  ASSERT_EQ(bounded.result, IkResult::RESULT_SUCCESS) << bounded.message;
  EXPECT_EQ(bounded.solutions.size(), 1U);

  request.max_solutions = 5;
  const auto invalid = callSolve(request)->result;
  EXPECT_EQ(invalid.result, IkResult::RESULT_BAD_REQUEST);
  EXPECT_TRUE(invalid.solutions.empty());
}

TEST_F(ServiceContractTest, EmptyBaseAndRootFrameModesProduceTheSameSolution) {
  IkRequest base_request = validRequest();
  base_request.frame_id.clear();
  const auto empty_base = callSolve(base_request)->result;
  ASSERT_EQ(empty_base.result, IkResult::RESULT_SUCCESS) << empty_base.message;

  IkRequest root_request = validRequest();
  const ArmChain& arm = chains_->arm(root_request.arm_id);
  ForwardKinematics forward_kinematics(arm.flange_chain());
  root_request.frame_id = chains_->root_frame();
  root_request.target_pose =
      toPose(arm.root_to_base() * forward_kinematics.compute(kNominalJoints));
  const auto root = callSolve(root_request)->result;
  ASSERT_EQ(root.result, IkResult::RESULT_SUCCESS) << root.message;
  ASSERT_EQ(root.solutions.size(), 1U);
  ASSERT_EQ(empty_base.solutions.size(), 1U);
  EXPECT_EQ(root.solutions.front().positions, empty_base.solutions.front().positions);
  EXPECT_EQ(root.solutions.front().redundancy_value, empty_base.solutions.front().redundancy_value);
  EXPECT_EQ(root.solutions.front().seed_distance, empty_base.solutions.front().seed_distance);
  EXPECT_EQ(root.solutions.front().branch, empty_base.solutions.front().branch);
  EXPECT_LE(root.solutions.front().position_error, 1.0e-15);
  EXPECT_LE(root.solutions.front().orientation_error, 1.0e-15);
  EXPECT_LE(empty_base.solutions.front().position_error, 1.0e-15);
  EXPECT_LE(empty_base.solutions.front().orientation_error, 1.0e-15);
}

TEST_F(ServiceContractTest, UnknownArmFrameAndHandlessTcpHaveDistinctResults) {
  IkRequest request = validRequest();
  request.arm_id = "panda3";
  EXPECT_EQ(callSolve(request)->result.result, IkResult::RESULT_UNKNOWN_ARM);

  request = validRequest();
  request.frame_id = "map";
  EXPECT_EQ(callSolve(request)->result.result, IkResult::RESULT_UNKNOWN_FRAME);

  request = validRequest();
  request.tip_frame = IkRequest::TIP_HAND_TCP;
  EXPECT_EQ(callSolve(request)->result.result, IkResult::RESULT_UNSUPPORTED_TIP);
}

TEST_F(ServiceContractTest, InvalidEnumsAndUnavailableAnalyticBackendAreBadRequests) {
  IkRequest request = validRequest();
  request.tip_frame = 2;
  EXPECT_EQ(callSolve(request)->result.result, IkResult::RESULT_BAD_REQUEST);

  request = validRequest();
  request.redundancy_mode = 2;
  EXPECT_EQ(callSolve(request)->result.result, IkResult::RESULT_BAD_REQUEST);

  request = validRequest();
  request.solver = 3;
  EXPECT_EQ(callSolve(request)->result.result, IkResult::RESULT_BAD_REQUEST);

  request = validRequest();
  request.solver = IkRequest::SOLVER_ANALYTIC;
  const auto analytic = callSolve(request)->result;
  EXPECT_EQ(analytic.result, IkResult::RESULT_BAD_REQUEST);
  EXPECT_EQ(analytic.solver_used, IkRequest::SOLVER_ANALYTIC);
  EXPECT_TRUE(analytic.solutions.empty());
}

TEST_F(ServiceContractTest, QuaternionIsNormalizedAndInvalidPoseValuesAreRejected) {
  IkRequest request = validRequest();
  request.target_pose.orientation.x *= 2.0;
  request.target_pose.orientation.y *= 2.0;
  request.target_pose.orientation.z *= 2.0;
  request.target_pose.orientation.w *= 2.0;
  EXPECT_EQ(callSolve(request)->result.result, IkResult::RESULT_SUCCESS);

  request = validRequest();
  request.target_pose.orientation.x = 0.0;
  request.target_pose.orientation.y = 0.0;
  request.target_pose.orientation.z = 0.0;
  request.target_pose.orientation.w = 0.0;
  EXPECT_EQ(callSolve(request)->result.result, IkResult::RESULT_BAD_REQUEST);

  request = validRequest();
  request.target_pose.orientation.w = 1.0e-7;
  request.target_pose.orientation.x = 0.0;
  request.target_pose.orientation.y = 0.0;
  request.target_pose.orientation.z = 0.0;
  EXPECT_EQ(callSolve(request)->result.result, IkResult::RESULT_BAD_REQUEST);

  request = validRequest();
  request.target_pose.orientation.w = 1.0e7;
  request.target_pose.orientation.x = 0.0;
  request.target_pose.orientation.y = 0.0;
  request.target_pose.orientation.z = 0.0;
  EXPECT_EQ(callSolve(request)->result.result, IkResult::RESULT_BAD_REQUEST);

  request = validRequest();
  request.target_pose.orientation.w = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(callSolve(request)->result.result, IkResult::RESULT_BAD_REQUEST);

  request = validRequest();
  request.target_pose.position.x = std::numeric_limits<double>::infinity();
  EXPECT_EQ(callSolve(request)->result.result, IkResult::RESULT_BAD_REQUEST);
}

TEST_F(ServiceContractTest, SeedToleranceAndMarginValidationUseContractCodes) {
  IkRequest request = validRequest();
  request.seed_positions[3] = 0.0;
  EXPECT_EQ(callSolve(request)->result.result, IkResult::RESULT_SEED_OUT_OF_LIMITS);

  request = validRequest();
  request.seed_positions[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(callSolve(request)->result.result, IkResult::RESULT_BAD_REQUEST);

  request = validRequest();
  request.position_tolerance = 1.0e-5;
  request.orientation_tolerance = 1.0e-4;
  request.joint_limit_margin = 0.01;
  EXPECT_EQ(callSolve(request)->result.result, IkResult::RESULT_SUCCESS);

  for (const auto selector : {0U, 1U, 2U}) {
    request = validRequest();
    if (selector == 0U) {
      request.position_tolerance = 0.011;
    } else if (selector == 1U) {
      request.orientation_tolerance = 0.11;
    } else {
      request.joint_limit_margin = 0.101;
    }
    EXPECT_EQ(callSolve(request)->result.result, IkResult::RESULT_BAD_REQUEST);
  }

  request = validRequest();
  request.position_tolerance = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(callSolve(request)->result.result, IkResult::RESULT_BAD_REQUEST);
  request = validRequest();
  request.position_tolerance = -1.0e-6;
  EXPECT_EQ(callSolve(request)->result.result, IkResult::RESULT_BAD_REQUEST);
  request = validRequest();
  request.orientation_tolerance = std::numeric_limits<double>::infinity();
  EXPECT_EQ(callSolve(request)->result.result, IkResult::RESULT_BAD_REQUEST);
  request = validRequest();
  request.orientation_tolerance = -1.0e-6;
  EXPECT_EQ(callSolve(request)->result.result, IkResult::RESULT_BAD_REQUEST);
  request = validRequest();
  request.joint_limit_margin = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(callSolve(request)->result.result, IkResult::RESULT_BAD_REQUEST);
  request = validRequest();
  request.joint_limit_margin = -1.0e-6;
  EXPECT_EQ(callSolve(request)->result.result, IkResult::RESULT_BAD_REQUEST);

  std::array<double, kPandaJointCount> near_limit = kNominalJoints;
  near_limit[0] = kPandaPositionLowerLimits[0] + 0.001;
  request = validRequest(near_limit);
  request.joint_limit_margin = 0.01;
  EXPECT_EQ(callSolve(request)->result.result, IkResult::RESULT_SEED_OUT_OF_LIMITS);
}

TEST_F(ServiceContractTest, BothRedundancyModesHoldTheRequestedQ7Exactly) {
  IkRequest from_seed = validRequest();
  const auto seeded = callSolve(from_seed)->result;
  ASSERT_EQ(seeded.result, IkResult::RESULT_SUCCESS) << seeded.message;
  ASSERT_EQ(seeded.solutions.size(), 1U);
  EXPECT_EQ(seeded.solutions.front().positions[6], from_seed.seed_positions[6]);

  std::array<double, kPandaJointCount> target_joints = kNominalJoints;
  target_joints[6] = 0.25;
  IkRequest fixed = validRequest(target_joints);
  fixed.seed_positions[6] = 0.0;
  fixed.redundancy_mode = IkRequest::REDUNDANCY_FIXED;
  fixed.redundancy_value = target_joints[6];
  const auto fixed_result = callSolve(fixed)->result;
  ASSERT_EQ(fixed_result.result, IkResult::RESULT_SUCCESS) << fixed_result.message;
  ASSERT_EQ(fixed_result.solutions.size(), 1U);
  EXPECT_EQ(fixed_result.solutions.front().positions[6], fixed.redundancy_value);
  EXPECT_EQ(fixed_result.solutions.front().redundancy_value, fixed.redundancy_value);

  fixed.redundancy_value = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(callSolve(fixed)->result.result, IkResult::RESULT_BAD_REQUEST);
  fixed.redundancy_value = kPandaPositionUpperLimits[6] + 0.1;
  EXPECT_EQ(callSolve(fixed)->result.result, IkResult::RESULT_BAD_REQUEST);
}

TEST_F(ServiceContractTest, ProvenGeometricAndJointLimitExclusionsReachTheWire) {
  IkRequest far = validRequest();
  far.target_pose.position.x = 0.18096185973712453;
  far.target_pose.position.y = -1.2227414573010629;
  far.target_pose.position.z = 0.9348903352668751;
  far.target_pose.orientation.x = 0.4922055611082705;
  far.target_pose.orientation.y = 0.18939754829650607;
  far.target_pose.orientation.z = 0.2860513525304495;
  far.target_pose.orientation.w = 0.800023048436022;
  // clang-format off
  far.seed_positions = {2.3322051274129203, -0.7195985475583473, -1.3624407049661704,
                        -2.927251957966345, -1.777438077902987, 0.3800066499618793,
                        1.0600304066639459};
  // clang-format on
  far.max_solutions = 0;
  EXPECT_EQ(callSolve(far)->result.result, IkResult::RESULT_UNREACHABLE);

  IkRequest limits = validRequest();
  limits.target_pose.position.x = -0.12119447946640212;
  limits.target_pose.position.y = 0.028288279325829557;
  limits.target_pose.position.z = 0.31627974511761137;
  limits.target_pose.orientation.x = -0.787613608860796;
  limits.target_pose.orientation.y = 0.28554092178532176;
  limits.target_pose.orientation.z = 0.25642778984119324;
  limits.target_pose.orientation.w = 0.4820539116327372;
  // clang-format off
  limits.seed_positions = {-1.7470302247752467, -1.6742992383832216, 2.3452560189342924,
                           -3.0518, 0.8281143679798251, 0.610783598200198,
                           -2.437307451459673};
  // clang-format on
  EXPECT_EQ(callSolve(limits)->result.result, IkResult::RESULT_LIMITS_VIOLATED);
}

// ---------------------------------------------------------------------------------------------
// Hand-TCP success path.
//
// Regression for post-MVP verification finding 3: before this fixture existed, TIP_HAND_TCP was
// only ever exercised against a hand-less description to obtain RESULT_UNSUPPORTED_TIP, and the
// remaining hand coverage stopped at the chain model.  Nothing solved a hand-TCP target end to end
// through the service, so a tip transform applied in the wrong direction would have shipped green.
// ---------------------------------------------------------------------------------------------
class HandTcpServiceContractTest : public ServiceContractTest {
 protected:
  const char* urdfPath() const override { return PANDA_IK_DUAL_HAND_TEST_URDF; }

  // The fixed panda_link8 -> <arm>_hand_tcp offset from franka_description's hand.xacro.
  static constexpr double kFlangeToTcpOffset = 0.1034;

  KDL::Frame handTcpPose(const std::string& arm_id,
                         const std::array<double, kPandaJointCount>& joints) const {
    const ArmChain& arm = chains_->arm(arm_id);
    ForwardKinematics flange_forward_kinematics(arm.flange_chain());
    return flange_forward_kinematics.compute(joints) * arm.flange_to_hand_tcp().value();
  }

  KDL::Frame flangePose(const std::string& arm_id,
                        const std::array<double, kPandaJointCount>& joints) const {
    ForwardKinematics flange_forward_kinematics(chains_->arm(arm_id).flange_chain());
    return flange_forward_kinematics.compute(joints);
  }

  IkRequest handTcpRequest(const std::array<double, kPandaJointCount>& target_joints,
                           const std::array<double, kPandaJointCount>& seed_joints,
                           const std::string& arm_id) const {
    IkRequest request;
    request.frame_id = chains_->arm(arm_id).base_frame();
    request.arm_id = arm_id;
    request.tip_frame = IkRequest::TIP_HAND_TCP;
    request.target_pose = toPose(handTcpPose(arm_id, target_joints));
    request.seed_positions = seed_joints;
    request.redundancy_mode = IkRequest::REDUNDANCY_FROM_SEED;
    request.redundancy_value = 0.0;
    request.max_solutions = 1;
    request.solver = IkRequest::SOLVER_DEFAULT;
    request.position_tolerance = 0.0;
    request.orientation_tolerance = 0.0;
    request.joint_limit_margin = 0.0;
    return request;
  }
};

TEST_F(HandTcpServiceContractTest, ChainInfoAdvertisesTheHandTcpFrameOnBothArms) {
  const auto response = callChainInfo();
  ASSERT_EQ(response->chains.size(), 2U);
  for (std::size_t arm_index = 0; arm_index < response->chains.size(); ++arm_index) {
    const std::string arm_id = arm_index == 0 ? "panda1" : "panda2";
    EXPECT_EQ(response->chains[arm_index].hand_tcp_frame, arm_id + "_hand_tcp");
  }
}

TEST_F(HandTcpServiceContractTest, HandTcpTargetsSolveAndReportErrorAtTheRequestedTip) {
  // The node defaults from README section 5.7; the request leaves both tolerances at 0.0.
  constexpr double kDefaultPositionTolerance = 1.0e-4;
  constexpr double kDefaultOrientationTolerance = 1.0e-3;
  constexpr std::array<std::array<double, kPandaJointCount>, 3> kSeedOffsets{{
      {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
      {{0.01, -0.01, 0.01, 0.01, -0.01, 0.01, 0.0}},
      {{-0.05, 0.04, -0.03, 0.05, 0.03, -0.04, 0.0}},
  }};

  for (const std::string& arm_id : {std::string("panda1"), std::string("panda2")}) {
    const ArmChain& arm = chains_->arm(arm_id);
    ASSERT_TRUE(arm.flange_to_hand_tcp().has_value()) << arm_id;

    for (std::size_t offset_index = 0; offset_index < kSeedOffsets.size(); ++offset_index) {
      SCOPED_TRACE(arm_id + " seed offset " + std::to_string(offset_index));
      std::array<double, kPandaJointCount> seed = kNominalJoints;
      for (std::size_t joint = 0; joint < kPandaJointCount; ++joint) {
        seed[joint] += kSeedOffsets[offset_index][joint];
      }

      const IkRequest request = handTcpRequest(kNominalJoints, seed, arm_id);
      const auto response = callSolve(request);
      ASSERT_EQ(response->result.result, IkResult::RESULT_SUCCESS) << response->result.message;
      ASSERT_EQ(response->result.solutions.size(), 1U);
      const IkSolution& solution = response->result.solutions.front();

      // Every returned joint value is inside the URDF limits (contract invariant 2).
      for (std::size_t joint = 0; joint < kPandaJointCount; ++joint) {
        EXPECT_GE(solution.positions[joint], arm.position_lower()[joint]);
        EXPECT_LE(solution.positions[joint], arm.position_upper()[joint]);
      }

      // The error the service reports is measured at the requested tip, not at the flange.
      std::array<double, kPandaJointCount> returned{};
      for (std::size_t joint = 0; joint < kPandaJointCount; ++joint) {
        returned[joint] = solution.positions[joint];
      }
      const KDL::Frame target_tcp = handTcpPose(arm_id, kNominalJoints);
      const PoseError tcp_error = computePoseError(target_tcp, handTcpPose(arm_id, returned));
      EXPECT_LE(tcp_error.position, kDefaultPositionTolerance);
      EXPECT_LE(tcp_error.orientation, kDefaultOrientationTolerance);
      // The service walks a dedicated base -> hand_tcp KDL chain while this test post-multiplies
      // the fixed flange offset, so the two agree only to floating-point composition order --
      // observed differences are ~1e-8 m.  That is five orders of magnitude below the 0.1034 m
      // flange offset, so the comparison still discriminates a TCP-measured error from a
      // flange-measured one, which is the property under test.
      EXPECT_NEAR(solution.position_error, tcp_error.position, 1.0e-6);
      EXPECT_NEAR(solution.orientation_error, tcp_error.orientation, 1.0e-6);
      EXPECT_LE(solution.position_error, kDefaultPositionTolerance);
      EXPECT_LE(solution.orientation_error, kDefaultOrientationTolerance);

      // The tip transform is applied in the correct direction: the solved flange sits exactly one
      // hand-TCP offset away from the TCP target.  Flipping the transform would put the flange on
      // the target and the TCP 0.1034 m past it.
      const KDL::Frame solved_flange = flangePose(arm_id, returned);
      EXPECT_NEAR((solved_flange.p - target_tcp.p).Norm(), kFlangeToTcpOffset, 1.0e-4);
      EXPECT_GT((solved_flange.p - target_tcp.p).Norm(), kDefaultPositionTolerance);

      EXPECT_EQ(solution.branch, IkSolution::BRANCH_NUMERIC);
      EXPECT_EQ(response->result.solver_used, IkRequest::SOLVER_NUMERIC);
    }
  }
}

TEST_F(HandTcpServiceContractTest, ExactHandTcpSeedRoundTripsBitExactly) {
  const IkRequest request = handTcpRequest(kNominalJoints, kNominalJoints, "panda1");
  const auto response = callSolve(request);
  ASSERT_EQ(response->result.result, IkResult::RESULT_SUCCESS) << response->result.message;
  ASSERT_EQ(response->result.solutions.size(), 1U);
  const IkSolution& solution = response->result.solutions.front();
  for (std::size_t joint = 0; joint < kPandaJointCount; ++joint) {
    EXPECT_DOUBLE_EQ(solution.positions[joint], kNominalJoints[joint]);
  }
  EXPECT_EQ(solution.redundancy_value, kNominalJoints[kPandaJointCount - 1]);
}

TEST(ServiceStatusMappingTest, EveryBackendStatusMapsToTheFrozenWireCode) {
  constexpr std::array<std::pair<SolveStatus, std::uint8_t>, 9> kMappings{{
      {SolveStatus::Success, IkResult::RESULT_SUCCESS},
      {SolveStatus::BadRequest, IkResult::RESULT_BAD_REQUEST},
      {SolveStatus::SeedOutOfLimits, IkResult::RESULT_SEED_OUT_OF_LIMITS},
      {SolveStatus::GeometricallyUnreachable, IkResult::RESULT_UNREACHABLE},
      {SolveStatus::JointLimitsViolated, IkResult::RESULT_LIMITS_VIOLATED},
      {SolveStatus::ToleranceNotMet, IkResult::RESULT_TOLERANCE_NOT_MET},
      {SolveStatus::IterationBudgetExhausted, IkResult::RESULT_ITERATION_BUDGET_EXHAUSTED},
      {SolveStatus::NoAcceptableSolution, IkResult::RESULT_NO_ACCEPTABLE_SOLUTION},
      {SolveStatus::InternalError, IkResult::RESULT_INTERNAL_ERROR},
  }};
  for (const auto& [status, expected] : kMappings) {
    EXPECT_EQ(solveStatusToResultCode(status), expected);
  }
  EXPECT_EQ(solveStatusToResultCode(static_cast<SolveStatus>(255)),
            IkResult::RESULT_INTERNAL_ERROR);
}

}  // namespace
}  // namespace franka_ik
