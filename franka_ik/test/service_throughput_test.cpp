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
#include "service_test_support.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <geometry_msgs/msg/pose.hpp>
#include <kdl/frames.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "franka_ik/service_node.hpp"
#include "franka_ik_interfaces/msg/ik_request.hpp"
#include "franka_ik_interfaces/msg/ik_result.hpp"
#include "franka_ik_interfaces/srv/solve_ik.hpp"

#ifndef PANDA_IK_DUAL_TEST_URDF
#error "PANDA_IK_DUAL_TEST_URDF must name the rendered dual-arm IK wrapper"
#endif

namespace franka_ik {
namespace {

using namespace std::chrono_literals;
using IkRequest = franka_ik_interfaces::msg::IkRequest;
using IkResult = franka_ik_interfaces::msg::IkResult;
using SolveIk = franka_ik_interfaces::srv::SolveIk;

double percentile(const std::vector<double>& sorted, const double probability) {
  const std::size_t index =
      static_cast<std::size_t>(std::ceil(probability * static_cast<double>(sorted.size())) - 1.0);
  return sorted.at(std::min(index, sorted.size() - 1));
}

geometry_msgs::msg::Pose toPose(const KDL::Frame& frame) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = frame.p.x();
  pose.position.y = frame.p.y();
  pose.position.z = frame.p.z();
  frame.M.GetQuaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z,
                        pose.orientation.w);
  return pose;
}

class ServiceThroughputTest : public ::testing::Test {
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

  void SetUp() override {
    rclcpp::NodeOptions service_options;
    service_options.use_global_arguments(false);
    service_options.parameter_overrides({
        rclcpp::Parameter("robot_description", test::readTextFile(PANDA_IK_DUAL_TEST_URDF)),
        rclcpp::Parameter("arm_ids", std::vector<std::string>{"panda1", "panda2"}),
        rclcpp::Parameter("default_solver", "numeric"),
    });
    service_node_ = std::make_shared<ServiceNode>(service_options);

    rclcpp::NodeOptions client_options;
    client_options.use_global_arguments(false);
    client_node_ = std::make_shared<rclcpp::Node>("franka_ik_throughput_client", client_options);
    solve_client_ = client_node_->create_client<SolveIk>("/franka_ik_service/solve_ik");
    executor_.add_node(service_node_);
    executor_.add_node(client_node_);
    ASSERT_TRUE(solve_client_->wait_for_service(3s));
  }

  void TearDown() override {
    executor_.remove_node(client_node_);
    executor_.remove_node(service_node_);
    solve_client_.reset();
    client_node_.reset();
    service_node_.reset();
  }

  IkRequest requestFor(const test::Witness& witness, const std::size_t index) const {
    IkRequest request;
    request.frame_id = "panda1_link0";
    request.arm_id = "panda1";
    request.tip_frame = IkRequest::TIP_FLANGE;
    request.target_pose = toPose(witness.target);
    request.seed_positions = test::perturbSeed(witness.seed, 0.01, index);
    request.redundancy_mode = IkRequest::REDUNDANCY_FROM_SEED;
    request.redundancy_value = 0.0;
    request.max_solutions = 1;
    request.solver = IkRequest::SOLVER_DEFAULT;
    request.position_tolerance = 0.0;
    request.orientation_tolerance = 0.0;
    request.joint_limit_margin = 0.0;
    return request;
  }

  std::shared_ptr<SolveIk::Response> call(const IkRequest& request) {
    auto service_request = std::make_shared<SolveIk::Request>();
    service_request->request = request;
    auto future = solve_client_->async_send_request(service_request);
    if (executor_.spin_until_future_complete(future, 5s) != rclcpp::FutureReturnCode::SUCCESS) {
      throw std::runtime_error("SolveIk request did not complete");
    }
    return future.get();
  }

  std::shared_ptr<ServiceNode> service_node_;
  std::shared_ptr<rclcpp::Node> client_node_;
  rclcpp::Client<SolveIk>::SharedPtr solve_client_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

TEST_F(ServiceThroughputTest, SequentialLocalhostShmCallsMeetRateAndLatencyGates) {
  const auto& witnesses = test::corpus().reachable_random;
  ASSERT_EQ(witnesses.size(), 2000U);

  for (std::size_t index = 0; index < 16; ++index) {
    const auto response = call(requestFor(witnesses[index], index));
    ASSERT_EQ(response->result.result, IkResult::RESULT_SUCCESS) << response->result.message;
  }

  std::vector<double> latency_microseconds;
  latency_microseconds.reserve(witnesses.size());
  std::size_t successes = 0;
  const auto throughput_begin = std::chrono::steady_clock::now();
  for (std::size_t index = 0; index < witnesses.size(); ++index) {
    const auto begin = std::chrono::steady_clock::now();
    const auto response = call(requestFor(witnesses[index], index));
    const auto end = std::chrono::steady_clock::now();
    latency_microseconds.push_back(std::chrono::duration<double, std::micro>(end - begin).count());
    successes += response->result.result == IkResult::RESULT_SUCCESS;
  }
  const auto throughput_end = std::chrono::steady_clock::now();

  std::sort(latency_microseconds.begin(), latency_microseconds.end());
  const double p50 = percentile(latency_microseconds, 0.50);
  const double p90 = percentile(latency_microseconds, 0.90);
  const double p95 = percentile(latency_microseconds, 0.95);
  const double p99 = percentile(latency_microseconds, 0.99);
  const double maximum = latency_microseconds.back();
  const double elapsed_seconds =
      std::chrono::duration<double>(throughput_end - throughput_begin).count();
  const double calls_per_second = static_cast<double>(witnesses.size()) / elapsed_seconds;

  std::cout << std::fixed << std::setprecision(3) << "service Release round-trip [us]: p50=" << p50
            << " p90=" << p90 << " p95=" << p95 << " p99=" << p99 << " max=" << maximum
            << " rate_hz=" << calls_per_second << " successes=" << successes << '/'
            << witnesses.size() << '\n';

  EXPECT_GE(static_cast<double>(successes) / witnesses.size(), 0.99);
  EXPECT_GE(calls_per_second, 200.0);
  // The product target is 2 ms; this 4x ceiling catches meaningful regressions without turning
  // normal loaded-host scheduling noise into a CI failure.
  EXPECT_LE(p99, 8000.0);
}

}  // namespace
}  // namespace franka_ik
