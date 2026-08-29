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

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstring>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "franka_ik/numeric_backend.hpp"

namespace franka_ik {
namespace {

struct ResultFingerprint {
  SolveStatus status;
  std::uint16_t iterations;
  int return_code;
  bool invoked;
  std::vector<IkCandidate> solutions;
};

ResultFingerprint fingerprint(const SolveOutput& output) {
  return {output.status, output.iterations, output.diagnostics.solver_return_code,
          output.diagnostics.solver_invoked, output.solutions};
}

void expectBitIdentical(const ResultFingerprint& expected, const SolveOutput& actual) {
  ASSERT_EQ(actual.status, expected.status);
  ASSERT_EQ(actual.iterations, expected.iterations);
  ASSERT_EQ(actual.diagnostics.solver_return_code, expected.return_code);
  ASSERT_EQ(actual.diagnostics.solver_invoked, expected.invoked);
  ASSERT_EQ(actual.solutions.size(), expected.solutions.size());
  for (std::size_t index = 0; index < expected.solutions.size(); ++index) {
    const auto& left = actual.solutions[index];
    const auto& right = expected.solutions[index];
    for (std::size_t joint = 0; joint < left.positions.size(); ++joint) {
      EXPECT_EQ(std::memcmp(&left.positions[joint], &right.positions[joint], sizeof(double)), 0);
    }
    EXPECT_EQ(std::memcmp(&left.redundancy_value, &right.redundancy_value, sizeof(double)), 0);
    EXPECT_EQ(std::memcmp(&left.position_error, &right.position_error, sizeof(double)), 0);
    EXPECT_EQ(std::memcmp(&left.orientation_error, &right.orientation_error, sizeof(double)), 0);
    EXPECT_EQ(std::memcmp(&left.seed_distance, &right.seed_distance, sizeof(double)), 0);
    EXPECT_EQ(left.branch, right.branch);
  }
}

std::vector<const test::Witness*> fullCorpusInStableOrder() {
  auto result = test::corpus().allReachable();
  for (const auto* set : {&test::corpus().unreachable_far, &test::corpus().unreachable_limits}) {
    for (const auto& witness : *set) {
      result.push_back(&witness);
    }
  }
  return result;
}

class SaturatingCpuLoad {
 public:
  SaturatingCpuLoad() {
    const unsigned int concurrency = std::max(1U, std::thread::hardware_concurrency());
    const unsigned int worker_count = concurrency > 1U ? concurrency - 1U : 1U;
    workers_.reserve(worker_count);
    for (unsigned int worker = 0; worker < worker_count; ++worker) {
      workers_.emplace_back([this, worker]() {
        volatile double accumulator = 1.0 + static_cast<double>(worker);
        while (!stop_.load(std::memory_order_relaxed)) {
          accumulator = accumulator * 1.0000001 + 0.0000001;
        }
      });
    }
  }

  ~SaturatingCpuLoad() {
    stop_.store(true, std::memory_order_relaxed);
    for (auto& worker : workers_) {
      worker.join();
    }
  }

  SaturatingCpuLoad(const SaturatingCpuLoad&) = delete;
  SaturatingCpuLoad& operator=(const SaturatingCpuLoad&) = delete;

 private:
  std::atomic<bool> stop_{false};
  std::vector<std::thread> workers_;
};

TEST(DeterminismTest, EntireCorpusIsBitIdenticalAcrossFiftyRuns) {
  test::Model model;
  NumericBackend backend(model.arm());
  const auto witnesses = fullCorpusInStableOrder();
  ASSERT_EQ(witnesses.size(), 7100U);

  std::vector<ResultFingerprint> baseline;
  baseline.reserve(witnesses.size());
  for (const auto* witness : witnesses) {
    SolveOutput output;
    backend.solve(test::exactInput(*witness), output);
    baseline.push_back(fingerprint(output));
  }

  for (std::size_t run = 1; run < 50; ++run) {
    for (std::size_t index = 0; index < witnesses.size(); ++index) {
      SCOPED_TRACE("run=" + std::to_string(run) + " witness=" + witnesses[index]->id);
      SolveOutput output;
      backend.solve(test::exactInput(*witnesses[index]), output);
      expectBitIdentical(baseline[index], output);
    }
  }
}

TEST(DeterminismTest, CpuLoadDoesNotChangeNumericResults) {
  test::Model model;
  NumericBackend backend(model.arm());
  const auto witnesses = fullCorpusInStableOrder();
  ASSERT_EQ(witnesses.size(), 7100U);
  std::vector<ResultFingerprint> baseline;
  baseline.reserve(witnesses.size());
  for (const auto* witness : witnesses) {
    SolveOutput output;
    backend.solve(test::exactInput(*witness), output);
    baseline.push_back(fingerprint(output));
  }

  SaturatingCpuLoad load;
  for (std::size_t run = 0; run < 50; ++run) {
    for (std::size_t index = 0; index < witnesses.size(); ++index) {
      SCOPED_TRACE("loaded_run=" + std::to_string(run) + " witness=" + witnesses[index]->id);
      SolveOutput output;
      backend.solve(test::exactInput(*witnesses[index]), output);
      expectBitIdentical(baseline[index], output);
    }
  }
}

}  // namespace
}  // namespace franka_ik
