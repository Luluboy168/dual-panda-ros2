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
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <vector>

#include <gtest/gtest.h>

#include "franka_ik/numeric_backend.hpp"

namespace franka_ik {
namespace {

constexpr std::uint16_t kTunedIterationBudget = 40;

double percentile(const std::vector<double>& sorted, const double probability) {
  const std::size_t index =
      static_cast<std::size_t>(std::ceil(probability * static_cast<double>(sorted.size())) - 1.0);
  return sorted.at(std::min(index, sorted.size() - 1));
}

TEST(SolveLatencyTest, NumericReachableRandomDistributionMeetsGenerousGate) {
  test::Model model;
  NumericBackend backend(model.arm());
  const auto& witnesses = test::corpus().reachable_random;
  ASSERT_EQ(witnesses.size(), 2000U);

  // Warm allocations and instruction/data caches outside the measured distribution.
  for (std::size_t index = 0; index < 16; ++index) {
    SolveInput input = test::exactInput(witnesses[index]);
    input.seed_positions = test::perturbSeed(input.seed_positions, 0.01, index);
    input.position_tolerance = 1.0e-4;
    input.orientation_tolerance = 1.0e-3;
    input.numeric_eps = 1.0e-6;
    input.numeric_max_iterations = kTunedIterationBudget;
    SolveOutput output;
    backend.solve(input, output);
  }

  std::vector<double> latency_microseconds;
  latency_microseconds.reserve(witnesses.size());
  std::vector<std::uint16_t> iteration_counts;
  iteration_counts.reserve(witnesses.size());
  std::size_t successes = 0;
  std::size_t raw_limit_violations = 0;
  for (std::size_t index = 0; index < witnesses.size(); ++index) {
    SolveInput input = test::exactInput(witnesses[index]);
    input.seed_positions = test::perturbSeed(input.seed_positions, 0.01, index);
    input.position_tolerance = 1.0e-4;
    input.orientation_tolerance = 1.0e-3;
    input.numeric_eps = 1.0e-6;
    input.numeric_max_iterations = kTunedIterationBudget;
    SolveOutput output;
    const auto begin = std::chrono::steady_clock::now();
    backend.solve(input, output);
    const auto end = std::chrono::steady_clock::now();
    latency_microseconds.push_back(std::chrono::duration<double, std::micro>(end - begin).count());
    iteration_counts.push_back(output.iterations);
    successes += output.status == SolveStatus::Success;
    raw_limit_violations += output.diagnostics.solver_invoked &&
                            output.diagnostics.raw_candidate_finite &&
                            !output.diagnostics.raw_candidate_within_limits;
  }
  std::sort(latency_microseconds.begin(), latency_microseconds.end());
  const double p50 = percentile(latency_microseconds, 0.50);
  const double p90 = percentile(latency_microseconds, 0.90);
  const double p95 = percentile(latency_microseconds, 0.95);
  const double p99 = percentile(latency_microseconds, 0.99);
  const double maximum = latency_microseconds.back();
  std::sort(iteration_counts.begin(), iteration_counts.end());
  const auto iteration_percentile = [&iteration_counts](const double probability) {
    const std::size_t index = static_cast<std::size_t>(
        std::ceil(probability * static_cast<double>(iteration_counts.size())) - 1.0);
    return iteration_counts.at(std::min(index, iteration_counts.size() - 1));
  };
  const std::size_t budget_hits = static_cast<std::size_t>(
      std::count(iteration_counts.begin(), iteration_counts.end(), kTunedIterationBudget));
  std::cout << std::fixed << std::setprecision(3) << "numeric Release latency [us]: p50=" << p50
            << " p90=" << p90 << " p95=" << p95 << " p99=" << p99 << " max=" << maximum
            << " successes=" << successes << '/' << witnesses.size()
            << " raw_limit_violations=" << raw_limit_violations << '\n';
  std::cout << "numeric convergence iterations: p50=" << iteration_percentile(0.50)
            << " p90=" << iteration_percentile(0.90) << " p95=" << iteration_percentile(0.95)
            << " p99=" << iteration_percentile(0.99) << " max=" << iteration_counts.back()
            << " budget_hits=" << budget_hits << '/' << iteration_counts.size() << '\n';

  EXPECT_GE(static_cast<double>(successes) / witnesses.size(), 0.99);
  // The product target is 1.5 ms; this 4x ceiling is intentionally resistant to loaded-host
  // flakes while catching an order-of-magnitude regression.
  EXPECT_LE(p99, 6000.0);
  EXPECT_EQ(budget_hits, 0U);
}

TEST(SolveLatencyTest, TunedIterationBudgetRetainsWholeReachableCorpusDecisionRate) {
  test::Model model;
  NumericBackend backend(model.arm());
  const auto witnesses = test::corpus().allReachable();
  ASSERT_EQ(witnesses.size(), 6700U);

  std::vector<std::uint16_t> iteration_counts;
  iteration_counts.reserve(witnesses.size());
  std::size_t successes = 0;
  std::size_t exhausted = 0;
  for (std::size_t index = 0; index < witnesses.size(); ++index) {
    SolveInput input = test::exactInput(*witnesses[index]);
    input.seed_positions = test::perturbSeed(input.seed_positions, 0.01, index);
    input.position_tolerance = 1.0e-4;
    input.orientation_tolerance = 1.0e-3;
    input.numeric_eps = 1.0e-6;
    input.numeric_max_iterations = kTunedIterationBudget;
    SolveOutput output;
    backend.solve(input, output);
    iteration_counts.push_back(output.iterations);
    successes += output.status == SolveStatus::Success;
    exhausted += output.status == SolveStatus::IterationBudgetExhausted;
  }

  std::sort(iteration_counts.begin(), iteration_counts.end());
  const auto iteration_percentile = [&iteration_counts](const double probability) {
    const std::size_t index = static_cast<std::size_t>(
        std::ceil(probability * static_cast<double>(iteration_counts.size())) - 1.0);
    return iteration_counts.at(std::min(index, iteration_counts.size() - 1));
  };
  std::cout << "numeric whole-corpus iterations at tuned budget " << kTunedIterationBudget
            << ": p50=" << iteration_percentile(0.50) << " p90=" << iteration_percentile(0.90)
            << " p95=" << iteration_percentile(0.95) << " p99=" << iteration_percentile(0.99)
            << " max=" << iteration_counts.back() << " successes=" << successes << '/'
            << witnesses.size() << " exhausted=" << exhausted << '\n';

  EXPECT_GE(static_cast<double>(successes) / witnesses.size(), 0.99);
  EXPECT_EQ(exhausted, 0U);
}

}  // namespace
}  // namespace franka_ik
