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

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "support/offline_controller_manager_harness.hpp"

namespace franka_example_controllers::test_support
{

#if defined(__SANITIZE_ADDRESS__)
inline constexpr bool kStressSanitizerInstrumented = true;
#elif defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(undefined_behavior_sanitizer)
inline constexpr bool kStressSanitizerInstrumented = true;
#else
inline constexpr bool kStressSanitizerInstrumented = false;
#endif
#else
inline constexpr bool kStressSanitizerInstrumented = false;
#endif

struct StressOptions
{
  uint64_t activation_cycles{0};
  uint64_t mode_transactions{0};
  uint64_t duration_seconds{0};
  uint32_t rate_hz{0};
  uint64_t seed{0};
};

struct CycleMetric
{
  uint64_t cycle_index{0};
  uint64_t scheduled_offset_ns{0};
  int64_t start_lateness_ns{0};
  uint64_t update_duration_ns{0};
};

struct DurationSummary
{
  uint64_t minimum_ns{0};
  uint64_t mean_ns{0};
  uint64_t median_ns{0};
  uint64_t p95_ns{0};
  uint64_t p99_ns{0};
  uint64_t p999_ns{0};
  uint64_t maximum_ns{0};
};

struct ResourceHighWater
{
  uint64_t samples{0};
  uint64_t rss_kib{0};
  uint64_t file_descriptors{0};
  uint64_t threads{0};
  bool sampling_failed{false};
  uint64_t rss_sample_period_ms{0};
  bool rss_steady_gate_applicable{false};
  bool rss_steady_gate_passed{false};
  uint64_t rss_steady_window_samples{0};
  uint64_t rss_early_median_kib{0};
  uint64_t rss_late_median_kib{0};
  int64_t rss_late_growth_kib{0};
  uint64_t rss_late_growth_limit_kib{0};
};

struct RosEntitySnapshot
{
  uint64_t nodes{0};
  uint64_t topics{0};
  uint64_t publishers{0};
  uint64_t subscriptions{0};
  uint64_t services{0};
};

struct ResourceSnapshot
{
  uint64_t rss_kib{0};
  uint64_t file_descriptors{0};
  uint64_t threads{0};
  RosEntitySnapshot ros{};
};

struct BackendMetric
{
  std::string arm_id;
  uint64_t accepted_states{0};
  uint64_t dropped_states{0};
  uint64_t accepted_commands{0};
  uint64_t rejected_commands{0};
  uint64_t command_queue_depth{0};
  uint64_t command_queue_capacity{0};
  uint64_t accepted_mode_requests{0};
  uint64_t rejected_mode_requests{0};
  uint64_t accepted_non_none_mode_requests{0};
  uint64_t rejected_non_none_mode_requests{0};
  uint64_t accepted_safe_snapshots{0};
  uint64_t accepted_non_safe_snapshots{0};
  bool state_queue_saturated{false};
  bool command_queue_saturated{false};
  uint64_t recovery_attempts{0};
  uint64_t recovery_successes{0};
  uint64_t recovery_failures{0};
  uint8_t requested_mode{0};
  uint8_t active_mode{0};
  bool faulted{false};
};

struct StressMetrics
{
  static constexpr uint64_t kSchemaVersion = 2;

  StressOptions requested{};
  uint64_t activation_successes{0};
  uint64_t deactivation_successes{0};
  uint64_t valid_transactions_requested{0};
  uint64_t valid_transactions_succeeded{0};
  uint64_t valid_transactions_failed{0};
  uint64_t invalid_transactions_requested{0};
  uint64_t invalid_transactions_rejected{0};
  uint64_t invalid_transactions_accepted{0};
  uint64_t accepted_mode_requests{0};
  uint64_t rejected_mode_requests{0};
  uint64_t accepted_non_none_mode_requests{0};
  uint64_t rejected_non_none_mode_requests{0};
  uint64_t accepted_safe_snapshots{0};
  uint64_t accepted_non_safe_snapshots{0};
  uint64_t scheduled_cycles{0};
  uint64_t completed_cycles{0};
  uint64_t deadline_misses{0};
  uint64_t functional_deadline_miss_limit{0};
  bool functional_deadline_gate_applicable{false};
  bool functional_deadline_gate_passed{false};
  uint64_t elapsed_ns{0};
  uint64_t broadcaster_messages_after_activation{0};
  uint64_t broadcaster_messages_after_transactions{0};
  uint64_t broadcaster_messages_after_timed_run{0};
  DurationSummary update_timing{};
  DurationSummary switch_timing{};
  ResourceSnapshot pre_harness{};
  ResourceSnapshot warm{};
  ResourceSnapshot post_run{};
  ResourceSnapshot post_cleanup{};
  ResourceHighWater resource_high_water{};
  bool sanitizer_instrumented{kStressSanitizerInstrumented};
  bool rss_threshold_applicable{!kStressSanitizerInstrumented};
  bool rss_threshold_passed{false};
  int64_t cold_retained_rss_kib{0};
  uint64_t cold_retained_rss_limit_kib{64U * 1024U};
  std::vector<BackendMetric> backends{};
  uint64_t final_active_controllers{0};
  bool final_global_fault{false};
  uint64_t controllers_unloaded{0};
  BackendCleanupSnapshot cleanup{};
  bool success{false};
  std::vector<CycleMetric> cycles{};
};

void validateStressOptions(const StressOptions & options);
[[nodiscard]] StressMetrics runOfflineReleaseStress(const StressOptions & options);
[[nodiscard]] std::string stressMetricsJson(const StressMetrics & metrics);
[[nodiscard]] std::string stressCyclesCsv(const StressMetrics & metrics);

}  // namespace franka_example_controllers::test_support
