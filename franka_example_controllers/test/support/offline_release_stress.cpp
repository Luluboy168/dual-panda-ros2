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

#include "support/offline_release_stress.hpp"

#include <dirent.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <franka_msgs/msg/franka_model.hpp>
#include <franka_msgs/msg/franka_state.hpp>
#include <fstream>
#include <iomanip>
#include <lifecycle_msgs/msg/state.hpp>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

namespace franka_example_controllers::test_support
{
namespace
{

constexpr uint64_t kNanosecondsPerSecond = 1'000'000'000ULL;
constexpr uint64_t kMaximumScheduledCycles = 10'000'000ULL;
constexpr auto kResourceSamplePeriod = std::chrono::seconds(1);
constexpr uint64_t kFormalSteadyDurationSeconds = 3600;
constexpr size_t kRssSteadyWindowSamples = 300;
constexpr uint64_t kRssLateGrowthLimitKib = 8U * 1024U;
constexpr uint64_t kPostCleanupRosDiscoveryTolerance = 1;

uint64_t countDirectoryEntries(const char * path)
{
  DIR * directory = opendir(path);
  if (directory == nullptr) {
    throw std::runtime_error("failed to inspect Linux process resources");
  }
  uint64_t count = 0;
  while (const auto * entry = readdir(directory)) {
    const std::string name(entry->d_name);
    if (name != "." && name != "..") {
      ++count;
    }
  }
  (void)closedir(directory);
  return count;
}

uint64_t residentSetKib()
{
  std::ifstream status("/proc/self/status");
  std::string key;
  while (status >> key) {
    if (key == "VmRSS:") {
      uint64_t value = 0;
      std::string unit;
      status >> value >> unit;
      if (unit != "kB") {
        throw std::runtime_error("unexpected Linux RSS unit");
      }
      return value;
    }
    status.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
  throw std::runtime_error("Linux RSS was unavailable");
}

RosEntitySnapshot rosEntities(rclcpp::Node & node)
{
  RosEntitySnapshot snapshot{};
  snapshot.nodes = node.get_node_names().size();
  const auto topics = node.get_topic_names_and_types();
  snapshot.topics = topics.size();
  for (const auto & topic : topics) {
    snapshot.publishers += node.get_publishers_info_by_topic(topic.first).size();
    snapshot.subscriptions += node.get_subscriptions_info_by_topic(topic.first).size();
  }
  snapshot.services = node.get_service_names_and_types().size();
  return snapshot;
}

ResourceSnapshot resources(rclcpp::Node & node)
{
  ResourceSnapshot snapshot{};
  snapshot.rss_kib = residentSetKib();
  snapshot.file_descriptors = countDirectoryEntries("/proc/self/fd");
  snapshot.threads = countDirectoryEntries("/proc/self/task");
  snapshot.ros = rosEntities(node);
  return snapshot;
}

struct ProcSample
{
  uint64_t rss_kib{0};
  uint64_t file_descriptors{0};
  uint64_t threads{0};
};

class ProcResourceSampler
{
public:
  explicit ProcResourceSampler(uint64_t duration_seconds)
  : duration_seconds_(duration_seconds), samples_(duration_seconds + 2U)
  {
  }

  ~ProcResourceSampler() { stop(); }

  void start()
  {
    worker_ = std::thread([this]() {
      auto next = std::chrono::steady_clock::now();
      while (!stop_.load(std::memory_order_acquire) && sample_count_ < samples_.size()) {
        try {
          auto & sample = samples_[sample_count_];
          sample.rss_kib = residentSetKib();
          sample.file_descriptors = countDirectoryEntries("/proc/self/fd");
          sample.threads = countDirectoryEntries("/proc/self/task");
          ++sample_count_;
        } catch (...) {
          sampling_failed_ = true;
          return;
        }
        next += kResourceSamplePeriod;
        std::this_thread::sleep_until(next);
      }
    });
  }

  void stop() noexcept
  {
    stop_.store(true, std::memory_order_release);
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  [[nodiscard]] ResourceHighWater highWater() const noexcept
  {
    ResourceHighWater result{};
    result.samples = sample_count_;
    result.sampling_failed = sampling_failed_;
    result.rss_sample_period_ms = static_cast<uint64_t>(kResourceSamplePeriod.count()) * 1000U;
    result.rss_steady_gate_applicable =
      !kStressSanitizerInstrumented && duration_seconds_ >= kFormalSteadyDurationSeconds;
    result.rss_steady_window_samples = kRssSteadyWindowSamples;
    result.rss_late_growth_limit_kib = kRssLateGrowthLimitKib;
    for (size_t index = 0; index < sample_count_; ++index) {
      result.rss_kib = std::max(result.rss_kib, samples_[index].rss_kib);
      result.file_descriptors = std::max(result.file_descriptors, samples_[index].file_descriptors);
      result.threads = std::max(result.threads, samples_[index].threads);
    }
    if (sample_count_ >= 2U * kRssSteadyWindowSamples) {
      std::array<uint64_t, kRssSteadyWindowSamples> early{};
      std::array<uint64_t, kRssSteadyWindowSamples> late{};
      for (size_t index = 0; index < kRssSteadyWindowSamples; ++index) {
        early[index] = samples_[index].rss_kib;
        late[index] = samples_[sample_count_ - kRssSteadyWindowSamples + index].rss_kib;
      }
      std::sort(early.begin(), early.end());
      std::sort(late.begin(), late.end());
      result.rss_early_median_kib = early[kRssSteadyWindowSamples / 2U];
      result.rss_late_median_kib = late[kRssSteadyWindowSamples / 2U];
      result.rss_late_growth_kib = static_cast<int64_t>(result.rss_late_median_kib) -
                                   static_cast<int64_t>(result.rss_early_median_kib);
      result.rss_steady_gate_passed =
        sample_count_ >= duration_seconds_ &&
        result.rss_late_growth_kib <= static_cast<int64_t>(kRssLateGrowthLimitKib);
    } else {
      result.rss_steady_gate_passed =
        !kStressSanitizerInstrumented && !result.rss_steady_gate_applicable;
    }
    return result;
  }

private:
  uint64_t duration_seconds_{0};
  std::vector<ProcSample> samples_{};
  size_t sample_count_{0};
  bool sampling_failed_{false};
  std::atomic_bool stop_{false};
  std::thread worker_{};
};

uint64_t quantile(const std::vector<uint64_t> & sorted, double probability)
{
  if (sorted.empty()) {
    return 0;
  }
  const auto rank = static_cast<size_t>(std::ceil(probability * sorted.size()));
  return sorted.at(std::max<size_t>(1, rank) - 1);
}

DurationSummary summarize(std::vector<uint64_t> values)
{
  DurationSummary result{};
  if (values.empty()) {
    return result;
  }
  std::sort(values.begin(), values.end());
  long double sum = 0.0L;
  for (const auto value : values) {
    sum += static_cast<long double>(value);
  }
  result.minimum_ns = values.front();
  result.mean_ns = static_cast<uint64_t>(sum / static_cast<long double>(values.size()));
  result.median_ns = quantile(values, 0.5);
  result.p95_ns = quantile(values, 0.95);
  result.p99_ns = quantile(values, 0.99);
  result.p999_ns = quantile(values, 0.999);
  result.maximum_ns = values.back();
  return result;
}

uint64_t nextRandom(uint64_t & state) noexcept
{
  state ^= state << 13U;
  state ^= state >> 7U;
  state ^= state << 17U;
  return state;
}

std::vector<std::string> activeControllers(OfflineControllerManagerHarness & harness)
{
  std::vector<std::string> result;
  for (const auto & name : harness.loadedControllerNames()) {
    if (harness.lifecycleId(name) == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      result.push_back(name);
    }
  }
  return result;
}

void appendDurationSummary(
  std::ostringstream & output, const char * name, const DurationSummary & summary,
  bool trailing_comma)
{
  output << "\"" << name << "\":{\"min_ns\":" << summary.minimum_ns
         << ",\"mean_ns\":" << summary.mean_ns << ",\"median_ns\":" << summary.median_ns
         << ",\"p95_ns\":" << summary.p95_ns << ",\"p99_ns\":" << summary.p99_ns
         << ",\"p99_9_ns\":" << summary.p999_ns << ",\"max_ns\":" << summary.maximum_ns << "}"
         << (trailing_comma ? "," : "");
}

void appendResourceSnapshot(
  std::ostringstream & output, const char * name, const ResourceSnapshot & snapshot,
  bool trailing_comma)
{
  output << "\"" << name << "\":{\"rss_kib\":" << snapshot.rss_kib
         << ",\"file_descriptors\":" << snapshot.file_descriptors
         << ",\"threads\":" << snapshot.threads << ",\"ros\":{\"nodes\":" << snapshot.ros.nodes
         << ",\"topics\":" << snapshot.ros.topics << ",\"publishers\":" << snapshot.ros.publishers
         << ",\"subscriptions\":" << snapshot.ros.subscriptions
         << ",\"services\":" << snapshot.ros.services << "}}" << (trailing_comma ? "," : "");
}

const char * jsonBool(bool value) noexcept { return value ? "true" : "false"; }

}  // namespace

void validateStressOptions(const StressOptions & options)
{
  if (options.activation_cycles == 0 || options.activation_cycles > 100'000) {
    throw std::invalid_argument("activation cycles must be in [1,100000]");
  }
  if (options.mode_transactions == 0 || options.mode_transactions > 1'000'000) {
    throw std::invalid_argument("mode transactions must be in [1,1000000]");
  }
  if (options.duration_seconds == 0 || options.duration_seconds > 7'200) {
    throw std::invalid_argument("duration seconds must be in [1,7200]");
  }
  if (options.rate_hz != 1'000) {
    throw std::invalid_argument("rate Hz must be exactly 1000");
  }
  if (options.seed == 0) {
    throw std::invalid_argument("seed must be nonzero");
  }
  if (options.duration_seconds > kMaximumScheduledCycles / static_cast<uint64_t>(options.rate_hz)) {
    throw std::invalid_argument("requested timed cycle storage exceeds its fixed bound");
  }
}

StressMetrics runOfflineReleaseStress(const StressOptions & options)
{
  validateStressOptions(options);
  StressMetrics metrics{};
  metrics.requested = options;
  metrics.scheduled_cycles = options.duration_seconds * static_cast<uint64_t>(options.rate_hz);
  metrics.cycles.resize(metrics.scheduled_cycles);
  std::vector<uint64_t> switch_durations(options.mode_transactions, 0);
  auto graph_probe = std::make_shared<rclcpp::Node>("offline_release_resource_probe");
  metrics.pre_harness = resources(*graph_probe);

  auto harness = std::make_unique<OfflineControllerManagerHarness>(kArmCount);
  std::array<std::atomic_uint64_t, 4> broadcaster_message_counts{};
  std::vector<rclcpp::SubscriptionBase::SharedPtr> broadcaster_subscriptions;
  broadcaster_subscriptions.reserve(4);
  for (size_t arm = 1; arm <= kArmCount; ++arm) {
    const auto prefix = "panda" + std::to_string(arm);
    const auto state_index = (arm - 1U) * 2U;
    const auto model_index = state_index + 1U;
    broadcaster_subscriptions.push_back(
      harness->clientNode().create_subscription<franka_msgs::msg::FrankaState>(
        "/" + prefix + "_state_broadcaster/robot_state", rclcpp::SystemDefaultsQoS(),
        [&, state_index](franka_msgs::msg::FrankaState::ConstSharedPtr) {
          broadcaster_message_counts[state_index].fetch_add(1, std::memory_order_relaxed);
        }));
    broadcaster_subscriptions.push_back(
      harness->clientNode().create_subscription<franka_msgs::msg::FrankaModel>(
        "/" + prefix + "_model_broadcaster/robot_model", rclcpp::SystemDefaultsQoS(),
        [&, model_index](franka_msgs::msg::FrankaModel::ConstSharedPtr) {
          broadcaster_message_counts[model_index].fetch_add(1, std::memory_order_relaxed);
        }));
  }
  const auto total_broadcaster_messages = [&]() {
    uint64_t total = 0;
    for (const auto & count : broadcaster_message_counts) {
      total += count.load(std::memory_order_relaxed);
    }
    return total;
  };
  harness->loadReviewedControllers();
  const auto broadcaster_names = harness->loadBroadcasters(kArmCount);
  const auto broadcasters_active = [&]() {
    return std::all_of(broadcaster_names.begin(), broadcaster_names.end(), [&](const auto & name) {
      return harness->lifecycleId(name) == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    });
  };
  if (harness->switchControllers(broadcaster_names, {}) != controller_interface::return_type::OK) {
    throw std::runtime_error("actual Franka broadcaster activation failed");
  }
  if (!harness->pumpUntil([&]() { return total_broadcaster_messages() >= 4U; })) {
    throw std::runtime_error("actual Franka broadcasters did not publish an initial sample");
  }
  const auto initial_broadcaster_messages = total_broadcaster_messages();

  for (uint64_t cycle = 0; cycle < options.activation_cycles; ++cycle) {
    if (
      harness->switchControllers({"velocity_controller"}, {}) ==
      controller_interface::return_type::OK) {
      ++metrics.activation_successes;
    }
    if (
      harness->switchControllers({}, {"velocity_controller"}) ==
      controller_interface::return_type::OK) {
      ++metrics.deactivation_successes;
    }
  }
  if (!harness->pumpUntil([&]() {
        return broadcasters_active() && total_broadcaster_messages() > initial_broadcaster_messages;
      })) {
    throw std::runtime_error(
      "broadcasters did not remain active and publish within the bounded post-cycle observation");
  }
  metrics.broadcaster_messages_after_activation = total_broadcaster_messages();

  const std::array<std::string, 2> reviewed_controllers{
    "velocity_controller", "impedance_controller"};
  int active_controller = -1;
  uint64_t random_state = options.seed;
  for (uint64_t transaction = 0; transaction < options.mode_transactions; ++transaction) {
    const bool request_invalid = nextRandom(random_state) % 5U == 0U;
    std::vector<std::string> activate;
    std::vector<std::string> deactivate;
    if (request_invalid) {
      ++metrics.invalid_transactions_requested;
      if (active_controller >= 0) {
        activate.push_back(reviewed_controllers.at(static_cast<size_t>(active_controller)));
      } else {
        deactivate.push_back(reviewed_controllers.front());
      }
    } else {
      ++metrics.valid_transactions_requested;
      int next_controller =
        static_cast<int>(nextRandom(random_state) % reviewed_controllers.size());
      if (next_controller == active_controller) {
        next_controller = (next_controller + 1) % static_cast<int>(reviewed_controllers.size());
      }
      activate.push_back(reviewed_controllers.at(static_cast<size_t>(next_controller)));
      if (active_controller >= 0) {
        deactivate.push_back(reviewed_controllers.at(static_cast<size_t>(active_controller)));
      }
      active_controller = next_controller;
    }

    const auto before = std::chrono::steady_clock::now();
    const auto result = harness->switchControllers(activate, deactivate);
    const auto after = std::chrono::steady_clock::now();
    switch_durations.at(transaction) = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(after - before).count());
    if (request_invalid) {
      if (result == controller_interface::return_type::ERROR) {
        ++metrics.invalid_transactions_rejected;
      } else {
        ++metrics.invalid_transactions_accepted;
      }
    } else if (result == controller_interface::return_type::OK) {
      ++metrics.valid_transactions_succeeded;
    } else {
      ++metrics.valid_transactions_failed;
      active_controller = -1;
      for (size_t index = 0; index < reviewed_controllers.size(); ++index) {
        if (
          harness->lifecycleId(reviewed_controllers.at(index)) ==
          lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
          active_controller = static_cast<int>(index);
          break;
        }
      }
    }
  }
  metrics.broadcaster_messages_after_transactions = total_broadcaster_messages();
  if (
    !broadcasters_active() || metrics.broadcaster_messages_after_transactions <=
                                metrics.broadcaster_messages_after_activation) {
    throw std::runtime_error("broadcasters did not advance through mode transactions");
  }
  if (
    active_controller < 0 && harness->switchControllers({reviewed_controllers.front()}, {}) !=
                               controller_interface::return_type::OK) {
    throw std::runtime_error("failed to establish timed-run reviewed controller");
  }

  const auto warmup_start = std::chrono::steady_clock::now();
  for (size_t warmup = 0; warmup < 100; ++warmup) {
    std::this_thread::sleep_until(warmup_start + std::chrono::milliseconds(warmup));
    (void)harness->cycle();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  ProcResourceSampler resource_sampler(options.duration_seconds);
  resource_sampler.start();
  metrics.warm = resources(*graph_probe);

  const auto timed_start = std::chrono::steady_clock::now();
  for (uint64_t cycle = 0; cycle < metrics.scheduled_cycles; ++cycle) {
    const auto scheduled_offset =
      (cycle * kNanosecondsPerSecond) / static_cast<uint64_t>(options.rate_hz);
    const auto scheduled_time = timed_start + std::chrono::nanoseconds(scheduled_offset);
    std::this_thread::sleep_until(scheduled_time);
    const auto update_start = std::chrono::steady_clock::now();
    const auto update_result = harness->cycle();
    const auto update_end = std::chrono::steady_clock::now();
    auto & sample = metrics.cycles[cycle];
    sample.cycle_index = cycle;
    sample.scheduled_offset_ns = scheduled_offset;
    sample.start_lateness_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(update_start - scheduled_time).count();
    sample.update_duration_ns = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(update_end - update_start).count());
    if (update_result == controller_interface::return_type::OK) {
      ++metrics.completed_cycles;
    }
    const auto deadline =
      scheduled_time + std::chrono::nanoseconds(kNanosecondsPerSecond / options.rate_hz);
    if (update_end > deadline) {
      ++metrics.deadline_misses;
    }
  }
  const auto requested_end = timed_start + std::chrono::seconds(options.duration_seconds);
  std::this_thread::sleep_until(requested_end);
  const auto timed_end = std::chrono::steady_clock::now();
  metrics.elapsed_ns = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(timed_end - timed_start).count());
  // This deliberately broad functional gate detects a loop that is no longer
  // operating at its requested cadence under an ordinary loaded host. It also
  // applies to the one-hour run; stricter formal timing interpretation is separate.
  metrics.functional_deadline_gate_applicable = true;
  metrics.functional_deadline_miss_limit = (metrics.scheduled_cycles + 3U) / 4U;
  metrics.functional_deadline_gate_passed =
    !metrics.functional_deadline_gate_applicable ||
    metrics.deadline_misses <= metrics.functional_deadline_miss_limit;
  metrics.post_run = resources(*graph_probe);
  resource_sampler.stop();
  metrics.resource_high_water = resource_sampler.highWater();
  metrics.broadcaster_messages_after_timed_run = total_broadcaster_messages();
  if (
    !broadcasters_active() || metrics.broadcaster_messages_after_timed_run <=
                                metrics.broadcaster_messages_after_transactions) {
    throw std::runtime_error("broadcasters did not advance through the timed phase");
  }

  std::vector<uint64_t> update_durations(metrics.cycles.size(), 0);
  for (size_t index = 0; index < metrics.cycles.size(); ++index) {
    update_durations[index] = metrics.cycles[index].update_duration_ns;
  }
  metrics.update_timing = summarize(std::move(update_durations));
  metrics.switch_timing = summarize(std::move(switch_durations));

  harness->deactivateAndUnloadAll();
  metrics.controllers_unloaded = 3U + broadcaster_names.size();
  metrics.final_active_controllers = activeControllers(*harness).size();
  metrics.final_global_fault = harness->productionHardware().globalFaultDiagnostic().latched();
  // Stop the injected hardware before sampling its final single-threaded
  // counters. stop() drains the bounded synthetic command queue.
  harness->deactivateHardware();

  // The synthetic backend's detailed accepted-count accessors are deliberately single-threaded.
  // Sample them only after the control/switch work has stopped and all controllers are unloaded.
  const auto backend_count = harness->armCount();
  metrics.backends.reserve(backend_count);
  for (size_t arm = 1; arm <= backend_count; ++arm) {
    const auto arm_id = "panda" + std::to_string(arm);
    const auto backend = harness->backend(arm_id);
    const auto diagnostics = backend->diagnostics();
    BackendMetric metric{};
    metric.arm_id = arm_id;
    metric.accepted_states = diagnostics.accepted_state_samples;
    metric.dropped_states = diagnostics.dropped_state_samples;
    metric.accepted_commands = backend->acceptedCommandCount();
    metric.rejected_commands = diagnostics.rejected_command_samples;
    metric.command_queue_depth = backend->commandQueueDepth();
    metric.command_queue_capacity = backend->commandQueueCapacity();
    metric.accepted_mode_requests = backend->acceptedModeRequestCount();
    metric.rejected_mode_requests = backend->rejectedModeRequestCount();
    metric.accepted_non_none_mode_requests = backend->acceptedNonNoneModeRequestCount();
    metric.rejected_non_none_mode_requests = backend->rejectedNonNoneModeRequestCount();
    metric.accepted_safe_snapshots = backend->acceptedSafeSnapshotCount();
    metric.accepted_non_safe_snapshots = backend->acceptedUnsafeSnapshotCount();
    metrics.accepted_mode_requests += metric.accepted_mode_requests;
    metrics.rejected_mode_requests += metric.rejected_mode_requests;
    metrics.accepted_non_none_mode_requests += metric.accepted_non_none_mode_requests;
    metrics.rejected_non_none_mode_requests += metric.rejected_non_none_mode_requests;
    metrics.accepted_safe_snapshots += metric.accepted_safe_snapshots;
    metrics.accepted_non_safe_snapshots += metric.accepted_non_safe_snapshots;
    metric.state_queue_saturated = diagnostics.state_queue_saturated;
    metric.command_queue_saturated = diagnostics.command_queue_saturated;
    metric.recovery_attempts = diagnostics.recovery_attempts;
    metric.recovery_successes = diagnostics.recovery_successes;
    metric.recovery_failures = diagnostics.recovery_failures;
    metric.requested_mode = static_cast<uint8_t>(diagnostics.requested_mode);
    metric.active_mode = static_cast<uint8_t>(diagnostics.active_mode);
    metric.faulted = backend->hasFault();
    metrics.backends.push_back(std::move(metric));
  }

  broadcaster_subscriptions.clear();
  harness->shutdown();
  metrics.cleanup = harness->cleanupSnapshot();
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  metrics.post_cleanup = resources(*graph_probe);
  metrics.cold_retained_rss_kib = static_cast<int64_t>(metrics.post_cleanup.rss_kib) -
                                  static_cast<int64_t>(metrics.pre_harness.rss_kib);

  const bool backend_final_safe = std::all_of(
    metrics.backends.begin(), metrics.backends.end(), [](const BackendMetric & backend) {
      return backend.requested_mode == static_cast<uint8_t>(franka_hardware::ControlMode::None) &&
             backend.active_mode == static_cast<uint8_t>(franka_hardware::ControlMode::None) &&
             !backend.faulted && !backend.state_queue_saturated &&
             !backend.command_queue_saturated && backend.accepted_states > 0 &&
             backend.dropped_states == 0 && backend.accepted_commands > 0 &&
             backend.rejected_commands == 0 && backend.accepted_mode_requests > 0 &&
             backend.rejected_mode_requests == 0 && backend.accepted_non_none_mode_requests > 0 &&
             backend.rejected_non_none_mode_requests == 0 && backend.accepted_safe_snapshots > 0 &&
             backend.accepted_non_safe_snapshots > 0 && backend.recovery_failures == 0 &&
             backend.command_queue_depth == 0 &&
             backend.accepted_safe_snapshots + backend.accepted_non_safe_snapshots ==
               backend.accepted_commands;
    });
  const bool rss_thresholds_satisfied =
    metrics.post_run.rss_kib <= metrics.warm.rss_kib + 32U * 1024U &&
    metrics.cold_retained_rss_kib <= static_cast<int64_t>(metrics.cold_retained_rss_limit_kib) &&
    (!metrics.resource_high_water.rss_steady_gate_applicable ||
     metrics.resource_high_water.rss_steady_gate_passed) &&
    metrics.resource_high_water.rss_kib <= metrics.warm.rss_kib + 32U * 1024U;
  metrics.rss_threshold_passed = metrics.rss_threshold_applicable && rss_thresholds_satisfied;
  metrics.success =
    metrics.activation_successes == options.activation_cycles &&
    metrics.deactivation_successes == options.activation_cycles &&
    metrics.valid_transactions_succeeded == metrics.valid_transactions_requested &&
    metrics.valid_transactions_failed == 0 &&
    metrics.invalid_transactions_rejected == metrics.invalid_transactions_requested &&
    metrics.invalid_transactions_accepted == 0 && metrics.accepted_mode_requests > 0 &&
    metrics.rejected_mode_requests == 0 && metrics.accepted_non_none_mode_requests > 0 &&
    metrics.rejected_non_none_mode_requests == 0 && metrics.accepted_safe_snapshots > 0 &&
    metrics.accepted_non_safe_snapshots > 0 &&
    metrics.accepted_safe_snapshots + metrics.accepted_non_safe_snapshots ==
      std::accumulate(
        metrics.backends.begin(), metrics.backends.end(), uint64_t{0},
        [](uint64_t total, const BackendMetric & backend) {
          return total + backend.accepted_commands;
        }) &&
    metrics.completed_cycles == metrics.scheduled_cycles &&
    metrics.functional_deadline_gate_passed &&
    metrics.elapsed_ns >= options.duration_seconds * kNanosecondsPerSecond &&
    metrics.final_active_controllers == 0 && !metrics.final_global_fault &&
    metrics.controllers_unloaded == 7 && metrics.cleanup.constructed == kArmCount &&
    metrics.cleanup.stopped == kArmCount && metrics.cleanup.destroyed == kArmCount &&
    metrics.post_run.threads == metrics.warm.threads &&
    metrics.post_run.file_descriptors <= metrics.warm.file_descriptors + 2U &&
    metrics.post_cleanup.threads <= metrics.pre_harness.threads + 1U &&
    metrics.post_cleanup.file_descriptors <= metrics.pre_harness.file_descriptors + 2U &&
    (!metrics.rss_threshold_applicable || metrics.rss_threshold_passed) &&
    metrics.resource_high_water.samples > 0U && !metrics.resource_high_water.sampling_failed &&
    metrics.resource_high_water.file_descriptors <= metrics.warm.file_descriptors + 4U &&
    metrics.resource_high_water.threads <= metrics.warm.threads &&
    metrics.post_run.ros.nodes <= metrics.warm.ros.nodes + 2U &&
    metrics.post_run.ros.topics <= metrics.warm.ros.topics + 2U &&
    metrics.post_run.ros.publishers <= metrics.warm.ros.publishers + 2U &&
    metrics.post_run.ros.subscriptions <= metrics.warm.ros.subscriptions + 2U &&
    metrics.post_run.ros.services <= metrics.warm.ros.services + 2U &&
    metrics.post_cleanup.ros.publishers <=
      metrics.pre_harness.ros.publishers + kPostCleanupRosDiscoveryTolerance &&
    metrics.post_cleanup.ros.subscriptions <=
      metrics.pre_harness.ros.subscriptions + kPostCleanupRosDiscoveryTolerance &&
    metrics.post_cleanup.ros.services <=
      metrics.pre_harness.ros.services + kPostCleanupRosDiscoveryTolerance &&
    metrics.post_cleanup.ros.nodes <=
      metrics.pre_harness.ros.nodes + kPostCleanupRosDiscoveryTolerance &&
    metrics.post_cleanup.ros.topics <=
      metrics.pre_harness.ros.topics + kPostCleanupRosDiscoveryTolerance &&
    backend_final_safe;
  return metrics;
}

std::string stressMetricsJson(const StressMetrics & metrics)
{
  std::ostringstream output;
  output << "{\"schema_version\":" << StressMetrics::kSchemaVersion
         << ",\"success\":" << jsonBool(metrics.success) << ",\"seed\":" << metrics.requested.seed
         << ",\"activation\":{\"requested_cycles\":" << metrics.requested.activation_cycles
         << ",\"activations_succeeded\":" << metrics.activation_successes
         << ",\"deactivations_succeeded\":" << metrics.deactivation_successes
         << "},\"transactions\":{\"requested\":" << metrics.requested.mode_transactions
         << ",\"valid_requested\":" << metrics.valid_transactions_requested
         << ",\"valid_succeeded\":" << metrics.valid_transactions_succeeded
         << ",\"valid_failed\":" << metrics.valid_transactions_failed
         << ",\"invalid_requested\":" << metrics.invalid_transactions_requested
         << ",\"invalid_rejected\":" << metrics.invalid_transactions_rejected
         << ",\"invalid_accepted\":" << metrics.invalid_transactions_accepted
         << "},\"mode_requests\":{\"accepted\":" << metrics.accepted_mode_requests
         << ",\"rejected\":" << metrics.rejected_mode_requests
         << ",\"accepted_non_none\":" << metrics.accepted_non_none_mode_requests
         << ",\"rejected_non_none\":" << metrics.rejected_non_none_mode_requests
         << "},\"command_snapshots\":{\"safe\":" << metrics.accepted_safe_snapshots
         << ",\"non_safe\":" << metrics.accepted_non_safe_snapshots
         << "},\"broadcasters\":{\"after_activation\":"
         << metrics.broadcaster_messages_after_activation
         << ",\"after_transactions\":" << metrics.broadcaster_messages_after_transactions
         << ",\"after_timed_run\":" << metrics.broadcaster_messages_after_timed_run
         << "},\"timed_run\":{\"requested_seconds\":" << metrics.requested.duration_seconds
         << ",\"rate_hz\":" << metrics.requested.rate_hz
         << ",\"scheduled_cycles\":" << metrics.scheduled_cycles
         << ",\"completed_cycles\":" << metrics.completed_cycles
         << ",\"deadline_misses\":" << metrics.deadline_misses
         << ",\"elapsed_ns\":" << metrics.elapsed_ns
         << ",\"functional_deadline_miss_limit\":" << metrics.functional_deadline_miss_limit
         << ",\"functional_deadline_gate_applicable\":"
         << jsonBool(metrics.functional_deadline_gate_applicable)
         << ",\"functional_deadline_gate_passed\":"
         << jsonBool(metrics.functional_deadline_gate_passed) << "},\"timing\":{";
  appendDurationSummary(output, "update", metrics.update_timing, true);
  appendDurationSummary(output, "switch", metrics.switch_timing, false);
  output << "},\"resources\":{";
  appendResourceSnapshot(output, "pre_harness", metrics.pre_harness, true);
  appendResourceSnapshot(output, "warm", metrics.warm, true);
  appendResourceSnapshot(output, "post_run", metrics.post_run, true);
  appendResourceSnapshot(output, "post_cleanup", metrics.post_cleanup, false);
  output << ",\"sanitizer_instrumented\":" << jsonBool(metrics.sanitizer_instrumented)
         << ",\"rss_threshold_applicable\":" << jsonBool(metrics.rss_threshold_applicable)
         << ",\"rss_threshold_passed\":" << jsonBool(metrics.rss_threshold_passed)
         << ",\"periodic_high_water\":{\"samples\":" << metrics.resource_high_water.samples
         << ",\"rss_kib\":" << metrics.resource_high_water.rss_kib
         << ",\"file_descriptors\":" << metrics.resource_high_water.file_descriptors
         << ",\"threads\":" << metrics.resource_high_water.threads
         << ",\"sampling_failed\":" << jsonBool(metrics.resource_high_water.sampling_failed)
         << ",\"rss_sample_period_ms\":" << metrics.resource_high_water.rss_sample_period_ms
         << ",\"rss_steady_gate_applicable\":"
         << jsonBool(metrics.resource_high_water.rss_steady_gate_applicable)
         << ",\"rss_steady_gate_passed\":"
         << jsonBool(metrics.resource_high_water.rss_steady_gate_passed)
         << ",\"rss_steady_window_samples\":"
         << metrics.resource_high_water.rss_steady_window_samples
         << ",\"rss_early_median_kib\":" << metrics.resource_high_water.rss_early_median_kib
         << ",\"rss_late_median_kib\":" << metrics.resource_high_water.rss_late_median_kib
         << ",\"rss_late_growth_kib\":" << metrics.resource_high_water.rss_late_growth_kib
         << ",\"rss_late_growth_limit_kib\":"
         << metrics.resource_high_water.rss_late_growth_limit_kib
         << "},\"post_cleanup_ros_discovery_tolerance\":" << kPostCleanupRosDiscoveryTolerance
         << ",\"cold_retained_rss_kib\":" << metrics.cold_retained_rss_kib
         << ",\"cold_retained_rss_limit_kib\":" << metrics.cold_retained_rss_limit_kib
         << "},\"backends\":[";
  for (size_t index = 0; index < metrics.backends.size(); ++index) {
    const auto & backend = metrics.backends[index];
    output << "{\"arm_id\":\"" << backend.arm_id
           << "\",\"accepted_states\":" << backend.accepted_states
           << ",\"dropped_states\":" << backend.dropped_states
           << ",\"accepted_commands\":" << backend.accepted_commands
           << ",\"rejected_commands\":" << backend.rejected_commands
           << ",\"command_queue_depth\":" << backend.command_queue_depth
           << ",\"command_queue_capacity\":" << backend.command_queue_capacity
           << ",\"accepted_mode_requests\":" << backend.accepted_mode_requests
           << ",\"rejected_mode_requests\":" << backend.rejected_mode_requests
           << ",\"accepted_non_none_mode_requests\":" << backend.accepted_non_none_mode_requests
           << ",\"rejected_non_none_mode_requests\":" << backend.rejected_non_none_mode_requests
           << ",\"accepted_safe_snapshots\":" << backend.accepted_safe_snapshots
           << ",\"accepted_non_safe_snapshots\":" << backend.accepted_non_safe_snapshots
           << ",\"state_queue_saturated\":" << jsonBool(backend.state_queue_saturated)
           << ",\"command_queue_saturated\":" << jsonBool(backend.command_queue_saturated)
           << ",\"recovery_attempts\":" << backend.recovery_attempts
           << ",\"recovery_successes\":" << backend.recovery_successes
           << ",\"recovery_failures\":" << backend.recovery_failures
           << ",\"requested_mode\":" << static_cast<unsigned int>(backend.requested_mode)
           << ",\"active_mode\":" << static_cast<unsigned int>(backend.active_mode)
           << ",\"faulted\":" << jsonBool(backend.faulted) << "}"
           << (index + 1 == metrics.backends.size() ? "" : ",");
  }
  output << "],\"final\":{\"active_controllers\":" << metrics.final_active_controllers
         << ",\"global_fault\":" << jsonBool(metrics.final_global_fault)
         << "},\"cleanup\":{\"controllers_unloaded\":" << metrics.controllers_unloaded
         << ",\"backends_constructed\":" << metrics.cleanup.constructed
         << ",\"backends_stopped\":" << metrics.cleanup.stopped
         << ",\"backends_destroyed\":" << metrics.cleanup.destroyed << "}}\n";
  return output.str();
}

std::string stressCyclesCsv(const StressMetrics & metrics)
{
  std::ostringstream output;
  output << "cycle_index,scheduled_offset_ns,start_lateness_ns,update_duration_ns\n";
  for (const auto & cycle : metrics.cycles) {
    output << cycle.cycle_index << ',' << cycle.scheduled_offset_ns << ','
           << cycle.start_lateness_ns << ',' << cycle.update_duration_ns << '\n';
  }
  return output.str();
}

}  // namespace franka_example_controllers::test_support
