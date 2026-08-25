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

#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>

#include "support/offline_release_stress.hpp"

namespace
{

using franka_example_controllers::test_support::StressOptions;

constexpr char kUsage[] =
  "usage: franka_offline_release_stress --activation-cycles N --mode-transactions N "
  "--duration-seconds N --rate-hz N --seed N --metrics-json /abs/file "
  "--cycles-csv /abs/file";

uint64_t parseUnsigned(const std::string & text)
{
  if (text.empty()) {
    throw std::invalid_argument("empty unsigned integer");
  }
  uint64_t value = 0;
  for (const char character : text) {
    if (character < '0' || character > '9') {
      throw std::invalid_argument("invalid unsigned integer");
    }
    const auto digit = static_cast<uint64_t>(character - '0');
    if (value > (std::numeric_limits<uint64_t>::max() - digit) / 10U) {
      throw std::invalid_argument("unsigned integer overflow");
    }
    value = value * 10U + digit;
  }
  return value;
}

struct ParsedArguments
{
  StressOptions options{};
  std::string metrics_json;
  std::string cycles_csv;
};

ParsedArguments parseArguments(int argc, char ** argv)
{
  if (argc != 15) {
    throw std::invalid_argument("exactly seven option-value pairs are required");
  }
  std::map<std::string, std::string> values;
  for (int index = 1; index < argc; index += 2) {
    const std::string option(argv[index]);
    const std::string value(argv[index + 1]);
    if (
      option != "--activation-cycles" && option != "--mode-transactions" &&
      option != "--duration-seconds" && option != "--rate-hz" && option != "--seed" &&
      option != "--metrics-json" && option != "--cycles-csv") {
      throw std::invalid_argument("unknown option");
    }
    if (!values.emplace(option, value).second) {
      throw std::invalid_argument("duplicate option");
    }
  }
  if (
    values.size() != 7U || values.count("--activation-cycles") == 0U ||
    values.count("--mode-transactions") == 0U || values.count("--duration-seconds") == 0U ||
    values.count("--rate-hz") == 0U || values.count("--seed") == 0U ||
    values.count("--metrics-json") == 0U || values.count("--cycles-csv") == 0U) {
    throw std::invalid_argument("required option missing");
  }

  ParsedArguments result{};
  result.options.activation_cycles = parseUnsigned(values.at("--activation-cycles"));
  result.options.mode_transactions = parseUnsigned(values.at("--mode-transactions"));
  result.options.duration_seconds = parseUnsigned(values.at("--duration-seconds"));
  const auto rate = parseUnsigned(values.at("--rate-hz"));
  if (rate > std::numeric_limits<uint32_t>::max()) {
    throw std::invalid_argument("rate overflow");
  }
  result.options.rate_hz = static_cast<uint32_t>(rate);
  result.options.seed = parseUnsigned(values.at("--seed"));
  result.metrics_json = values.at("--metrics-json");
  result.cycles_csv = values.at("--cycles-csv");
  if (
    result.metrics_json.empty() || result.cycles_csv.empty() ||
    result.metrics_json.front() != '/' || result.cycles_csv.front() != '/' ||
    result.metrics_json == result.cycles_csv ||
    result.metrics_json.find('\n') != std::string::npos ||
    result.cycles_csv.find('\n') != std::string::npos) {
    throw std::invalid_argument("output paths must be distinct absolute paths");
  }
  franka_example_controllers::test_support::validateStressOptions(result.options);
  return result;
}

void writeFile(const std::string & path, const std::string & contents)
{
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  if (!output || !(output << contents)) {
    throw std::runtime_error("stress artifact write failed");
  }
  output.close();
  if (!output) {
    throw std::runtime_error("stress artifact close failed");
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << kUsage << '\n';
    return 0;
  }
  ParsedArguments arguments{};
  try {
    arguments = parseArguments(argc, argv);
  } catch (const std::exception &) {
    std::cerr << "franka_offline_release_stress: invalid arguments\n";
    return 2;
  }

  try {
    rclcpp::init(argc, argv);
    const auto metrics =
      franka_example_controllers::test_support::runOfflineReleaseStress(arguments.options);
    const auto metrics_json = franka_example_controllers::test_support::stressMetricsJson(metrics);
    const auto cycles_csv = franka_example_controllers::test_support::stressCyclesCsv(metrics);
    writeFile(arguments.metrics_json, metrics_json);
    writeFile(arguments.cycles_csv, cycles_csv);
    rclcpp::shutdown();
    if (!metrics.success) {
      std::cerr << "franka_offline_release_stress: measured invariant failed\n";
      return 1;
    }
    std::cout << "franka_offline_release_stress: PASS\n";
    return 0;
  } catch (const std::exception &) {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    std::cerr << "franka_offline_release_stress: execution failed\n";
    return 1;
  }
}
