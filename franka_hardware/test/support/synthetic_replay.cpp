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

#include "support/synthetic_replay.hpp"

#include <charconv>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <system_error>

namespace franka_hardware::test_support {
namespace {

constexpr size_t kMaximumNumericTokenBytes = 64;

constexpr char asciiLower(char character) noexcept {
  return character >= 'A' && character <= 'Z' ? static_cast<char>(character - 'A' + 'a')
                                              : character;
}

bool containsCaseInsensitive(std::string_view content, std::string_view token) noexcept {
  if (token.empty() || token.size() > content.size()) {
    return false;
  }
  for (size_t offset = 0; offset + token.size() <= content.size(); ++offset) {
    size_t index = 0;
    for (; index < token.size(); ++index) {
      if (asciiLower(content[offset + index]) != asciiLower(token[index])) {
        break;
      }
    }
    if (index == token.size()) {
      return true;
    }
  }
  return false;
}

bool containsForbiddenToken(std::string_view content) noexcept {
  for (const char character : content) {
    if (character == '#' || character == '\"' || character == '\'' || character == '/' ||
        character == '\\' || character == '\r' || character == ':') {
      return true;
    }
  }
  for (const auto token :
       {std::string_view{"address"}, std::string_view{"hostname"}, std::string_view{"robot_ip"},
        std::string_view{"path"}, std::string_view{"://"}}) {
    if (containsCaseInsensitive(content, token)) {
      return true;
    }
  }
  size_t token_offset = 0;
  while (token_offset < content.size()) {
    const size_t token_end = content.find_first_of(",\n", token_offset);
    const auto token = content.substr(token_offset, token_end == std::string_view::npos
                                                        ? std::string_view::npos
                                                        : token_end - token_offset);
    size_t dots = 0;
    size_t digits = 0;
    bool address_candidate = !token.empty();
    for (const char character : token) {
      if (character >= '0' && character <= '9') {
        ++digits;
      } else if (character == '.' && digits != 0) {
        ++dots;
        digits = 0;
      } else {
        address_candidate = false;
        break;
      }
    }
    if (address_candidate && dots == 3 && digits != 0) {
      return true;
    }
    if (token_end == std::string_view::npos) {
      break;
    }
    token_offset = token_end + 1;
  }
  return false;
}

SyntheticReplayParseResult fail(SyntheticReplayError error, size_t line) noexcept {
  SyntheticReplayParseResult result;
  result.error = error;
  result.line = line;
  return result;
}

bool splitFields(std::string_view line,
                 std::array<std::string_view, kSyntheticReplayFieldCount + 1>& fields,
                 size_t& field_count) noexcept {
  field_count = 0;
  size_t offset = 0;
  for (;;) {
    const size_t comma = line.find(',', offset);
    if (field_count == fields.size()) {
      return false;
    }
    fields[field_count++] = line.substr(
        offset, comma == std::string_view::npos ? std::string_view::npos : comma - offset);
    if (comma == std::string_view::npos) {
      return true;
    }
    offset = comma + 1;
  }
}

bool parseTimestamp(std::string_view token, uint64_t& value) noexcept {
  if (token.empty() || token.size() > 19 || token.front() == '+' || token.front() == '-') {
    return false;
  }
  int64_t parsed = 0;
  const auto result = std::from_chars(token.data(), token.data() + token.size(), parsed);
  if (result.ec != std::errc{} || result.ptr != token.data() + token.size() || parsed <= 0) {
    return false;
  }
  value = static_cast<uint64_t>(parsed);
  return true;
}

bool parseFiniteDouble(std::string_view token, double& value) noexcept {
  if (token.empty() || token.size() > kMaximumNumericTokenBytes) {
    return false;
  }
  const auto result =
      std::from_chars(token.data(), token.data() + token.size(), value, std::chars_format::general);
  return result.ec == std::errc{} && result.ptr == token.data() + token.size() &&
         std::isfinite(value);
}

void populateState(franka::RobotState& state,
                   uint8_t arm_marker,
                   uint64_t steady_ns,
                   const std::array<std::string_view, kSyntheticReplayFieldCount + 1>& fields,
                   size_t first_value) noexcept {
  state = makeSyntheticRobotState(arm_marker, steady_ns / 1'000'000U);
  for (size_t index = 0; index < 7; ++index) {
    (void)parseFiniteDouble(fields[first_value + index], state.q[index]);
    (void)parseFiniteDouble(fields[first_value + 7 + index], state.dq[index]);
    (void)parseFiniteDouble(fields[first_value + 14 + index], state.tau_J[index]);
  }
  state.q_d = state.q;
  state.dq_d = state.dq;
  state.theta = state.q;
  state.dtheta = state.dq;
}

}  // namespace

SyntheticReplayParseResult parseSyntheticReplay(std::string_view content) noexcept {
  if (content.size() > kSyntheticReplayMaximumBytes) {
    return fail(SyntheticReplayError::InputTooLarge, 0);
  }
  if (containsForbiddenToken(content)) {
    return fail(SyntheticReplayError::ForbiddenToken, 0);
  }

  SyntheticReplayParseResult result;
  size_t line_number = 1;
  size_t offset = 0;
  uint64_t previous_timestamp = 0;
  bool header_seen = false;
  bool arms_seen = false;
  while (offset < content.size()) {
    const size_t newline = content.find('\n', offset);
    const auto line = content.substr(
        offset, newline == std::string_view::npos ? std::string_view::npos : newline - offset);
    if (line.empty()) {
      return fail(SyntheticReplayError::BlankLine, line_number);
    }
    if (line_number == 1) {
      if (line != kSyntheticReplayHeader) {
        return fail(SyntheticReplayError::InvalidHeader, line_number);
      }
      header_seen = true;
    } else if (line_number == 2) {
      if (line != kSyntheticReplayArms) {
        return fail(SyntheticReplayError::InvalidArms, line_number);
      }
      arms_seen = true;
    } else {
      if (result.replay.size == kSyntheticReplayMaximumFrames) {
        return fail(SyntheticReplayError::FrameCount, line_number);
      }
      std::array<std::string_view, kSyntheticReplayFieldCount + 1> fields{};
      size_t field_count = 0;
      if (!splitFields(line, fields, field_count) || field_count != kSyntheticReplayFieldCount) {
        return fail(SyntheticReplayError::FieldCount, line_number);
      }
      if (fields[0] != "frame") {
        return fail(SyntheticReplayError::UnknownRow, line_number);
      }
      uint64_t timestamp = 0;
      if (!parseTimestamp(fields[1], timestamp)) {
        return fail(SyntheticReplayError::InvalidTimestamp, line_number);
      }
      if (timestamp <= previous_timestamp) {
        return fail(SyntheticReplayError::NonIncreasingTimestamp, line_number);
      }
      for (size_t index = 2; index < kSyntheticReplayFieldCount; ++index) {
        double value = 0.0;
        if (!parseFiniteDouble(fields[index], value)) {
          return fail(SyntheticReplayError::NonFiniteValue, line_number);
        }
      }

      auto& frame = result.replay.frames[result.replay.size++];
      frame.steady_ns = timestamp;
      populateState(frame.states[0], 1, timestamp, fields, 2);
      populateState(frame.states[1], 2, timestamp, fields, 23);
      previous_timestamp = timestamp;
    }

    if (newline == std::string_view::npos) {
      offset = content.size();
    } else {
      offset = newline + 1;
    }
    ++line_number;
  }

  if (!header_seen) {
    return fail(SyntheticReplayError::InvalidHeader, line_number);
  }
  if (!arms_seen) {
    return fail(SyntheticReplayError::InvalidArms, line_number);
  }
  if (result.replay.size == 0) {
    return fail(SyntheticReplayError::FrameCount, line_number);
  }
  return result;
}

SyntheticFrankaArmBackendConfig replayBackendConfig(const SyntheticReplay& replay,
                                                    size_t arm_index) {
  if (replay.size == 0 || replay.size > kSyntheticReplayMaximumFrames ||
      arm_index >= kSyntheticReplayArmCount) {
    throw std::invalid_argument("synthetic replay backend selection is invalid");
  }
  SyntheticFrankaArmBackendConfig config =
      SyntheticFrankaArmBackendConfig::forArm(static_cast<uint8_t>(arm_index + 1));
  config.initial_state = replay.frames[0].states[arm_index];
  config.initial_state_steady_ns = replay.frames[0].steady_ns;
  config.replay_states.reserve(replay.size - 1);
  config.replay_state_steady_ns.reserve(replay.size - 1);
  for (size_t index = 1; index < replay.size; ++index) {
    config.replay_states.push_back(replay.frames[index].states[arm_index]);
    config.replay_state_steady_ns.push_back(replay.frames[index].steady_ns);
  }
  return config;
}

}  // namespace franka_hardware::test_support
