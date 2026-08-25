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

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

#include "support/synthetic_franka_arm_backend.hpp"

namespace franka_hardware::test_support {

inline constexpr std::string_view kSyntheticReplayHeader = "multipanda_synthetic_replay_v1";
inline constexpr std::string_view kSyntheticReplayArms = "arms,panda1,panda2";
inline constexpr size_t kSyntheticReplayArmCount = 2;
inline constexpr size_t kSyntheticReplayFieldCount = 44;
inline constexpr size_t kSyntheticReplayMaximumFrames = 32;
inline constexpr size_t kSyntheticReplayMaximumBytes = 64 * 1024;

struct SyntheticReplayFrame {
  uint64_t steady_ns{0};
  std::array<franka::RobotState, kSyntheticReplayArmCount> states{};
};

struct SyntheticReplay {
  std::array<SyntheticReplayFrame, kSyntheticReplayMaximumFrames> frames{};
  size_t size{0};
};

enum class SyntheticReplayError : uint8_t {
  None,
  InputTooLarge,
  ForbiddenToken,
  InvalidHeader,
  InvalidArms,
  BlankLine,
  UnknownRow,
  FieldCount,
  InvalidTimestamp,
  NonIncreasingTimestamp,
  NonFiniteValue,
  FrameCount,
};

struct SyntheticReplayParseResult {
  SyntheticReplay replay{};
  SyntheticReplayError error{SyntheticReplayError::None};
  size_t line{0};

  [[nodiscard]] constexpr bool ok() const noexcept { return error == SyntheticReplayError::None; }
};

[[nodiscard]] SyntheticReplayParseResult parseSyntheticReplay(std::string_view content) noexcept;
[[nodiscard]] SyntheticFrankaArmBackendConfig replayBackendConfig(const SyntheticReplay& replay,
                                                                  size_t arm_index);

}  // namespace franka_hardware::test_support
