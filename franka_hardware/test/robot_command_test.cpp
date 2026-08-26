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

#include "franka_hardware/real/robot_command.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <new>
#include <random>
#include <string>

namespace franka_hardware {
namespace {

TEST(RobotCommandTest, SafeSnapshotUsesZerosAndMeasuredState) {
  franka::RobotState state;
  for (size_t index = 0; index < state.q.size(); ++index) {
    state.q[index] = static_cast<double>(index) + 0.25;
  }
  for (size_t index = 0; index < state.O_T_EE.size(); ++index) {
    state.O_T_EE[index] = static_cast<double>(index) + 1.5;
  }

  const auto command = makeSafeRobotCommand(state);

  EXPECT_TRUE(std::all_of(command.efforts.begin(), command.efforts.end(),
                          [](double value) { return value == 0.0; }));
  EXPECT_TRUE(std::all_of(command.joint_velocities.begin(), command.joint_velocities.end(),
                          [](double value) { return value == 0.0; }));
  EXPECT_TRUE(std::all_of(command.cartesian_velocities.begin(), command.cartesian_velocities.end(),
                          [](double value) { return value == 0.0; }));
  EXPECT_EQ(command.joint_positions, state.q);
  EXPECT_EQ(command.cartesian_positions, state.O_T_EE);
}

// ---------------------------------------------------------------------------
// Fixed-seed fuzz test: hostile RobotState inputs (NaN, +-Inf, subnormals,
// extreme magnitudes) fed through makeSafeRobotCommand(). This is the only
// command-construction surface robot_command.hpp exposes; the invariant it
// must preserve under adversarial state is that dynamic command fields
// (effort/velocity) are always exactly zero regardless of measured-state
// content, and that position fields are copied bit-exactly (NaN payloads are
// compared by bit pattern since NaN != NaN under IEEE-754 equality).
// ---------------------------------------------------------------------------

bool sameBits(double lhs, double rhs) {
  std::uint64_t lhs_bits = 0;
  std::uint64_t rhs_bits = 0;
  std::memcpy(&lhs_bits, &lhs, sizeof(lhs_bits));
  std::memcpy(&rhs_bits, &rhs, sizeof(rhs_bits));
  return lhs_bits == rhs_bits;
}

double randomHostileDouble(std::mt19937_64& engine) {
  switch (engine() % 9U) {
    case 0:
      return std::numeric_limits<double>::quiet_NaN();
    case 1:
      return std::numeric_limits<double>::infinity();
    case 2:
      return -std::numeric_limits<double>::infinity();
    case 3:
      return 0.0;
    case 4:
      return -0.0;
    case 5:
      return std::numeric_limits<double>::denorm_min();
    case 6:
      return std::numeric_limits<double>::max();
    case 7:
      return std::numeric_limits<double>::lowest();
    default: {
      std::uniform_real_distribution<double> dist(-1e9, 1e9);
      return dist(engine);
    }
  }
}

template <typename Array>
void fillHostile(Array& values, std::mt19937_64& engine) {
  for (auto& value : values) {
    value = randomHostileDouble(engine);
  }
}

TEST(RobotCommandTest, FixedSeedHostileStatesAlwaysZeroDynamicFieldsAndPreservePositions) {
  constexpr std::array<std::uint64_t, 3> kSeeds{0x524f424fU, 0x434f4d4dU, 0x68617a617264ULL};
  constexpr std::size_t kCasesPerSeed = 4000;
  std::size_t total_cases = 0;

  for (const auto seed : kSeeds) {
    std::cout << "RobotCommand hostile-state property seed=" << seed << " cases=" << kCasesPerSeed
              << '\n';
    std::mt19937_64 engine(seed);
    for (std::size_t case_index = 0; case_index < kCasesPerSeed; ++case_index) {
      franka::RobotState state{};
      fillHostile(state.q, engine);
      fillHostile(state.O_T_EE, engine);
      // Poison fields that must NOT leak into the safe command: if these ever
      // start influencing makeSafeRobotCommand's output, this test must fail.
      fillHostile(state.dq, engine);
      fillHostile(state.tau_J, engine);
      fillHostile(state.tau_J_d, engine);
      fillHostile(state.dtau_J, engine);

      SCOPED_TRACE("seed=" + std::to_string(seed) + " case=" + std::to_string(case_index));
      const auto command = makeSafeRobotCommand(state);

      for (double value : command.efforts) {
        EXPECT_TRUE(sameBits(value, 0.0));
      }
      for (double value : command.joint_velocities) {
        EXPECT_TRUE(sameBits(value, 0.0));
      }
      for (double value : command.cartesian_velocities) {
        EXPECT_TRUE(sameBits(value, 0.0));
      }
      for (std::size_t index = 0; index < command.joint_positions.size(); ++index) {
        EXPECT_TRUE(sameBits(command.joint_positions[index], state.q[index]));
      }
      for (std::size_t index = 0; index < command.cartesian_positions.size(); ++index) {
        EXPECT_TRUE(sameBits(command.cartesian_positions[index], state.O_T_EE[index]));
      }
      ++total_cases;
    }
  }

  std::cout << "RobotCommand hostile-state property total generated cases=" << total_cases
            << '\n';
  EXPECT_EQ(total_cases, 12000U);
}

TEST(RobotCommandTest, DefaultConstructionIsAllZeroRegardlessOfPriorStackContents) {
  // Deliberately poison the stack slot before placement-constructing over it,
  // so a missing/incomplete zero-initialization would be observable.
  alignas(RobotCommand) unsigned char storage[sizeof(RobotCommand)];
  std::memset(storage, 0xAA, sizeof(storage));
  auto* command = new (storage) RobotCommand();

  for (double value : command->efforts) {
    EXPECT_TRUE(sameBits(value, 0.0));
  }
  for (double value : command->joint_positions) {
    EXPECT_TRUE(sameBits(value, 0.0));
  }
  for (double value : command->joint_velocities) {
    EXPECT_TRUE(sameBits(value, 0.0));
  }
  for (double value : command->cartesian_positions) {
    EXPECT_TRUE(sameBits(value, 0.0));
  }
  for (double value : command->cartesian_velocities) {
    EXPECT_TRUE(sameBits(value, 0.0));
  }
  command->~RobotCommand();
}

}  // namespace
}  // namespace franka_hardware
