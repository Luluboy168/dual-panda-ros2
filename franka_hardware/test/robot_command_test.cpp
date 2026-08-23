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

namespace franka_hardware
{
namespace
{

TEST(RobotCommandTest, SafeSnapshotUsesZerosAndMeasuredState)
{
  franka::RobotState state;
  for (size_t index = 0; index < state.q.size(); ++index) {
    state.q[index] = static_cast<double>(index) + 0.25;
  }
  for (size_t index = 0; index < state.O_T_EE.size(); ++index) {
    state.O_T_EE[index] = static_cast<double>(index) + 1.5;
  }

  const auto command = makeSafeRobotCommand(state);

  EXPECT_TRUE(std::all_of(
    command.efforts.begin(), command.efforts.end(), [](double value) { return value == 0.0; }));
  EXPECT_TRUE(std::all_of(
    command.joint_velocities.begin(), command.joint_velocities.end(),
    [](double value) { return value == 0.0; }));
  EXPECT_TRUE(std::all_of(
    command.cartesian_velocities.begin(), command.cartesian_velocities.end(),
    [](double value) { return value == 0.0; }));
  EXPECT_EQ(command.joint_positions, state.q);
  EXPECT_EQ(command.cartesian_positions, state.O_T_EE);
}

}  // namespace
}  // namespace franka_hardware
