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

#include <franka/robot_state.h>

#include <array>

namespace franka_hardware {

struct RobotCommand {
  std::array<double, 7> efforts{};
  std::array<double, 7> joint_positions{};
  std::array<double, 7> joint_velocities{};
  std::array<double, 16> cartesian_positions{};
  std::array<double, 6> cartesian_velocities{};
};

inline RobotCommand makeSafeRobotCommand(const franka::RobotState& state) noexcept {
  RobotCommand command;
  command.joint_positions = state.q;
  command.cartesian_positions = state.O_T_EE;
  return command;
}

}  // namespace franka_hardware
