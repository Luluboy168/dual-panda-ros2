// Copyright (c) 2017 Franka Emika GmbH
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

#include <ostream>
#include <type_traits>

namespace franka_hardware {

enum class ControlMode {
  None = 0,
  JointTorque = (1 << 0),
  JointPosition = (1 << 1),
  JointVelocity = (1 << 2),
  CartesianVelocity = (1 << 3),
  CartesianPose = (1 << 4),
};

std::ostream& operator<<(std::ostream& ostream, ControlMode mode);

// Implement operators for BitmaskType concept
constexpr ControlMode operator&(ControlMode left, ControlMode right) {
  return static_cast<ControlMode>(static_cast<std::underlying_type_t<ControlMode>>(left) &
                                  static_cast<std::underlying_type_t<ControlMode>>(right));
}

constexpr ControlMode operator|(ControlMode left, ControlMode right) {
  return static_cast<ControlMode>(static_cast<std::underlying_type_t<ControlMode>>(left) |
                                  static_cast<std::underlying_type_t<ControlMode>>(right));
}

constexpr ControlMode operator^(ControlMode left, ControlMode right) {
  return static_cast<ControlMode>(static_cast<std::underlying_type_t<ControlMode>>(left) ^
                                  static_cast<std::underlying_type_t<ControlMode>>(right));
}

constexpr ControlMode operator~(ControlMode mode) {
  return static_cast<ControlMode>(~static_cast<std::underlying_type_t<ControlMode>>(mode));
}

constexpr ControlMode& operator&=(ControlMode& left, ControlMode right) {
  return left = left & right;
}

constexpr ControlMode& operator|=(ControlMode& left, ControlMode right) {
  return left = left | right;
}

}  // namespace franka_hardware
