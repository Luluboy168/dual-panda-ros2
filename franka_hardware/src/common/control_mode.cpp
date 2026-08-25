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

#include <franka_hardware/common/control_mode.h>

#include <algorithm>
#include <iterator>
#include <string>
#include <vector>

namespace franka_hardware {

std::ostream& operator<<(std::ostream& ostream, ControlMode mode) {
  if (mode == ControlMode::None) {
    ostream << "<none>";
  } else {
    std::vector<std::string> names;
    if ((mode & ControlMode::JointTorque) != ControlMode::None) {
      names.emplace_back("joint_torque");
    }
    if ((mode & ControlMode::JointPosition) != ControlMode::None) {
      names.emplace_back("joint_position");
    }
    if ((mode & ControlMode::JointVelocity) != ControlMode::None) {
      names.emplace_back("joint_velocity");
    }
    if ((mode & ControlMode::CartesianVelocity) != ControlMode::None) {
      names.emplace_back("cartesian_velocity");
    }
    if ((mode & ControlMode::CartesianPose) != ControlMode::None) {
      names.emplace_back("cartesian_pose");
    }
    std::copy(names.cbegin(), names.cend() - 1, std::ostream_iterator<std::string>(ostream, ", "));
    ostream << names.back();
  }
  return ostream;
}

}  // namespace franka_hardware
