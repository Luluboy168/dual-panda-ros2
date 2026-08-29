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

namespace franka_ik {

inline constexpr std::size_t kPandaJointCount = 7;
inline constexpr std::size_t kPandaArmIdMaxLength = 64;

// Literal limits from franka_description/robots/common/panda_arm.xacro. RobotChains validates
// every configured arm against these values so a changed description cannot silently move the IK
// fence.
inline constexpr std::array<double, kPandaJointCount> kPandaPositionLowerLimits{
    {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973}};
inline constexpr std::array<double, kPandaJointCount> kPandaPositionUpperLimits{
    {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973}};
inline constexpr std::array<double, kPandaJointCount> kPandaVelocityLimits{
    {2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61}};
inline constexpr std::array<double, kPandaJointCount> kPandaEffortLimits{
    {87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0}};

}  // namespace franka_ik
