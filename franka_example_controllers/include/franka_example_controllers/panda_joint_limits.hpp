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

#include <franka/rate_limiting.h>

namespace franka_example_controllers {

constexpr size_t kPandaJointCount = 7;
constexpr size_t kPandaArmIdMaxLength = 64;

// Absolute Panda limits. Per-controller configuration may be stricter, never looser.
constexpr std::array<double, kPandaJointCount> kPandaAbsoluteEffortCeilings{
    {87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0}};
constexpr std::array<double, kPandaJointCount> kPandaPositionLowerLimits{
    {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973}};
constexpr std::array<double, kPandaJointCount> kPandaPositionUpperLimits{
    {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973}};
constexpr std::array<double, kPandaJointCount> kPandaAbsoluteJointVelocityCeilings{
    {2.175, 2.175, 2.175, 2.175, 2.610, 2.610, 2.610}};

// The velocity command controller retains libfranka 0.9.2's deliberately reduced FCI ceilings
// and acceleration limits. Keeping both kinds of ceiling here makes the distinction explicit and
// prevents the selected controllers from acquiring independent copies of either limit set.
constexpr std::array<double, kPandaJointCount> kPandaFciJointVelocityCeilings =
    franka::kMaxJointVelocity;
constexpr std::array<double, kPandaJointCount> kPandaFciJointAccelerationCeilings =
    franka::kMaxJointAcceleration;

}  // namespace franka_example_controllers
