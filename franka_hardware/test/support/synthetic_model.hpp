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
#include <cstdint>

#include "franka_hardware/common/model_base.hpp"

namespace franka_hardware::test_support
{

// This model is deliberately conspicuous and test-owned. Its values are deterministic markers,
// not identified robot dynamics or a recording of physical hardware.
class SyntheticModel final : public ModelBase
{
public:
  explicit SyntheticModel(uint8_t arm_marker, double coriolis_scale = 1.0);

  [[nodiscard]] uint8_t armMarker() const noexcept { return arm_marker_; }

private:
  std::array<double, 16> poseImpl(
    franka::Frame frame, const std::array<double, 7> & q, const std::array<double, 16> & F_T_EE,
    const std::array<double, 16> & EE_T_K) const override;
  std::array<double, 42> bodyJacobianImpl(
    franka::Frame frame, const std::array<double, 7> & q, const std::array<double, 16> & F_T_EE,
    const std::array<double, 16> & EE_T_K) const override;
  std::array<double, 42> zeroJacobianImpl(
    franka::Frame frame, const std::array<double, 7> & q, const std::array<double, 16> & F_T_EE,
    const std::array<double, 16> & EE_T_K) const override;
  std::array<double, 49> massImpl(
    const std::array<double, 7> & q, const std::array<double, 9> & I_total, double m_total,
    const std::array<double, 3> & F_x_Ctotal) const override;
  std::array<double, 7> coriolisImpl(
    const std::array<double, 7> & q, const std::array<double, 7> & dq,
    const std::array<double, 9> & I_total, double m_total,
    const std::array<double, 3> & F_x_Ctotal) const override;
  std::array<double, 7> gravityImpl(
    const std::array<double, 7> & q, double m_total, const std::array<double, 3> & F_x_Ctotal,
    const std::array<double, 3> & gravity_earth) const override;

  uint8_t arm_marker_;
  double coriolis_scale_;
};

}  // namespace franka_hardware::test_support
