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

#include "support/synthetic_model.hpp"

#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace franka_hardware::test_support
{

SyntheticModel::SyntheticModel(uint8_t arm_marker, double coriolis_scale)
: arm_marker_(arm_marker), coriolis_scale_(coriolis_scale)
{
  if (arm_marker_ == 0) {
    throw std::invalid_argument("synthetic model arm marker must be nonzero");
  }
  if (!std::isfinite(coriolis_scale_) || coriolis_scale_ <= 0.0) {
    throw std::invalid_argument("synthetic model Coriolis scale must be finite and positive");
  }
}

std::array<double, 16> SyntheticModel::poseImpl(
  franka::Frame frame, const std::array<double, 7> & /*q*/,
  const std::array<double, 16> & /*F_T_EE*/, const std::array<double, 16> & /*EE_T_K*/) const
{
  std::array<double, 16> result{};
  result[0] = 1.0;
  result[5] = 1.0;
  result[10] = 1.0;
  result[15] = 1.0;
  const auto frame_marker = static_cast<double>(static_cast<uint8_t>(frame)) * 0.001;
  result[12] = 0.30 + static_cast<double>(arm_marker_) * 0.10 + frame_marker;
  result[13] = static_cast<double>(arm_marker_) * 0.01;
  result[14] = 0.50 + frame_marker;
  return result;
}

std::array<double, 42> SyntheticModel::bodyJacobianImpl(
  franka::Frame frame, const std::array<double, 7> & /*q*/,
  const std::array<double, 16> & /*F_T_EE*/, const std::array<double, 16> & /*EE_T_K*/) const
{
  std::array<double, 42> result{};
  const double base = static_cast<double>(arm_marker_) * 1000.0 +
                      static_cast<double>(static_cast<uint8_t>(frame)) * 100.0;
  for (size_t index = 0; index < result.size(); ++index) {
    result[index] = base + static_cast<double>(index + 1);
  }
  return result;
}

std::array<double, 42> SyntheticModel::zeroJacobianImpl(
  franka::Frame frame, const std::array<double, 7> & /*q*/,
  const std::array<double, 16> & /*F_T_EE*/, const std::array<double, 16> & /*EE_T_K*/) const
{
  std::array<double, 42> result{};
  const double base = static_cast<double>(arm_marker_) * 2000.0 +
                      static_cast<double>(static_cast<uint8_t>(frame)) * 100.0;
  for (size_t index = 0; index < result.size(); ++index) {
    result[index] = base + static_cast<double>(index + 1);
  }
  return result;
}

std::array<double, 49> SyntheticModel::massImpl(
  const std::array<double, 7> & /*q*/, const std::array<double, 9> & /*I_total*/,
  double /*m_total*/, const std::array<double, 3> & /*F_x_Ctotal*/) const
{
  std::array<double, 49> result{};
  for (size_t joint = 0; joint < 7; ++joint) {
    result[joint * 7 + joint] =
      static_cast<double>(arm_marker_) * 100.0 + static_cast<double>(joint + 1);
  }
  return result;
}

std::array<double, 7> SyntheticModel::coriolisImpl(
  const std::array<double, 7> & /*q*/, const std::array<double, 7> & /*dq*/,
  const std::array<double, 9> & /*I_total*/, double /*m_total*/,
  const std::array<double, 3> & /*F_x_Ctotal*/) const
{
  std::array<double, 7> result{};
  for (size_t joint = 0; joint < result.size(); ++joint) {
    result[joint] =
      (static_cast<double>(arm_marker_) * 10.0 + static_cast<double>(joint + 1)) * coriolis_scale_;
  }
  return result;
}

std::array<double, 7> SyntheticModel::gravityImpl(
  const std::array<double, 7> & /*q*/, double /*m_total*/,
  const std::array<double, 3> & /*F_x_Ctotal*/,
  const std::array<double, 3> & /*gravity_earth*/) const
{
  std::array<double, 7> result{};
  for (size_t joint = 0; joint < result.size(); ++joint) {
    result[joint] = static_cast<double>(arm_marker_) * 20.0 + static_cast<double>(joint + 1);
  }
  return result;
}

}  // namespace franka_hardware::test_support
