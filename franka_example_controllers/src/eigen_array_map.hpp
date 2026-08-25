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

#include <Eigen/Core>

namespace franka_example_controllers {
namespace detail {

template <typename EigenType, size_t Size>
Eigen::Map<const EigenType> makeEigenMap(const std::array<double, Size>& values) noexcept {
  static_assert(EigenType::SizeAtCompileTime == static_cast<int>(Size),
                "Eigen type and std::array sizes must match");
  return Eigen::Map<const EigenType>(values.data());
}

template <typename EigenType, size_t Size>
Eigen::Map<const EigenType> makeEigenMap(std::array<double, Size>&&) = delete;

template <typename EigenType, size_t Size>
Eigen::Map<const EigenType> makeEigenMap(const std::array<double, Size>&&) = delete;

}  // namespace detail
}  // namespace franka_example_controllers
