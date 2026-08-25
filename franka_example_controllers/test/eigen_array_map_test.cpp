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

#include <gtest/gtest.h>

#include <array>
#include <cstddef>

#include <Eigen/Core>

#include "eigen_array_map.hpp"

namespace franka_example_controllers {
namespace {

using Vector7d = Eigen::Matrix<double, 7, 1>;

class GetterByValueFixture {
 public:
  [[nodiscard]] std::array<double, 7> getCoriolisForceVector() const {
    return {11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0};
  }
};

TEST(EigenArrayMapTest, GetterResultOwnerOutlivesMapUse) {
  GetterByValueFixture model;
  const auto coriolis_array = model.getCoriolisForceVector();
  const auto coriolis = detail::makeEigenMap<Vector7d>(coriolis_array);

  std::array<double, 256> unrelated_stack_values{};
  for (size_t index = 0; index < unrelated_stack_values.size(); ++index) {
    unrelated_stack_values[index] = static_cast<double>(index);
  }

  EXPECT_DOUBLE_EQ(coriolis(0), 11.0);
  EXPECT_DOUBLE_EQ(coriolis(3), 14.0);
  EXPECT_DOUBLE_EQ(coriolis(6), 17.0);
  EXPECT_DOUBLE_EQ(unrelated_stack_values.back(), 255.0);
}

}  // namespace
}  // namespace franka_example_controllers
