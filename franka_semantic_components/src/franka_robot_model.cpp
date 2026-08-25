// Copyright (c) 2023 Franka Robotics GmbH
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

#include "franka_semantic_components/franka_robot_model.hpp"

#include <cstring>
#include <stdexcept>
namespace
{

// Example implementation of bit_cast: https://en.cppreference.com/w/cpp/numeric/bit_cast
template <class To, class From>
std::
  enable_if_t<
    sizeof(To) == sizeof(From) && std::is_trivially_copyable_v<From> &&
      std::is_trivially_copyable_v<To>,
    To>
  bit_cast(const From & src) noexcept
{
  static_assert(
    std::is_trivially_constructible_v<To>,
    "This implementation additionally requires "
    "destination type to be trivially constructible");

  To dst;
  std::memcpy(&dst, &src, sizeof(To));
  return dst;
}

}  // namespace

namespace franka_semantic_components
{
// order of args is correct
FrankaRobotModel::FrankaRobotModel(const std::string & model_name, const std::string & robot_name)
: SemanticComponentInterface(model_name, 2)
{
  arm_id_ = robot_name;
  interface_names_.emplace_back(model_name);
  interface_names_.emplace_back(arm_id_ + "/" + robot_state_interface_name_);
}

bool FrankaRobotModel::update_state_and_model()
{
  auto franka_state_interface = std::find_if(
    state_interfaces_.begin(), state_interfaces_.end(),
    [this](const auto & interface) { return interface.get().get_name() == interface_names_[1]; });

  auto franka_model_interface = std::find_if(
    state_interfaces_.begin(), state_interfaces_.end(),
    [this](const auto & interface) { return interface.get().get_name() == interface_names_[0]; });

  if (
    franka_state_interface == state_interfaces_.end() ||
    franka_model_interface == state_interfaces_.end()) {
    robot_model = nullptr;
    robot_state = nullptr;
    initialized = false;
    return false;
  }

  try {
    const auto model_value = (*franka_model_interface).get().get_optional<double>(1);
    const auto state_value = (*franka_state_interface).get().get_optional<double>(1);
    if (!model_value || !state_value) {
      robot_model = nullptr;
      robot_state = nullptr;
      initialized = false;
      return false;
    }
    auto * const decoded_model = bit_cast<franka_hardware::ModelBase *>(*model_value);
    auto * const decoded_state = bit_cast<franka::RobotState *>(*state_value);
    if (decoded_model == nullptr || decoded_state == nullptr) {
      robot_model = nullptr;
      robot_state = nullptr;
      initialized = false;
      return false;
    }
    robot_model = decoded_model;
    robot_state = decoded_state;
    initialized = true;
  } catch (...) {
    robot_model = nullptr;
    robot_state = nullptr;
    initialized = false;
    return false;
  }
  return true;
}

void FrankaRobotModel::initialize()
{
  if (!update_state_and_model()) {
    throw std::runtime_error("Franka robot model interfaces are unavailable");
  }
}

bool FrankaRobotModel::get_values_as_message(franka_msgs::msg::FrankaModel & message)
{
  if (!update_state_and_model()) {
    return false;
  }
  constexpr auto frame = franka::Frame::kEndEffector;
  message.coriolis = robot_model->coriolis(*robot_state);
  message.mass = robot_model->mass(*robot_state);
  message.ee_body_jacobian = robot_model->bodyJacobian(frame, *robot_state);
  message.ee_zero_jacobian = robot_model->zeroJacobian(frame, *robot_state);
  return true;
}

void FrankaRobotModel::release_interfaces()
{
  SemanticComponentInterface::release_interfaces();
  robot_model = nullptr;
  robot_state = nullptr;
  initialized = false;
}
}  // namespace franka_semantic_components
