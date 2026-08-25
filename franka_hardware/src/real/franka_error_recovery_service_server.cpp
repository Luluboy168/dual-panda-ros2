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

#include <franka_hardware/real/franka_error_recovery_service_server.hpp>

#include <exception>
#include <functional>
#include <utility>

namespace franka_hardware {

FrankaErrorRecoveryServiceServer::FrankaErrorRecoveryServiceServer(
    const rclcpp::NodeOptions& options,
    std::shared_ptr<FrankaArmBackend> backend,
    std::string prefix)
    : rclcpp::Node(prefix + "error_recovery_service_server", options),
      backend_(std::move(backend)) {
  error_recovery_service_ = create_service<franka_msgs::srv::ErrorRecovery>(
      "~/error_recovery", std::bind(&FrankaErrorRecoveryServiceServer::triggerAutomaticRecovery,
                                    this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "Error recovery service started");
}

void FrankaErrorRecoveryServiceServer::triggerAutomaticRecovery(
    const franka_msgs::srv::ErrorRecovery::Request::SharedPtr& /*request*/,
    const franka_msgs::srv::ErrorRecovery::Response::SharedPtr& response) {
  response->success = false;
  response->error.clear();
  if (!this->backend_->hasFault()) {
    RCLCPP_INFO(this->get_logger(), "No errors detected; error recovery is not necessary.");
    response->error = "No errors";
    response->success = false;
  } else {
    try {
      if (!this->backend_->recoverToReading()) {
        response->error = "Recovery did not reach a safe state-only mode";
        response->success = false;
        RCLCPP_ERROR(this->get_logger(), "%s", response->error.c_str());
        return;
      }
      response->error.clear();
      response->success = true;
      RCLCPP_INFO(this->get_logger(), "Successfully recovered to state-only reading.");
    } catch (const franka::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error recovery failed: %s", e.what());
      response->error = e.what();
      response->success = false;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Unexpected recovery failure: %s", e.what());
      response->error = e.what();
      response->success = false;
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Unknown recovery failure");
      response->error = "unknown recovery failure";
      response->success = false;
    }
  }
};

}  // namespace franka_hardware
