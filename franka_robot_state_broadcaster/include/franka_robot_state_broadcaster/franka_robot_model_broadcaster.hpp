#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "franka_msgs/msg/franka_model.hpp"
#include "franka_semantic_components/franka_robot_model.hpp"
#include "franka_semantic_components/franka_robot_state.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_publisher.hpp"

namespace franka_robot_state_broadcaster
{

class FrankaRobotModelBroadcaster : public controller_interface::ControllerInterface
{
public:
  // Lifecycle state transition functions
  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  void release_interfaces() override;

  // Update loop
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Configuration
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

protected:
  std::string arm_id;
  std::string state_interface_name{"robot_state"};
  std::string model_interface_name{"robot_model"};
  std::string full_state_interface_name_;
  std::string full_model_interface_name_;
  std::shared_ptr<rclcpp::Publisher<franka_msgs::msg::FrankaModel>> franka_model_publisher;
  std::shared_ptr<realtime_tools::RealtimePublisher<franka_msgs::msg::FrankaModel>>
    realtime_franka_model_publisher;
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model;
  size_t arm_count;
  rclcpp::Time last_pub_;
  int64_t frequency{0};
  int64_t publish_interval_ns_{0};
  bool publish_time_initialized_{false};

private:
  void reset_semantic_and_cadence();
  void reset_cadence();
  void cleanup_non_realtime_resources();
};

}  // namespace franka_robot_state_broadcaster
