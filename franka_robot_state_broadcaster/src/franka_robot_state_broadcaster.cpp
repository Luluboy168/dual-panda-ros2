#include "franka_robot_state_broadcaster/franka_robot_state_broadcaster.hpp"

#include <stddef.h>

#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcpputils/split.hpp"
#include "rcutils/logging_macros.h"
#include "std_msgs/msg/header.hpp"

namespace franka_robot_state_broadcaster
{
namespace
{

constexpr int64_t kMinimumPublishFrequency = 1;
constexpr int64_t kMaximumPublishFrequency = 1000;
constexpr int64_t kNanosecondsPerSecond = 1'000'000'000;

template <typename Message>
class RealtimePublisherUnlockGuard
{
public:
  explicit RealtimePublisherUnlockGuard(
    realtime_tools::RealtimePublisher<Message> * publisher) noexcept
  : publisher_(publisher)
  {
  }
  ~RealtimePublisherUnlockGuard()
  {
    if (publisher_ != nullptr) {
      publisher_->unlock();
    }
  }
  void disarm() noexcept { publisher_ = nullptr; }

private:
  realtime_tools::RealtimePublisher<Message> * publisher_;
};

}  // namespace

controller_interface::CallbackReturn FrankaRobotStateBroadcaster::on_init()
{
  try {
    auto_declare<std::string>("arm_id", "panda");
    auto_declare<int>("frequency", 30);

  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FrankaRobotStateBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  cleanup_non_realtime_resources();
  arm_id = get_node()->get_parameter("arm_id").as_string();
  frequency = get_node()->get_parameter("frequency").as_int();
  if (frequency < kMinimumPublishFrequency || frequency > kMaximumPublishFrequency) {
    RCLCPP_ERROR(get_node()->get_logger(), "frequency must be an integer in [1, 1000]");
    return CallbackReturn::ERROR;
  }
  publish_interval_ns_ = kNanosecondsPerSecond / frequency;
  reset_cadence();
  full_state_interface_name_ = arm_id + "/" + state_interface_name;
  franka_robot_state = std::make_unique<franka_semantic_components::FrankaRobotState>(
    franka_semantic_components::FrankaRobotState(full_state_interface_name_, arm_id));

  try {
    franka_state_publisher = get_node()->create_publisher<franka_msgs::msg::FrankaState>(
      "~/" + state_interface_name, rclcpp::SystemDefaultsQoS());
    realtime_franka_state_publisher =
      std::make_shared<realtime_tools::RealtimePublisher<franka_msgs::msg::FrankaState>>(
        franka_state_publisher);
    ;
  } catch (const std::exception & e) {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    cleanup_non_realtime_resources();
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(
    get_node()->get_logger(), "%s franka state broadcaster configuration successful",
    arm_id.c_str());
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FrankaRobotStateBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!franka_robot_state) {
    return CallbackReturn::ERROR;
  }
  franka_robot_state->release_interfaces();
  if (
    state_interfaces_.size() != 1 ||
    state_interfaces_.front().get_name() != full_state_interface_name_) {
    reset_cadence();
    return CallbackReturn::ERROR;
  }
  if (!franka_robot_state->assign_loaned_state_interfaces(state_interfaces_)) {
    franka_robot_state->release_interfaces();
    reset_cadence();
    return CallbackReturn::ERROR;
  }
  if (franka_robot_state->get_robot_state_ptr() == nullptr) {
    franka_robot_state->release_interfaces();
    reset_cadence();
    return CallbackReturn::ERROR;
  }
  reset_cadence();
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FrankaRobotStateBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (franka_robot_state) {
    franka_robot_state->release_interfaces();
  }
  reset_cadence();
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FrankaRobotStateBroadcaster::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  cleanup_non_realtime_resources();
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FrankaRobotStateBroadcaster::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  reset_semantic_and_cadence();
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FrankaRobotStateBroadcaster::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  reset_semantic_and_cadence();
  return CallbackReturn::SUCCESS;
}

void FrankaRobotStateBroadcaster::reset_semantic_and_cadence()
{
  if (franka_robot_state) {
    franka_robot_state->release_interfaces();
  }
  reset_cadence();
}

void FrankaRobotStateBroadcaster::reset_cadence()
{
  publish_time_initialized_ = false;
  last_pub_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
}

void FrankaRobotStateBroadcaster::cleanup_non_realtime_resources()
{
  reset_semantic_and_cadence();
  realtime_franka_state_publisher.reset();
  franka_state_publisher.reset();
  franka_robot_state.reset();
  frequency = 0;
  publish_interval_ns_ = 0;
}

void FrankaRobotStateBroadcaster::release_interfaces()
{
  if (franka_robot_state) {
    franka_robot_state->release_interfaces();
  }
  reset_cadence();
  controller_interface::ControllerInterface::release_interfaces();
}

controller_interface::InterfaceConfiguration
FrankaRobotStateBroadcaster::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
FrankaRobotStateBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = franka_robot_state->get_state_interface_names();
  return state_interfaces_config;
}

controller_interface::return_type FrankaRobotStateBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (!realtime_franka_state_publisher) {
    return controller_interface::return_type::ERROR;
  }
  // The first update and a backward-time update establish their epoch by attempting an immediate
  // sample using the manager-provided time. Failed/contended attempts do not consume that epoch.
  const bool reset_epoch =
    !publish_time_initialized_ || time.nanoseconds() < last_pub_.nanoseconds();
  if (!reset_epoch && time.nanoseconds() - last_pub_.nanoseconds() < publish_interval_ns_) {
    return controller_interface::return_type::OK;
  }
  if (!realtime_franka_state_publisher->trylock()) {
    return controller_interface::return_type::OK;
  }
  RealtimePublisherUnlockGuard<franka_msgs::msg::FrankaState> unlock_guard(
    realtime_franka_state_publisher.get());
  try {
    realtime_franka_state_publisher->msg_.header.stamp = time;
    if (
      !franka_robot_state ||
      !franka_robot_state->get_values_as_message(realtime_franka_state_publisher->msg_)) {
      return controller_interface::return_type::ERROR;
    }
  } catch (...) {
    return controller_interface::return_type::ERROR;
  }
  unlock_guard.disarm();
  realtime_franka_state_publisher->unlockAndPublish();
  last_pub_ = time;
  publish_time_initialized_ = true;
  return controller_interface::return_type::OK;
}

}  // namespace franka_robot_state_broadcaster

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(
  franka_robot_state_broadcaster::FrankaRobotStateBroadcaster,
  controller_interface::ControllerInterface)
