#include "franka_robot_state_broadcaster/franka_robot_model_broadcaster.hpp"

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

namespace
{
template <class T, size_t N>
std::ostream & operator<<(std::ostream & ostream, const std::array<T, N> & array)
{
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}
}  // anonymous namespace

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

controller_interface::CallbackReturn FrankaRobotModelBroadcaster::on_init()
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

// configure works
controller_interface::CallbackReturn FrankaRobotModelBroadcaster::on_configure(
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
  full_model_interface_name_ = arm_id + "/" + model_interface_name;
  franka_robot_model = std::make_unique<franka_semantic_components::FrankaRobotModel>(
    franka_semantic_components::FrankaRobotModel(full_model_interface_name_, arm_id));

  try {
    franka_model_publisher = get_node()->create_publisher<franka_msgs::msg::FrankaModel>(
      "~/" + model_interface_name, rclcpp::SystemDefaultsQoS());
    realtime_franka_model_publisher =
      std::make_shared<realtime_tools::RealtimePublisher<franka_msgs::msg::FrankaModel>>(
        franka_model_publisher);
    ;
  } catch (const std::exception & e) {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    cleanup_non_realtime_resources();
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(
    get_node()->get_logger(), "%s franka model broadcaster configuration successful",
    arm_id.c_str());
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FrankaRobotModelBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!franka_robot_model) {
    return CallbackReturn::ERROR;
  }
  franka_robot_model->release_interfaces();
  if (state_interfaces_.size() != 2) {
    reset_cadence();
    return CallbackReturn::ERROR;
  }
  size_t model_count = 0;
  size_t state_count = 0;
  for (const auto & interface : state_interfaces_) {
    model_count += interface.get_name() == full_model_interface_name_ ? 1U : 0U;
    state_count += interface.get_name() == full_state_interface_name_ ? 1U : 0U;
  }
  if (model_count != 1 || state_count != 1) {
    reset_cadence();
    return CallbackReturn::ERROR;
  }
  if (!franka_robot_model->assign_loaned_state_interfaces(state_interfaces_)) {
    franka_robot_model->release_interfaces();
    reset_cadence();
    return CallbackReturn::ERROR;
  }
  if (!franka_robot_model->update_state_and_model()) {
    franka_robot_model->release_interfaces();
    reset_cadence();
    return CallbackReturn::ERROR;
  }
  reset_cadence();
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FrankaRobotModelBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (franka_robot_model) {
    franka_robot_model->release_interfaces();
  }
  reset_cadence();
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FrankaRobotModelBroadcaster::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  cleanup_non_realtime_resources();
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FrankaRobotModelBroadcaster::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  reset_semantic_and_cadence();
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FrankaRobotModelBroadcaster::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  reset_semantic_and_cadence();
  return CallbackReturn::SUCCESS;
}

void FrankaRobotModelBroadcaster::reset_semantic_and_cadence()
{
  if (franka_robot_model) {
    franka_robot_model->release_interfaces();
  }
  reset_cadence();
}

void FrankaRobotModelBroadcaster::reset_cadence()
{
  publish_time_initialized_ = false;
  last_pub_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
}

void FrankaRobotModelBroadcaster::cleanup_non_realtime_resources()
{
  reset_semantic_and_cadence();
  realtime_franka_model_publisher.reset();
  franka_model_publisher.reset();
  franka_robot_model.reset();
  frequency = 0;
  publish_interval_ns_ = 0;
}

void FrankaRobotModelBroadcaster::release_interfaces()
{
  if (franka_robot_model) {
    franka_robot_model->release_interfaces();
  }
  reset_cadence();
  controller_interface::ControllerInterface::release_interfaces();
}

controller_interface::InterfaceConfiguration
FrankaRobotModelBroadcaster::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
FrankaRobotModelBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & name : franka_robot_model->get_state_interface_names()) {
    state_interfaces_config.names.push_back(name);
  }
  return state_interfaces_config;
}

controller_interface::return_type FrankaRobotModelBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (!realtime_franka_model_publisher) {
    return controller_interface::return_type::ERROR;
  }
  // The first update and a backward-time update establish their epoch by attempting an immediate
  // sample using the manager-provided time. Failed/contended attempts do not consume that epoch.
  const bool reset_epoch =
    !publish_time_initialized_ || time.nanoseconds() < last_pub_.nanoseconds();
  if (!reset_epoch && time.nanoseconds() - last_pub_.nanoseconds() < publish_interval_ns_) {
    return controller_interface::return_type::OK;
  }
  if (!realtime_franka_model_publisher->trylock()) {
    return controller_interface::return_type::OK;
  }
  RealtimePublisherUnlockGuard<franka_msgs::msg::FrankaModel> unlock_guard(
    realtime_franka_model_publisher.get());
  try {
    realtime_franka_model_publisher->msg_.header.stamp = time;
    if (
      !franka_robot_model ||
      !franka_robot_model->get_values_as_message(realtime_franka_model_publisher->msg_)) {
      return controller_interface::return_type::ERROR;
    }
  } catch (...) {
    return controller_interface::return_type::ERROR;
  }
  unlock_guard.disarm();
  realtime_franka_model_publisher->unlockAndPublish();
  last_pub_ = time;
  publish_time_initialized_ = true;
  return controller_interface::return_type::OK;
}
}  // namespace franka_robot_state_broadcaster

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(
  franka_robot_state_broadcaster::FrankaRobotModelBroadcaster,
  controller_interface::ControllerInterface)
