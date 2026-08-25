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

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "franka_hardware/real/franka_multi_hardware_interface.hpp"
#include "franka_msgs/srv/error_recovery.hpp"
#include "franka_msgs/srv/set_cartesian_stiffness.hpp"
#include "franka_msgs/srv/set_force_torque_collision_behavior.hpp"
#include "franka_msgs/srv/set_full_collision_behavior.hpp"
#include "franka_msgs/srv/set_joint_stiffness.hpp"
#include "franka_msgs/srv/set_load.hpp"
#include "franka_msgs/srv/set_stiffness_frame.hpp"
#include "franka_msgs/srv/set_tcp_frame.hpp"
#include "support/synthetic_franka_arm_backend.hpp"

namespace franka_hardware {
namespace {

using namespace std::chrono_literals;
using test_support::SyntheticCondition;
using test_support::SyntheticFailurePoint;
using test_support::SyntheticFrankaArmBackend;
using test_support::SyntheticFrankaArmBackendConfig;

hardware_interface::InterfaceInfo makeInterface(const std::string& name) {
  hardware_interface::InterfaceInfo interface{};
  interface.name = name;
  interface.data_type = "double";
  return interface;
}

hardware_interface::HardwareInfo makeHardwareInfo() {
  hardware_interface::HardwareInfo info{};
  info.name = "FrankaMultiHardwareInterface";
  info.type = "system";
  info.hardware_plugin_name = "franka_hardware/FrankaMultiHardwareInterface";
  info.hardware_parameters["robot_count"] = "2";
  for (size_t arm_slot = 1; arm_slot <= 2; ++arm_slot) {
    const auto slot = std::to_string(arm_slot);
    const auto arm_name = "panda" + slot;
    info.hardware_parameters["ns_" + slot] = arm_name;
    info.hardware_parameters["robot_ip_" + slot] = "offline-placeholder-" + slot;
    for (size_t joint_index = 1; joint_index <= FrankaMultiHardwareInterface::kNumberOfJoints;
         ++joint_index) {
      hardware_interface::ComponentInfo joint{};
      joint.name = arm_name + "_joint" + std::to_string(joint_index);
      joint.type = "joint";
      joint.command_interfaces = {makeInterface("effort"), makeInterface("position"),
                                  makeInterface("velocity")};
      joint.state_interfaces = {makeInterface("position"), makeInterface("velocity"),
                                makeInterface("effort")};
      info.joints.push_back(std::move(joint));
    }
  }
  return info;
}

class RclcppScope {
 public:
  RclcppScope() {
    if (!rclcpp::ok()) {
      int argc = 0;
      char** argv = nullptr;
      rclcpp::init(argc, argv);
      owns_context_ = true;
    }
  }

  RclcppScope(const RclcppScope&) = delete;
  RclcppScope& operator=(const RclcppScope&) = delete;

  ~RclcppScope() {
    if (owns_context_ && rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

 private:
  bool owns_context_{false};
};

struct ServiceBackendControl {
  size_t throw_read_call{0};
  size_t read_calls{0};
};

class ServiceBackend final : public FrankaArmBackend {
 public:
  ServiceBackend(std::shared_ptr<SyntheticFrankaArmBackend> backend,
                 std::shared_ptr<ServiceBackendControl> control)
      : backend_(std::move(backend)), control_(std::move(control)) {}

  bool startStateReading() override { return backend_->startStateReading(); }
  bool stop() override { return backend_->stop(); }
  franka::RobotState readLatestState() override {
    ++control_->read_calls;
    if (control_->throw_read_call != 0 && control_->read_calls == control_->throw_read_call) {
      throw std::runtime_error("injected service-test read failure");
    }
    return backend_->readLatestState();
  }
  ModelBase* model() noexcept override { return backend_->model(); }
  bool canPublishCommand() const noexcept override { return backend_->canPublishCommand(); }
  bool publishCommand(const RobotCommand& command) noexcept override {
    return backend_->publishCommand(command);
  }
  bool canRequestControlMode(ControlMode mode) const noexcept override {
    return backend_->canRequestControlMode(mode);
  }
  bool requestControlMode(ControlMode mode) noexcept override {
    return backend_->requestControlMode(mode);
  }
  ControlMode requestedControlMode() const noexcept override {
    return backend_->requestedControlMode();
  }
  ControlMode activeControlMode() const noexcept override { return backend_->activeControlMode(); }
  bool hasFault() const noexcept override { return backend_->hasFault(); }
  bool recoverToReading() override { return backend_->recoverToReading(); }
  FrankaArmBackendDiagnostics diagnostics() const noexcept override {
    return backend_->diagnostics();
  }
  void setJointStiffness(
      const franka_msgs::srv::SetJointStiffness::Request::SharedPtr& request) override {
    backend_->setJointStiffness(request);
  }
  void setCartesianStiffness(
      const franka_msgs::srv::SetCartesianStiffness::Request::SharedPtr& request) override {
    backend_->setCartesianStiffness(request);
  }
  void setLoad(const franka_msgs::srv::SetLoad::Request::SharedPtr& request) override {
    backend_->setLoad(request);
  }
  void setTCPFrame(const franka_msgs::srv::SetTCPFrame::Request::SharedPtr& request) override {
    backend_->setTCPFrame(request);
  }
  void setStiffnessFrame(
      const franka_msgs::srv::SetStiffnessFrame::Request::SharedPtr& request) override {
    backend_->setStiffnessFrame(request);
  }
  void setForceTorqueCollisionBehavior(
      const franka_msgs::srv::SetForceTorqueCollisionBehavior::Request::SharedPtr& request)
      override {
    backend_->setForceTorqueCollisionBehavior(request);
  }
  void setFullCollisionBehavior(
      const franka_msgs::srv::SetFullCollisionBehavior::Request::SharedPtr& request) override {
    backend_->setFullCollisionBehavior(request);
  }

 private:
  std::shared_ptr<SyntheticFrankaArmBackend> backend_;
  std::shared_ptr<ServiceBackendControl> control_;
};

struct BackendHarness {
  std::map<std::string, SyntheticFrankaArmBackendConfig> configurations;
  std::map<std::string, std::shared_ptr<ServiceBackendControl>> controls;
  std::map<std::string, std::weak_ptr<SyntheticFrankaArmBackend>> backends;

  BackendHarness() {
    for (uint8_t arm_slot = 1; arm_slot <= 2; ++arm_slot) {
      const auto arm_name = "panda" + std::to_string(arm_slot);
      configurations.emplace(arm_name, SyntheticFrankaArmBackendConfig::forArm(arm_slot));
      controls.emplace(arm_name, std::make_shared<ServiceBackendControl>());
    }
  }

  BackendFactory factory() {
    return [this](const std::string& arm_name, const std::string&,
                  const rclcpp::Logger&) -> std::shared_ptr<FrankaArmBackend> {
      auto backend = std::make_shared<SyntheticFrankaArmBackend>(configurations.at(arm_name));
      backends[arm_name] = backend;
      return std::make_shared<ServiceBackend>(std::move(backend), controls.at(arm_name));
    };
  }

  std::shared_ptr<SyntheticFrankaArmBackend> backend(const std::string& arm_name) const {
    return backends.at(arm_name).lock();
  }
};

std::string recoveryServiceName(const std::string& arm_name) {
  return "/" + arm_name + "_error_recovery_service_server/error_recovery";
}

std::string parameterServiceName(const std::string& arm_name, const std::string& endpoint) {
  return "/" + arm_name + "_param_service_server/" + endpoint;
}

const std::array<std::string, 7> kParameterEndpoints{"set_joint_stiffness",
                                                     "set_cartesian_stiffness",
                                                     "set_load",
                                                     "set_tcp_frame",
                                                     "set_stiffness_frame",
                                                     "set_force_torque_collision_behavior",
                                                     "set_full_collision_behavior"};

class ServiceCaller {
 public:
  ServiceCaller() : node_(std::make_shared<rclcpp::Node>("franka_service_test_client")) {}

  template <typename Service>
  typename Service::Response::SharedPtr call(const std::string& service_name,
                                             const typename Service::Request::SharedPtr& request) {
    auto client = node_->create_client<Service>(service_name);
    if (std::string(client->get_service_name()) != service_name) {
      throw std::runtime_error("client service name did not resolve exactly");
    }
    if (!client->wait_for_service(2s)) {
      throw std::runtime_error("service was not available: " + service_name);
    }
    auto pending = client->async_send_request(request);
    const auto result = rclcpp::spin_until_future_complete(node_, pending.future, 2s);
    if (result != rclcpp::FutureReturnCode::SUCCESS) {
      client->remove_pending_request(pending);
      throw std::runtime_error("service call did not complete: " + service_name);
    }
    return pending.get();
  }

  template <typename Service>
  typename rclcpp::Client<Service>::SharedPtr makeClient(const std::string& service_name) {
    return node_->create_client<Service>(service_name);
  }

  std::shared_ptr<rclcpp::Node> node() const { return node_; }

 private:
  std::shared_ptr<rclcpp::Node> node_;
};

template <size_t Size>
std::array<double, Size> values(double base) {
  std::array<double, Size> result{};
  for (size_t index = 0; index < Size; ++index) {
    result.at(index) = base + static_cast<double>(index);
  }
  return result;
}

template <typename Response>
void expectSuccess(const std::shared_ptr<Response>& response) {
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
  EXPECT_TRUE(response->error.empty());
}

template <typename Response>
void expectFailure(const std::shared_ptr<Response>& response) {
  ASSERT_NE(response, nullptr);
  EXPECT_FALSE(response->success);
  EXPECT_FALSE(response->error.empty());
}

void exerciseAllParameterServices(ServiceCaller& caller,
                                  const std::string& arm_name,
                                  const std::shared_ptr<SyntheticFrankaArmBackend>& backend,
                                  double base) {
  ASSERT_NE(backend, nullptr);
  auto joint = std::make_shared<franka_msgs::srv::SetJointStiffness::Request>();
  joint->joint_stiffness = values<7>(base + 10.0);
  expectSuccess(caller.call<franka_msgs::srv::SetJointStiffness>(
      parameterServiceName(arm_name, "set_joint_stiffness"), joint));
  EXPECT_EQ(backend->parameterSnapshot().joint_stiffness, joint->joint_stiffness);

  auto cartesian = std::make_shared<franka_msgs::srv::SetCartesianStiffness::Request>();
  cartesian->cartesian_stiffness = values<6>(base + 20.0);
  expectSuccess(caller.call<franka_msgs::srv::SetCartesianStiffness>(
      parameterServiceName(arm_name, "set_cartesian_stiffness"), cartesian));
  EXPECT_EQ(backend->parameterSnapshot().cartesian_stiffness, cartesian->cartesian_stiffness);

  auto load = std::make_shared<franka_msgs::srv::SetLoad::Request>();
  load->mass = base + 30.0;
  load->center_of_mass = values<3>(base + 31.0);
  load->load_inertia = values<9>(base + 34.0);
  expectSuccess(
      caller.call<franka_msgs::srv::SetLoad>(parameterServiceName(arm_name, "set_load"), load));
  EXPECT_DOUBLE_EQ(backend->parameterSnapshot().load_mass, load->mass);
  EXPECT_EQ(backend->parameterSnapshot().load_center_of_mass, load->center_of_mass);
  EXPECT_EQ(backend->parameterSnapshot().load_inertia, load->load_inertia);

  auto tcp = std::make_shared<franka_msgs::srv::SetTCPFrame::Request>();
  tcp->transformation = values<16>(base + 50.0);
  expectSuccess(caller.call<franka_msgs::srv::SetTCPFrame>(
      parameterServiceName(arm_name, "set_tcp_frame"), tcp));
  EXPECT_EQ(backend->parameterSnapshot().tcp_frame, tcp->transformation);

  auto stiffness = std::make_shared<franka_msgs::srv::SetStiffnessFrame::Request>();
  stiffness->transformation = values<16>(base + 70.0);
  expectSuccess(caller.call<franka_msgs::srv::SetStiffnessFrame>(
      parameterServiceName(arm_name, "set_stiffness_frame"), stiffness));
  EXPECT_EQ(backend->parameterSnapshot().stiffness_frame, stiffness->transformation);

  auto nominal = std::make_shared<franka_msgs::srv::SetForceTorqueCollisionBehavior::Request>();
  nominal->lower_torque_thresholds_nominal = values<7>(base + 90.0);
  nominal->upper_torque_thresholds_nominal = values<7>(base + 100.0);
  nominal->lower_force_thresholds_nominal = values<6>(base + 110.0);
  nominal->upper_force_thresholds_nominal = values<6>(base + 120.0);
  expectSuccess(caller.call<franka_msgs::srv::SetForceTorqueCollisionBehavior>(
      parameterServiceName(arm_name, "set_force_torque_collision_behavior"), nominal));
  EXPECT_EQ(backend->parameterSnapshot().lower_torque_thresholds_nominal,
            nominal->lower_torque_thresholds_nominal);
  EXPECT_EQ(backend->parameterSnapshot().upper_torque_thresholds_nominal,
            nominal->upper_torque_thresholds_nominal);
  EXPECT_EQ(backend->parameterSnapshot().lower_force_thresholds_nominal,
            nominal->lower_force_thresholds_nominal);
  EXPECT_EQ(backend->parameterSnapshot().upper_force_thresholds_nominal,
            nominal->upper_force_thresholds_nominal);

  auto full = std::make_shared<franka_msgs::srv::SetFullCollisionBehavior::Request>();
  full->lower_torque_thresholds_acceleration = values<7>(base + 130.0);
  full->upper_torque_thresholds_acceleration = values<7>(base + 140.0);
  full->lower_torque_thresholds_nominal = values<7>(base + 150.0);
  full->upper_torque_thresholds_nominal = values<7>(base + 160.0);
  full->lower_force_thresholds_acceleration = values<6>(base + 170.0);
  full->upper_force_thresholds_acceleration = values<6>(base + 180.0);
  full->lower_force_thresholds_nominal = values<6>(base + 190.0);
  full->upper_force_thresholds_nominal = values<6>(base + 200.0);
  expectSuccess(caller.call<franka_msgs::srv::SetFullCollisionBehavior>(
      parameterServiceName(arm_name, "set_full_collision_behavior"), full));
  EXPECT_EQ(backend->parameterSnapshot().lower_torque_thresholds_acceleration,
            full->lower_torque_thresholds_acceleration);
  EXPECT_EQ(backend->parameterSnapshot().upper_torque_thresholds_acceleration,
            full->upper_torque_thresholds_acceleration);
  EXPECT_EQ(backend->parameterSnapshot().lower_torque_thresholds_nominal,
            full->lower_torque_thresholds_nominal);
  EXPECT_EQ(backend->parameterSnapshot().upper_torque_thresholds_nominal,
            full->upper_torque_thresholds_nominal);
  EXPECT_EQ(backend->parameterSnapshot().lower_force_thresholds_acceleration,
            full->lower_force_thresholds_acceleration);
  EXPECT_EQ(backend->parameterSnapshot().upper_force_thresholds_acceleration,
            full->upper_force_thresholds_acceleration);
  EXPECT_EQ(backend->parameterSnapshot().lower_force_thresholds_nominal,
            full->lower_force_thresholds_nominal);
  EXPECT_EQ(backend->parameterSnapshot().upper_force_thresholds_nominal,
            full->upper_force_thresholds_nominal);
  EXPECT_EQ(backend->diagnostics().service_operation, BackendServiceOperation::Idle);
}

void expectAllParameterServicesFail(ServiceCaller& caller, const std::string& arm_name) {
  expectFailure(caller.call<franka_msgs::srv::SetJointStiffness>(
      parameterServiceName(arm_name, "set_joint_stiffness"),
      std::make_shared<franka_msgs::srv::SetJointStiffness::Request>()));
  expectFailure(caller.call<franka_msgs::srv::SetCartesianStiffness>(
      parameterServiceName(arm_name, "set_cartesian_stiffness"),
      std::make_shared<franka_msgs::srv::SetCartesianStiffness::Request>()));
  expectFailure(caller.call<franka_msgs::srv::SetLoad>(
      parameterServiceName(arm_name, "set_load"),
      std::make_shared<franka_msgs::srv::SetLoad::Request>()));
  expectFailure(caller.call<franka_msgs::srv::SetTCPFrame>(
      parameterServiceName(arm_name, "set_tcp_frame"),
      std::make_shared<franka_msgs::srv::SetTCPFrame::Request>()));
  expectFailure(caller.call<franka_msgs::srv::SetStiffnessFrame>(
      parameterServiceName(arm_name, "set_stiffness_frame"),
      std::make_shared<franka_msgs::srv::SetStiffnessFrame::Request>()));
  expectFailure(caller.call<franka_msgs::srv::SetForceTorqueCollisionBehavior>(
      parameterServiceName(arm_name, "set_force_torque_collision_behavior"),
      std::make_shared<franka_msgs::srv::SetForceTorqueCollisionBehavior::Request>()));
  expectFailure(caller.call<franka_msgs::srv::SetFullCollisionBehavior>(
      parameterServiceName(arm_name, "set_full_collision_behavior"),
      std::make_shared<franka_msgs::srv::SetFullCollisionBehavior::Request>()));
}

std::vector<std::string> modeInterfaces(const std::string& arm_name,
                                        const std::string& interface_name) {
  std::vector<std::string> interfaces;
  for (size_t joint = 1; joint <= FrankaMultiHardwareInterface::kNumberOfJoints; ++joint) {
    interfaces.push_back(arm_name + "_joint" + std::to_string(joint) + "/" + interface_name);
  }
  return interfaces;
}

void startMode(FrankaMultiHardwareInterface& hardware,
               const std::string& arm_name,
               const std::string& interface_name) {
  const auto interfaces = modeInterfaces(arm_name, interface_name);
  ASSERT_EQ(hardware.write(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.prepare_command_mode_switch(interfaces, {}),
            hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.perform_command_mode_switch(interfaces, {}),
            hardware_interface::return_type::OK);
}

void setCommandInterfaceValue(std::vector<hardware_interface::CommandInterface>& interfaces,
                              const std::string& full_name,
                              double value) {
  const auto found = std::find_if(
      interfaces.begin(), interfaces.end(),
      [&full_name](const auto& interface) { return interface.get_name() == full_name; });
  ASSERT_NE(found, interfaces.end());
  ASSERT_TRUE(found->template set_value<double>(value));
}

TEST(FrankaMultiHardwareInterfaceServiceTest, ExactUniqueServiceNamesExistAndCleanUp) {
  RclcppScope rclcpp_scope;
  ServiceCaller caller;
  BackendHarness harness;
  auto hardware = std::make_unique<FrankaMultiHardwareInterface>(harness.factory());
  ASSERT_EQ(hardware->on_init(makeHardwareInfo()), CallbackReturn::SUCCESS);

  std::set<std::string> expected_names;
  for (const auto& arm_name : {"panda1", "panda2"}) {
    expected_names.insert(recoveryServiceName(arm_name));
    for (const auto& endpoint : kParameterEndpoints) {
      expected_names.insert(parameterServiceName(arm_name, endpoint));
    }
  }
  ASSERT_EQ(expected_names.size(), 16U);
  auto graph = caller.node()->get_service_names_and_types();
  const auto graph_has_every_service = [&expected_names](const auto& names_and_types) {
    return std::all_of(expected_names.begin(), expected_names.end(),
                       [&names_and_types](const auto& service_name) {
                         return names_and_types.count(service_name) != 0;
                       });
  };
  const auto graph_deadline = std::chrono::steady_clock::now() + 2s;
  while (!graph_has_every_service(graph) && std::chrono::steady_clock::now() < graph_deadline) {
    std::this_thread::sleep_for(10ms);
    graph = caller.node()->get_service_names_and_types();
  }
  for (const auto& service_name : expected_names) {
    const auto found = graph.find(service_name);
    ASSERT_NE(found, graph.end()) << service_name;
    EXPECT_EQ(found->second.size(), 1U) << service_name;
  }
  const auto node_names = caller.node()->get_node_names();
  EXPECT_NE(
      std::find(node_names.begin(), node_names.end(), "/panda1_error_recovery_service_server"),
      node_names.end());

  auto recovery_client =
      caller.makeClient<franka_msgs::srv::ErrorRecovery>(recoveryServiceName("panda1"));
  ASSERT_TRUE(recovery_client->wait_for_service(2s));
  const auto backend1 = harness.backends.at("panda1");
  const auto backend2 = harness.backends.at("panda2");
  hardware.reset();
  EXPECT_TRUE(backend1.expired());
  EXPECT_TRUE(backend2.expired());
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (recovery_client->service_is_ready() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_FALSE(recovery_client->service_is_ready());
}

TEST(FrankaMultiHardwareInterfaceParameterServiceTest,
     AllSevenEndpointsCaptureExactRequestsInactiveAndStateReadingForBothArms) {
  RclcppScope rclcpp_scope;
  ServiceCaller caller;
  BackendHarness harness;
  FrankaMultiHardwareInterface hardware(harness.factory());
  ASSERT_EQ(hardware.on_init(makeHardwareInfo()), CallbackReturn::SUCCESS);
  exerciseAllParameterServices(caller, "panda1", harness.backend("panda1"), 1000.0);
  exerciseAllParameterServices(caller, "panda2", harness.backend("panda2"), 2000.0);

  ASSERT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  exerciseAllParameterServices(caller, "panda1", harness.backend("panda1"), 3000.0);
  exerciseAllParameterServices(caller, "panda2", harness.backend("panda2"), 4000.0);
  EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST(FrankaMultiHardwareInterfaceParameterServiceTest,
     ControllingAndFaultedArmsRejectWhileTheOtherArmRemainsIndependent) {
  RclcppScope rclcpp_scope;
  ServiceCaller caller;
  BackendHarness harness;
  FrankaMultiHardwareInterface hardware(harness.factory());
  ASSERT_EQ(hardware.on_init(makeHardwareInfo()), CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  auto request = std::make_shared<franka_msgs::srv::SetJointStiffness::Request>();
  request->joint_stiffness = values<7>(10.0);

  startMode(hardware, "panda1", "effort");
  expectAllParameterServicesFail(caller, "panda1");
  expectSuccess(caller.call<franka_msgs::srv::SetJointStiffness>(
      parameterServiceName("panda2", "set_joint_stiffness"), request));
  EXPECT_EQ(harness.backend("panda1")->diagnostics().service_operation,
            BackendServiceOperation::Idle);
  const auto effort = modeInterfaces("panda1", "effort");
  ASSERT_EQ(hardware.prepare_command_mode_switch({}, effort), hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.perform_command_mode_switch({}, effort), hardware_interface::return_type::OK);

  harness.backend("panda2")->injectFaultForTest();
  ASSERT_EQ(hardware.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            hardware_interface::return_type::ERROR);
  expectAllParameterServicesFail(caller, "panda2");
  expectSuccess(caller.call<franka_msgs::srv::SetJointStiffness>(
      parameterServiceName("panda1", "set_joint_stiffness"), request));
  EXPECT_EQ(harness.backend("panda2")->diagnostics().service_operation,
            BackendServiceOperation::Idle);
  EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST(FrankaMultiHardwareInterfaceParameterServiceTest,
     RecoveryParameterAndModeGatesArePerArmAndAlwaysReleaseForTheNextSuccess) {
  RclcppScope rclcpp_scope;
  ServiceCaller caller;
  BackendHarness harness;
  FrankaMultiHardwareInterface hardware(harness.factory());
  ASSERT_EQ(hardware.on_init(makeHardwareInfo()), CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  auto request = std::make_shared<franka_msgs::srv::SetJointStiffness::Request>();
  request->joint_stiffness = values<7>(20.0);

  for (const auto operation :
       {BackendServiceOperation::Parameter, BackendServiceOperation::Recovery,
        BackendServiceOperation::ModeRequest}) {
    for (const auto& held_arm : {"panda1", "panda2"}) {
      const std::string other_arm = held_arm == std::string("panda1") ? "panda2" : "panda1";
      ASSERT_TRUE(harness.backend(held_arm)->holdServiceOperationForTest(operation));
      expectAllParameterServicesFail(caller, held_arm);
      expectSuccess(caller.call<franka_msgs::srv::SetJointStiffness>(
          parameterServiceName(other_arm, "set_joint_stiffness"), request));
      harness.backend(held_arm)->releaseServiceOperationForTest();
      expectSuccess(caller.call<franka_msgs::srv::SetJointStiffness>(
          parameterServiceName(held_arm, "set_joint_stiffness"), request));
      EXPECT_EQ(harness.backend(held_arm)->diagnostics().service_operation,
                BackendServiceOperation::Idle);
    }
  }
  EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST(FrankaMultiHardwareInterfaceParameterServiceTest,
     InjectedFailureReleasesGateAndAFollowingSuccessHasNoStaleError) {
  RclcppScope rclcpp_scope;
  ServiceCaller caller;
  BackendHarness harness;
  harness.configurations.at("panda1").failure = {SyntheticFailurePoint::JointStiffness, 1};
  FrankaMultiHardwareInterface hardware(harness.factory());
  ASSERT_EQ(hardware.on_init(makeHardwareInfo()), CallbackReturn::SUCCESS);
  auto request = std::make_shared<franka_msgs::srv::SetJointStiffness::Request>();
  request->joint_stiffness = values<7>(30.0);

  expectFailure(caller.call<franka_msgs::srv::SetJointStiffness>(
      parameterServiceName("panda1", "set_joint_stiffness"), request));
  EXPECT_EQ(harness.backend("panda1")->diagnostics().service_operation,
            BackendServiceOperation::Idle);
  expectSuccess(caller.call<franka_msgs::srv::SetJointStiffness>(
      parameterServiceName("panda2", "set_joint_stiffness"), request));
  expectSuccess(caller.call<franka_msgs::srv::SetJointStiffness>(
      parameterServiceName("panda1", "set_joint_stiffness"), request));
  EXPECT_EQ(harness.backend("panda1")->parameterSnapshot().joint_stiffness,
            request->joint_stiffness);
}

TEST(FrankaMultiHardwareInterfaceRecoveryServiceTest,
     ActiveRecoveryIsPerArmNeverResumesEffortOrVelocityAndReadLazilyClearsLatch) {
  RclcppScope rclcpp_scope;
  ServiceCaller caller;
  for (const uint8_t arm_slot : {1, 2}) {
    SCOPED_TRACE(static_cast<int>(arm_slot));
    BackendHarness harness;
    FrankaMultiHardwareInterface hardware(harness.factory());
    ASSERT_EQ(hardware.on_init(makeHardwareInfo()), CallbackReturn::SUCCESS);
    ASSERT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    const auto arm_name = "panda" + std::to_string(arm_slot);
    startMode(hardware, arm_name, arm_slot == 1 ? "effort" : "velocity");
    ASSERT_NE(harness.backend(arm_name)->requestedControlMode(), ControlMode::None);
    harness.backend(arm_name)->injectFaultForTest();
    ASSERT_EQ(hardware.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
              hardware_interface::return_type::ERROR);
    EXPECT_EQ(hardware.globalFaultDiagnostic().origin_arm_slot, arm_slot);
    EXPECT_EQ(hardware.globalFaultDiagnostic().cause, GlobalFaultCause::BackendFault);

    auto response = caller.call<franka_msgs::srv::ErrorRecovery>(
        recoveryServiceName(arm_name),
        std::make_shared<franka_msgs::srv::ErrorRecovery::Request>());
    expectSuccess(response);
    const auto diagnostics = harness.backend(arm_name)->diagnostics();
    EXPECT_FALSE(harness.backend(arm_name)->hasFault());
    EXPECT_EQ(diagnostics.worker_state, BackendWorkerState::Running);
    EXPECT_FALSE(diagnostics.stopped);
    EXPECT_EQ(diagnostics.service_operation, BackendServiceOperation::Idle);
    EXPECT_EQ(diagnostics.requested_mode, ControlMode::None);
    EXPECT_EQ(diagnostics.active_mode, ControlMode::None);
    EXPECT_TRUE(hardware.globalFaultDiagnostic().latched());
    EXPECT_EQ(hardware.write(rclcpp::Time(0), rclcpp::Duration(0, 0)),
              hardware_interface::return_type::ERROR);
    EXPECT_EQ(hardware.prepare_command_mode_switch(modeInterfaces(arm_name, "effort"), {}),
              hardware_interface::return_type::ERROR);
    const auto other_arm = arm_slot == 1 ? std::string("panda2") : std::string("panda1");
    ASSERT_TRUE(harness.backend(other_arm)->holdServiceOperationForTest(
        BackendServiceOperation::Parameter));
    EXPECT_EQ(hardware.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
              hardware_interface::return_type::ERROR);
    EXPECT_TRUE(hardware.globalFaultDiagnostic().latched());
    harness.backend(other_arm)->releaseServiceOperationForTest();
    ASSERT_EQ(hardware.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
              hardware_interface::return_type::OK);
    EXPECT_FALSE(hardware.globalFaultDiagnostic().latched());
    EXPECT_EQ(harness.backend(arm_name)->requestedControlMode(), ControlMode::None);
    EXPECT_EQ(harness.backend(arm_name)->activeControlMode(), ControlMode::None);
    EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  }
}

TEST(FrankaMultiHardwareInterfaceRecoveryServiceTest,
     InactiveRecoveryRemainsStoppedAndSafeActivationLazilyClearsLatch) {
  RclcppScope rclcpp_scope;
  ServiceCaller caller;
  BackendHarness harness;
  FrankaMultiHardwareInterface hardware(harness.factory());
  ASSERT_EQ(hardware.on_init(makeHardwareInfo()), CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  startMode(hardware, "panda2", "velocity");
  harness.backend("panda2")->injectFaultForTest();
  ASSERT_EQ(hardware.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            hardware_interface::return_type::ERROR);
  ASSERT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  expectSuccess(caller.call<franka_msgs::srv::ErrorRecovery>(
      recoveryServiceName("panda2"), std::make_shared<franka_msgs::srv::ErrorRecovery::Request>()));
  const auto recovered = harness.backend("panda2")->diagnostics();
  EXPECT_TRUE(recovered.stopped);
  EXPECT_EQ(recovered.worker_state, BackendWorkerState::Stopped);
  EXPECT_EQ(recovered.requested_mode, ControlMode::None);
  EXPECT_EQ(recovered.active_mode, ControlMode::None);
  EXPECT_TRUE(hardware.globalFaultDiagnostic().latched());

  ASSERT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  EXPECT_FALSE(hardware.globalFaultDiagnostic().latched());
  EXPECT_EQ(harness.backend("panda2")->requestedControlMode(), ControlMode::None);
  EXPECT_EQ(harness.backend("panda2")->activeControlMode(), ControlMode::None);
  EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST(FrankaMultiHardwareInterfaceRecoveryServiceTest,
     OneArmRecoveryCannotClearOtherFaultAndFailedRecoveryStaysLatchedObservable) {
  RclcppScope rclcpp_scope;
  ServiceCaller caller;
  {
    BackendHarness harness;
    FrankaMultiHardwareInterface hardware(harness.factory());
    ASSERT_EQ(hardware.on_init(makeHardwareInfo()), CallbackReturn::SUCCESS);
    ASSERT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    harness.backend("panda1")->injectFaultForTest();
    harness.backend("panda2")->injectFaultForTest();
    ASSERT_EQ(hardware.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
              hardware_interface::return_type::ERROR);
    expectSuccess(caller.call<franka_msgs::srv::ErrorRecovery>(
        recoveryServiceName("panda1"),
        std::make_shared<franka_msgs::srv::ErrorRecovery::Request>()));
    EXPECT_FALSE(harness.backend("panda1")->hasFault());
    EXPECT_TRUE(harness.backend("panda2")->hasFault());
    EXPECT_EQ(hardware.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
              hardware_interface::return_type::ERROR);
    EXPECT_TRUE(hardware.globalFaultDiagnostic().latched());
    expectSuccess(caller.call<franka_msgs::srv::ErrorRecovery>(
        recoveryServiceName("panda2"),
        std::make_shared<franka_msgs::srv::ErrorRecovery::Request>()));
    EXPECT_EQ(hardware.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
              hardware_interface::return_type::OK);
    EXPECT_FALSE(hardware.globalFaultDiagnostic().latched());
    EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  }

  {
    BackendHarness harness;
    harness.configurations.at("panda1").failure = {SyntheticFailurePoint::Recovery, 1};
    FrankaMultiHardwareInterface hardware(harness.factory());
    ASSERT_EQ(hardware.on_init(makeHardwareInfo()), CallbackReturn::SUCCESS);
    ASSERT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    harness.backend("panda1")->injectFaultForTest();
    ASSERT_EQ(hardware.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
              hardware_interface::return_type::ERROR);
    expectFailure(caller.call<franka_msgs::srv::ErrorRecovery>(
        recoveryServiceName("panda1"),
        std::make_shared<franka_msgs::srv::ErrorRecovery::Request>()));
    EXPECT_TRUE(harness.backend("panda1")->hasFault());
    EXPECT_EQ(harness.backend("panda1")->diagnostics().service_operation,
              BackendServiceOperation::Idle);
    EXPECT_EQ(hardware.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
              hardware_interface::return_type::ERROR);
    expectSuccess(caller.call<franka_msgs::srv::ErrorRecovery>(
        recoveryServiceName("panda1"),
        std::make_shared<franka_msgs::srv::ErrorRecovery::Request>()));
    EXPECT_EQ(hardware.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
              hardware_interface::return_type::OK);
    EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  }
}

TEST(FrankaMultiHardwareInterfaceRecoveryServiceTest,
     RecoveryNeverClearsReadInvalidStateOrInvalidCommandLatches) {
  RclcppScope rclcpp_scope;
  ServiceCaller caller;
  enum class LocalFault : uint8_t { ReadFailure, InvalidState, InvalidCommand };
  for (const auto local_fault :
       {LocalFault::ReadFailure, LocalFault::InvalidState, LocalFault::InvalidCommand}) {
    SCOPED_TRACE(static_cast<int>(local_fault));
    BackendHarness harness;
    if (local_fault == LocalFault::ReadFailure) {
      harness.controls.at("panda1")->throw_read_call = 3;
    } else if (local_fault == LocalFault::InvalidState) {
      harness.configurations.at("panda1").failure = {SyntheticFailurePoint::InvalidNanState, 3};
    }
    FrankaMultiHardwareInterface hardware(harness.factory());
    ASSERT_EQ(hardware.on_init(makeHardwareInfo()), CallbackReturn::SUCCESS);
    ASSERT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    if (local_fault == LocalFault::InvalidCommand) {
      auto commands = hardware.export_command_interfaces();
      setCommandInterfaceValue(commands, "panda1_joint1/effort",
                               std::numeric_limits<double>::quiet_NaN());
      ASSERT_EQ(hardware.write(rclcpp::Time(0), rclcpp::Duration(0, 0)),
                hardware_interface::return_type::ERROR);
    } else {
      ASSERT_EQ(hardware.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
                hardware_interface::return_type::ERROR);
    }
    const auto original = hardware.globalFaultDiagnostic();
    ASSERT_NE(original.cause, GlobalFaultCause::BackendFault);
    expectFailure(caller.call<franka_msgs::srv::ErrorRecovery>(
        recoveryServiceName("panda1"),
        std::make_shared<franka_msgs::srv::ErrorRecovery::Request>()));
    EXPECT_EQ(hardware.read(rclcpp::Time(0), rclcpp::Duration(0, 0)),
              hardware_interface::return_type::ERROR);
    EXPECT_EQ(hardware.globalFaultDiagnostic().origin_arm_slot, original.origin_arm_slot);
    EXPECT_EQ(hardware.globalFaultDiagnostic().cause, original.cause);
    ASSERT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    ASSERT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    EXPECT_FALSE(hardware.globalFaultDiagnostic().latched());
    EXPECT_EQ(hardware.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  }
}

}  // namespace
}  // namespace franka_hardware
