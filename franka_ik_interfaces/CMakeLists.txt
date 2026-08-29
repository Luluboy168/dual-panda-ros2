# Copyright 2026 The multipanda_ros2 Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.16)
project(franka_ik_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ChainInfo.msg"
  "msg/IkRequest.msg"
  "msg/IkResult.msg"
  "msg/IkSolution.msg"
  "srv/GetChainInfo.srv"
  "srv/SolveIk.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_copyright(CMakeLists.txt msg package.xml srv test)
  ament_flake8(test)
  ament_lint_cmake(CMakeLists.txt)
  ament_pep257(test)
  ament_xmllint()

  ament_add_pytest_test(
    message_field_freeze_test
    test/message_field_freeze_test.py
    ENV
      FRANKA_IK_INTERFACES_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
      PYTHONDONTWRITEBYTECODE=1)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
