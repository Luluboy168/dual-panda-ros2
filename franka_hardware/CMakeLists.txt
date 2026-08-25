cmake_minimum_required(VERSION 3.5)
project(franka_hardware)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Default to release
set(default_build_type "Release")
# Respect the build type from user and ide
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
      STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror=unused-variable)
endif()

option(CHECK_TIDY "Adds clang-tidy tests" OFF)
option(BUILD_MUJOCO "Build the MuJoCo simulation hardware interface" OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(diagnostic_updater REQUIRED)
if(BUILD_MUJOCO)
find_package(mujoco_ros2_control REQUIRED)
endif()
find_package(std_srvs REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Franka REQUIRED) # Franka_DIR will provide the path to the libfranka installation
find_package(Boost COMPONENTS thread chrono REQUIRED)

# Mujoco dependencies
# set(OpenGL_GL_PREFERENCE GLVND)
# find_package(glfw3 3.3 REQUIRED)     # GLFW3 here,
# find_package(mujoco REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

set(FRANKA_SOURCE_COMMIT "unknown")
set(FRANKA_SOURCE_DIRTY false)
find_package(Git QUIET)
if(Git_FOUND)
  execute_process(
    COMMAND "${GIT_EXECUTABLE}" rev-parse --verify HEAD
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    RESULT_VARIABLE FRANKA_GIT_COMMIT_RESULT
    OUTPUT_VARIABLE FRANKA_GIT_COMMIT_CANDIDATE
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )
  if(FRANKA_GIT_COMMIT_RESULT EQUAL 0 AND
  FRANKA_GIT_COMMIT_CANDIDATE MATCHES "^[0-9a-f][0-9a-f]*$" AND
  FRANKA_GIT_COMMIT_CANDIDATE MATCHES "^........................................$")
    set(FRANKA_SOURCE_COMMIT "${FRANKA_GIT_COMMIT_CANDIDATE}")
  endif()
  execute_process(
    COMMAND "${GIT_EXECUTABLE}" status --porcelain --untracked-files=normal
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    RESULT_VARIABLE FRANKA_GIT_STATUS_RESULT
    OUTPUT_VARIABLE FRANKA_GIT_STATUS_OUTPUT
    ERROR_QUIET
  )
  if(FRANKA_GIT_STATUS_RESULT EQUAL 0 AND NOT FRANKA_GIT_STATUS_OUTPUT STREQUAL "")
    set(FRANKA_SOURCE_DIRTY true)
  endif()
endif()

set(FRANKA_ROS_DISTRO "unknown")
if(DEFINED ENV{ROS_DISTRO} AND "$ENV{ROS_DISTRO}" MATCHES "^[a-z][a-z0-9_]*$")
  string(LENGTH "$ENV{ROS_DISTRO}" FRANKA_ROS_DISTRO_LENGTH)
  if(FRANKA_ROS_DISTRO_LENGTH LESS_EQUAL 32)
    set(FRANKA_ROS_DISTRO "$ENV{ROS_DISTRO}")
  endif()
endif()

function(franka_sanitize_version OUTPUT_VARIABLE CANDIDATE)
  if("${CANDIDATE}" MATCHES "^[0-9]+\\.[0-9]+\\.[0-9]+$")
    set("${OUTPUT_VARIABLE}" "${CANDIDATE}" PARENT_SCOPE)
  else()
    set("${OUTPUT_VARIABLE}" "unknown" PARENT_SCOPE)
  endif()
endfunction()

franka_sanitize_version(FRANKA_RCLCPP_VERSION "${rclcpp_VERSION}")
franka_sanitize_version(FRANKA_ROS2_CONTROL_VERSION "${hardware_interface_VERSION}")
franka_sanitize_version(FRANKA_LIBFRANKA_VERSION "${Franka_VERSION}")

configure_file(
  cmake/build_provenance.hpp.in
  generated/franka_hardware/build_provenance.hpp
  @ONLY
)

###### Real robot ###############
add_library(franka_hardware
        SHARED
        src/real/franka_multi_hardware_interface.cpp
        src/real/franka_hardware_diagnostics_node.cpp
        src/real/real_franka_arm_backend.cpp
        src/real/robot.cpp
        src/real/franka_param_service_server.cpp
        src/real/franka_error_recovery_service_server.cpp
        src/common/franka_executor.cpp
        src/common/control_mode.cpp
        src/common/helper_functions.cpp
)

target_include_directories(
        franka_hardware
        PRIVATE
        include
        ${CMAKE_CURRENT_BINARY_DIR}/generated
        ${EIGEN3_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIRS}
)

ament_target_dependencies(
        franka_hardware
        hardware_interface
        diagnostic_updater
        Franka
        pluginlib
        rclcpp
        rclcpp_action
        rclcpp_components
        franka_msgs
        std_srvs
)

target_link_libraries(
        franka_hardware
        Boost::thread
        Boost::chrono
        ${YAML_CPP_LIBRARIES}
)

pluginlib_export_plugin_description_file(hardware_interface franka_hardware.xml)

########## Mujoco simulation robot ##########
if(BUILD_MUJOCO)
add_library(
  franka_mj_hardware
  SHARED
  src/sim/franka_mj_hardware_system.cpp
  src/sim/robot_sim.cpp
  src/sim/gripper_sim_action_server.cpp
  src/common/franka_executor.cpp
  src/common/control_mode.cpp
  src/common/helper_functions.cpp
)

target_compile_features(franka_mj_hardware PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(
  franka_mj_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

ament_target_dependencies(
  franka_mj_hardware
  "mujoco_ros2_control"
  "hardware_interface"
  "rclcpp"
  "rclcpp_components"
  "rclcpp_lifecycle"
  "rclcpp_action"
  "pluginlib"
  Franka
  franka_msgs
  std_srvs
)

target_link_libraries(
  franka_mj_hardware
  Boost::thread
  Boost::chrono
  ${YAML_CPP_LIBRARIES}
)

pluginlib_export_plugin_description_file(mujoco_ros2_control franka_mj_hardware.xml)
endif()

# Install scripts
install(
        TARGETS franka_hardware
        DESTINATION lib
)

install(
        FILES config/franka_multi_hardware_interface_contract.json
        DESTINATION share/${PROJECT_NAME}/config
)
if(BUILD_MUJOCO)
install(
        TARGETS franka_mj_hardware
        DESTINATION lib
)
endif()
install(
        DIRECTORY include/
        DESTINATION include
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(ament_cmake_clang_format REQUIRED)
    find_package(ament_cmake_copyright REQUIRED)
    find_package(ament_cmake_cppcheck REQUIRED)
    find_package(ament_cmake_flake8 REQUIRED)
    find_package(ament_cmake_lint_cmake REQUIRED)
    find_package(ament_cmake_pep257 REQUIRED)
    find_package(ament_cmake_xmllint REQUIRED)
    find_package(diagnostic_msgs REQUIRED)

    set(CPP_DIRECTORIES src include)
    ament_clang_format(CONFIG_FILE ../.clang-format ${CPP_DIRECTORIES})
    ament_copyright(${CPP_DIRECTORIES} package.xml)
    ament_cppcheck(${CPP_DIRECTORIES})
    ament_lint_cmake(CMakeLists.txt)
    ament_flake8()
    ament_pep257()
    ament_xmllint()
    ament_add_gtest(control_loop_worker_test test/control_loop_worker_test.cpp)
    target_include_directories(control_loop_worker_test PRIVATE include)
    target_link_libraries(control_loop_worker_test franka_hardware)
    ament_add_gtest(command_mode_switch_planner_test test/command_mode_switch_planner_test.cpp)
    target_include_directories(command_mode_switch_planner_test PRIVATE include)
    target_link_libraries(command_mode_switch_planner_test franka_hardware)
    ament_add_gtest(spsc_ring_buffer_test test/spsc_ring_buffer_test.cpp)
    target_include_directories(spsc_ring_buffer_test PRIVATE include)
    ament_add_gtest(robot_command_test test/robot_command_test.cpp)
    target_include_directories(robot_command_test PRIVATE include)
    ament_target_dependencies(robot_command_test Franka)
    ament_add_gtest(backend_factory_seam_test test/backend_factory_seam_test.cpp)
    target_include_directories(backend_factory_seam_test PRIVATE include)
    target_link_libraries(backend_factory_seam_test franka_hardware)
    add_library(synthetic_franka_arm_backend_support STATIC
        test/support/synthetic_franka_arm_backend.cpp
        test/support/synthetic_model.cpp
        test/support/synthetic_replay.cpp
    )
    target_include_directories(synthetic_franka_arm_backend_support PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/test
    )
    ament_target_dependencies(synthetic_franka_arm_backend_support Franka franka_msgs)
    ament_add_gtest(synthetic_franka_arm_backend_test
        test/test_synthetic_franka_arm_backend.cpp
    )
    target_link_libraries(synthetic_franka_arm_backend_test
        franka_hardware
        synthetic_franka_arm_backend_support
    )
    ament_target_dependencies(synthetic_franka_arm_backend_test Franka franka_msgs)
    ament_add_gtest(franka_multi_hardware_interface_lifecycle_test
        test/franka_multi_hardware_interface_lifecycle_test.cpp
    )
    target_compile_definitions(franka_multi_hardware_interface_lifecycle_test PRIVATE
        FRANKA_HARDWARE_INTERFACE_CONTRACT_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/franka_multi_hardware_interface_contract.json"
    )
    target_link_libraries(franka_multi_hardware_interface_lifecycle_test
        franka_hardware
        synthetic_franka_arm_backend_support
        ${YAML_CPP_LIBRARIES}
    )
    ament_target_dependencies(franka_multi_hardware_interface_lifecycle_test Franka franka_msgs)
    ament_add_gtest(franka_multi_hardware_interface_mode_fault_test
        test/franka_multi_hardware_interface_mode_fault_test.cpp
    )
    target_link_libraries(franka_multi_hardware_interface_mode_fault_test
        franka_hardware
        synthetic_franka_arm_backend_support
    )
    ament_target_dependencies(franka_multi_hardware_interface_mode_fault_test Franka franka_msgs)
    ament_add_gtest(franka_multi_hardware_interface_service_test
        test/franka_multi_hardware_interface_service_test.cpp
    )
    target_link_libraries(franka_multi_hardware_interface_service_test
        franka_hardware
        synthetic_franka_arm_backend_support
    )
    ament_target_dependencies(franka_multi_hardware_interface_service_test Franka franka_msgs)
    ament_add_gtest(franka_hardware_diagnostics_test
        test/franka_hardware_diagnostics_test.cpp
    )
    target_link_libraries(franka_hardware_diagnostics_test
        franka_hardware
        synthetic_franka_arm_backend_support
    )
    ament_target_dependencies(franka_hardware_diagnostics_test
        diagnostic_msgs
        diagnostic_updater
        Franka
        franka_msgs
    )
    ament_add_gtest(synthetic_replay_diagnostics_test
        test/synthetic_replay_diagnostics_test.cpp
    )
    target_compile_definitions(synthetic_replay_diagnostics_test PRIVATE
        "SYNTHETIC_REPLAY_FIXTURE_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/multipanda_synthetic_replay_v1.csv\""
    )
    target_link_libraries(synthetic_replay_diagnostics_test
        franka_hardware
        synthetic_franka_arm_backend_support
    )
    ament_target_dependencies(synthetic_replay_diagnostics_test
        diagnostic_msgs
        diagnostic_updater
        Franka
        franka_msgs
    )
    if(CHECK_TIDY)
        find_package(ament_cmake_clang_tidy REQUIRED)
        set(ament_cmake_clang_tidy_CONFIG_FILE ../.clang-tidy)
        ament_clang_tidy(${CMAKE_BINARY_DIR})
    endif()
endif()

ament_export_include_directories(
        include
)
ament_export_libraries(
        franka_hardware
)
ament_export_dependencies(
        hardware_interface
        diagnostic_updater
        pluginlib
        rclcpp
)
if(BUILD_MUJOCO)
ament_export_libraries(
        franka_mj_hardware
)
ament_export_dependencies(
        mujoco_ros2_control
)
endif()

ament_package()
