cmake_minimum_required(VERSION 3.5)
project(franka_example_controllers)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(CHECK_TIDY "Adds clang-tidy tests" OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(franka_hardware REQUIRED)
find_package(franka_semantic_components REQUIRED)
find_package(generate_parameter_library)
find_package(franka_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(
        ${PROJECT_NAME}
        SHARED
        src/comless/gravity_compensation_example_controller.cpp
        src/comless/joint_impedance_example_controller.cpp
        src/comless/cartesian_impedance_example_controller.cpp
        src/comless/move_to_start_example_controller.cpp
        src/comless/joint_position_example_controller.cpp
        src/comless/joint_velocity_example_controller.cpp
        src/comless/cartesian_velocity_example_controller.cpp
        src/comless/motion_generator.cpp
        src/comless/model_example_controller.cpp
        src/comless/multi_joint_impedance_example_controller.cpp
        src/comless/dual_joint_velocity_example_controller.cpp
        src/dual_arm_joint_hold_controller.cpp
        src/dual_arm_joint_impedance_controller.cpp
        src/dual_arm_joint_velocity_controller.cpp

        src/subscriber/cartesian_impedance_controller.cpp
        src/subscriber/joint_impedance_controller.cpp
)


target_include_directories(
        ${PROJECT_NAME}
        PUBLIC
        include
        ${EIGEN3_INCLUDE_DIRS}
        PRIVATE
        src
)
ament_target_dependencies(
        ${PROJECT_NAME}
        controller_interface
        control_msgs
        hardware_interface
        franka_hardware
        pluginlib
        rclcpp
        rclcpp_lifecycle
        realtime_tools
        std_srvs
        trajectory_msgs
        franka_semantic_components
)

generate_parameter_library(franka_example_controllers_parameters src/comless/model_example_controller_parameters.yaml)

pluginlib_export_plugin_description_file(
  controller_interface franka_example_controllers.xml)

install(
        TARGETS
        ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

install(
        DIRECTORY include/
        DESTINATION include
)

install(
        DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    find_package(ament_cmake_clang_format REQUIRED)
    find_package(ament_cmake_copyright REQUIRED)
    find_package(ament_cmake_cppcheck REQUIRED)
    find_package(ament_cmake_flake8 REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(ament_cmake_lint_cmake REQUIRED)
    find_package(ament_cmake_pep257 REQUIRED)
    find_package(ament_cmake_xmllint REQUIRED)
    find_package(controller_manager REQUIRED)
    find_package(controller_manager_msgs REQUIRED)
    find_package(franka_robot_state_broadcaster REQUIRED)
    find_package(yaml-cpp REQUIRED)

    set(CPP_DIRECTORIES src include)
    ament_clang_format(CONFIG_FILE ../.clang-format ${CPP_DIRECTORIES})
    ament_copyright(src ${CPP_DIRECTORIES} package.xml)
    ament_cppcheck(${CPP_DIRECTORIES})
    ament_lint_cmake(CMakeLists.txt)
    ament_flake8()
    ament_pep257()
    ament_xmllint()

    add_test(
        NAME no_eigen_map_on_temporary
        COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/test/test_no_eigen_map_on_temporary.py
            --source-root ${CMAKE_CURRENT_SOURCE_DIR}
    )
    ament_add_gtest(eigen_array_map_test test/eigen_array_map_test.cpp)
    target_include_directories(eigen_array_map_test PRIVATE src ${EIGEN3_INCLUDE_DIRS})

    ament_add_gtest(
        panda_joint_limits_consistency_test
        test/panda_joint_limits_consistency_test.cpp
    )
    target_compile_definitions(panda_joint_limits_consistency_test PRIVATE
        "PANDA_JOINT_LIMIT_POLICY_FILE=\"${CMAKE_CURRENT_SOURCE_DIR}/config/panda_joint_limits_v1.yaml\""
        "PANDA_ARM_XACRO_FILE=\"${CMAKE_CURRENT_SOURCE_DIR}/../franka_description/robots/common/panda_arm.xacro\""
        "PANDA_PINNED_LIBFRANKA_VERSION=\"${Franka_VERSION}\""
    )
    target_link_libraries(panda_joint_limits_consistency_test
        ${PROJECT_NAME}
        yaml-cpp
    )

    # ROS_DOMAIN_ID allocation for live-node tests (gap G4): every gtest below that
    # builds a live rclcpp node (directly, or via OfflineControllerManagerHarness's
    # ControllerManager("controller_manager", ...)) pins a unique ROS_DOMAIN_ID so
    # parallel test runs cannot collide. See the master table and the Fast-DDS
    # 0-232 range justification in franka_bringup/CMakeLists.txt.
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/test_logs)
    ament_add_gtest(dual_arm_joint_hold_controller_test
        test/dual_arm_joint_hold_controller_test.cpp
        ENV
            FASTDDS_BUILTIN_TRANSPORTS=SHM
            ROS_AUTOMATIC_DISCOVERY_RANGE=OFF
            ROS_DOMAIN_ID=205
            ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_logs
        APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}
        APPEND_LIBRARY_DIRS ${CMAKE_INSTALL_PREFIX}/lib)
    target_link_libraries(dual_arm_joint_hold_controller_test ${PROJECT_NAME})
    ament_target_dependencies(dual_arm_joint_hold_controller_test
        controller_interface
        franka_hardware
        hardware_interface
        pluginlib
        rclcpp)

    ament_add_gtest(dual_arm_joint_velocity_controller_test
        test/dual_arm_joint_velocity_controller_test.cpp
        ENV
            FASTDDS_BUILTIN_TRANSPORTS=SHM
            ROS_AUTOMATIC_DISCOVERY_RANGE=OFF
            ROS_DOMAIN_ID=206
            ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_logs
        APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}
        APPEND_LIBRARY_DIRS ${CMAKE_INSTALL_PREFIX}/lib)
    target_include_directories(dual_arm_joint_velocity_controller_test PRIVATE src)
    target_link_libraries(dual_arm_joint_velocity_controller_test ${PROJECT_NAME})
    ament_target_dependencies(dual_arm_joint_velocity_controller_test
        control_msgs
        controller_interface
        hardware_interface
        pluginlib
        rclcpp
        realtime_tools
        std_srvs)

    ament_add_gtest(dual_arm_joint_impedance_controller_test
        test/dual_arm_joint_impedance_controller_test.cpp
        ENV
            FASTDDS_BUILTIN_TRANSPORTS=SHM
            ROS_AUTOMATIC_DISCOVERY_RANGE=OFF
            ROS_DOMAIN_ID=207
            ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_logs
        APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}
        APPEND_LIBRARY_DIRS ${CMAKE_INSTALL_PREFIX}/lib)
    target_include_directories(dual_arm_joint_impedance_controller_test PRIVATE src)
    target_link_libraries(dual_arm_joint_impedance_controller_test ${PROJECT_NAME})
    ament_target_dependencies(dual_arm_joint_impedance_controller_test
        controller_interface
        franka_hardware
        hardware_interface
        pluginlib
        rclcpp
        realtime_tools
        std_srvs
        trajectory_msgs)

    # One-arm-mode (arm_count == 1) coverage. Separate binaries from the dual-mode tests above so
    # neither can regress the other; each builds its own arm_count == 1 controller instance and a
    # hardware fixture sized for exactly one arm.
    ament_add_gtest(dual_arm_joint_hold_controller_one_arm_test
        test/dual_arm_joint_hold_controller_one_arm_test.cpp
        ENV
            FASTDDS_BUILTIN_TRANSPORTS=SHM
            ROS_AUTOMATIC_DISCOVERY_RANGE=OFF
            ROS_DOMAIN_ID=214
            ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_logs
        APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}
        APPEND_LIBRARY_DIRS ${CMAKE_INSTALL_PREFIX}/lib)
    target_link_libraries(dual_arm_joint_hold_controller_one_arm_test ${PROJECT_NAME})
    ament_target_dependencies(dual_arm_joint_hold_controller_one_arm_test
        controller_interface
        franka_hardware
        hardware_interface
        pluginlib
        rclcpp)

    ament_add_gtest(dual_arm_joint_velocity_controller_one_arm_test
        test/dual_arm_joint_velocity_controller_one_arm_test.cpp
        ENV
            FASTDDS_BUILTIN_TRANSPORTS=SHM
            ROS_AUTOMATIC_DISCOVERY_RANGE=OFF
            ROS_DOMAIN_ID=215
            ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_logs
        APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}
        APPEND_LIBRARY_DIRS ${CMAKE_INSTALL_PREFIX}/lib)
    target_include_directories(dual_arm_joint_velocity_controller_one_arm_test PRIVATE src)
    target_link_libraries(dual_arm_joint_velocity_controller_one_arm_test ${PROJECT_NAME})
    ament_target_dependencies(dual_arm_joint_velocity_controller_one_arm_test
        control_msgs
        controller_interface
        hardware_interface
        pluginlib
        rclcpp
        realtime_tools
        std_srvs)

    ament_add_gtest(dual_arm_joint_impedance_controller_one_arm_test
        test/dual_arm_joint_impedance_controller_one_arm_test.cpp
        ENV
            FASTDDS_BUILTIN_TRANSPORTS=SHM
            ROS_AUTOMATIC_DISCOVERY_RANGE=OFF
            ROS_DOMAIN_ID=216
            ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_logs
        APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}
        APPEND_LIBRARY_DIRS ${CMAKE_INSTALL_PREFIX}/lib)
    target_include_directories(dual_arm_joint_impedance_controller_one_arm_test PRIVATE src)
    target_link_libraries(dual_arm_joint_impedance_controller_one_arm_test ${PROJECT_NAME})
    ament_target_dependencies(dual_arm_joint_impedance_controller_one_arm_test
        controller_interface
        franka_hardware
        hardware_interface
        pluginlib
        rclcpp
        realtime_tools
        std_srvs
        trajectory_msgs)

    ament_add_gtest(production_controller_manager_integration_test
        test/production_controller_manager_integration_test.cpp
        ENV
            FASTDDS_BUILTIN_TRANSPORTS=SHM
            ROS_AUTOMATIC_DISCOVERY_RANGE=OFF
            ROS_DOMAIN_ID=191
            ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_logs
        APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}
        APPEND_LIBRARY_DIRS ${CMAKE_INSTALL_PREFIX}/lib)
    target_compile_definitions(production_controller_manager_integration_test PRIVATE
        "PRODUCTION_CM_TEST_PARAMS_FILE=\"${CMAKE_CURRENT_SOURCE_DIR}/test/production_controller_manager_integration.yaml\""
        "SEMANTIC_STATE_SOURCE_FILE=\"${CMAKE_CURRENT_SOURCE_DIR}/../franka_semantic_components/src/franka_robot_state.cpp\""
        "SEMANTIC_MODEL_SOURCE_FILE=\"${CMAKE_CURRENT_SOURCE_DIR}/../franka_semantic_components/src/franka_robot_model.cpp\""
        "STATE_BROADCASTER_SOURCE_FILE=\"${CMAKE_CURRENT_SOURCE_DIR}/../franka_robot_state_broadcaster/src/franka_robot_state_broadcaster.cpp\""
        "MODEL_BROADCASTER_SOURCE_FILE=\"${CMAKE_CURRENT_SOURCE_DIR}/../franka_robot_state_broadcaster/src/franka_robot_model_broadcaster.cpp\"")
    add_library(franka_offline_test_support STATIC
        test/support/offline_controller_manager_harness.cpp
        test/support/offline_release_stress.cpp
        ../franka_hardware/test/support/synthetic_franka_arm_backend.cpp
        ../franka_hardware/test/support/synthetic_model.cpp
    )
    target_compile_features(franka_offline_test_support PUBLIC cxx_std_17)
    target_compile_definitions(franka_offline_test_support PRIVATE
        "OFFLINE_CM_TEST_PARAMS_FILE=\"${CMAKE_CURRENT_SOURCE_DIR}/test/production_controller_manager_integration.yaml\"")
    target_include_directories(franka_offline_test_support PUBLIC
        test
        ../franka_hardware/test
    )
    target_link_libraries(franka_offline_test_support ${PROJECT_NAME})
    ament_target_dependencies(franka_offline_test_support
        controller_interface
        controller_manager
        controller_manager_msgs
        franka_hardware
        franka_robot_state_broadcaster
        hardware_interface
        lifecycle_msgs
        rclcpp
        rclcpp_lifecycle
    )

    target_link_libraries(production_controller_manager_integration_test
        ${PROJECT_NAME}
        franka_offline_test_support
        yaml-cpp
    )
    ament_target_dependencies(production_controller_manager_integration_test
        control_msgs
        controller_interface
        controller_manager
        controller_manager_msgs
        franka_hardware
        franka_msgs
        franka_robot_state_broadcaster
        hardware_interface
        lifecycle_msgs
        rclcpp
        rclcpp_lifecycle
        std_srvs
        trajectory_msgs)

    add_executable(franka_offline_release_stress
        test/franka_offline_release_stress.cpp
    )
    target_compile_features(franka_offline_release_stress PRIVATE cxx_std_17)
    target_link_libraries(franka_offline_release_stress franka_offline_test_support)
    ament_target_dependencies(franka_offline_release_stress rclcpp)
    add_dependencies(production_controller_manager_integration_test
        franka_offline_release_stress)
    target_compile_definitions(production_controller_manager_integration_test PRIVATE
        "OFFLINE_STRESS_EXECUTABLE=\"$<TARGET_FILE:franka_offline_release_stress>\"")

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
        ${PROJECT_NAME}
)
ament_export_dependencies(
        controller_interface
        control_msgs
        pluginlib
        rclcpp
        rclcpp_lifecycle
        hardware_interface
        franka_hardware
        realtime_tools
        std_srvs
        trajectory_msgs
)
ament_package()
