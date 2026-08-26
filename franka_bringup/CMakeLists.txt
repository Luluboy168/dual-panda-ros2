cmake_minimum_required(VERSION 3.5)
project(franka_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/franka_rt_preflight.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME franka_rt_preflight
)

install(
  PROGRAMS scripts/franka_joint_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME franka_joint_state_publisher
)

install(
  PROGRAMS scripts/franka_status.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME franka_status
)

install(
  PROGRAMS scripts/franka_validate_controller_config.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME franka_validate_controller_config
)

install(
  PROGRAMS scripts/franka_record.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME franka_record
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  # ---------------------------------------------------------------------
  # ROS_DOMAIN_ID allocation for live-node tests (gap G4)
  #
  # Every test entry point that starts a live rclcpp/rclpy node - a launch
  # test, or a gtest that calls rclcpp::init() - must pin a UNIQUE
  # ROS_DOMAIN_ID via its test-registration ENV, so that concurrent/
  # parallel test runs (Phase 8) cannot collide. Most of these tests bring
  # up a node literally named "controller_manager"; the gtests that build
  # their own rclcpp::Node still open a live DDS participant on whatever
  # domain is in the environment, so they get pinned too.
  #
  # Safe range for this workspace's only installed RMW (rmw_fastrtps_cpp):
  # Fast-DDS computes its RTPS UDP ports as 7400 + 250 * domain_id (see
  # /opt/ros/jazzy/include/fastrtps/fastdds/rtps/common/PortParameters.h),
  # so any domain_id above 232 overflows a 16-bit port and is rejected by
  # Fast-DDS itself ("Calculated port number is too high ... domainId is
  # over 232"). 0-232 is therefore the hard ceiling, not just a convention.
  #
  # This is the single master table - the franka_example_controllers and
  # franka_hardware CMakeLists.txt files each carry a short comment
  # pointing back here. When adding a new live-node test, claim the next
  # free ID below and add a row here.
  #
  #   ID   Package                     Test target
  #   ---  --------------------------  ---------------------------------------------
  #   191  franka_example_controllers  production_controller_manager_integration_test  (pre-existing, 70491ea)
  #   197  franka_bringup              franka_status_localhost_test                     (pre-existing, 70491ea)
  #   200  franka_bringup              single_fake_hardware_smoke.launch_test.py
  #   201  franka_bringup              dual_fake_hardware_smoke.launch_test.py
  #   202  franka_bringup              dual_fake_hardware_controller_switch.launch_test.py
  #   203  franka_bringup              operator_fake_state_only.launch_test.py
  #   204  franka_bringup              dual_safe_controllers_fake_hardware.launch_test.py
  #   205  franka_example_controllers  dual_arm_joint_hold_controller_test
  #   206  franka_example_controllers  dual_arm_joint_velocity_controller_test
  #   207  franka_example_controllers  dual_arm_joint_impedance_controller_test
  #   208  franka_hardware             backend_factory_seam_test
  #   209  franka_hardware             franka_hardware_diagnostics_test
  #   210  franka_hardware             franka_multi_hardware_interface_mode_fault_test
  #   211  franka_hardware             franka_multi_hardware_interface_lifecycle_test
  #   212  franka_hardware             franka_multi_hardware_interface_service_test
  #   213  franka_hardware             synthetic_replay_diagnostics_test
  # ---------------------------------------------------------------------

  ament_add_pytest_test(
    franka_rt_preflight_test
    test/test_rt_preflight.py
    TIMEOUT 30
  )
  ament_add_pytest_test(
    franka_launch_arm_id_validation_test
    test/test_launch_arm_id_validation.py
    TIMEOUT 30
  )
  ament_add_pytest_test(
    franka_controller_config_validator_test
    test/test_controller_config_validator.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    franka_status_test
    test/test_franka_status.py
    TIMEOUT 60
  )
  file(MAKE_DIRECTORY
    ${CMAKE_CURRENT_BINARY_DIR}/status_localhost_ros_home
    ${CMAKE_CURRENT_BINARY_DIR}/status_localhost_ros_logs
  )
  ament_add_pytest_test(
    franka_status_localhost_test
    test/test_franka_status_localhost.py
    TIMEOUT 30
    ENV
      FASTDDS_BUILTIN_TRANSPORTS=SHM
      ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
      ROS_DOMAIN_ID=197
      ROS_HOME=${CMAKE_CURRENT_BINARY_DIR}/status_localhost_ros_home
      ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/status_localhost_ros_logs
  )
  ament_add_pytest_test(
    franka_operator_launch_profiles_test
    test/test_operator_launch_profiles.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    franka_bounded_recorder_test
    test/test_bounded_recorder.py
    TIMEOUT 60
  )
  add_launch_test(
    test/single_fake_hardware_smoke.launch_test.py
    TIMEOUT 90
    ENV
      ROS_DOMAIN_ID=200
  )
  add_launch_test(
    test/dual_fake_hardware_smoke.launch_test.py
    TIMEOUT 90
    ENV
      ROS_DOMAIN_ID=201
  )
  add_launch_test(
    test/dual_fake_hardware_controller_switch.launch_test.py
    TIMEOUT 120
    ENV
      ROS_DOMAIN_ID=202
  )
  file(MAKE_DIRECTORY
    ${CMAKE_CURRENT_BINARY_DIR}/operator_fake_state_only_ros_home
    ${CMAKE_CURRENT_BINARY_DIR}/operator_fake_state_only_ros_logs
  )
  add_launch_test(
    test/operator_fake_state_only.launch_test.py
    TIMEOUT 120
    ENV
      FASTDDS_BUILTIN_TRANSPORTS=SHM
      ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
      ROS_DOMAIN_ID=203
      ROS_HOME=${CMAKE_CURRENT_BINARY_DIR}/operator_fake_state_only_ros_home
      ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/operator_fake_state_only_ros_logs
  )
  file(MAKE_DIRECTORY
    ${CMAKE_CURRENT_BINARY_DIR}/dual_safe_controllers_ros_home
    ${CMAKE_CURRENT_BINARY_DIR}/dual_safe_controllers_ros_logs
  )
  add_launch_test(
    test/dual_safe_controllers_fake_hardware.launch_test.py
    TIMEOUT 180
    ENV
      FASTDDS_BUILTIN_TRANSPORTS=SHM
      ROS_DOMAIN_ID=204
      ROS_HOME=${CMAKE_CURRENT_BINARY_DIR}/dual_safe_controllers_ros_home
      ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/dual_safe_controllers_ros_logs
  )
endif()

ament_package()
