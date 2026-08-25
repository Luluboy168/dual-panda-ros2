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
  )
  add_launch_test(
    test/dual_fake_hardware_smoke.launch_test.py
    TIMEOUT 90
  )
  add_launch_test(
    test/dual_fake_hardware_controller_switch.launch_test.py
    TIMEOUT 120
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
      ROS_HOME=${CMAKE_CURRENT_BINARY_DIR}/dual_safe_controllers_ros_home
      ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/dual_safe_controllers_ros_logs
  )
endif()

ament_package()
