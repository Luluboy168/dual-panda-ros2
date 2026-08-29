# [THROWAWAY] Session C package scaffold; franka_web owns the merged build.
cmake_minimum_required(VERSION 3.8)
project(franka_ghost)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS scripts/franka_ghost_prep.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME franka_ghost_prep
)

install(
  PROGRAMS scripts/franka_ghost_dev_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME franka_ghost_dev_server
)

install(
  DIRECTORY web
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_pytest_test(${PROJECT_NAME}_urdf_export test/test_urdf_export.py)
  ament_add_pytest_test(${PROJECT_NAME}_mesh_convert test/test_mesh_convert.py)
  ament_add_pytest_test(${PROJECT_NAME}_apply_payload test/test_apply_payload.py)
  ament_add_pytest_test(${PROJECT_NAME}_dev_server test/test_dev_server.py)
  ament_add_pytest_test(${PROJECT_NAME}_module_purity test/test_module_purity.py)
  ament_add_pytest_test(${PROJECT_NAME}_browser test/browser/test_browser.py TIMEOUT 60)

  file(MAKE_DIRECTORY
    ${CMAKE_CURRENT_BINARY_DIR}/live_state_bridge_ros_home
    ${CMAKE_CURRENT_BINARY_DIR}/live_state_bridge_ros_logs
  )
  # Domain 223 is reserved for this registered live-node test. See the master
  # allocation table at franka_bringup/CMakeLists.txt:163-209; do not use the
  # Session C interactive domain (82) here.
  add_launch_test(
    test/live/test_live_state_bridge.py
    TIMEOUT 120
    ENV
      FASTDDS_BUILTIN_TRANSPORTS=SHM
      ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
      ROS_DOMAIN_ID=223
      ROS_HOME=${CMAKE_CURRENT_BINARY_DIR}/live_state_bridge_ros_home
      ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/live_state_bridge_ros_logs
  )
endif()

ament_package()
