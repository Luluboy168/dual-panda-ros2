cmake_minimum_required(VERSION 3.5)
project(franka_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(
    test/dual_fake_hardware_smoke.launch_test.py
    TIMEOUT 90
  )
endif()

ament_package()
