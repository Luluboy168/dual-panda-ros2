cmake_minimum_required(VERSION 3.5)
project(franka_web)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS scripts/franka_web_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME franka_web_server
)

install(
  DIRECTORY static
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__/*" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_test REQUIRED)

  # The browser UI tests use Node's built-in test runner (available since
  # Node.js 18).  Treat a missing/old runtime as a configuration error so a
  # normal package test cannot silently omit the frontend safety checks.
  find_program(
    FRANKA_WEB_NODE_EXECUTABLE
    NAMES node nodejs
    DOC "Node.js executable used by the franka_web frontend tests"
  )
  if(NOT FRANKA_WEB_NODE_EXECUTABLE)
    message(FATAL_ERROR
      "Node.js 18 or newer is required when BUILD_TESTING is enabled. "
      "Install the declared nodejs test dependency or set "
      "FRANKA_WEB_NODE_EXECUTABLE to a compatible executable."
    )
  endif()
  execute_process(
    COMMAND "${FRANKA_WEB_NODE_EXECUTABLE}" --version
    RESULT_VARIABLE _franka_web_node_version_result
    OUTPUT_VARIABLE _franka_web_node_version
    ERROR_VARIABLE _franka_web_node_version_error
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_STRIP_TRAILING_WHITESPACE
  )
  if(NOT _franka_web_node_version_result EQUAL 0)
    message(FATAL_ERROR
      "Could not run ${FRANKA_WEB_NODE_EXECUTABLE} --version: "
      "${_franka_web_node_version_error}"
    )
  endif()
  string(REGEX MATCH "^v([0-9]+)" _franka_web_node_version_match
    "${_franka_web_node_version}")
  if(NOT _franka_web_node_version_match OR CMAKE_MATCH_1 LESS 18)
    message(FATAL_ERROR
      "Node.js 18 or newer is required for the frontend tests; found "
      "${_franka_web_node_version}."
    )
  endif()

  ament_add_test(
    franka_web_frontend_fence_preview_test
    COMMAND
      "${FRANKA_WEB_NODE_EXECUTABLE}"
      --test
      "${CMAKE_CURRENT_SOURCE_DIR}/test/frontend_fence_preview_test.js"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    TIMEOUT 60
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
  ament_add_test(
    franka_web_frontend_settling_ui_test
    COMMAND
      "${FRANKA_WEB_NODE_EXECUTABLE}"
      --test
      "${CMAKE_CURRENT_SOURCE_DIR}/test/frontend_settling_ui_test.js"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    TIMEOUT 60
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )

  # ---------------------------------------------------------------------
  # ROS_DOMAIN_ID allocation for live-node tests
  #
  # The single master table for this workspace lives in
  # franka_bringup/CMakeLists.txt (reviewed core; IDs 191-218 taken there,
  # hard ceiling 232 for rmw_fastrtps_cpp). That file is not edited from
  # this branch: franka_web claims the IDs below locally, and the session
  # handoff asks a core-permitted session to add the matching rows to the
  # master table.
  #
  #   ID   Package     Test target
  #   ---  ----------  ------------------------------------------
  #   219  franka_web  e2e_fake_dual_state_only_test.py  (Stage 1)
  #   220  franka_web  e2e_fake_motion_mock_test.py      (Stage 2)
  # ---------------------------------------------------------------------

  ament_add_pytest_test(
    franka_web_copyright_test
    test/test_copyright.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    franka_web_flake8_test
    test/test_flake8.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    franka_web_pep257_test
    test/test_pep257.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    franka_web_config_test
    test/test_config.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    franka_web_settling_test
    test/test_settling.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    franka_web_profiles_test
    test/test_profiles.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    franka_web_lock_test
    test/test_lock.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    franka_web_sse_test
    test/test_sse.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    franka_web_launcher_test
    test/test_launcher.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    franka_web_preflight_test
    test/test_preflight.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    franka_web_recording_test
    test/test_recording.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    franka_web_health_test
    test/test_health_projection.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    franka_web_faults_test
    test/test_faults.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    franka_web_session_test
    test/test_session_state_machine.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    franka_web_http_api_test
    test/test_http_api.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    franka_web_review_regressions_test
    test/test_review_regressions.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    franka_web_jog_model_test
    test/test_jog_model.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    franka_web_gains_test
    test/test_gains.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    franka_web_jog_message_contract_test
    test/test_jog_message_contract.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    franka_web_motion_guards_test
    test/test_motion_guards.py
    TIMEOUT 120
  )
  file(MAKE_DIRECTORY
    ${CMAKE_CURRENT_BINARY_DIR}/e2e_motion_mock_ros_home
    ${CMAKE_CURRENT_BINARY_DIR}/e2e_motion_mock_ros_logs
  )
  ament_add_pytest_test(
    franka_web_e2e_fake_motion_mock_test
    test/e2e_fake_motion_mock_test.py
    TIMEOUT 300
    ENV
      FASTDDS_BUILTIN_TRANSPORTS=SHM
      ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
      ROS_DOMAIN_ID=220
      ROS_HOME=${CMAKE_CURRENT_BINARY_DIR}/e2e_motion_mock_ros_home
      ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/e2e_motion_mock_ros_logs
  )
  file(MAKE_DIRECTORY
    ${CMAKE_CURRENT_BINARY_DIR}/e2e_fake_dual_ros_home
    ${CMAKE_CURRENT_BINARY_DIR}/e2e_fake_dual_ros_logs
  )
  ament_add_pytest_test(
    franka_web_e2e_fake_dual_state_only_test
    test/e2e_fake_dual_state_only_test.py
    TIMEOUT 300
    ENV
      FASTDDS_BUILTIN_TRANSPORTS=SHM
      ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
      ROS_DOMAIN_ID=219
      ROS_HOME=${CMAKE_CURRENT_BINARY_DIR}/e2e_fake_dual_ros_home
      ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/e2e_fake_dual_ros_logs
  )
endif()

ament_package()
