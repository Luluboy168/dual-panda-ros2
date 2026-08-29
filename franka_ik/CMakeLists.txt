cmake_minimum_required(VERSION 3.16)
project(franka_ik VERSION 0.1.0)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(franka_ik_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(urdf REQUIRED)

add_library(franka_ik SHARED
  src/forward_kinematics.cpp
  src/numeric_backend.cpp
  src/reachability.cpp
  src/robot_chains.cpp)
target_include_directories(franka_ik
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(franka_ik
  PUBLIC
    kdl_parser::kdl_parser
    orocos-kdl
    urdf::urdf
  PRIVATE
    Eigen3::Eigen)

add_library(franka_ik_service SHARED src/service_node.cpp)
target_link_libraries(franka_ik_service PUBLIC franka_ik)
ament_target_dependencies(franka_ik_service
  PUBLIC
    franka_ik_interfaces
    geometry_msgs
    rcl_interfaces
    rclcpp
    rcutils)

add_executable(franka_ik_service_node src/franka_ik_service_node_main.cpp)
target_link_libraries(franka_ik_service_node PRIVATE franka_ik_service)
ament_target_dependencies(franka_ik_service_node PUBLIC rclcpp)

install(
  TARGETS franka_ik franka_ik_service
  EXPORT export_franka_ik
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(TARGETS franka_ik_service_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(DIRECTORY doc/ DESTINATION share/${PROJECT_NAME}/doc)

if(BUILD_TESTING)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(xacro REQUIRED)
  find_package(yaml-cpp REQUIRED)

  ament_clang_format(CONFIG_FILE ../.clang-format include src test)
  ament_copyright(include launch src test urdf package.xml)
  ament_cppcheck(include src)
  # ament_cppcheck skips itself outright on cppcheck 1.88/2.x unless
  # AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS is present in the environment (see
  # ament_cppcheck/main.py).  Without this the "cppcheck" ctest entry passes while
  # reporting every checked file as skipped, so the package gets zero static-analysis
  # coverage and a reproducer does not obviously notice.  Setting it here keeps the
  # coverage attached to the build rather than to a shell the operator must remember.
  set_tests_properties(cppcheck PROPERTIES
    ENVIRONMENT "AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1")
  ament_flake8(
    launch/franka_ik.launch.py
    test/contract_doc_sync_test.py
    test/data/generate_corpus.py
    test/franka_ik_launch_smoke.launch_test.py
    test/urdf_wrapper_equivalence_test.py)
  ament_lint_cmake(CMakeLists.txt)
  ament_pep257(
    launch/franka_ik.launch.py
    test/contract_doc_sync_test.py
    test/data/generate_corpus.py
    test/franka_ik_launch_smoke.launch_test.py
    test/urdf_wrapper_equivalence_test.py)
  ament_xmllint()

  ament_add_gtest(robot_chains_test test/robot_chains_test.cpp)
  target_link_libraries(robot_chains_test franka_ik)

  ament_add_gtest(joint_limits_match_urdf_test test/joint_limits_match_urdf_test.cpp)
  xacro_add_xacro_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/urdf/panda_ik_single.urdf.xacro
    ${CMAKE_CURRENT_BINARY_DIR}/panda_ik_single.test.urdf
    OUTPUT PANDA_IK_SINGLE_TEST_URDF)
  xacro_add_xacro_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/urdf/panda_ik_dual.urdf.xacro
    ${CMAKE_CURRENT_BINARY_DIR}/panda_ik_dual.test.urdf
    OUTPUT PANDA_IK_DUAL_TEST_URDF)
  # Hand-equipped rendering of the same dual wrapper, so the TIP_HAND_TCP success path can be
  # solved end to end through the service (post-MVP verification finding 3).
  xacro_add_xacro_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/urdf/panda_ik_dual.urdf.xacro
    ${CMAKE_CURRENT_BINARY_DIR}/panda_ik_dual_hand.test.urdf
    REMAP hand_1:=true hand_2:=true
    OUTPUT PANDA_IK_DUAL_HAND_TEST_URDF)
  add_custom_target(franka_ik_test_urdfs
    DEPENDS
      ${PANDA_IK_SINGLE_TEST_URDF}
      ${PANDA_IK_DUAL_TEST_URDF}
      ${PANDA_IK_DUAL_HAND_TEST_URDF})
  add_dependencies(joint_limits_match_urdf_test franka_ik_test_urdfs)
  target_compile_definitions(joint_limits_match_urdf_test PRIVATE
    "PANDA_IK_SINGLE_TEST_URDF=\"${PANDA_IK_SINGLE_TEST_URDF}\""
    "PANDA_IK_DUAL_TEST_URDF=\"${PANDA_IK_DUAL_TEST_URDF}\"")
  target_link_libraries(joint_limits_match_urdf_test franka_ik)

  ament_add_gtest(joint_limits_match_policy_test test/joint_limits_match_policy_test.cpp)
  target_compile_definitions(joint_limits_match_policy_test PRIVATE
    "PANDA_JOINT_LIMIT_POLICY_FILE=\"${CMAKE_CURRENT_SOURCE_DIR}/../franka_example_controllers/config/panda_joint_limits_v1.yaml\"")
  target_link_libraries(joint_limits_match_policy_test franka_ik yaml-cpp)

  ament_add_gtest(forward_kinematics_test test/forward_kinematics_test.cpp)
  add_dependencies(forward_kinematics_test franka_ik_test_urdfs)
  target_compile_definitions(forward_kinematics_test PRIVATE
    "FRANKA_IK_CORPUS_FILE=\"${CMAKE_CURRENT_SOURCE_DIR}/test/data/corpus_v1.json\""
    "PANDA_IK_SINGLE_TEST_URDF=\"${PANDA_IK_SINGLE_TEST_URDF}\"")
  target_link_libraries(forward_kinematics_test franka_ik)

  ament_add_gtest(
    unreachable_classification_test
    test/unreachable_classification_test.cpp)
  add_dependencies(unreachable_classification_test franka_ik_test_urdfs)
  target_compile_definitions(unreachable_classification_test PRIVATE
    "FRANKA_IK_CORPUS_FILE=\"${CMAKE_CURRENT_SOURCE_DIR}/test/data/corpus_v1.json\""
    "PANDA_IK_SINGLE_TEST_URDF=\"${PANDA_IK_SINGLE_TEST_URDF}\""
    "PANDA_IK_DUAL_TEST_URDF=\"${PANDA_IK_DUAL_TEST_URDF}\"")
  target_link_libraries(unreachable_classification_test franka_ik yaml-cpp)

  foreach(numeric_test
      numeric_backend_round_trip_test
      limit_margin_test
      seed_validation_test
      determinism_test
      solve_latency_test)
    if(numeric_test STREQUAL "determinism_test")
      ament_add_gtest(${numeric_test} test/${numeric_test}.cpp TIMEOUT 180)
    else()
      ament_add_gtest(${numeric_test} test/${numeric_test}.cpp)
    endif()
    add_dependencies(${numeric_test} franka_ik_test_urdfs)
    target_compile_definitions(${numeric_test} PRIVATE
      "FRANKA_IK_CORPUS_FILE=\"${CMAKE_CURRENT_SOURCE_DIR}/test/data/corpus_v1.json\""
      "PANDA_IK_SINGLE_TEST_URDF=\"${PANDA_IK_SINGLE_TEST_URDF}\"")
    target_link_libraries(${numeric_test} franka_ik yaml-cpp)
  endforeach()
  set_tests_properties(solve_latency_test PROPERTIES LABELS "performance")

  ament_add_gtest(service_contract_test test/service_contract_test.cpp TIMEOUT 120)
  add_dependencies(service_contract_test franka_ik_test_urdfs)
  target_compile_definitions(service_contract_test PRIVATE
    "PANDA_IK_DUAL_TEST_URDF=\"${PANDA_IK_DUAL_TEST_URDF}\""
    "PANDA_IK_DUAL_HAND_TEST_URDF=\"${PANDA_IK_DUAL_HAND_TEST_URDF}\"")
  target_link_libraries(service_contract_test franka_ik_service)

  ament_add_gtest(parameter_validation_test test/parameter_validation_test.cpp TIMEOUT 120)
  add_dependencies(parameter_validation_test franka_ik_test_urdfs)
  target_compile_definitions(parameter_validation_test PRIVATE
    "PANDA_IK_SINGLE_TEST_URDF=\"${PANDA_IK_SINGLE_TEST_URDF}\"")
  target_link_libraries(parameter_validation_test franka_ik_service)

  ament_add_gtest(service_throughput_test test/service_throughput_test.cpp TIMEOUT 120)
  add_dependencies(service_throughput_test franka_ik_test_urdfs)
  target_compile_definitions(service_throughput_test PRIVATE
    "FRANKA_IK_CORPUS_FILE=\"${CMAKE_CURRENT_SOURCE_DIR}/test/data/corpus_v1.json\""
    "PANDA_IK_SINGLE_TEST_URDF=\"${PANDA_IK_SINGLE_TEST_URDF}\""
    "PANDA_IK_DUAL_TEST_URDF=\"${PANDA_IK_DUAL_TEST_URDF}\"")
  target_link_libraries(service_throughput_test franka_ik_service yaml-cpp)
  set_tests_properties(service_throughput_test PROPERTIES LABELS "performance")

  # Domain allocation: 221 belongs to franka_ik's registered contract/parameter tests and 222
  # belongs to its Stage-6 smoke test.  The reviewed master table remains in franka_bringup.
  file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/test_logs)
  set(franka_ik_service_test_environment
    "FASTDDS_BUILTIN_TRANSPORTS=SHM"
    "ROS_AUTOMATIC_DISCOVERY_RANGE=OFF"
    "ROS_DOMAIN_ID=221"
    "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_logs")
  set_tests_properties(
    service_contract_test
    parameter_validation_test
    service_throughput_test
    PROPERTIES ENVIRONMENT "${franka_ik_service_test_environment}")

  file(MAKE_DIRECTORY
    ${CMAKE_CURRENT_BINARY_DIR}/launch_smoke_ros_home
    ${CMAKE_CURRENT_BINARY_DIR}/launch_smoke_ros_logs)
  add_launch_test(
    test/franka_ik_launch_smoke.launch_test.py
    TIMEOUT 60
    ENV
      FASTDDS_BUILTIN_TRANSPORTS=SHM
      ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
      ROS_DOMAIN_ID=222
      ROS_HOME=${CMAKE_CURRENT_BINARY_DIR}/launch_smoke_ros_home
      ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/launch_smoke_ros_logs
      PYTHONDONTWRITEBYTECODE=1)

  ament_add_pytest_test(
    contract_doc_sync_test
    test/contract_doc_sync_test.py
    TIMEOUT 30
    ENV
      FRANKA_IK_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
      FRANKA_IK_INTERFACES_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../franka_ik_interfaces
      PYTHONDONTWRITEBYTECODE=1)

  ament_add_pytest_test(
    urdf_wrapper_equivalence_test
    test/urdf_wrapper_equivalence_test.py
    TIMEOUT 60
    ENV
      FRANKA_IK_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
      FRANKA_DESCRIPTION_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../franka_description
      PYTHONDONTWRITEBYTECODE=1)
endif()

ament_export_targets(export_franka_ik HAS_LIBRARY_TARGET)
ament_export_dependencies(franka_ik_interfaces kdl_parser orocos_kdl_vendor rclcpp urdf)
ament_package()
