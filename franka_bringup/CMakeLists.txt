cmake_minimum_required(VERSION 3.5)
project(franka_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# ---------------------------------------------------------------------
# Install-time exclusions (pre-Phase-9 review, real-IP launch surface)
#
# These stay in the source tree (still readable, importable by tests that
# resolve them via a source-tree path, still `git`-tracked) but are not
# copied into the installed share/ directory, so `ros2 launch
# franka_bringup ...` cannot reach them and nothing accidentally points a
# real robot IP at them. Each PATTERN below names the exact reviewed
# finding driving it and the condition under which it may be re-included.
#
# EXCLUDE reason 1 - post-MVP, after the 3 multimode P1s are fixed:
#   MultiModeController has 3 known open P1s; these launch files/configs
#   are the only way to load it against a real robot_ip.
#     launch/real/multimode_franka.launch.py
#     launch/real/dual_multimode_franka.launch.py
#     launch/mixed/                                  (both mixed multimode launches)
#     config/real/single_multimode.yaml
#     config/real/dual_multimode.yaml
#     config/mixed/                                  (multimode configs for the above)
#
# EXCLUDE reason 2 - superseded by operator profiles + one-arm mode:
#   franka.launch.py is the ungated legacy single-arm demo launch: no
#   operator-profile gating, and single_controllers.yaml registers the
#   banned joint-position controller. franka_bringup/launch/operator/ and
#   launch/real/one_arm_franka.launch.py are the supported replacements.
#     launch/real/franka.launch.py
#     config/real/single_controllers.yaml
#
# TOTAL: the 7 PATTERN EXCLUDE clauses below remove 10 files from the
# installed tree (verified by diffing installed share/ against the source
# tree): launch/real/{franka,multimode_franka,dual_multimode_franka}.launch.py
# (3), both launch/mixed/*.launch.py files (2), config/real/
# {single_controllers,single_multimode,dual_multimode}.yaml (3), both
# config/mixed/*.yaml files (2). "mixed/*" is one PATTERN covering 4 of
# the 10 files (2 launch + 2 config); every other PATTERN covers exactly
# one file. Do not describe this as "8 targets" - that number undercounts
# the mixed/ directory contents.
#
# NOTE: franka_bringup/test/test_launch_arm_id_validation.py and
# franka_bringup/test/single_fake_hardware_smoke.launch_test.py both used to
# resolve launch/real/franka.launch.py via get_package_share_directory().
# Both now resolve that one file from the source tree instead (the same
# _SOURCE_ROOT mechanism already used by test_operator_launch_profiles.py),
# since franka.launch.py is deliberately not installed.
#
# PATTERN CAVEAT (found during pre-Phase-9 build verification): this target
# is built with `colcon build --symlink-install`, which routes install()
# through ROS 2's ament_cmake_symlink_install override instead of CMake's
# native install(DIRECTORY). That override matches each PATTERN relative to
# its OWN top-level DIRECTORY entry (i.e. relative to "config" or "launch"
# separately), whereas plain CMake install(DIRECTORY) matches PATTERN
# against the full source path (so a "launch/..." or "config/..." prefix
# is required there). A single prefix-free pattern such as
# "real/franka.launch.py" satisfies both: under the symlink override it is
# looked up as "<dir>/real/franka.launch.py" (matches only under launch/,
# since only launch/ has a real/franka.launch.py); under plain CMake it
# still matches because PATTERN only requires the suffix to be preceded by
# *a* slash, not specifically by "launch/". Do not reintroduce a
# "launch/" or "config/" prefix on these patterns without re-testing under
# --symlink-install (the CMake-3.28-docs-only review that produced the
# original prefixed patterns did not catch this because it did not build
# with --symlink-install).
#
# __pycache__ LEAK (found by the final pre-Phase-9 skeptic pass, after this
# record had already claimed the gating above was complete): the two tests
# that resolve launch/real/franka.launch.py from the SOURCE tree via
# importlib (see the NOTE above) make CPython write a
# launch/real/__pycache__/franka.launch.cpython-3xx.pyc next to the source
# file. With --symlink-install nothing previously excluded __pycache__, so
# that compiled, loadable/executable copy of the excluded file was swept
# into the installed share/ tree regardless of the PATTERN EXCLUDEs above -
# reproducible from a byte-for-byte clean rebuild, not merely "stale from a
# previous test run" (see franka_bringup/test/*.py for the matching
# sys.dont_write_bytecode source-side half of this fix).
#
# Getting the exclude PATTERN right took a second empirical round beyond
# the "match relative to top-level DIRECTORY entry" lesson above: the
# --symlink-install override implements PATTERN/PATTERN_EXCLUDE with
# `file(GLOB_RECURSE ... "<dir>/<pattern>")` (see
# ament_cmake_symlink_install.cmake.in), and GLOB_RECURSE only ever
# enumerates FILES, never bare directory names. A literal, wildcard-free
# pattern like "real/__pycache__" therefore matches nothing (no installed
# file has exactly that relative path) and silently fails to exclude the
# .pyc files sitting inside it - verified empirically (rebuilt with that
# exact pattern; the .pyc still installed). A trailing "/*" is required so
# the glob enumerates the files inside the directory. "mixed/*" above
# already covers a nested mixed/__pycache__ for the same reason (any
# __pycache__ under mixed/ is just more files matched by that wildcard).
# One explicit "<subdir>/__pycache__/*" PATTERN per launch/config
# subdirectory is used - rather than a single "*/__pycache__/*" - because
# that single-level-wildcard form was tried first and, empirically,
# GLOB_RECURSE does not expand a bare leading "*" to match arbitrary
# top-level subdirectory names the way "mixed/*" (a wildcard immediately
# following a literal name already anchored to a real directory) does;
# verified by clean-rebuilding with "*/__pycache__" EXCLUDE and observing
# the .pyc files were still installed. sim/ carries no __pycache__ today
# (no .py file has been imported from it) but is included pre-emptively
# since a future test could resolve a launch/sim or config/sim file from
# the source tree the same way franka.launch.py is resolved today; this
# was verified against a synthetic launch/sim/__pycache__/probe.pyc
# (removed after the check) alongside real/ and operator/ probes.
# ---------------------------------------------------------------------
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "real/multimode_franka.launch.py" EXCLUDE
  PATTERN "real/dual_multimode_franka.launch.py" EXCLUDE
  PATTERN "mixed/*" EXCLUDE
  PATTERN "real/single_multimode.yaml" EXCLUDE
  PATTERN "real/dual_multimode.yaml" EXCLUDE
  PATTERN "real/franka.launch.py" EXCLUDE
  PATTERN "real/single_controllers.yaml" EXCLUDE
  PATTERN "real/__pycache__/*" EXCLUDE
  PATTERN "operator/__pycache__/*" EXCLUDE
  PATTERN "sim/__pycache__/*" EXCLUDE
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
  #   200  franka_bringup              single_fake_hardware_smoke.launch_test.py (RETIRED - see comment above its add_launch_test)
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
  #   214  franka_example_controllers  dual_arm_joint_hold_controller_one_arm_test
  #   215  franka_example_controllers  dual_arm_joint_velocity_controller_one_arm_test
  #   216  franka_example_controllers  dual_arm_joint_impedance_controller_one_arm_test
  #   217  franka_bringup              single_safe_controllers_fake_hardware.launch_test.py
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
  # DORMANT during the MVP (pre-Phase-9 readiness pass, 2026-08-26): this
  # test's generate_test_description() IncludeLaunchDescription()s
  # launch/real/franka.launch.py, which loads config/real/
  # single_controllers.yaml - both deliberately excluded from install
  # above (EXCLUDE reason 2). The test file itself resolves
  # franka.launch.py from the source tree (see the NOTE above), so it
  # still imports fine, but that source-tree franka.launch.py in turn
  # resolves single_controllers.yaml via FindPackageShare against the
  # INSTALLED share/ directory, where the file no longer exists - so the
  # launch fails to bring up joint_state_broadcaster and 3 of the 6
  # in-process test_* cases fail (test_spawner_exits_cleanly,
  # test_no_motion_controller_is_loaded,
  # test_expected_joint_states_publish_without_duplicates); the 1
  # post_shutdown test_managed_processes_exit_cleanly still passes.
  # Registration commented out rather than resurrecting the dormant
  # single_controllers.yaml path. Static coverage of franka.launch.py's
  # arm_id validation remains live via
  # franka_launch_arm_id_validation_test (source-tree resolution, see
  # test_launch_arm_id_validation.py); live single-arm fake-hardware
  # coverage is single_safe_controllers_fake_hardware.launch_test.py
  # (ID 217, passes 6/6). Reinstate this registration together with the
  # legacy launch/real/franka.launch.py + config/real/
  # single_controllers.yaml install surface, post-MVP, after the
  # multimode P1 fixes (EXCLUDE reason 1) make it worth restoring the
  # legacy demo path too.
  # add_launch_test(
  #   test/single_fake_hardware_smoke.launch_test.py
  #   TIMEOUT 90
  #   ENV
  #     ROS_DOMAIN_ID=200
  # )
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
  file(MAKE_DIRECTORY
    ${CMAKE_CURRENT_BINARY_DIR}/single_safe_controllers_ros_home
    ${CMAKE_CURRENT_BINARY_DIR}/single_safe_controllers_ros_logs
  )
  add_launch_test(
    test/single_safe_controllers_fake_hardware.launch_test.py
    TIMEOUT 180
    ENV
      FASTDDS_BUILTIN_TRANSPORTS=SHM
      ROS_DOMAIN_ID=217
      ROS_HOME=${CMAKE_CURRENT_BINARY_DIR}/single_safe_controllers_ros_home
      ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/single_safe_controllers_ros_logs
  )
endif()

ament_package()
