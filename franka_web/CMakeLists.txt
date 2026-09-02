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

# The frontend is plain static files: no bundler, no transpiler, no package
# manager, nothing to build.  They are installed as they are and served from
# share/franka_web/static by the server.  This rule also carries the vendored
# static/fonts/*.woff2 files, so the console keeps its look with no network.
install(
  DIRECTORY static
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__/*" EXCLUDE
)

# The 3D scene's assets are GENERATED here and installed beside the static
# files; they are not committed.  A cold run costs about a second and a warm
# one about a tenth of that -- the generator re-hashes its inputs and returns
# without work -- which is a small price for a tree that cannot go stale
# behind a description change.  That staleness is not theoretical: the
# base-separation fix moved both arms, and a committed model.urdf would have
# kept drawing the old lab with a green test suite.
#
# The target runs on EVERY build rather than being keyed to an output file,
# because the inputs live in another package's share directory and the
# generator's own input hashing is the authority on whether work is needed.
set(GHOST_ASSET_DIR ${CMAKE_CURRENT_BINARY_DIR}/ghost_assets)
add_custom_target(
  franka_web_ghost_assets ALL
  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/franka_web_ghost_assets.py
          ${GHOST_ASSET_DIR}
  COMMENT "Generating the franka_web 3D scene assets"
  VERBATIM
)
install(
  DIRECTORY ${GHOST_ASSET_DIR}/
  DESTINATION share/${PROJECT_NAME}/static/ghost/assets
)

# The fully commented configuration example.  Nothing reads it at runtime --
# it is documentation an operator copies to ~/.config/franka_web/config.yaml.
install(
  FILES config/config.example.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

# The operator documentation ships with the package, so an installed
# workspace answers "how do I run this" without the source tree.
install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_test REQUIRED)

  # ---------------------------------------------------------------------
  # ROS_DOMAIN_ID allocation for live-node tests
  #
  # The single master table for this workspace lives in
  # franka_bringup/CMakeLists.txt (reviewed core; IDs 191-218 taken there,
  # hard ceiling 232 for rmw_fastrtps_cpp), and every row below now has its
  # matching row there. 225 and 226 belong to franka_robotiq, which is an
  # ament_python package with no CMakeLists to pin them from: they are set
  # in franka_robotiq/test/conftest.py and are listed here so one reader
  # sees the whole contiguous block.
  #
  #   ID   Package         Test target
  #   ---  --------------  ----------------------------------------
  #   219  franka_web      e2e_fake_dual_state_only_test.py  (Stage 1)
  #   220  franka_web      e2e_fake_motion_mock_test.py      (Stage 2)
  #   221  franka_ik       contract / parameter tests        (TAKEN)
  #   222  franka_ik       Stage-6 smoke test                (TAKEN)
  #   223  franka_ghost    browser bridge test               (TAKEN)
  #   224  franka_web      e2e_fake_simulate_console_test.py (Stage 3)
  #   225  franka_robotiq  test_node_contract + test_action_contract (conftest)
  #   226  franka_robotiq  test_launch.py subprocesses               (conftest)
  #   227  franka_web      e2e_fake_dual_gripper_row_test.py
  #   228  franka_web      ghost_ik_live_test.py             (ghost IK round trip)
  #
  # 228 rather than the 225 the ghost contract reserved: 225 and 226 were
  # taken by franka_robotiq while the ghost was being planned, and two tests
  # sharing a domain see each other's graph.  228-232 were the free block.
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
    franka_web_defaults_test
    test/test_defaults.py
    TIMEOUT 60
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
    franka_web_logbus_test
    test/test_logbus.py
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
    franka_web_ghost_scene_api_test
    test/test_ghost_scene_api.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    franka_web_ghost_solve_test
    test/test_ghost_solve.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    franka_web_ghost_assets_test
    test/test_ghost_assets.py
    TIMEOUT 300
  )
  ament_add_pytest_test(
    franka_web_static_lane_test
    test/test_static_lane.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    franka_web_ros_bridge_ik_mapping_test
    test/test_ros_bridge_ik_mapping.py
    TIMEOUT 60
  )
  # The browser suite lives beside the JavaScript it drives; it skips itself
  # when Chromium is absent, so a machine without one stays green.
  ament_add_pytest_test(
    franka_web_browser_test
    test/browser/test_browser.py
    TIMEOUT 300
  )
  ament_add_pytest_test(
    franka_web_jog_model_test
    test/test_jog_model.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    franka_web_travel_test
    test/test_travel.py
    TIMEOUT 60
  )
  # The Apply gates read the tree rather than running it, and one of them
  # loads the real cell model to prove check_path really resamples -- the
  # external fact G3's whole safety argument rests on.
  ament_add_pytest_test(
    franka_web_apply_gates_test
    test/test_apply_gates.py
    TIMEOUT 120
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
  # The Simulate console battery.  XDG_CONFIG_HOME points at an empty build
  # directory so a developer's own ~/.config/franka_web/config.yaml cannot
  # change the zero-config boot case.  That directory is never cleaned
  # between ctest runs: the test must treat it as read-only and empty, and
  # write every config file it needs to a per-case temporary directory
  # passed explicitly with --config.
  file(MAKE_DIRECTORY
    ${CMAKE_CURRENT_BINARY_DIR}/e2e_simulate_console_ros_home
    ${CMAKE_CURRENT_BINARY_DIR}/e2e_simulate_console_ros_logs
    ${CMAKE_CURRENT_BINARY_DIR}/e2e_simulate_console_xdg_config
  )
  ament_add_pytest_test(
    franka_web_e2e_fake_simulate_console_test
    test/e2e_fake_simulate_console_test.py
    TIMEOUT 600
    ENV
      FASTDDS_BUILTIN_TRANSPORTS=SHM
      ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
      ROS_DOMAIN_ID=224
      ROS_HOME=${CMAKE_CURRENT_BINARY_DIR}/e2e_simulate_console_ros_home
      ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/e2e_simulate_console_ros_logs
      XDG_CONFIG_HOME=${CMAKE_CURRENT_BINARY_DIR}/e2e_simulate_console_xdg_config
  )

  # The gripper row beside STANDING fake gripper nodes.  The file starts the
  # nodes itself with `ros2 launch franka_robotiq dual_robotiq.launch.py
  # use_fake:=true`, so it needs its own domain: 227, which the file also
  # checks for and SKIPS itself without.  Half 2 (the row filling and
  # refusing beside a Watch session) skips until bring-up day - see the
  # file's docstring and doc/MOUNTING.md step 8.12.
  file(MAKE_DIRECTORY
    ${CMAKE_CURRENT_BINARY_DIR}/e2e_gripper_row_ros_home
    ${CMAKE_CURRENT_BINARY_DIR}/e2e_gripper_row_ros_logs
  )
  ament_add_pytest_test(
    franka_web_e2e_fake_dual_gripper_row_test
    test/e2e_fake_dual_gripper_row_test.py
    TIMEOUT 600
    ENV
      FASTDDS_BUILTIN_TRANSPORTS=SHM
      ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
      ROS_DOMAIN_ID=227
      ROS_HOME=${CMAKE_CURRENT_BINARY_DIR}/e2e_gripper_row_ros_home
      ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/e2e_gripper_row_ros_logs
  )

  # The live IK round trip: a real franka_ik node and the real server, on
  # their own domain.  The file skips itself unless that domain is set, so a
  # bare pytest can never publish onto someone else's graph.
  file(MAKE_DIRECTORY
    ${CMAKE_CURRENT_BINARY_DIR}/ghost_ik_ros_home
    ${CMAKE_CURRENT_BINARY_DIR}/ghost_ik_ros_logs
  )
  ament_add_pytest_test(
    franka_web_ghost_ik_live_test
    test/ghost_ik_live_test.py
    TIMEOUT 300
    ENV
      FASTDDS_BUILTIN_TRANSPORTS=SHM
      ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
      ROS_DOMAIN_ID=228
      ROS_HOME=${CMAKE_CURRENT_BINARY_DIR}/ghost_ik_ros_home
      ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ghost_ik_ros_logs
  )

  # This package's own gate: whatever ships must not reference a Node.js
  # runtime.  The INSTALLED tree is scanned (this file is not installed, so
  # the check cannot match its own patterns).  grep -R, not -r: with
  # --symlink-install the installed files are symlinks into the source tree.
  # No ';' anywhere in the body -- ament_add_test expands COMMAND unquoted
  # and CMake would re-split it into separate argv elements, which silently
  # turns the whole check into an unconditional pass.
  # The two search patterns are assembled at run time from $nw so that this
  # file does not itself contain the strings it forbids: the packaging sweep
  # and the structural build-file check both grep THIS file for them, and a
  # literal here would make the gate report its own test body.  Shell
  # variables are written without braces -- CMake would expand ${nw}.
  ament_add_test(
    franka_web_no_node_dependency_test
    COMMAND bash -c "set -u
nw=node
roots=''
for d in '${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}' '${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}' ${CMAKE_INSTALL_PREFIX}/lib/python3*/site-packages/${PROJECT_NAME}
do
  if [ -e \"$d\" ]
  then
    roots=\"$roots $d\"
  fi
done
if [ -z \"$roots\" ]
then
  echo 'installed franka_web tree not found - build with an install step before testing' >&2
  exit 1
fi
if grep -RInI -e \"$nw\"js -e \"$nw --test\" $roots
then
  echo 'the installed package still references a Node.js runtime' >&2
  exit 1
fi
exit 0"
    TIMEOUT 60
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )

  # This package's own gate: the layout the server and the README promise.
  # One vendored font is on the list so a dropped static/fonts/ subtree is
  # caught here; the exact whole-directory list is asserted by the Python
  # static-surface test, and the served headers with it.
  ament_add_test(
    franka_web_install_layout_test
    COMMAND bash -c "set -u
rc=0
for f in '${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/package.xml' '${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/README.md' '${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/static/index.html' '${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/static/fonts/archivo-var.woff2' '${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config/config.example.yaml'
do
  if [ ! -f \"$f\" ]
  then
    echo \"missing installed file: $f\" >&2
    rc=1
  fi
done
if [ ! -x '${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/franka_web_server' ]
then
  echo 'franka_web_server is not installed as an executable' >&2
  rc=1
fi
exit $rc"
    TIMEOUT 60
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
endif()

ament_package()
