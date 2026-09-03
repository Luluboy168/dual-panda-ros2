cmake_minimum_required(VERSION 3.5)
project(franka_workspace_model)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# The cell file and the derived link geometry install side by side, because the
# cell file names link_geometry_v1.yaml relative to its own directory. This is
# the documented, stable install location a consumer may rely on; the module
# level default_cell_model_path() resolves it without hard-coding a filename.
install(
  DIRECTORY cell doc
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)

  # The whole suite is offline: no robot, no running ROS graph, no launch. It
  # resolves the repository from the source tree so that the hash interlock in
  # CONTRACT A rule A.3.6 is exercised against the description this package was
  # generated from.
  # The timeout is 1800 s, not 600, because the suite now carries three
  # measured populations rather than a handful of poses: two GJK ensembles
  # (20 000 uniform placements and 1 200 bisected near-touching ones) and the
  # acceptance census.  Those are the instruments that would have caught the
  # defects this package has already shipped once, and the honest way to afford
  # them is a longer timeout - never a smaller draw count, which is what a
  # 600 s ceiling would quietly buy.
  ament_add_pytest_test(
    franka_workspace_model_test
    test
    TIMEOUT 1800
    ENV FRANKA_WSM_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
