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

endif()

ament_package()
