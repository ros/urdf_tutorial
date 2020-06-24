cmake_minimum_required(VERSION 3.5)
project(urdf_tutorial)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY images launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
