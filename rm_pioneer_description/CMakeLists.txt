cmake_minimum_required(VERSION 3.8)
project(rm_pioneer_description)

# find dependencies
find_package(ament_cmake REQUIRED)

# Install files
install(DIRECTORY
        launch
        meshes
        urdf
        DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
