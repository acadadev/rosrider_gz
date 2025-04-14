cmake_minimum_required(VERSION 3.5)

project(rosrider_gz_bringup)

find_package(ament_cmake REQUIRED)
find_package(rosrider_gz_description REQUIRED)
find_package(rosrider_gz_gazebo REQUIRED)

install(
  DIRECTORY
    launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY
    config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
