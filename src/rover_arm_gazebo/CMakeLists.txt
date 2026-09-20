cmake_minimum_required(VERSION 3.8)
project(rover_arm_gazebo)

find_package(ament_cmake REQUIRED)

add_subdirectory(plugins)

install(DIRECTORY
  worlds
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
