cmake_minimum_required(VERSION 3.8)
project(my_robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY config launch maps rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(
  PROGRAMS
    scripts/nav2_cmd_vel_relay.py
    scripts/world_to_odom.py
    scripts/map_to_odom.py
    scripts/map_publisher.py
    scripts/joint_states_restamper.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES Nav2RealRover.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
