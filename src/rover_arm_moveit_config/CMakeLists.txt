cmake_minimum_required(VERSION 3.8)
project(rover_arm_moveit_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(control_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)


if(APPLE)
  find_library(ATOMIC_LIBRARY atomic)
  if(NOT ATOMIC_LIBRARY)
    message(FATAL_ERROR "libatomic not found")
  endif()
  add_library(atomic UNKNOWN IMPORTED)
  set_target_properties(atomic PROPERTIES IMPORTED_LOCATION "${ATOMIC_LIBRARY}")
endif()

find_package(Python3 REQUIRED COMPONENTS Development)

add_executable(demo_motion src/demo_motion.cpp)
ament_target_dependencies(demo_motion
  rclcpp
  moveit_ros_planning_interface
  control_msgs
)
target_link_libraries(demo_motion
  Python3::Python
  ${ATOMIC_LIBRARY}
)

install(DIRECTORY
    config
    launch
    rviz
    DESTINATION share/${PROJECT_NAME}
)

add_executable(panel_interaction_demo src/panel_interaction_demo.cpp)
ament_target_dependencies(panel_interaction_demo
  rclcpp
  moveit_ros_planning_interface
  control_msgs
  nav2_msgs
)
target_link_libraries(panel_interaction_demo
  Python3::Python
  ${ATOMIC_LIBRARY}
)

add_executable(ik_grasp_probe src/ik_grasp_probe.cpp)
ament_target_dependencies(ik_grasp_probe
  rclcpp
  moveit_ros_planning_interface
)
target_link_libraries(ik_grasp_probe
  Python3::Python
  ${ATOMIC_LIBRARY}
)

install(
  TARGETS demo_motion panel_interaction_demo ik_grasp_probe
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
