cmake_minimum_required(VERSION 3.8)
project(rover_arm_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)

if(APPLE)
  find_library(ATOMIC_LIBRARY atomic)
  if(NOT ATOMIC_LIBRARY)
    message(FATAL_ERROR "libatomic not found")
  endif()
  add_library(atomic UNKNOWN IMPORTED)
  set_target_properties(atomic PROPERTIES IMPORTED_LOCATION "${ATOMIC_LIBRARY}")
endif()

add_executable(send_trajectory reference_generator/send_trajectory.cpp)
ament_target_dependencies(send_trajectory
  kdl_parser
  rclcpp
  trajectory_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
    launch
    meshes
    rviz
    urdf
    bringup
    doc
    DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS send_trajectory
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    test/test_topic_based_hw_interface.py
    teleop/arm_keyboard_teleop.py
    teleop/arm_joy_teleop.py
    teleop/arm_joy_publisher.py
    scripts/gripper_mimic_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
