cmake_minimum_required(VERSION 3.8)
project(rover_arm_panel_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PanelState.msg"
  "srv/SetSwitch.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

install(DIRECTORY
    config
    launch
    urdf
    materials
    rviz
    meshes
    DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/panel_gui.py
    scripts/panel_scene_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
