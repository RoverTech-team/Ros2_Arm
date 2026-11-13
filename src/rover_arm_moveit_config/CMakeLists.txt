cmake_minimum_required(VERSION 3.8)
project(rover_arm_moveit_config)

# ============================================================
# COMPILER
# ============================================================

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================
# DIPENDENZE
# ============================================================

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)

find_package(control_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(Python3 REQUIRED COMPONENTS Development)

# ============================================================
# SUPPORTO APPLE
# ============================================================

if(APPLE)
  find_library(ATOMIC_LIBRARY atomic)

  if(NOT ATOMIC_LIBRARY)
    message(FATAL_ERROR "libatomic not found")
  endif()

  add_library(atomic UNKNOWN IMPORTED)

  set_target_properties(atomic PROPERTIES
    IMPORTED_LOCATION "${ATOMIC_LIBRARY}"
  )
endif()

# ============================================================
# DEMO MOTION
# ============================================================

add_executable(
  demo_motion
  src/demo_motion.cpp
)

ament_target_dependencies(
  demo_motion
  rclcpp
  moveit_ros_planning_interface
  control_msgs
)

target_link_libraries(
  demo_motion
  Python3::Python
  ${ATOMIC_LIBRARY}
)

# ============================================================
# PANEL INTERACTION DEMO - VERSIONE ORIGINALE
# ============================================================

add_executable(
  panel_interaction_demo
  src/panel_interaction_demo.cpp
)

ament_target_dependencies(
  panel_interaction_demo
  rclcpp
  moveit_ros_planning_interface
  control_msgs
  nav2_msgs
)

target_link_libraries(
  panel_interaction_demo
  Python3::Python
  ${ATOMIC_LIBRARY}
)

# ============================================================
# PANEL INTERACTION DEMO - RICERCA AUTONOMA ARUCO + CENTRAGGIO
# ============================================================

# Nuovo C++:
# - Ricerca autonoma degli ArUco
# - Movimento progressivo del braccio
# - Stop quando entrambi gli ArUco sono visibili
# - Centraggio del pannello
# - Segnale /panel/align_done per la stima finale
#   effettuata dal nodo Python con ArUco + Depth

add_executable(
  panel_interaction_demo_aim_aruco
  src/panel_interaction_demo_aim_aruco.cpp
)

ament_target_dependencies(
  panel_interaction_demo_aim_aruco
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_core
  control_msgs
  sensor_msgs
  std_msgs
  trajectory_msgs
)

target_link_libraries(
  panel_interaction_demo_aim_aruco
  Python3::Python
  ${ATOMIC_LIBRARY}
)


# ============================================================
# IK GRASP PROBE
# ============================================================

add_executable(
  ik_grasp_probe
  src/ik_grasp_probe.cpp
)

ament_target_dependencies(
  ik_grasp_probe
  rclcpp
  moveit_ros_planning_interface
)

target_link_libraries(
  ik_grasp_probe
  Python3::Python
  ${ATOMIC_LIBRARY}
)

# ============================================================
# INSTALLAZIONE CONFIGURAZIONI
# ============================================================

install(
  DIRECTORY
    config
    launch
    rviz
  DESTINATION share/${PROJECT_NAME}
)

# ============================================================
# INSTALLAZIONE ESEGUIBILI
# ============================================================

install(
  TARGETS
    demo_motion
    panel_interaction_demo
    panel_interaction_demo_aim_aruco
    ik_grasp_probe

  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
# ============================================================
# AMENT
# ============================================================

ament_package()
