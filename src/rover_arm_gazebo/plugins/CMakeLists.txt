cmake_minimum_required(VERSION 3.8)

find_package(gz-sim8 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# On macOS, there is no separate libatomic. If rclcpp/rcutils
# brings in -latomic, provide a stub so the linker doesn't fail.
if(APPLE AND NOT TARGET atomic)
  if(NOT EXISTS "${CMAKE_CURRENT_BINARY_DIR}/libatomic.a")
    file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/atomic_stub.c" "int __atomic_dummy;\n")
    execute_process(COMMAND
      ${CMAKE_C_COMPILER} -arch arm64 -c
      "${CMAKE_CURRENT_BINARY_DIR}/atomic_stub.c"
      -o "${CMAKE_CURRENT_BINARY_DIR}/atomic_stub.o")
    execute_process(COMMAND
      ar rcs "${CMAKE_CURRENT_BINARY_DIR}/libatomic.a"
      "${CMAKE_CURRENT_BINARY_DIR}/atomic_stub.o")
  endif()
  add_library(atomic STATIC IMPORTED GLOBAL)
  set_target_properties(atomic PROPERTIES
    IMPORTED_LOCATION "${CMAKE_CURRENT_BINARY_DIR}/libatomic.a")
endif()

add_library(lever_switch_plugin SHARED
  src/lever_switch_plugin.cpp
)
target_include_directories(lever_switch_plugin PRIVATE
  ${std_msgs_INCLUDE_DIRS}
)
target_link_libraries(lever_switch_plugin
  gz-sim8::gz-sim8
  rclcpp::rclcpp
  ${std_msgs_LIBRARIES}
)

add_library(rotary_switch_plugin SHARED
  src/rotary_switch_plugin.cpp
)
target_include_directories(rotary_switch_plugin PRIVATE
  ${std_msgs_INCLUDE_DIRS}
)
target_link_libraries(rotary_switch_plugin
  gz-sim8::gz-sim8
  rclcpp::rclcpp
  ${std_msgs_LIBRARIES}
)

install(TARGETS
  lever_switch_plugin
  rotary_switch_plugin
  DESTINATION lib/${PROJECT_NAME}
)
