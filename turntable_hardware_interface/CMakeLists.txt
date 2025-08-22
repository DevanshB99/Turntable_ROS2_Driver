cmake_minimum_required(VERSION 3.8)
project(turntable_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
include_directories(
  include
)

add_library(turntable_system SHARED
  src/turntable_system.cpp
)

ament_target_dependencies(turntable_system
  hardware_interface
  pluginlib
  rclcpp
  std_msgs
  sensor_msgs
)

target_include_directories(turntable_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# This is crucial for the plugin to be discovered
pluginlib_export_plugin_description_file(hardware_interface turntable_hardware_interface_plugin.xml)

install(
  TARGETS turntable_system
  EXPORT export_turntable_system
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES
  config/turntable_hardware_params.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

install(FILES turntable_hardware_interface_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(turntable_system)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  std_msgs
  sensor_msgs
)
ament_package()

