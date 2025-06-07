cmake_minimum_required(VERSION 3.8)
project(turntable_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install full urdf folder including subfolders like 'inc'
install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
  FILES_MATCHING PATTERN "*.xacro"
)

install(
  DIRECTORY config/turntable
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
