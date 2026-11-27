cmake_minimum_required(VERSION 3.8)
project(hsr_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes launch config worlds
  DESTINATION share/${PROJECT_NAME})

# Make package://hsr_description/... meshes resolvable by Gazebo after sourcing the workspace
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
