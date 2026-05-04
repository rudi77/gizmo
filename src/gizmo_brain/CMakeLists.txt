cmake_minimum_required(VERSION 3.8)
project(gizmo_brain)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/gizmo_brain_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gizmo_brain_node)

ament_package()
