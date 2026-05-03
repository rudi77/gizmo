cmake_minimum_required(VERSION 3.8)
project(gizmo_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/gizmo_gait_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gizmo_gait_node)

ament_package()
