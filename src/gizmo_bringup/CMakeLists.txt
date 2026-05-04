cmake_minimum_required(VERSION 3.8)
project(gizmo_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/gizmo_gait_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gizmo_gait_node)

install(PROGRAMS
  scripts/gizmo_face_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gizmo_face_node)

install(PROGRAMS
  scripts/gizmo_stt_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gizmo_stt_node)

install(PROGRAMS
  scripts/gizmo_tts_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gizmo_tts_node)

ament_package()
