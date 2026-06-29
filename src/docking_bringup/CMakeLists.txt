cmake_minimum_required(VERSION 3.8)
project(docking_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/dock_metrics.py
  scripts/dock_eval.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
