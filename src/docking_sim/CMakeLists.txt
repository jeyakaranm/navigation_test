cmake_minimum_required(VERSION 3.8)
project(docking_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY worlds launch rviz models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
