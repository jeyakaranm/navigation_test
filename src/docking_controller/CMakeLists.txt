cmake_minimum_required(VERSION 3.8)
project(docking_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(magic_enum REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  std_srvs
  visualization_msgs
)

include_directories(
  include
)

add_library(line_fit SHARED
  src/line_fit.cpp
)

add_library(docking_controller SHARED
  src/docking_controller.cpp
)
target_link_libraries(docking_controller line_fit magic_enum::magic_enum)
ament_target_dependencies(docking_controller ${dependencies})

add_executable(docking_node
  src/docking_node.cpp
)
target_link_libraries(docking_node docking_controller)
ament_target_dependencies(docking_node ${dependencies})

install(TARGETS docking_controller line_fit
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS docking_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(docking_controller line_fit)
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_line_fit test/test_line_fit.cpp)
  target_link_libraries(test_line_fit line_fit)
endif()

ament_package()
