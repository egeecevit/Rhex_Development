cmake_minimum_required(VERSION 3.8)
project(control_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)

install(
  PROGRAMS src/controller_server_network.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch src
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
