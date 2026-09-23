cmake_minimum_required(VERSION 3.8)
project(reactive_turtlebot)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

add_executable(reactive_turtlebot
  src/reactive_turtlebot.cpp
)

ament_target_dependencies(reactive_turtlebot
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  ros_gz_interfaces
)

install(TARGETS
  reactive_turtlebot
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
