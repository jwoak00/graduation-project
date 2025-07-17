cmake_minimum_required(VERSION 3.8)
project(offboard_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(offboard_align_land src/OffboardAlignLand.cpp)
ament_target_dependencies(offboard_align_land rclcpp px4_msgs geometry_msgs)

install(TARGETS
  offboard_align_land
  DESTINATION lib/${PROJECT_NAME})

ament_package()
