cmake_minimum_required(VERSION 3.8)
project(my_hole_detector_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io segmentation)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(hole_detector_node src/hole_detector_node.cpp)

ament_target_dependencies(hole_detector_node
  rclcpp
  sensor_msgs
  cv_bridge
  pcl_conversions
  visualization_msgs
  PCL
)

target_link_libraries(hole_detector_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

install(TARGETS hole_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

