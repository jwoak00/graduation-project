cmake_minimum_required(VERSION 3.8 FATAL_ERROR) # CMake 최소 버전 명시
project(cannyedge_test1_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 빌드 타입 설정 (선택 사항)
# set(CMAKE_BUILD_TYPE Release)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HoleInfo.msg"
  "msg/HoleInfoArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
