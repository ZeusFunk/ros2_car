cmake_minimum_required(VERSION 3.5)
project(garbage_classification)

# 默认需要C++17标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GarbageClassification.srv"
)

add_executable(server src/server.cpp)
ament_target_dependencies(server
  rclcpp
  rosidl_default_runtime
)

add_executable(client src/client.cpp)
ament_target_dependencies(client
  rclcpp
  rosidl_default_runtime
)

install(TARGETS server client
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(
  rosidl_default_runtime
  rclcpp
)

ament_package()