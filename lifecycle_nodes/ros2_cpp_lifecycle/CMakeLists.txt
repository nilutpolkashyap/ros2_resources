cmake_minimum_required(VERSION 3.8)
project(ros2_cpp_lifecycle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
    rclcpp_lifecycle
    lifecycle_msgs
    rclcpp
	std_msgs
)

add_executable(lifecycle_publisher_node src/lifecycle_publisher.cpp)
ament_target_dependencies(lifecycle_publisher_node ${dependencies})

install(TARGETS
  lifecycle_publisher_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
