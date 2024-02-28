cmake_minimum_required(VERSION 3.8)
project(ros2_cpp_with_headers)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
  rclcpp
  std_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED src/ros2_cpp_with_headers/MyLibrary.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(my_publisher src/main.cpp)
ament_target_dependencies(my_publisher ${dependencies})
target_link_libraries(my_publisher ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  my_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})
ament_package()
