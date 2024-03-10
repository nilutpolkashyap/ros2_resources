cmake_minimum_required(VERSION 3.8)
project(ros2_opencv_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

set(dependencies
    rclcpp
    cv_bridge
    sensor_msgs
    OpenCV
)

add_executable(webcam_publisher_node src/webcam_publisher.cpp)
ament_target_dependencies(webcam_publisher_node ${dependencies})

add_executable(webcam_subscriber_node src/webcam_subscriber.cpp)
ament_target_dependencies(webcam_subscriber_node ${dependencies})

install(TARGETS
  webcam_publisher_node
  webcam_subscriber_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
