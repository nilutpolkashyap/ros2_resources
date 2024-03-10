cmake_minimum_required(VERSION 3.8)
project(behavior_tree_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
    rclcpp
    behaviortree_cpp
    std_msgs 
    ament_index_cpp
)

add_executable(grasp_place src/grasp_place_bt.cpp)
ament_target_dependencies(grasp_place ${dependencies})

install(TARGETS
  grasp_place
  DESTINATION lib/${PROJECT_NAME})

# install(DIRECTORY include/
#   DESTINATION include/
# )

install(DIRECTORY behavior_tree_xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_include_directories(share)
ament_export_dependencies(${dependencies})

ament_package()
