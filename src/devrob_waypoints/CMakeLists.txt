cmake_minimum_required(VERSION 3.8)
project(devrob_waypoints)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(waypoint_node src/waypoint_node.cpp)
target_compile_features(waypoint_node PUBLIC cxx_std_17)

ament_target_dependencies(
  waypoint_node
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

# Link yaml-cpp library
target_link_libraries(waypoint_node yaml-cpp)

install(
  TARGETS waypoint_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

