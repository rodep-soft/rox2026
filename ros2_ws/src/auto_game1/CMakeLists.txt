cmake_minimum_required(VERSION 3.8)
project(auto_game1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate Action / Message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Kick.action"
  "action/Dribble.action"
  "action/Pass.action"
  DEPENDENCIES std_msgs
)

# Include directories
include_directories(
  include
)

# Executable target
add_executable(auto_game1_node
  src/auto_game1_node.cpp
  src/main.cpp
)

ament_target_dependencies(auto_game1_node
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  sensor_msgs
  nav2_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Link generated interfaces
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(auto_game1_node "${cpp_typesupport_target}")

install(TARGETS
  auto_game1_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Testing section (omitted)

ament_package()
