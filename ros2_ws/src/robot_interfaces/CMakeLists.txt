cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotCommand.msg"
  DEPENDENCIES geometry_msgs
)

add_executable(mecanum_controller_node
  mecanum_controller/src/mecanum_controller_node.cpp
)
target_include_directories(mecanum_controller_node PRIVATE mecanum_controller/include)
ament_target_dependencies(mecanum_controller_node
  rclcpp
  std_msgs
)
rosidl_target_interfaces(mecanum_controller_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS mecanum_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY mecanum_controller/include/
  DESTINATION include
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
