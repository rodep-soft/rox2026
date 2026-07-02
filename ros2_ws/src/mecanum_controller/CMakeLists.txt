cmake_minimum_required(VERSION 3.8)
project(mecanum_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(mecanum_controller_node src/main.cpp src/mecanum_controller_node.cpp)

add_executable(base_controller_node src/base_controller.cpp)

target_include_directories(
  mecanum_controller_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  mecanum_controller_node rclcpp sensor_msgs geometry_msgs std_msgs)
ament_target_dependencies(
  base_controller_node rclcpp sensor_msgs geometry_msgs)
install(TARGETS mecanum_controller_node DESTINATION lib/${PROJECT_NAME})

install(TARGETS base_controller_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
