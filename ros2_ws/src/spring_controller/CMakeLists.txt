cmake_minimum_required(VERSION 3.8)
project(spring_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(spring_edulite_controller src/spring_edulite_controller.cpp)
ament_target_dependencies(
  spring_edulite_controller rclcpp std_msgs)
target_include_directories(
  spring_edulite_controller
  PRIVATE include)

add_executable(spring_joy_sub src/spring_joy_sub.cpp)
ament_target_dependencies(
  spring_joy_sub rclcpp sensor_msgs std_msgs)
target_include_directories(
  spring_joy_sub
  PRIVATE include)

install(TARGETS spring_joy_sub spring_edulite_controller DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
