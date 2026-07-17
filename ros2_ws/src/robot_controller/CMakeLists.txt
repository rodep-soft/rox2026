cmake_minimum_required(VERSION 3.8)
project(robot_controller)

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

add_executable(spring_controller_node
  spring_controller/src/spring_edulite_controller.cpp
)
target_include_directories(spring_controller_node PRIVATE spring_controller/include)
ament_target_dependencies(spring_controller_node
  rclcpp
  std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(mecanum_controller_node "${cpp_typesupport_target}")
target_link_libraries(spring_controller_node "${cpp_typesupport_target}")

install(TARGETS mecanum_controller_node spring_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY mecanum_controller/include/
  DESTINATION include
)

install(DIRECTORY spring_controller/include/
  DESTINATION include
)

ament_export_dependencies(rosidl_default_runtime)

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
