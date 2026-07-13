cmake_minimum_required(VERSION 3.8)
project(can_send_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(can_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(can_send_node
  src/can_send_node.cpp
)
target_include_directories(can_send_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(can_send_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(can_send_node
  can_msgs
  rclcpp
)

install(TARGETS
  can_send_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
