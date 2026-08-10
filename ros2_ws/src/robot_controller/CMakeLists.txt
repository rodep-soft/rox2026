cmake_minimum_required(VERSION 3.8)
project(robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(actuator_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

function(add_controller target directory implementation)
  set(library_target ${target}_implementation)

  add_library(${library_target} STATIC ${directory}/src/${implementation})
  target_include_directories(
    ${library_target}
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${directory}/include>)
  ament_target_dependencies(${library_target} ${ARGN})

  add_executable(${target} ${directory}/src/main.cpp)
  target_link_libraries(${target} PRIVATE ${library_target})

  set(controller_nodes ${controller_nodes} ${target} PARENT_SCOPE)
endfunction()

add_controller(
  mecanum_controller_node
  mecanum_controller
  mecanum_controller_node.cpp
  actuator_msgs geometry_msgs rclcpp std_msgs)
add_controller(
  spring_controller_node
  spring_controller
  spring_edulite_controller.cpp
  actuator_msgs rclcpp std_msgs)
add_controller(
  belt_controller_node
  belt_controller
  belt_controller.cpp
  actuator_msgs rclcpp rcl_interfaces std_msgs)
add_controller(
  dribble_controller_node
  dribble_controller
  dribble_controller.cpp
  actuator_msgs rclcpp rcl_interfaces std_msgs)
add_controller(
  game2_tactical_shooter_node
  game2_shooter
  game2_tactical_shooter_node.cpp
  geometry_msgs rclcpp sensor_msgs std_msgs tf2 tf2_ros)

install(TARGETS ${controller_nodes} DESTINATION lib/${PROJECT_NAME})

set(controller_include_directories
    mecanum_controller
    spring_controller
    belt_controller
    dribble_controller
    game2_shooter)
foreach(directory IN LISTS controller_include_directories)
  install(DIRECTORY ${directory}/include/ DESTINATION include)
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest QUIET)
  if(ament_cmake_gtest_FOUND)
    ament_add_gtest(test_robot_controller test/test_robot_controller.cpp)
    target_link_libraries(
      test_robot_controller
      belt_controller_node_implementation
      dribble_controller_node_implementation
      mecanum_controller_node_implementation)
  endif()
endif()

ament_package()
