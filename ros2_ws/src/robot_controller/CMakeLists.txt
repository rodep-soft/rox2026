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
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

function(add_controller target directory implementation)
  set(library_target ${target}_implementation)

  add_library(${library_target} STATIC ${directory}/src/${implementation})
  target_include_directories(${library_target}
                             PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${directory}/include>)
  ament_target_dependencies(${library_target} ${ARGN})

  add_executable(${target} ${directory}/src/main.cpp)
  target_link_libraries(${target} PRIVATE ${library_target})

  set(controller_nodes
      ${controller_nodes} ${target}
      PARENT_SCOPE)
endfunction()

add_controller(
  mecanum_controller_node
  mecanum_controller
  mecanum_controller_node.cpp
  actuator_msgs
  geometry_msgs
  rclcpp
  std_msgs)
add_controller(spring_controller_node spring_controller spring_edulite_controller.cpp actuator_msgs rclcpp std_msgs)
add_controller(
  belt_controller_node
  belt_controller
  belt_controller.cpp
  actuator_msgs
  rclcpp
  rcl_interfaces
  std_msgs
  robot_msgs)
add_controller(
  dribble_controller_node
  dribble_controller
  dribble_controller.cpp
  actuator_msgs
  geometry_msgs
  rclcpp
  rcl_interfaces
  std_msgs
  robot_msgs)
add_controller(
  game2_auto_node
  auto_controllers/game2_shooter
  game2_auto_node.cpp
  geometry_msgs
  rclcpp
  rcl_interfaces
  sensor_msgs
  std_msgs
  robot_msgs
  apriltag_msgs
  tf2
  tf2_ros)
add_controller(
  game1_auto_node
  auto_controllers/game1_shooter
  game1_auto_node.cpp
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  robot_msgs)
add_controller(led_controller_node led_controller led_controller.cpp rclcpp std_msgs robot_msgs)

add_executable(mecanum_odometry_node mecanum_controller/src/mecanum_odometry_node.cpp)
target_include_directories(
  mecanum_odometry_node
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/mecanum_controller/include)
ament_target_dependencies(
  mecanum_odometry_node actuator_msgs nav_msgs rclcpp tf2 tf2_ros)

add_executable(heading_hold_node heading_hold/src/heading_hold_node.cpp)
ament_target_dependencies(
  heading_hold_node
  geometry_msgs
  rcl_interfaces
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs)

add_executable(test_game1_wp_move_node auto_controllers/game1_shooter/src/test_game1_wp_move_node.cpp)
ament_target_dependencies(
  test_game1_wp_move_node geometry_msgs nav_msgs rclcpp std_msgs)

list(APPEND controller_nodes mecanum_odometry_node heading_hold_node test_game1_wp_move_node)
install(TARGETS ${controller_nodes} DESTINATION lib/${PROJECT_NAME})

set(controller_include_directories
    mecanum_controller
    spring_controller
    led_controller
    belt_controller
    dribble_controller
    auto_controllers/game1_shooter
    auto_controllers/game2_shooter)
foreach(directory IN LISTS controller_include_directories)
  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${directory}/include/ DESTINATION include)
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
      test_robot_controller belt_controller_node_implementation dribble_controller_node_implementation
      mecanum_controller_node_implementation spring_controller_node_implementation)
    ament_add_gtest(test_mecanum_odometry test/test_mecanum_odometry.cpp)
    target_include_directories(test_mecanum_odometry PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/mecanum_controller/include)
  endif()
endif()

ament_package()
