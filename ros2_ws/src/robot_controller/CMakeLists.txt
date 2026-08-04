cmake_minimum_required(VERSION 3.8)
project(robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(mecanum_controller_node mecanum_controller/src/mecanum_controller_node.cpp
                                       mecanum_controller/src/main.cpp)
target_include_directories(
  mecanum_controller_node
  PRIVATE mecanum_controller/include)
ament_target_dependencies(
  mecanum_controller_node geometry_msgs rclcpp std_msgs)

add_executable(spring_controller_node spring_controller/src/spring_edulite_controller.cpp)
target_include_directories(
  spring_controller_node
  PRIVATE spring_controller/include)
ament_target_dependencies(
  spring_controller_node rclcpp std_msgs)

add_executable(belt_controller_node belt_controller/src/belt_controller.cpp belt_controller/src/main.cpp)
target_include_directories(
  belt_controller_node
  PRIVATE belt_controller/include)
ament_target_dependencies(
  belt_controller_node rclcpp std_msgs)

add_executable(dribbler_controller_node dribbler_controller/src/dribbler_controller.cpp
                                        dribbler_controller/src/main.cpp)
target_include_directories(
  dribbler_controller_node
  PRIVATE dribbler_controller/include)
ament_target_dependencies(
  dribbler_controller_node rclcpp std_msgs)

add_executable(arm_position_controller_node arm_position_controller/src/arm_position_controller.cpp
                                            arm_position_controller/src/main.cpp)
target_include_directories(
  arm_position_controller_node
  PRIVATE arm_position_controller/include)
ament_target_dependencies(
  arm_position_controller_node rclcpp std_msgs)

install(TARGETS mecanum_controller_node spring_controller_node belt_controller_node dribbler_controller_node
                arm_position_controller_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY mecanum_controller/include/ DESTINATION include)
install(DIRECTORY spring_controller/include/ DESTINATION include)
install(DIRECTORY belt_controller/include/ DESTINATION include)
install(DIRECTORY dribbler_controller/include/ DESTINATION include)
install(DIRECTORY arm_position_controller/include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest QUIET)
  if(ament_cmake_gtest_FOUND)
    ament_add_gtest(
      test_robot_controller test/test_robot_controller.cpp belt_controller/src/belt_controller.cpp
      arm_position_controller/src/arm_position_controller.cpp dribbler_controller/src/dribbler_controller.cpp
      mecanum_controller/src/mecanum_controller_node.cpp)
    target_include_directories(test_robot_controller PRIVATE belt_controller/include arm_position_controller/include
                                                             dribbler_controller/include mecanum_controller/include)
    ament_target_dependencies(test_robot_controller rclcpp std_msgs geometry_msgs)
  endif()
endif()

ament_package()
