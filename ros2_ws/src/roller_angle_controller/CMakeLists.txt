cmake_minimum_required(VERSION 3.8)
project(roller_angle_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_msgs REQUIRED)

# 実行ファイル名とソースコードの指定
add_executable(roller_angle_controller_node src/roller_angle_controller.cpp)

# 【修正】ROS 2の標準的な依存関係のリンク方法に変更しました
ament_target_dependencies(
  roller_angle_controller_node rclcpp sensor_msgs std_msgs can_msgs)

# インストール設定
install(TARGETS roller_angle_controller_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
