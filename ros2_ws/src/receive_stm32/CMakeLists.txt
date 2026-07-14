cmake_minimum_required(VERSION 3.8)
project(receive_stm32)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 依存パッケージの検索設定 (追加)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(limit_sw_node src/receive_stm32_node.cpp)

# 3. 実行ファイルへの依存関係の紐付け (追加)
ament_target_dependencies(
  limit_sw_node rclcpp can_msgs std_msgs)

# 4. ros2 run で動かせるようにするためのインストール設定 (追加)
install(TARGETS limit_sw_node DESTINATION lib/${PROJECT_NAME})

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
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
