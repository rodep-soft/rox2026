cmake_minimum_required(VERSION 3.8)
project(imu_bno055)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 必要なパッケージ
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# 実行ファイル
add_executable(bno055_node src/bno055.cpp src/imu_node.cpp)

# ヘッダファイルの場所
target_include_directories(
  bno055_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

# C++17
target_compile_features(
  bno055_node
  PUBLIC cxx_std_17)

# ROS2ライブラリ
ament_target_dependencies(
  bno055_node rclcpp)

# インストール
install(TARGETS bno055_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

ament_package()
