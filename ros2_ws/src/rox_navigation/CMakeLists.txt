cmake_minimum_required(VERSION 3.8)
project(rox_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(tag_localization_node src/tag_localization_node.cpp)
ament_target_dependencies(tag_localization_node geometry_msgs rclcpp tf2 tf2_geometry_msgs tf2_ros)

install(TARGETS tag_localization_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY behavior_trees maps urdf DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
