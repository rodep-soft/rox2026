cmake_minimum_required(VERSION 3.8)
project(hardware_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(can_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# STM32 Protocol Library
add_library(stm32_protocol src/protocol/stm32_protocol.cpp)
target_compile_features(stm32_protocol PUBLIC cxx_std_17)
target_include_directories(stm32_protocol
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(stm32_protocol can_msgs)

# EduLite 05 Protocol Library
add_library(edulite05_protocol src/protocol/edulite05_protocol.cpp)
target_compile_features(edulite05_protocol PUBLIC cxx_std_17)
target_include_directories(edulite05_protocol
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/edulite05_driver>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(edulite05_protocol can_msgs rclcpp std_msgs)

# STM32 Node Executable
add_executable(stm32_node src/nodes/stm32_node.cpp)
target_compile_features(stm32_node PUBLIC cxx_std_17)
target_link_libraries(stm32_node stm32_protocol)
ament_target_dependencies(stm32_node can_msgs rclcpp std_msgs)

# EduLite 05 Node Executable
add_executable(edulite05_node src/nodes/edulite05_node.cpp)
target_compile_features(edulite05_node PUBLIC cxx_std_17)
target_link_libraries(edulite05_node edulite05_protocol)
ament_target_dependencies(edulite05_node can_msgs rclcpp std_msgs)

install(
  TARGETS
    stm32_protocol
    stm32_node
    edulite05_protocol
    edulite05_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(can_msgs rclcpp std_msgs)
ament_export_libraries(stm32_protocol edulite05_protocol)
ament_package()

