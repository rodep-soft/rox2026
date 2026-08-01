cmake_minimum_required(VERSION 3.8)
project(hardware_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(can_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

function(add_protocol_library target source)
  add_library(${target} ${source})
  target_include_directories(${target} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                                              $<INSTALL_INTERFACE:include>)
  if(ARGN)
    ament_target_dependencies(${target} ${ARGN})
  endif()
endfunction()

function(add_hardware_node target source)
  add_executable(${target} ${source})
  target_include_directories(${target} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  if(ARGN)
    target_link_libraries(${target} ${ARGN})
  endif()
  ament_target_dependencies(${target} can_msgs rclcpp sensor_msgs std_msgs)
endfunction()

add_protocol_library(stm32_protocol src/protocol/stm32_protocol.cpp can_msgs)
add_protocol_library(edulite05_protocol src/protocol/edulite05_protocol.cpp)

add_hardware_node(stm32_node src/nodes/stm32_node.cpp stm32_protocol)
add_hardware_node(edulite05_node src/nodes/edulite05_node.cpp edulite05_protocol)
add_hardware_node(vesc_node src/nodes/vesc_node.cpp)

set(protocol_libraries stm32_protocol edulite05_protocol)
set(driver_nodes stm32_node edulite05_node vesc_node)

install(
  TARGETS ${protocol_libraries}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS ${driver_nodes} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(can_msgs)
ament_package()
