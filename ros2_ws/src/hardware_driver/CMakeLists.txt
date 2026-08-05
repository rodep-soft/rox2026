cmake_minimum_required(VERSION 3.8)
project(hardware_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(can_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
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
  add_library(${target}_component SHARED ${source})
  target_include_directories(${target}_component PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  if(ARGN)
    target_link_libraries(${target}_component ${ARGN})
  endif()
  ament_target_dependencies(${target}_component can_msgs rclcpp rclcpp_components sensor_msgs std_msgs)
endfunction()

add_protocol_library(stm32_protocol src/protocol/stm32_protocol.cpp can_msgs)
add_protocol_library(edulite05_protocol src/protocol/edulite05_protocol.cpp)

add_hardware_node(stm32_node src/nodes/stm32_node.cpp stm32_protocol)
rclcpp_components_register_node(stm32_node_component PLUGIN "stm32_driver::Stm32Node" EXECUTABLE stm32_node)
target_include_directories(stm32_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(stm32_node stm32_protocol)
ament_target_dependencies(stm32_node can_msgs rclcpp rclcpp_components sensor_msgs std_msgs)

add_hardware_node(edulite05_node src/nodes/edulite05_node.cpp edulite05_protocol)
rclcpp_components_register_node(edulite05_node_component PLUGIN "Ed05DriverNode" EXECUTABLE edulite05_node)
target_include_directories(edulite05_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(edulite05_node edulite05_protocol)
ament_target_dependencies(edulite05_node can_msgs rclcpp rclcpp_components sensor_msgs std_msgs)

add_hardware_node(vesc_node src/nodes/vesc_node.cpp)
rclcpp_components_register_node(vesc_node_component PLUGIN "vesc_driver::Node" EXECUTABLE vesc_node)
target_include_directories(vesc_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(vesc_node can_msgs rclcpp rclcpp_components sensor_msgs std_msgs)

set(protocol_libraries stm32_protocol edulite05_protocol stm32_node_component edulite05_node_component vesc_node_component)
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
