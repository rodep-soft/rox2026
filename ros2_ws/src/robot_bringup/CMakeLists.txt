cmake_minimum_required(VERSION 3.8)
project(robot_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
file(GLOB launch_files "${CMAKE_CURRENT_SOURCE_DIR}/launch/*.launch.py")
install(FILES ${launch_files} DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY launch/navigate/ DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
