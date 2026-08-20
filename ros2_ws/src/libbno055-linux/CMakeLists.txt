cmake_minimum_required(VERSION 3.10)
project(libbno055_linux VERSION 1.10.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Source files
set(SOURCES
    src/core/bno055.cpp
    src/core/bno055_c.cpp
)

# Header files
set(HEADERS
    include/libbno055-linux/bno055.hpp
    include/libbno055-linux/bno055_c.h
    include/libbno055-linux/controllers/heading_controller.hpp
)

# Add library
add_library(libbno055-linux SHARED ${SOURCES} ${HEADERS})

# Prevent duplicate "lib" prefix (e.g. liblibbno055-linux.so)
set_target_properties(libbno055-linux PROPERTIES
    PREFIX ""
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)

# Automatically sync rust/cpp/ bundled C++ sources with the main source tree.
# This prevents source drift between the Rust crate's embedded copy and the canonical source.
add_custom_command(TARGET libbno055-linux POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/src/core/bno055.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/rust/cpp/src/bno055.cpp
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/src/core/bno055_c.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/rust/cpp/src/bno055_c.cpp
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/include/libbno055-linux/bno055.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/rust/cpp/include/libbno055-linux/bno055.hpp
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/include/libbno055-linux/bno055_c.h
        ${CMAKE_CURRENT_SOURCE_DIR}/rust/cpp/include/libbno055-linux/bno055_c.h
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/include/libbno055-linux/transport.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/rust/cpp/include/libbno055-linux/transport.hpp
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/include/libbno055-linux/mock_transport.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/rust/cpp/include/libbno055-linux/mock_transport.hpp
    COMMENT "Syncing rust/cpp/ bundled sources with main source tree"
    VERBATIM
)
if(ENABLE_CLANG_TIDY AND CLANG_TIDY_EXE)
    set_target_properties(libbno055-linux PROPERTIES
        CXX_CLANG_TIDY "${CLANG_TIDY_EXE}"
    )
endif()

# Include directories
target_include_directories(libbno055-linux
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(libbno055-linux PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Clang-Tidy static analysis integration
option(ENABLE_CLANG_TIDY "Enable clang-tidy static analysis" ON)
if(ENABLE_CLANG_TIDY)
    find_program(CLANG_TIDY_EXE NAMES "clang-tidy")
    if(CLANG_TIDY_EXE)
        message(STATUS "clang-tidy enabled: ${CLANG_TIDY_EXE}")
    else()
        message(WARNING "clang-tidy not found, static analysis disabled.")
    endif()
endif()

# Dependencies
find_package(Threads REQUIRED)
target_link_libraries(libbno055-linux PRIVATE Threads::Threads)

# Optional: libgpiod for modern GPIO IRQ support (replaces deprecated sysfs /sys/class/gpio)
# Required: libgpiod >= 2.0 (gpiod_chip_open / gpiod_line_request API)
# Ubuntu/Debian: sudo apt install libgpiod-dev
# Arch Linux:    sudo pacman -S libgpiod
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(LIBGPIOD QUIET libgpiod>=2.0)
endif()
if(LIBGPIOD_FOUND)
    message(STATUS "libgpiod ${LIBGPIOD_VERSION} found -- enabling modern character device GPIO IRQ support.")
    target_compile_definitions(libbno055-linux PRIVATE HAVE_LIBGPIOD)
    target_include_directories(libbno055-linux PRIVATE ${LIBGPIOD_INCLUDE_DIRS})
    target_link_libraries(libbno055-linux PRIVATE ${LIBGPIOD_LIBRARIES})
else()
    message(STATUS "libgpiod not found -- falling back to legacy sysfs GPIO (deprecated since Linux 5.10).")
    message(STATUS "Install libgpiod >= 2.0 to enable the modern GPIO character device IRQ path.")
endif()

# Example executables
option(BUILD_EXAMPLES "Build example executables" ON)

if(BUILD_EXAMPLES)
    add_executable(calibrate_imu examples/cpp/calibrate.cpp)
    target_link_libraries(calibrate_imu PRIVATE libbno055-linux)

    add_executable(read_data_noexcept examples/cpp/read_data_noexcept.cpp)
    target_link_libraries(read_data_noexcept PRIVATE libbno055-linux)

    add_executable(read_all_data examples/cpp/read_all_data.cpp)
    target_link_libraries(read_all_data PRIVATE libbno055-linux)

    add_executable(benchmark_imu examples/cpp/benchmark_imu.cpp)
    target_link_libraries(benchmark_imu PRIVATE libbno055-linux)

    add_executable(heading_control_demo examples/cpp/heading_control_demo.cpp)
    target_link_libraries(heading_control_demo PRIVATE libbno055-linux)

    add_executable(c_demo examples/c/c_demo.c)
    target_link_libraries(c_demo PRIVATE libbno055-linux)
endif()

# Python Bindings (pybind11)
option(BUILD_PYTHON_BINDINGS "Build Python bindings using pybind11" ON)
if(BUILD_PYTHON_BINDINGS)
    find_package(pybind11 QUIET)
    if(NOT pybind11_FOUND)
        message(STATUS "pybind11 not found locally. Fetching pybind11 from GitHub...")
        include(FetchContent)
        FetchContent_Declare(
            pybind11
            GIT_REPOSITORY https://github.com/pybind/pybind11.git
            GIT_TAG        v2.12.0
        )
        FetchContent_MakeAvailable(pybind11)
    endif()

    pybind11_add_module(libbno055_py src/python/bindings.cpp)
    target_link_libraries(libbno055_py PRIVATE libbno055-linux)
    set_target_properties(libbno055_py PROPERTIES OUTPUT_NAME "libbno055")
endif()

# Optional ROS 2 integration (conditional build of ROS 2 publisher samples)
find_package(rclcpp QUIET)
find_package(rclcpp_components QUIET)
find_package(rclcpp_lifecycle QUIET)
find_package(sensor_msgs QUIET)
find_package(geometry_msgs QUIET)
find_package(diagnostic_msgs QUIET)
find_package(std_srvs QUIET)
find_package(tf2 QUIET)
find_package(tf2_ros QUIET)
find_package(tf2_geometry_msgs QUIET)

if(rclcpp_FOUND AND sensor_msgs_FOUND AND geometry_msgs_FOUND AND diagnostic_msgs_FOUND AND std_srvs_FOUND AND tf2_ros_FOUND)
    find_package(ament_cmake REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    set(ament_cmake_uncrustify_FOUND TRUE)
    set(ament_cmake_pep257_FOUND TRUE)
    set(ament_cmake_xmllint_FOUND TRUE)
    set(ament_cmake_flake8_FOUND TRUE)
    message(STATUS "ROS 2 environment detected. Building ROS 2 publisher and control node samples.")
    
    # --- 1. Standalone Executable (Publisher & Heading Control) ---
    add_executable(bno055_publisher_node src/ros2/ros2_publisher_node.cpp)
    target_link_libraries(bno055_publisher_node PUBLIC
        libbno055-linux
        rclcpp::rclcpp
        ${sensor_msgs_TARGETS}
        ${geometry_msgs_TARGETS}
        ${diagnostic_msgs_TARGETS}
        ${std_srvs_TARGETS}
        ${tf2_ros_TARGETS}
        ${tf2_geometry_msgs_TARGETS}
    )

    add_executable(bno055_heading_control_node src/ros2/ros2_heading_control_node.cpp)
    target_link_libraries(bno055_heading_control_node PUBLIC
        libbno055-linux
        rclcpp::rclcpp
        ${sensor_msgs_TARGETS}
        ${geometry_msgs_TARGETS}
        ${diagnostic_msgs_TARGETS}
        ${std_srvs_TARGETS}
        ${tf2_TARGETS}
        ${tf2_ros_TARGETS}
        ${tf2_geometry_msgs_TARGETS}
    )

    install(TARGETS bno055_publisher_node bno055_heading_control_node
        DESTINATION lib/${PROJECT_NAME}
    )

    # --- 2. Composable Node Shared Library (Publisher & Heading Control) ---
    if(rclcpp_components_FOUND)
        message(STATUS "rclcpp_components detected. Building Composable Node libraries.")
        add_library(bno055_ros2_components SHARED src/ros2/ros2_publisher_node.cpp)
        target_compile_definitions(bno055_ros2_components PRIVATE BNO055_ROS2_BUILDING_COMPONENT)
        target_link_libraries(bno055_ros2_components PUBLIC
            libbno055-linux
            rclcpp::rclcpp
            rclcpp_components::component
            ${sensor_msgs_TARGETS}
            ${geometry_msgs_TARGETS}
            ${diagnostic_msgs_TARGETS}
            ${std_srvs_TARGETS}
            ${tf2_ros_TARGETS}
            ${tf2_geometry_msgs_TARGETS}
        )
        rclcpp_components_register_node(bno055_ros2_components PLUGIN "bno055_ros2::BNO055PublisherNode" EXECUTABLE bno055_publisher_node_container)

        add_library(bno055_heading_control_components SHARED src/ros2/ros2_heading_control_node.cpp)
        target_compile_definitions(bno055_heading_control_components PRIVATE BNO055_ROS2_BUILDING_COMPONENT)
        target_link_libraries(bno055_heading_control_components PUBLIC
            libbno055-linux
            rclcpp::rclcpp
            rclcpp_components::component
            ${sensor_msgs_TARGETS}
            ${geometry_msgs_TARGETS}
            ${diagnostic_msgs_TARGETS}
            ${std_srvs_TARGETS}
            ${tf2_TARGETS}
            ${tf2_ros_TARGETS}
            ${tf2_geometry_msgs_TARGETS}
        )
        rclcpp_components_register_node(bno055_heading_control_components PLUGIN "bno055_ros2::BNO055HeadingControlNode" EXECUTABLE bno055_heading_control_container)

        install(TARGETS bno055_ros2_components bno055_heading_control_components
            ARCHIVE DESTINATION lib
            LIBRARY DESTINATION lib
            RUNTIME DESTINATION bin
        )
    endif()

    if(rclcpp_lifecycle_FOUND)
        message(STATUS "ROS 2 Lifecycle detected. Building Lifecycle Node sample.")
        # --- 3. Standalone Executable (Lifecycle) ---
        add_executable(bno055_lifecycle_publisher_node src/ros2/ros2_lifecycle_publisher_node.cpp)
        target_link_libraries(bno055_lifecycle_publisher_node PUBLIC
            libbno055-linux
            rclcpp::rclcpp
            rclcpp_lifecycle::rclcpp_lifecycle
            ${sensor_msgs_TARGETS}
            ${geometry_msgs_TARGETS}
            ${diagnostic_msgs_TARGETS}
            ${std_srvs_TARGETS}
            ${tf2_ros_TARGETS}
            ${tf2_geometry_msgs_TARGETS}
        )

        add_executable(bno055_lifecycle_heading_control_node src/ros2/ros2_lifecycle_heading_control_node.cpp)
        target_link_libraries(bno055_lifecycle_heading_control_node PUBLIC
            libbno055-linux
            rclcpp::rclcpp
            rclcpp_lifecycle::rclcpp_lifecycle
            ${sensor_msgs_TARGETS}
            ${geometry_msgs_TARGETS}
            ${diagnostic_msgs_TARGETS}
            ${std_srvs_TARGETS}
        )

        install(TARGETS bno055_lifecycle_publisher_node bno055_lifecycle_heading_control_node
            DESTINATION lib/${PROJECT_NAME}
        )

        # --- 4. Composable Node Shared Library (Lifecycle) ---
        if(rclcpp_components_FOUND)
            message(STATUS "Building Lifecycle Composable Node library.")
            add_library(bno055_ros2_lifecycle_components SHARED src/ros2/ros2_lifecycle_publisher_node.cpp)
            target_compile_definitions(bno055_ros2_lifecycle_components PRIVATE BNO055_ROS2_BUILDING_COMPONENT)
            target_link_libraries(bno055_ros2_lifecycle_components PUBLIC
                libbno055-linux
                rclcpp::rclcpp
                rclcpp_lifecycle::rclcpp_lifecycle
                rclcpp_components::component
                ${sensor_msgs_TARGETS}
                ${geometry_msgs_TARGETS}
                ${diagnostic_msgs_TARGETS}
                ${std_srvs_TARGETS}
                ${tf2_ros_TARGETS}
                ${tf2_geometry_msgs_TARGETS}
            )
            rclcpp_components_register_node(bno055_ros2_lifecycle_components PLUGIN "bno055_ros2::BNO055LifecyclePublisherNode" EXECUTABLE bno055_lifecycle_publisher_node_container)

            add_library(bno055_lifecycle_heading_control_components SHARED src/ros2/ros2_lifecycle_heading_control_node.cpp)
            target_compile_definitions(bno055_lifecycle_heading_control_components PRIVATE BNO055_ROS2_BUILDING_COMPONENT)
            target_link_libraries(bno055_lifecycle_heading_control_components PUBLIC
                libbno055-linux
                rclcpp::rclcpp
                rclcpp_lifecycle::rclcpp_lifecycle
                rclcpp_components::component
                ${sensor_msgs_TARGETS}
                ${geometry_msgs_TARGETS}
                ${diagnostic_msgs_TARGETS}
                ${std_srvs_TARGETS}
            )
            rclcpp_components_register_node(bno055_lifecycle_heading_control_components PLUGIN "bno055_ros2::BNO055LifecycleHeadingControlNode" EXECUTABLE bno055_lifecycle_heading_control_container)

            install(TARGETS bno055_ros2_lifecycle_components bno055_lifecycle_heading_control_components
                ARCHIVE DESTINATION lib
                LIBRARY DESTINATION lib
                RUNTIME DESTINATION bin
            )
        endif()
    endif()

    # Install ROS 2 configuration and launch files
    install(DIRECTORY config launch
        DESTINATION share/libbno055_linux
    )
else()
    message(STATUS "ROS 2 not found. Skipping ROS 2 publisher node sample.")
endif()


# Installation definitions
include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

# Install targets
install(TARGETS libbno055-linux
    EXPORT libbno055-linuxTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

if(BUILD_EXAMPLES)
    install(TARGETS calibrate_imu read_data_noexcept read_all_data benchmark_imu heading_control_demo c_demo
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
endif()

# Install headers
install(DIRECTORY include/libbno055-linux
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Export targets
install(EXPORT libbno055-linuxTargets
    FILE libbno055-linuxTargets.cmake
    NAMESPACE libbno055-linux::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/libbno055-linux
)

# Configure config file
configure_package_config_file(
    cmake/libbno055-linuxConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/libbno055-linuxConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/libbno055-linux
)

# Write version file
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/libbno055-linuxConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

# Install config files
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/libbno055-linuxConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/libbno055-linuxConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/libbno055-linux
)

# Testing
option(BUILD_TESTING "Build unit tests" OFF)
if(BUILD_TESTING AND NOT rclcpp_FOUND)
    # Try to find system-installed GTest (e.g., provided by apt, ROS, vcpkg, or Conan)
    # This prevents network access during offline builds (like ROS buildfarms)
    find_package(GTest QUIET)

    if(NOT GTest_FOUND)
        message(STATUS "GTest not found locally. Fetching GoogleTest from GitHub...")
        include(FetchContent)
        FetchContent_Declare(
          googletest
          GIT_REPOSITORY https://github.com/google/googletest.git
          GIT_TAG        release-1.12.1
        )
        set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
        
        # Disable clang-tidy for third-party GoogleTest to prevent false-positive errors
        set(BACKUP_CLANG_TIDY "${CMAKE_CXX_CLANG_TIDY}")
        set(CMAKE_CXX_CLANG_TIDY "")
        FetchContent_MakeAvailable(googletest)
        set(CMAKE_CXX_CLANG_TIDY "${BACKUP_CLANG_TIDY}")
    endif()

    enable_testing()

    add_executable(libbno055-linux_tests
        tests/test_bno055.cpp
        tests/test_heading_controller.cpp
    )

    if(TARGET GTest::gtest_main)
        target_link_libraries(libbno055-linux_tests
            PRIVATE
                libbno055-linux
                GTest::gtest_main
        )
    else()
        target_link_libraries(libbno055-linux_tests
            PRIVATE
                libbno055-linux
                gtest_main
        )
    endif()

    include(GoogleTest)
    gtest_discover_tests(libbno055-linux_tests)

    # If ROS 2 is available, also build the ROS 2 node integration tests
    if(rclcpp_FOUND AND sensor_msgs_FOUND AND rclcpp_lifecycle_FOUND AND diagnostic_msgs_FOUND AND std_srvs_FOUND)
        message(STATUS "ROS 2 detected. Building ROS 2 node integration tests.")
        add_executable(libbno055-linux_ros2_tests
            tests/test_ros2_nodes.cpp
        )
        target_compile_definitions(libbno055-linux_ros2_tests PRIVATE ROS2_NODE_TESTING)
        
        if(TARGET GTest::gtest_main)
            target_link_libraries(libbno055-linux_ros2_tests PUBLIC libbno055-linux GTest::gtest_main)
        else()
            target_link_libraries(libbno055-linux_ros2_tests PUBLIC libbno055-linux gtest_main)
        endif()
        target_link_libraries(libbno055-linux_ros2_tests PUBLIC
            rclcpp::rclcpp
            rclcpp_lifecycle::rclcpp_lifecycle
            ${sensor_msgs_TARGETS}
            ${diagnostic_msgs_TARGETS}
            ${std_srvs_TARGETS}
        )
        gtest_discover_tests(libbno055-linux_ros2_tests)
    endif()
endif()

if(rclcpp_FOUND AND sensor_msgs_FOUND)
    ament_package()
endif()

# ---------------------------------------------------------
# CPack Packaging for non-ROS users (.deb / .rpm)
# ---------------------------------------------------------
set(CPACK_PACKAGE_NAME "libbno055-linux")
set(CPACK_PACKAGE_VENDOR "lazytatzv")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "A C++17 library for the BNO055 sensor over I2C.")
set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")

# Exclude ROS components from the standalone debian package
if(rclcpp_FOUND)
    set(CPACK_COMPONENTS_ALL_IN_ONE_PACKAGE 1)
endif()

# Debian specific
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64") # Note: Will be overridden by build system if cross-compiling
set(CPACK_DEBIAN_PACKAGE_DEPENDS "")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "lazytatzv")
set(CPACK_DEBIAN_PACKAGE_SECTION "devel")

set(CPACK_GENERATOR "DEB;TGZ")
include(CPack)
