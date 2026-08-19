cmake_minimum_required(VERSION 3.16)
project(test_package CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(libbno055-linux REQUIRED)

add_executable(test_package src/test_package.cpp)
target_link_libraries(test_package PRIVATE libbno055-linux::libbno055-linux)
