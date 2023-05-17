cmake_minimum_required(VERSION 3.12)

project(arm_actions)

find_package(orocos_kdl REQUIRED)

INCLUDE_DIRECTORIES(
  ${orocos_kdl_INCLUDE_DIRS}  
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_INSTALL_PREFIX}/../outputs/${PROJECT_NAME}/)

add_executable(geometry src/geometry.cpp)
TARGET_LINK_LIBRARIES(geometry orocos-kdl)
