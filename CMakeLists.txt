cmake_minimum_required(VERSION 3.12)

project(MS_VS_DSL)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/../install)

# add subdirectories
add_subdirectory(orocos_kinematics_dynamics/orocos_kdl)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/orocos_kinematics_dynamics/orocos_kdl/src/)

add_subdirectory(test_package)