cmake_minimum_required(VERSION 3.12)

project(MS_VS_DSL)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/../install)

# add subdirectories
add_subdirectory(orocos_kinematics_dynamics/orocos_kdl)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/orocos_kinematics_dynamics/orocos_kdl/src/)

add_subdirectory(kdl_parser/kdl_parser)

add_compile_definitions(_OS_UNIX)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/kdl_parser/kdl_parser/include/)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api/include
    ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api/include/client
    ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api/include/common
    ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api/include/messages
    ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api/include/client_stubs
    ${CMAKE_CURRENT_SOURCE_DIR}/kinova_api/include/google
)

add_subdirectory(arm_actions)