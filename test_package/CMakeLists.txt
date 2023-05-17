cmake_minimum_required(VERSION 3.12)

project(test)

find_package(orocos_kdl REQUIRED)

# INCLUDE_DIRECTORIES(${PROJ_SOURCE_DIR}/src ${PROJ_BINARY_DIR}/src)

INCLUDE_DIRECTORIES(
  ${orocos_kdl_INCLUDE_DIRS}  
)

add_executable(geometry geometry.cpp )
TARGET_LINK_LIBRARIES(geometry orocos-kdl)
