cmake_minimum_required(VERSION 3.12)

project(arm_actions)

find_package(orocos_kdl REQUIRED)

INCLUDE_DIRECTORIES(
  ${orocos_kdl_INCLUDE_DIRS}  
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_INSTALL_PREFIX}/../outputs/${PROJECT_NAME}/)

# add all cpp files in src folder
file(GLOB SOURCES "src/*.cpp")

# make all cpp files in src folder as executable
foreach(source ${SOURCES})
    get_filename_component(name ${source} NAME_WE)
    add_executable(${name} ${source})
    TARGET_LINK_LIBRARIES(${name} orocos-kdl kdl_parser)
endforeach(source ${SOURCES})