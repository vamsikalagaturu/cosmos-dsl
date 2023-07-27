cmake_minimum_required(VERSION 3.12)

project(arm_actions)

find_package(orocos_kdl REQUIRED)

INCLUDE_DIRECTORIES(
  ${orocos_kdl_INCLUDE_DIRS}  
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_INSTALL_PREFIX}/../outputs/${PROJECT_NAME}/)

# create logs folder in outputs folder if not exists
file(MAKE_DIRECTORY ${CMAKE_INSTALL_PREFIX}/../outputs/logs)

# include all hpp files in include folder
include_directories(include)

# add pid_controller.cpp as a library
add_library(pid_controller src/utils/pid_controller.cpp)

# add logger.cpp as library
add_library(logger src/utils/logger.cpp)
target_link_libraries(logger ${orocos_kdl_LIBRARIES})

# add utils.cpp as library
add_library(utils src/utils/utils.cpp)
target_link_libraries(utils ${orocos_kdl_LIBRARIES} kdl_parser logger)

# add solver_utils.cpp as library
add_library(solver_utils src/utils/solver_utils.cpp)
target_link_libraries(solver_utils ${orocos_kdl_LIBRARIES} logger)

# add math_utils.cpp as library
add_library(math_utils src/utils/math_utils.cpp)
target_link_libraries(math_utils ${orocos_kdl_LIBRARIES} logger orocos-kdl)

add_library(monitor src/utils/monitors.cpp)
target_link_libraries(monitor ${orocos_kdl_LIBRARIES} logger utils math_utils)

add_library(gnu_plotter src/utils/gnu_plotter.cpp)
target_link_libraries(gnu_plotter boost_filesystem boost_system boost_iostreams orocos-kdl ${orocos_kdl_LIBRARIES})

add_library(kinova_mediator src/utils/kinova_mediator.cpp)
target_link_libraries(kinova_mediator ${orocos_kdl_LIBRARIES} logger)

# add all cpp files in src folder
file(GLOB SOURCES "src/*.cpp") 
# file(GLOB SOURCES "src/utils/*.cpp")

# make all cpp files in src folder as executable
foreach(source ${SOURCES})
    get_filename_component(name ${source} NAME_WE)
    add_executable(${name} ${source})
    TARGET_LINK_LIBRARIES(${name} 
      orocos-kdl boost_filesystem boost_system boost_iostreams 
      pid_controller logger utils solver_utils gnu_plotter yaml-cpp monitor math_utils
      /home/batsy/rnd/src/kinova_api/lib/release/libKortexApiCpp.a)
endforeach(source ${SOURCES})

# export urdf file to outputs folder
# file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/urdf DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})