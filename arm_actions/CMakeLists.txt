cmake_minimum_required(VERSION 3.12)

project(arm_actions)

find_package(orocos_kdl REQUIRED)

INCLUDE_DIRECTORIES(
  ${orocos_kdl_INCLUDE_DIRS}  
)

# get project root directory
get_filename_component(PROJECT_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
message(STATUS "Project root directory: ${PROJECT_ROOT_DIR}")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_INSTALL_PREFIX}/../outputs/${PROJECT_NAME}/)

# create logs folder in outputs folder if not exists
file(MAKE_DIRECTORY ${CMAKE_INSTALL_PREFIX}/../outputs/logs)

set(MUJOCO_PATH "${PROJECT_ROOT_DIR}/mujoco/mujoco-2.3.7")

include_directories(${MUJOCO_PATH}/include)
link_directories(${MUJOCO_PATH}/bin)

find_package(glfw3 REQUIRED)
include_directories(${GLFW3_INCLUDE_DIRS})

# include all hpp files in include folder
include_directories(include)

# add pid_controller.cpp as a library
add_library(pid_controller SHARED src/utils/pid_controller.cpp)

# add logger.cpp as library
add_library(logger SHARED src/utils/logger.cpp)
target_link_libraries(logger ${orocos_kdl_LIBRARIES})

# add utils.cpp as library
add_library(utils SHARED src/utils/utils.cpp)
target_link_libraries(utils ${orocos_kdl_LIBRARIES} kdl_parser logger)

# add solver_utils.cpp as library
add_library(solver_utils SHARED src/utils/solver_utils.cpp)
target_link_libraries(solver_utils ${orocos_kdl_LIBRARIES} logger)

# add math_utils.cpp as library
add_library(math_utils SHARED src/utils/math_utils.cpp)
target_link_libraries(math_utils ${orocos_kdl_LIBRARIES} logger orocos-kdl)

# add tf_utils.cpp as library
add_library(tf_utils SHARED src/utils/tf_utils.cpp)
target_link_libraries(tf_utils ${orocos_kdl_LIBRARIES} logger orocos-kdl)

add_library(monitor SHARED src/utils/monitors.cpp)
target_link_libraries(monitor ${orocos_kdl_LIBRARIES} logger utils math_utils)

add_library(gnu_plotter SHARED src/utils/gnu_plotter.cpp)
target_link_libraries(gnu_plotter boost_filesystem boost_system boost_iostreams orocos-kdl ${orocos_kdl_LIBRARIES})

add_library(kinova_mediator SHARED src/utils/kinova_mediator.cpp)
target_link_libraries(kinova_mediator ${orocos_kdl_LIBRARIES} orocos-kdl kdl_parser logger 
      ${PROJECT_ROOT_DIR}/kinova_api/lib/release/libKortexApiCpp.a pthread)

add_library(mujoco_env SHARED src/utils/mujoco_env.cpp)
target_link_libraries(mujoco_env ${MUJOCO_PATH}/lib/libmujoco.so glfw GL ${orocos_kdl_LIBRARIES} logger)

# add all cpp files in src folder
file(GLOB SOURCES "src/*.cpp") 

# make all cpp files in src folder as executable
foreach(source ${SOURCES})
    get_filename_component(name ${source} NAME_WE)
    add_executable(${name} ${source})
    TARGET_LINK_LIBRARIES(${name} 
      orocos-kdl boost_filesystem boost_system boost_iostreams
      pid_controller logger utils solver_utils gnu_plotter yaml-cpp monitor math_utils tf_utils
      mujoco_env kinova_mediator)
endforeach(source ${SOURCES})

# export urdf file to outputs folder
# file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/urdf DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})