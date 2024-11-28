################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

################################################################################
# Packages
################################################################################
# find_package(gazebo REQUIRED)
# find_package(ros_gz_bridge REQUIRED)
# find_package(ros_gz_image REQUIRED)
# find_package(ros_gz_sim REQUIRED)
# find_package(ignition-math6 REQUIRED)
# get_cmake_property(_variableNames VARIABLES)
# list (SORT _variableNames)
# foreach (_variableName ${_variableNames})
#     message(STATUS "${_variableName}=${${_variableName}}")
# endforeach()

################################################################################
# Build
################################################################################
link_directories(${IGNITION_MATH6_LIBRARY_DIRS})

include_directories(${IGNITION_MATH6_INCLUDE_DIRS})

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(obstacle1 SHARED obstacle1.cc)
target_link_libraries(obstacle1 ${GAZEBO_LIBRARIES})

add_library(obstacle2 SHARED obstacle2.cc)
target_link_libraries(obstacle2 ${GAZEBO_LIBRARIES})

add_library(obstacles SHARED obstacles.cc)
target_link_libraries(obstacles ${GAZEBO_LIBRARIES})
