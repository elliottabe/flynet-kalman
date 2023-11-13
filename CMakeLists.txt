cmake_minimum_required (VERSION 3.0)
project(KALMAN)

include(ExternalProject)

set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find default python libraries and interpreter
find_package(PythonLibs 3.8 COMPONENTS Interpreter Development REQUIRED)
find_package(Armadillo REQUIRED)
find_package(Boost COMPONENTS python numpy REQUIRED)

include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIR})

message(STATUS "PYTHON_INCLUDE_DIR = " ${PYTHON_INCLUDE_DIRS})
message(STATUS "PYTHON_LIBRARIES   = " ${PYTHON_LIBRARIES})
message(STATUS "Boost_LIBRARIES    = " ${Boost_LIBRARIES})

# Build and link the pylib module
add_library(_kalman_lib SHARED flynet_kalman/kalman_lib.cpp flynet_kalman/kalman_filter.cpp)
target_link_libraries(_kalman_lib ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} ${ARMADILLO_LIBRARIES} )

# Tweaks the name of the library to match what Python expects
set_target_properties(_kalman_lib PROPERTIES SUFFIX .so)
set_target_properties(_kalman_lib PROPERTIES PREFIX "")

install(TARGETS _kalman_lib LIBRARY DESTINATION flynet_kalman)
