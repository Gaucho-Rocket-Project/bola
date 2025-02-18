cmake_minimum_required(VERSION 3.10)

# Project Name and Language
project(v1 LANGUAGES C CXX)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Define directories
set(INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")
set(SRC_DIR "${PROJECT_SOURCE_DIR}/src")
include_directories(${INCLUDE_DIR})

# Find all source files
file(GLOB CPP_SOURCES "${SRC_DIR}/*.cpp")
file(GLOB C_SOURCES "${SRC_DIR}/*.c")

# Separate main and test sources
set(MAIN "${SRC_DIR}/main.cpp")
set(TEST "${SRC_DIR}/test.cpp")

# Exclude main and test from common sources
list(REMOVE_ITEM CPP_SOURCES ${MAIN} ${TEST})

# Create a library for common utilities
add_library(utils STATIC ${C_SOURCES} ${CPP_SOURCES})

# Define the main executable
add_executable(main ${MAIN})
target_link_libraries(main utils wiringPi pthread)

# Define the test executable
add_executable(test ${TEST})
target_link_libraries(test utils wiringPi pthread)
