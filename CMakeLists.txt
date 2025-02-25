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

# Check if WiringPi is available
find_path(WIRINGPI_INCLUDE_DIR NAMES wiringPi.h)
find_library(WIRINGPI_LIBRARY NAMES wiringPi)

if(WIRINGPI_INCLUDE_DIR AND WIRINGPI_LIBRARY)
    message(STATUS "WiringPi found: ${WIRINGPI_INCLUDE_DIR}")
    set(USE_WIRINGPI TRUE)
else()
    message(WARNING "WiringPi not found! Using test implementation.")
    set(USE_WIRINGPI FALSE)
endif()

# Create a library for common utilities
add_library(utils STATIC ${C_SOURCES} ${CPP_SOURCES})

# Create the ICM20948 driver library
add_library(icm20948 STATIC ${SRC_DIR}/icm20948.c)
target_include_directories(icm20948 PRIVATE ${INCLUDE_DIR})

# Define the main executable (use WiringPi if available)
add_executable(main ${MAIN})
if(USE_WIRINGPI)
    target_compile_definitions(main PRIVATE USE_WIRINGPI)
    target_link_libraries(main utils ${WIRINGPI_LIBRARY} icm20948 pthread)
else()
    target_link_libraries(main utils icm20948 pthread)  # No WiringPi if not found
endif()

# Define the test executable (always uses dummy WiringPi)
add_executable(test ${TEST} ${SRC_DIR}/wiringPiTest.cpp)
target_compile_definitions(test PRIVATE USE_WIRINGPI=0)  # Explicitly disable WiringPi
target_link_libraries(test utils icm20948 pthread)
