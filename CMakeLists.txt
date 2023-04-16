cmake_minimum_required(VERSION 3.0)

# Set the project name
project(test_rotm2euler)

# Set the path to the mecl library
set(MECL_DIR "./mecl")

# Find the mecl library
#find_package(mecl REQUIRED PATHS ${MECL_DIR})

# Set the include directories
#include_directories(${MECL_INCLUDE_DIRS})
# Set the include directories
include_directories(
    ${MECL_DIR}
    ${MECL_DIR}/* # Include all subdirectories in mecl
)
# Set the source files
#set(SOURCE_FILES "./src/rot2Euler.cpp")

# Create the executable
add_executable(test_rotm2euler rot2Eurler.cpp)
