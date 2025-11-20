cmake_minimum_required(VERSION 3.22)
project(pmw3389_rpl4_lib)

# Option to build examples
option(BUILD_EXAMPLES "Build example programs" OFF)

add_library(${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC
    include
)

file(GLOB SRCS src/*.c*)
target_sources(${PROJECT_NAME} PRIVATE ${SRCS})

# No default dependencies - users provide peripheral implementation

# Build examples if enabled
if(BUILD_EXAMPLES)
    # Add RPL4 library only when building examples
    add_subdirectory(RPL4)
    add_subdirectory(example)
endif()
