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

target_link_libraries(${PROJECT_NAME}
    rpl4
)

# Build examples if enabled
if(BUILD_EXAMPLES)
    add_subdirectory(example)
endif()
