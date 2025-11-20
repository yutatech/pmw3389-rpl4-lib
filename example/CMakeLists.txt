cmake_minimum_required(VERSION 3.22)
project(pmw3389_example)

# Create executable
add_executable(${PROJECT_NAME} 
    main.cc
    peripheral_rpl4.cc
)

# Link against the pmw3389-rpl4-lib library and RPL4
target_link_libraries(${PROJECT_NAME}
    pmw3389_rpl4_lib
    rpl4
)
