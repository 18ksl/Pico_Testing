cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(my_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(my_code
    my_code.c
)

target_link_libraries(my_code pico_stdlib)

pico_add_extra_outputs(my_code)

