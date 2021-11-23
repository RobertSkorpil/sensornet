cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)
pico_sdk_init()

project(base_station C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
)

add_executable(base_station src/main.cc)

target_link_libraries(base_station hardware_gpio pico_stdlib pico_time)

pico_enable_stdio_usb(base_station 1)

pico_add_extra_outputs(base_station)
