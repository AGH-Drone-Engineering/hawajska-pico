cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(hawajska C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(main
    ${CMAKE_CURRENT_LIST_DIR}/src/main.c
    ${CMAKE_CURRENT_LIST_DIR}/src/gyro.c
    ${CMAKE_CURRENT_LIST_DIR}/src/sbus.c
    ${CMAKE_CURRENT_LIST_DIR}/src/motors.c
)
target_link_libraries(main pico_stdlib hardware_spi hardware_uart hardware_pwm)
pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)
pico_add_extra_outputs(main)
