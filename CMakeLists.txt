cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(main)

pico_sdk_init()

add_executable(main    
    main.c
    minmea.c
)

pico_add_extra_outputs(main)

target_link_libraries(main 
    pico_stdlib 
    hardware_uart
    pico_cyw43_arch_none
)

add_custom_target(flash
    COMMAND /bin/bash ../flash
    DEPENDS main
)

pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 1)