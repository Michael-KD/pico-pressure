# Example firmware for the Raspberry Pi Pico + REYAX RYLR998.
#
# This is a standalone pico-sdk project (NOT built by the host build at
# the repo root — the host build only compiles the library and tests,
# including the platform-free part of the RYLR998 driver).
#
# Build (needs the pico-sdk and arm-none-eabi-gcc):
#   export PICO_SDK_PATH=/path/to/pico-sdk
#   cmake -B build -G Ninja examples
#   cmake --build build
# Flash examples/build/mesh_field_node.uf2 to each field node (set a
# unique NODE_ID in node_config.h first) and mesh_base_station.uf2 to
# the gateway Pico.
cmake_minimum_required(VERSION 3.16)

include(pico_sdk_import.cmake)

project(mesh_examples C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

# The mesh library, reused as-is. Its CMakeLists skips host tests when
# it is not the top-level project.
add_subdirectory(.. mesh)

# Portable RYLR998 AT driver core + Pico UART glue.
add_library(rylr998 STATIC
    radios/RYLR998/rylr998.c
    rylr998_pico.c
)
target_include_directories(rylr998 PUBLIC
    radios/RYLR998
    ${CMAKE_CURRENT_SOURCE_DIR}   # node_config.h, rylr998_pico.h
)
target_link_libraries(rylr998 PUBLIC
    mesh
    pico_stdlib
    hardware_uart
    hardware_irq
)

function(mesh_example name)
    add_executable(${name} ${ARGN})
    target_link_libraries(${name} PRIVATE mesh rylr998 pico_stdlib)
    target_compile_options(${name} PRIVATE -Wall -Wextra)
    # Logs and (for the base station) the CLI go over USB serial.
    pico_enable_stdio_usb(${name} 1)
    pico_enable_stdio_uart(${name} 0)  # uart pins belong to the radio
    pico_add_extra_outputs(${name})    # .uf2 for drag-and-drop flashing
endfunction()

mesh_example(mesh_field_node main.c)
target_link_libraries(mesh_field_node PRIVATE hardware_adc pico_unique_id)

mesh_example(mesh_base_station base_station.c)
