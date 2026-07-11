cmake_minimum_required(VERSION 3.16)
project(pico_pressure_mesh C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

add_library(mesh STATIC
    src/frame.c
    src/mic_null.c
    src/mic_cmac.c
    src/txqueue.c
    src/sched.c
    src/neighbor.c
    src/tree.c
    src/beacon.c
    src/flood.c
    src/command.c
    src/telemetry.c
    src/mesh.c
)
target_include_directories(mesh PUBLIC include)

if(MSVC)
    target_compile_options(mesh PRIVATE /W4 /WX)
else()
    target_compile_options(mesh PRIVATE -Wall -Wextra -Werror -Wshadow)
endif()

# Host-side tests and simulator. Skipped in cross builds (e.g., pico-sdk)
# unless testing is explicitly requested.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    enable_testing()
    add_subdirectory(tests)
endif()
