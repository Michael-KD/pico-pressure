function(mesh_add_test name)
    add_executable(${name} ${ARGN})
    target_link_libraries(${name} PRIVATE mesh)
    # Unit tests may reach library internals via src/mesh_internal.h.
    target_include_directories(${name} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/src)
    if(MSVC)
        target_compile_options(${name} PRIVATE /W4 /WX)
    else()
        target_compile_options(${name} PRIVATE -Wall -Wextra -Werror -Wshadow)
    endif()
    add_test(NAME ${name} COMMAND ${name})
endfunction()

mesh_add_test(test_frame unit/test_frame.c)
mesh_add_test(test_mic unit/test_mic.c)
mesh_add_test(test_neighbor unit/test_neighbor.c)
mesh_add_test(test_tree unit/test_tree.c)
mesh_add_test(test_flood unit/test_flood.c)
mesh_add_test(test_sched unit/test_sched.c)
mesh_add_test(test_bulk unit/test_bulk.c)

# The RYLR998 example driver core is platform-free, so its AT state
# machine is tested here on the host (the Pico glue only moves bytes).
mesh_add_test(test_rylr998 unit/test_rylr998.c
    ${CMAKE_SOURCE_DIR}/examples/radios/RYLR998/rylr998.c)
target_include_directories(test_rylr998 PRIVATE
    ${CMAKE_SOURCE_DIR}/examples/radios/RYLR998)

mesh_add_test(test_scenarios sim/test_scenarios.c sim/sim_net.c)
target_include_directories(test_scenarios PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/sim)

# Interactive demo (not a test): prints tree formation, a command flood,
# and self-healing on a 10-node fleet.
add_executable(mesh_demo sim/demo.c sim/sim_net.c)
target_link_libraries(mesh_demo PRIVATE mesh)
target_include_directories(mesh_demo PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/sim
    ${CMAKE_SOURCE_DIR}/src)
if(NOT MSVC)
    target_compile_options(mesh_demo PRIVATE -Wall -Wextra -Werror -Wshadow)
endif()
