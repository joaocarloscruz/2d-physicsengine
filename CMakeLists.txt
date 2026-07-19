cmake_minimum_required(VERSION 3.10)
project(Engine CXX)

# Source directory for library
set(PHYSICS_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/physics)
# Source directory for tests
set(TESTS_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/tests)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${TESTS_SRC_DIR}
)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Emscripten links the engine into the generated WebAssembly module. Native
# consumers continue to receive the shared library used by the visualizer.
if(EMSCRIPTEN)
    set(ENGINE_LIBRARY_TYPE STATIC)
else()
    set(ENGINE_LIBRARY_TYPE SHARED)
endif()

# --- Engine Library
add_library(Engine ${ENGINE_LIBRARY_TYPE}
    src/engine.cpp
    ${PHYSICS_SRC_DIR}/math/vector2.cpp
    ${PHYSICS_SRC_DIR}/math/matrix2x2.cpp
    ${PHYSICS_SRC_DIR}/core/rigidbody.cpp
    ${PHYSICS_SRC_DIR}/core/simulation_config.cpp
    ${PHYSICS_SRC_DIR}/core/fixed_step_runner.cpp
    ${PHYSICS_SRC_DIR}/core/world.cpp
    ${PHYSICS_SRC_DIR}/core/forces/gravity.cpp
    ${PHYSICS_SRC_DIR}/core/forces/drag.cpp
    ${PHYSICS_SRC_DIR}/core/particles/particle.cpp
    ${PHYSICS_SRC_DIR}/core/particles/particle_system.cpp
    ${PHYSICS_SRC_DIR}/core/particles/particle_spatial_grid.cpp
    ${PHYSICS_SRC_DIR}/core/fluids/fluid_particle.cpp
    ${PHYSICS_SRC_DIR}/core/fluids/fluid_particle_spatial_grid.cpp
    ${PHYSICS_SRC_DIR}/core/fluids/sph_kernels.cpp
    ${PHYSICS_SRC_DIR}/core/fluids/wcsph_solver.cpp
    ${PHYSICS_SRC_DIR}/core/fluids/fluid_boundary.cpp
    ${PHYSICS_SRC_DIR}/core/collisions/collision_dispatcher.cpp
    ${PHYSICS_SRC_DIR}/core/collisions/narrow_phase/collision_circle_circle.cpp
    ${PHYSICS_SRC_DIR}/core/collisions/narrow_phase/collision_polygon_polygon.cpp
    ${PHYSICS_SRC_DIR}/core/collisions/narrow_phase/collision_circle_polygon.cpp
    ${PHYSICS_SRC_DIR}/core/collisions/collision_resolver.cpp
    ${PHYSICS_SRC_DIR}/core/collisions/broad_phase/sweep_and_prune.cpp
    ${PHYSICS_SRC_DIR}/core/collisions/broad_phase/uniform_grid.cpp
)

if(WIN32)
    set_target_properties(Engine PROPERTIES
        WINDOWS_EXPORT_ALL_SYMBOLS ON
    )
endif()

if(EMSCRIPTEN)
    # --- WebAssembly module
    add_executable(physics_engine_wasm
        src/wasm/bindings.cpp
    )

    target_link_libraries(physics_engine_wasm PRIVATE Engine embind)
    target_compile_options(physics_engine_wasm PRIVATE -fexceptions)
    target_link_options(physics_engine_wasm PRIVATE
        -fexceptions
        "-sMODULARIZE=1"
        "-sEXPORT_NAME=createPhysicsEngineModule"
        "-sENVIRONMENT=web,node"
        "-sALLOW_MEMORY_GROWTH=1"
        "-sDISABLE_EXCEPTION_CATCHING=0"
    )

    set(WASM_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/wasm")
    set_target_properties(physics_engine_wasm PROPERTIES
        OUTPUT_NAME "physics_engine"
        SUFFIX ".js"
        RUNTIME_OUTPUT_DIRECTORY "${WASM_OUTPUT_DIRECTORY}"
    )

    add_custom_command(TARGET physics_engine_wasm POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/examples/wasm-smoke/index.html"
            "${WASM_OUTPUT_DIRECTORY}/index.html"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/examples/wasm-smoke/smoke.js"
            "${WASM_OUTPUT_DIRECTORY}/smoke.js"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/examples/wasm-smoke/smoke-test.cjs"
            "${WASM_OUTPUT_DIRECTORY}/smoke-test.cjs"
        COMMENT "Copying WebAssembly smoke tests"
    )
else()
    # --- Native Test Executable
    add_executable(run_tests
        ${TESTS_SRC_DIR}/main_tests.cpp
        ${TESTS_SRC_DIR}/test_math.cpp
        ${TESTS_SRC_DIR}/test_rigidbody.cpp
        ${TESTS_SRC_DIR}/test_world.cpp
        ${TESTS_SRC_DIR}/test_forces.cpp
        ${TESTS_SRC_DIR}/test_engine.cpp
        ${TESTS_SRC_DIR}/test_collision.cpp
        ${TESTS_SRC_DIR}/test_broad_phase.cpp
        ${TESTS_SRC_DIR}/test_simulation.cpp
        ${TESTS_SRC_DIR}/test_collision_listener.cpp
        ${TESTS_SRC_DIR}/test_particles.cpp
        ${TESTS_SRC_DIR}/test_fluid_particles.cpp
        ${TESTS_SRC_DIR}/test_sph_kernels.cpp
        ${TESTS_SRC_DIR}/test_wcsph_solver.cpp
        ${TESTS_SRC_DIR}/test_fluid_boundaries.cpp
        ${TESTS_SRC_DIR}/test_physics_invariants.cpp
        ${TESTS_SRC_DIR}/test_collision_properties.cpp
        ${TESTS_SRC_DIR}/test_collision_filter.cpp
        ${TESTS_SRC_DIR}/test_simulation_config.cpp
        ${TESTS_SRC_DIR}/test_fixed_step.cpp
        ${TESTS_SRC_DIR}/test_simulation_statistics.cpp
        ${TESTS_SRC_DIR}/test_contact_manifold.cpp
        ${TESTS_SRC_DIR}/test_broad_phase_differential.cpp
        ${TESTS_SRC_DIR}/test_stress.cpp
        ${TESTS_SRC_DIR}/test_validation.cpp
        ${TESTS_SRC_DIR}/catch_amalgamated.cpp
    )

    target_link_libraries(run_tests PRIVATE Engine)

    add_executable(numerical_validation
        ${TESTS_SRC_DIR}/numerical_validation.cpp
    )
    target_link_libraries(numerical_validation PRIVATE Engine)

    enable_testing()
    add_test(NAME engine_tests COMMAND run_tests)
    add_test(
        NAME numerical_validation_fast
        COMMAND numerical_validation
            --mode fast
            --output ${CMAKE_BINARY_DIR}/numerical-validation-fast.json
    )
    set_tests_properties(numerical_validation_fast PROPERTIES
        LABELS "fast;numerical"
    )
endif()
