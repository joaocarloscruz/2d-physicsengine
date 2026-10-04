cmake_minimum_required(VERSION 3.16)
project(PhysicsEngine VERSION 0.2.0 LANGUAGES CXX)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(CTest)
option(BUILD_SHARED_LIBS "Build the native shared library" ON)

# Source directory for library
set(PHYSICS_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/physics)
# Source directory for tests
set(TESTS_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/tests)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Emscripten links the engine into the generated WebAssembly module. Native
# consumers continue to receive the shared library used by the visualizer.
if(EMSCRIPTEN)
    set(ENGINE_LIBRARY_TYPE STATIC)
elseif(BUILD_SHARED_LIBS)
    set(ENGINE_LIBRARY_TYPE SHARED)
else()
    set(ENGINE_LIBRARY_TYPE STATIC)
endif()

# --- Engine Library
add_library(Engine ${ENGINE_LIBRARY_TYPE}
    src/engine.cpp
    ${PHYSICS_SRC_DIR}/math/vector2.cpp
    ${PHYSICS_SRC_DIR}/math/matrix2x2.cpp
    ${PHYSICS_SRC_DIR}/core/rigidbody.cpp
    ${PHYSICS_SRC_DIR}/core/charged_particle.cpp
    ${PHYSICS_SRC_DIR}/core/nbody_gravity.cpp
    ${PHYSICS_SRC_DIR}/core/simulation_config.cpp
    ${PHYSICS_SRC_DIR}/core/fixed_step_runner.cpp
    ${PHYSICS_SRC_DIR}/core/world.cpp
    ${PHYSICS_SRC_DIR}/core/joints.cpp
    ${PHYSICS_SRC_DIR}/core/prismatic_joint.cpp
    ${PHYSICS_SRC_DIR}/core/softbody.cpp
    ${PHYSICS_SRC_DIR}/core/thermal_network.cpp
    ${PHYSICS_SRC_DIR}/core/state_export.cpp
    ${PHYSICS_SRC_DIR}/core/spatial_queries.cpp
    ${PHYSICS_SRC_DIR}/core/forces/gravity.cpp
    ${PHYSICS_SRC_DIR}/core/forces/drag.cpp
    ${PHYSICS_SRC_DIR}/core/particles/particle.cpp
    ${PHYSICS_SRC_DIR}/core/particles/particle_system.cpp
    ${PHYSICS_SRC_DIR}/core/particles/particle_spatial_grid.cpp
    ${PHYSICS_SRC_DIR}/core/fluids/fluid_particle.cpp
    ${PHYSICS_SRC_DIR}/core/fluids/fluid_particle_spatial_grid.cpp
    ${PHYSICS_SRC_DIR}/core/fluids/sph_kernels.cpp
    ${PHYSICS_SRC_DIR}/core/fluids/wcsph_solver.cpp
    ${PHYSICS_SRC_DIR}/core/fluids/fluid_solver.cpp
    ${PHYSICS_SRC_DIR}/core/fluids/dfsph_solver.cpp
    ${PHYSICS_SRC_DIR}/core/fluids/fluid_boundary.cpp
    ${PHYSICS_SRC_DIR}/core/fluids/fluid_rigid_coupler.cpp
    ${PHYSICS_SRC_DIR}/core/fluids/coupled_fluid_simulation.cpp
    ${PHYSICS_SRC_DIR}/core/collisions/collision_dispatcher.cpp
    ${PHYSICS_SRC_DIR}/core/collisions/continuous_collision.cpp
    ${PHYSICS_SRC_DIR}/core/collisions/narrow_phase/collision_circle_circle.cpp
    ${PHYSICS_SRC_DIR}/core/collisions/narrow_phase/collision_polygon_polygon.cpp
    ${PHYSICS_SRC_DIR}/core/collisions/narrow_phase/collision_circle_polygon.cpp
    ${PHYSICS_SRC_DIR}/core/collisions/collision_resolver.cpp
    ${PHYSICS_SRC_DIR}/core/collisions/broad_phase/sweep_and_prune.cpp
    ${PHYSICS_SRC_DIR}/core/collisions/broad_phase/uniform_grid.cpp
)

add_library(PhysicsEngine::Engine ALIAS Engine)
target_compile_features(Engine PUBLIC cxx_std_17)
target_include_directories(Engine PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
set_target_properties(Engine PROPERTIES VERSION ${PROJECT_VERSION} SOVERSION 0)
install(TARGETS Engine EXPORT PhysicsEngineTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT PhysicsEngineTargets NAMESPACE PhysicsEngine::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/PhysicsEngine)
configure_package_config_file(cmake/PhysicsEngineConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/PhysicsEngineConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/PhysicsEngine)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/PhysicsEngineConfigVersion.cmake
    VERSION ${PROJECT_VERSION} COMPATIBILITY SameMinorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/PhysicsEngineConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/PhysicsEngineConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/PhysicsEngine)

if(WIN32)
    target_compile_definitions(Engine PUBLIC _USE_MATH_DEFINES)
    set_target_properties(Engine PROPERTIES
        WINDOWS_EXPORT_ALL_SYMBOLS ON
    )
endif()

if(NOT EMSCRIPTEN)
    option(PHYSICS_BUILD_EXAMPLES "Build reproducible experiment" ON)
    if(PHYSICS_BUILD_EXAMPLES)
        add_executable(replay_experiment examples/replay_experiment.cpp)
        target_link_libraries(replay_experiment PRIVATE PhysicsEngine::Engine)
        add_executable(softbody_rope examples/softbody_rope.cpp)
        target_link_libraries(softbody_rope PRIVATE PhysicsEngine::Engine)
        add_executable(thermal_network_demo examples/thermal_network_demo.cpp)
        target_link_libraries(thermal_network_demo PRIVATE PhysicsEngine::Engine)
        add_executable(gravity_binary examples/gravity_binary.cpp)
        target_link_libraries(gravity_binary PRIVATE PhysicsEngine::Engine)
        if(BUILD_TESTING)
            add_test(NAME gravity_binary_smoke COMMAND gravity_binary)
            add_test(NAME thermal_network_smoke COMMAND thermal_network_demo)
            add_test(NAME softbody_rope_smoke COMMAND softbody_rope)
            add_test(NAME reproducible_replay COMMAND ${CMAKE_COMMAND}
                -DPROGRAM=$<TARGET_FILE:replay_experiment>
                -DOUTPUT_DIR=${CMAKE_CURRENT_BINARY_DIR}
                -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/VerifyReplay.cmake)
        endif()
    endif()
endif()

if(EMSCRIPTEN)
    target_compile_options(Engine PUBLIC -fexceptions)
    # --- WebAssembly module
    add_executable(physics_engine_wasm
        src/wasm/bindings.cpp
        src/wasm/softbody_bindings.cpp
        src/wasm/thermal_bindings.cpp
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
elseif(BUILD_TESTING)
    # --- Native Test Executable
    # Visual Studio project names are case-insensitive: run_tests would collide
    # with CTest's generated RUN_TESTS target. Preserve the executable filename.
    add_executable(engine_tests_runner
        ${TESTS_SRC_DIR}/main_tests.cpp
        ${TESTS_SRC_DIR}/test_math.cpp
        ${TESTS_SRC_DIR}/test_rigidbody.cpp
        ${TESTS_SRC_DIR}/test_rigidbody_numerics.cpp
        ${TESTS_SRC_DIR}/test_polygon_centering.cpp
        ${TESTS_SRC_DIR}/test_contact_numerics.cpp
        ${TESTS_SRC_DIR}/test_charged_particle.cpp
        ${TESTS_SRC_DIR}/test_nbody_gravity.cpp
        ${TESTS_SRC_DIR}/test_world.cpp
        ${TESTS_SRC_DIR}/test_forces.cpp
        ${TESTS_SRC_DIR}/test_engine.cpp
        ${TESTS_SRC_DIR}/test_collision.cpp
        ${TESTS_SRC_DIR}/test_broad_phase.cpp
        ${TESTS_SRC_DIR}/test_grid_safety.cpp
        ${TESTS_SRC_DIR}/test_simulation.cpp
        ${TESTS_SRC_DIR}/test_collision_listener.cpp
        ${TESTS_SRC_DIR}/test_collision_events.cpp
        ${TESTS_SRC_DIR}/test_ownership.cpp
        ${TESTS_SRC_DIR}/test_ccd.cpp
        ${TESTS_SRC_DIR}/test_joints.cpp
        ${TESTS_SRC_DIR}/test_prismatic_joint.cpp
        ${TESTS_SRC_DIR}/test_softbody.cpp
        ${TESTS_SRC_DIR}/test_thermal_network.cpp
        ${TESTS_SRC_DIR}/test_dfsph_solver.cpp
        ${TESTS_SRC_DIR}/test_fluid_viscosity.cpp
        ${TESTS_SRC_DIR}/test_state_export.cpp
        ${TESTS_SRC_DIR}/test_particles.cpp
        ${TESTS_SRC_DIR}/test_fluid_particles.cpp
        ${TESTS_SRC_DIR}/test_sph_kernels.cpp
        ${TESTS_SRC_DIR}/test_wcsph_solver.cpp
        ${TESTS_SRC_DIR}/test_fluid_boundaries.cpp
        ${TESTS_SRC_DIR}/test_fluid_rigid_coupling.cpp
        ${TESTS_SRC_DIR}/test_coupled_fluid_simulation.cpp
        ${TESTS_SRC_DIR}/test_fluid_physics_validation.cpp
        ${TESTS_SRC_DIR}/test_fluid_consistency_diagnostic.cpp
        ${TESTS_SRC_DIR}/test_fluid_kernel_family.cpp
        ${TESTS_SRC_DIR}/test_physics_invariants.cpp
        ${TESTS_SRC_DIR}/test_collision_properties.cpp
        ${TESTS_SRC_DIR}/test_collision_filter.cpp
        ${TESTS_SRC_DIR}/test_spatial_queries.cpp
        ${TESTS_SRC_DIR}/test_simulation_config.cpp
        ${TESTS_SRC_DIR}/test_fixed_step.cpp
        ${TESTS_SRC_DIR}/test_simulation_statistics.cpp
        ${TESTS_SRC_DIR}/test_contact_manifold.cpp
        ${TESTS_SRC_DIR}/test_broad_phase_differential.cpp
        ${TESTS_SRC_DIR}/test_stress.cpp
        ${TESTS_SRC_DIR}/test_validation.cpp
        ${TESTS_SRC_DIR}/catch_amalgamated.cpp
    )

    set_target_properties(engine_tests_runner PROPERTIES OUTPUT_NAME run_tests)
    target_link_libraries(engine_tests_runner PRIVATE Engine)
    target_include_directories(engine_tests_runner PRIVATE ${TESTS_SRC_DIR})

    add_executable(numerical_validation
        ${TESTS_SRC_DIR}/numerical_validation.cpp
    )
    target_link_libraries(numerical_validation PRIVATE Engine)

    add_executable(fluid_benchmark
        benchmarks/fluid_benchmark.cpp
    )
    target_link_libraries(fluid_benchmark PRIVATE Engine)
    add_executable(fluid_solver_comparison benchmarks/fluid_solver_comparison.cpp)
    target_link_libraries(fluid_solver_comparison PRIVATE Engine)
    add_executable(fluid_consistency_diagnostic benchmarks/fluid_consistency_diagnostic.cpp)
    target_link_libraries(fluid_consistency_diagnostic PRIVATE Engine)

    enable_testing()
    add_test(NAME engine_tests COMMAND engine_tests_runner)
    add_test(
        NAME numerical_validation_fast
        COMMAND numerical_validation
            --mode fast
            --output ${CMAKE_BINARY_DIR}/numerical-validation-fast.json
    )
    set_tests_properties(numerical_validation_fast PROPERTIES
        LABELS "fast;numerical"
    )
    add_test(
        NAME fluid_benchmark_smoke
        COMMAND fluid_benchmark
            --particles 1000
            --warmup 0
            --samples 1
            --output ${CMAKE_BINARY_DIR}/fluid-benchmark-smoke.json
    )
    set_tests_properties(fluid_benchmark_smoke PROPERTIES
        LABELS "fast;performance"
    )
    add_test(NAME fluid_consistency_diagnostic_smoke
        COMMAND fluid_consistency_diagnostic --quick
            --output ${CMAKE_BINARY_DIR}/fluid-consistency-quick.json)
    set_tests_properties(fluid_consistency_diagnostic_smoke PROPERTIES
        LABELS "fast;numerical;diagnostic"
    )
    add_test(NAME fluid_cubic_diagnostic_smoke
        COMMAND fluid_consistency_diagnostic --quick --family cubic
            --output ${CMAKE_BINARY_DIR}/fluid-cubic-quick.json)
    set_tests_properties(fluid_cubic_diagnostic_smoke PROPERTIES
        LABELS "fast;numerical;diagnostic"
    )
endif()
