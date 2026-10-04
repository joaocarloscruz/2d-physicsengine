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
    # The module-local exception-safe Embind adapter intentionally targets the
    # exact SDK used by hosted CI. Upgrades require reviewing its runtime ABI.
    if(NOT EMSCRIPTEN_VERSION STREQUAL "6.0.3")
        message(FATAL_ERROR "PhysicsEngine WASM requires Emscripten 6.0.3 (exception boundary adapter)")
    endif()
    option(PHYSICS_WASM_BOUNDARY_TEST_PROBES "Include test-only WASM allocation/lifetime probes" OFF)
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
    ${PHYSICS_SRC_DIR}/core/wave_membrane.cpp
    ${PHYSICS_SRC_DIR}/core/maxwell_grid.cpp
    ${PHYSICS_SRC_DIR}/core/state_export.cpp
    ${PHYSICS_SRC_DIR}/core/spatial_queries.cpp
    ${PHYSICS_SRC_DIR}/core/forces/gravity.cpp
    ${PHYSICS_SRC_DIR}/core/forces/drag.cpp
    ${PHYSICS_SRC_DIR}/core/particles/particle.cpp
    ${PHYSICS_SRC_DIR}/core/particles/particle_system.cpp
    ${PHYSICS_SRC_DIR}/core/particles/particle_spatial_grid.cpp
    ${PHYSICS_SRC_DIR}/core/fluids/fluid_particle.cpp
    ${PHYSICS_SRC_DIR}/core/fluids/periodic_mac_grid.cpp
    ${PHYSICS_SRC_DIR}/core/fluids/periodic_mac_diffusion.cpp
    ${PHYSICS_SRC_DIR}/core/fluids/periodic_scalar_transport.cpp
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
        add_executable(thermal_radiation_demo examples/thermal_radiation_demo.cpp)
        target_link_libraries(thermal_radiation_demo PRIVATE PhysicsEngine::Engine)
        add_executable(gravity_binary examples/gravity_binary.cpp)
        target_link_libraries(gravity_binary PRIVATE PhysicsEngine::Engine)
        add_executable(wave_membrane_demo examples/wave_membrane_demo.cpp)
        target_link_libraries(wave_membrane_demo PRIVATE PhysicsEngine::Engine)
        add_executable(mac_projection_demo examples/mac_projection_demo.cpp)
        target_link_libraries(mac_projection_demo PRIVATE PhysicsEngine::Engine)
        add_executable(mac_diffusion_demo examples/mac_diffusion_demo.cpp)
        target_link_libraries(mac_diffusion_demo PRIVATE PhysicsEngine::Engine)
        add_executable(scalar_transport_demo examples/scalar_transport_demo.cpp)
        target_link_libraries(scalar_transport_demo PRIVATE PhysicsEngine::Engine)
        add_executable(maxwell_grid_demo examples/maxwell_grid_demo.cpp)
        target_link_libraries(maxwell_grid_demo PRIVATE PhysicsEngine::Engine)
        if(BUILD_TESTING)
            add_test(NAME gravity_binary_smoke COMMAND gravity_binary)
            add_test(NAME thermal_network_smoke COMMAND thermal_network_demo)
            add_test(NAME thermal_radiation_smoke COMMAND thermal_radiation_demo)
            add_test(NAME wave_membrane_smoke COMMAND wave_membrane_demo)
            add_test(NAME mac_projection_smoke COMMAND mac_projection_demo)
            add_test(NAME mac_diffusion_smoke COMMAND mac_diffusion_demo)
            add_test(NAME scalar_transport_smoke COMMAND scalar_transport_demo)
            add_test(NAME maxwell_grid_smoke COMMAND maxwell_grid_demo)
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
        src/wasm/gravity_bindings.cpp
        src/wasm/wave_bindings.cpp
        src/wasm/mac_bindings.cpp
        src/wasm/maxwell_bindings.cpp
        src/wasm/query_bindings.cpp
    )

    # The custom library must follow -lembind so its definitions replace the
    # SDK wrappers rather than being replaced by them during library loading.
    target_link_libraries(physics_engine_wasm PRIVATE Engine embind
        "--js-library=${CMAKE_CURRENT_SOURCE_DIR}/src/wasm/embind_boundary.js")
    if(PHYSICS_WASM_BOUNDARY_TEST_PROBES)
        target_sources(physics_engine_wasm PRIVATE tests/wasm_boundary_probe.cpp)
        target_link_options(physics_engine_wasm PRIVATE
            "-sEXPORTED_FUNCTIONS=['_emscripten_stack_get_current']")
    endif()
    target_compile_options(physics_engine_wasm PRIVATE -fexceptions)
    target_link_options(physics_engine_wasm PRIVATE
        -fexceptions
        "-sMODULARIZE=1"
        "-sEXPORT_NAME=createPhysicsEngineModule"
        "-sENVIRONMENT=web,node"
        "-sALLOW_MEMORY_GROWTH=1"
        "-sDISABLE_EXCEPTION_CATCHING=0"
    )
    set_property(TARGET physics_engine_wasm APPEND PROPERTY LINK_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/src/wasm/embind_boundary.js")

    set(WASM_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/wasm")
    set_target_properties(physics_engine_wasm PROPERTIES
        OUTPUT_NAME "physics_engine"
        SUFFIX ".js"
        RUNTIME_OUTPUT_DIRECTORY "${WASM_OUTPUT_DIRECTORY}"
    )

    # Harness edits must be visible even when the module needs no relink.
    # Explicit outputs also let incremental builds restore a missing asset.
    set(WASM_ASSET_OUTPUTS)
    foreach(asset IN ITEMS index.html smoke.js smoke-test.cjs boundary-stress.cjs)
        set(asset_source "${CMAKE_CURRENT_SOURCE_DIR}/examples/wasm-smoke/${asset}")
        set(asset_output "${WASM_OUTPUT_DIRECTORY}/${asset}")
        add_custom_command(OUTPUT "${asset_output}"
            COMMAND ${CMAKE_COMMAND} -E make_directory "${WASM_OUTPUT_DIRECTORY}"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different "${asset_source}" "${asset_output}"
            DEPENDS "${asset_source}"
            COMMENT "Updating WebAssembly asset ${asset}"
            VERBATIM
        )
        list(APPEND WASM_ASSET_OUTPUTS "${asset_output}")
    endforeach()
    add_custom_target(physics_wasm_assets DEPENDS ${WASM_ASSET_OUTPUTS})
    add_dependencies(physics_engine_wasm physics_wasm_assets)
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
        ${TESTS_SRC_DIR}/test_circle_polygon_numerics.cpp
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
        ${TESTS_SRC_DIR}/test_ccd_numerics.cpp
        ${TESTS_SRC_DIR}/test_rigid_contact_benchmark.cpp
        ${TESTS_SRC_DIR}/test_contact_load_projection.cpp
        ${TESTS_SRC_DIR}/test_joints.cpp
        ${TESTS_SRC_DIR}/test_joint_arithmetic.cpp
        ${TESTS_SRC_DIR}/test_prismatic_joint.cpp
        ${TESTS_SRC_DIR}/test_softbody.cpp
        ${TESTS_SRC_DIR}/test_thermal_network.cpp
        ${TESTS_SRC_DIR}/test_thermal_radiation.cpp
        ${TESTS_SRC_DIR}/test_wave_membrane.cpp
        ${TESTS_SRC_DIR}/test_maxwell_grid.cpp
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
        ${TESTS_SRC_DIR}/test_fluid_disorder.cpp
        ${TESTS_SRC_DIR}/test_fluid_kernel_family.cpp
        ${TESTS_SRC_DIR}/test_fluid_wall_audit.cpp
        ${TESTS_SRC_DIR}/test_wall_pressure_modes.cpp
        ${TESTS_SRC_DIR}/test_planar_reflected_operator.cpp
        ${TESTS_SRC_DIR}/test_periodic_mac_grid.cpp
        ${TESTS_SRC_DIR}/test_mac_diffusion.cpp
        ${TESTS_SRC_DIR}/test_scalar_transport.cpp
        ${TESTS_SRC_DIR}/test_physics_invariants.cpp
        ${TESTS_SRC_DIR}/test_collision_properties.cpp
        ${TESTS_SRC_DIR}/test_collision_filter.cpp
        ${TESTS_SRC_DIR}/test_spatial_queries.cpp
        ${TESTS_SRC_DIR}/test_circle_queries.cpp
        ${TESTS_SRC_DIR}/test_simulation_config.cpp
        ${TESTS_SRC_DIR}/test_fixed_step.cpp
        ${TESTS_SRC_DIR}/test_simulation_statistics.cpp
        ${TESTS_SRC_DIR}/test_contact_manifold.cpp
        ${TESTS_SRC_DIR}/test_polygon_manifold_numerics.cpp
        ${TESTS_SRC_DIR}/test_contact_arithmetic.cpp
        ${TESTS_SRC_DIR}/test_two_point_contact.cpp
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
    add_executable(broad_phase_benchmark benchmarks/broad_phase_benchmark.cpp)
    target_link_libraries(broad_phase_benchmark PRIVATE PhysicsEngine::Engine)
    add_test(NAME broad_phase_benchmark_smoke COMMAND broad_phase_benchmark)
    set_tests_properties(broad_phase_benchmark_smoke PROPERTIES LABELS "fast;performance")
    add_executable(rigid_contact_benchmark benchmarks/rigid_contact_benchmark.cpp)
    target_link_libraries(rigid_contact_benchmark PRIVATE PhysicsEngine::Engine)
    add_executable(contact_load_benchmark benchmarks/rigid_contact_benchmark.cpp)
    target_compile_definitions(contact_load_benchmark PRIVATE PHYSICS_CONTACT_LOAD_EXPERIMENT)
    target_link_libraries(contact_load_benchmark PRIVATE PhysicsEngine::Engine)
    add_executable(contact_load_diagnostic benchmarks/contact_load_diagnostic.cpp)
    target_link_libraries(contact_load_diagnostic PRIVATE PhysicsEngine::Engine)
    add_test(NAME contact_load_diagnostic_smoke COMMAND ${CMAKE_COMMAND}
        -DPROGRAM=$<TARGET_FILE:contact_load_diagnostic>
        -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/VerifyContactLoadJson.cmake)
    set_tests_properties(contact_load_diagnostic_smoke PROPERTIES LABELS "fast;numerical;diagnostic")
    add_executable(contact_integration_diagnostic benchmarks/contact_integration_diagnostic.cpp)
    target_link_libraries(contact_integration_diagnostic PRIVATE PhysicsEngine::Engine)
    add_test(NAME contact_integration_diagnostic_smoke COMMAND contact_integration_diagnostic)
    set_tests_properties(contact_integration_diagnostic_smoke PROPERTIES LABELS "fast;numerical;diagnostic")
    add_test(NAME rigid_contact_benchmark_smoke COMMAND rigid_contact_benchmark --quick
        --output ${CMAKE_BINARY_DIR}/rigid-contact-smoke.json)
    set_tests_properties(rigid_contact_benchmark_smoke PROPERTIES LABELS "fast;numerical;diagnostic")
    add_test(NAME rigid_contact_benchmark_invalid_iterations COMMAND rigid_contact_benchmark --iterations 1.5)
    add_test(NAME rigid_contact_benchmark_budget_rejection COMMAND rigid_contact_benchmark --quick --dt 0.0001 --duration 8)
    set_tests_properties(rigid_contact_benchmark_invalid_iterations rigid_contact_benchmark_budget_rejection
        PROPERTIES WILL_FAIL TRUE LABELS "fast;diagnostic")
    add_executable(fluid_solver_comparison benchmarks/fluid_solver_comparison.cpp)
    target_link_libraries(fluid_solver_comparison PRIVATE Engine)
    add_executable(fluid_consistency_diagnostic benchmarks/fluid_consistency_diagnostic.cpp)
    target_link_libraries(fluid_consistency_diagnostic PRIVATE Engine)
    add_executable(fluid_disorder_diagnostic benchmarks/fluid_disorder_diagnostic.cpp)
    target_link_libraries(fluid_disorder_diagnostic PRIVATE Engine)
    add_executable(fluid_wall_diagnostic benchmarks/fluid_wall_diagnostic.cpp)
    target_link_libraries(fluid_wall_diagnostic PRIVATE Engine)
    add_executable(planar_reflected_diagnostic benchmarks/planar_reflected_diagnostic.cpp)
    target_link_libraries(planar_reflected_diagnostic PRIVATE Engine)

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
    add_test(NAME fluid_disorder_diagnostic_smoke
        COMMAND fluid_disorder_diagnostic --quick --output ${CMAKE_BINARY_DIR}/fluid-disorder-quick.json)
    set_tests_properties(fluid_disorder_diagnostic_smoke PROPERTIES LABELS "fast;numerical;diagnostic")
    add_test(NAME fluid_cubic_diagnostic_smoke
        COMMAND fluid_consistency_diagnostic --quick --family cubic
            --output ${CMAKE_BINARY_DIR}/fluid-cubic-quick.json)
    set_tests_properties(fluid_cubic_diagnostic_smoke PROPERTIES
        LABELS "fast;numerical;diagnostic"
    )
    add_test(NAME fluid_wall_diagnostic_smoke COMMAND fluid_wall_diagnostic --quick
        --output ${CMAKE_BINARY_DIR}/fluid-wall-quick.json)
    set_tests_properties(fluid_wall_diagnostic_smoke PROPERTIES LABELS "fast;numerical;diagnostic")
    add_test(NAME fluid_signed_wall_diagnostic_smoke COMMAND fluid_wall_diagnostic --quick --signed-pressure
        --output ${CMAKE_BINARY_DIR}/fluid-wall-signed-quick.json)
    set_tests_properties(fluid_signed_wall_diagnostic_smoke PROPERTIES LABELS "fast;numerical;diagnostic")
    add_test(NAME planar_reflected_diagnostic_smoke COMMAND planar_reflected_diagnostic --quick
        --output ${CMAKE_BINARY_DIR}/planar-reflected-quick.json)
    set_tests_properties(planar_reflected_diagnostic_smoke PROPERTIES LABELS "fast;numerical;diagnostic")
endif()
