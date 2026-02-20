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

# --- Engine Library
add_library(Engine SHARED
    src/engine.cpp
    ${PHYSICS_SRC_DIR}/math/vector2.cpp
    ${PHYSICS_SRC_DIR}/math/matrix2x2.cpp
    ${PHYSICS_SRC_DIR}/core/rigidbody.cpp
    ${PHYSICS_SRC_DIR}/core/world.cpp
    ${PHYSICS_SRC_DIR}/core/forces/gravity.cpp
    ${PHYSICS_SRC_DIR}/core/forces/drag.cpp
    ${PHYSICS_SRC_DIR}/core/collisions/collision_dispatcher.cpp
    ${PHYSICS_SRC_DIR}/core/collisions/narrow_phase/collision_circle_circle.cpp
    ${PHYSICS_SRC_DIR}/core/collisions/narrow_phase/collision_polygon_polygon.cpp
    ${PHYSICS_SRC_DIR}/core/collisions/narrow_phase/collision_circle_polygon.cpp
    ${PHYSICS_SRC_DIR}/core/collisions/collision_resolver.cpp
    ${PHYSICS_SRC_DIR}/core/collisions/broad_phase/sweep_and_prune.cpp
)

set_target_properties(Engine PROPERTIES
    WINDOWS_EXPORT_ALL_SYMBOLS ON
)

# --- Test Executable
add_executable(run_tests
    ${TESTS_SRC_DIR}/main_tests.cpp
    ${TESTS_SRC_DIR}/test_math.cpp
    ${TESTS_SRC_DIR}/test_rigidbody.cpp
    ${TESTS_SRC_DIR}/test_world.cpp
    ${TESTS_SRC_DIR}/test_forces.cpp
    ${TESTS_SRC_DIR}/test_engine.cpp
    ${TESTS_SRC_DIR}/test_collision.cpp
    ${TESTS_SRC_DIR}/test_simulation.cpp
    ${TESTS_SRC_DIR}/catch_amalgamated.cpp
)

# Link the test executable with the physics library
target_link_libraries(run_tests
    Engine
)