cmake_minimum_required(VERSION 3.10)
project(PhysicsEngine CXX)

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

# --- PhysicsEngine Library
add_library(PhysicsEngine STATIC
    ${PHYSICS_SRC_DIR}/math/vector2.cpp
    ${PHYSICS_SRC_DIR}/math/matrix2x2.cpp
    ${PHYSICS_SRC_DIR}/core/rigidbody.cpp
    ${PHYSICS_SRC_DIR}/core/world.cpp
)

# --- Test Executable
add_executable(run_tests
    ${TESTS_SRC_DIR}/main_tests.cpp
    ${TESTS_SRC_DIR}/test_math.cpp
    ${TESTS_SRC_DIR}/test_rigidbody.cpp
    ${TESTS_SRC_DIR}/catch_amalgamated.cpp
)

# Link the test executable with the physics library
target_link_libraries(run_tests
    PhysicsEngine
)