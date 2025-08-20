cmake_minimum_required(VERSION 3.10)
project(PhysicsEngine CXX)

# Source directory for library
set(PHYSICS_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/physics)
# Source directory for tests
set(TESTS_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/tests)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${TESTS_SRC_DIR}/test_suite
)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- PhysicsEngine Library
add_library(PhysicsEngine STATIC
    ${PHYSICS_SRC_DIR}/math/vector2.cpp
    ${PHYSICS_SRC_DIR}/math/matrix2x2.cpp
)

# --- Test Executable
add_executable(run_tests
    ${TESTS_SRC_DIR}/test_suite/main_tests.cpp
    ${TESTS_SRC_DIR}/test_suite/test_math.cpp
    ${TESTS_SRC_DIR}/test_suite/catch_amalgamated.cpp
)

# Link the test executable with the physics library
target_link_libraries(run_tests
    PhysicsEngine
)