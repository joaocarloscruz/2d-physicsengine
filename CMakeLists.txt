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
    src/physics_engine.cpp
    ${PHYSICS_SRC_DIR}/math/vector2.cpp
    ${PHYSICS_SRC_DIR}/math/matrix2x2.cpp
    ${PHYSICS_SRC_DIR}/core/rigidbody.cpp
    ${PHYSICS_SRC_DIR}/core/world.cpp
    ${PHYSICS_SRC_DIR}/core/forces/gravity.cpp
)

# --- Test Executable
add_executable(run_tests
    ${TESTS_SRC_DIR}/main_tests.cpp
    ${TESTS_SRC_DIR}/test_math.cpp
    ${TESTS_SRC_DIR}/test_rigidbody.cpp
    ${TESTS_SRC_DIR}/test_world.cpp
    ${TESTS_SRC_DIR}/test_forces.cpp
    ${TESTS_SRC_DIR}/test_engine.cpp
    ${TESTS_SRC_DIR}/catch_amalgamated.cpp
)

# Link the test executable with the physics library
target_link_libraries(run_tests
    PhysicsEngine
)

# --- Installation Rules ---

include(GNUInstallDirs)

# Install the library target
install(TARGETS PhysicsEngine
    EXPORT PhysicsEngineTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# Install all the public headers
install(FILES
    include/physics_engine.h
    include/physics/core/force_generator.h
    include/physics/core/rigidbody.h
    include/physics/core/shape.h
    include/physics/core/world.h
    include/physics/core/forces/gravity.h
    include/physics/math/matrix2x2.h
    include/physics/math/vector2.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/PhysicsEngine
)

# --- CMake Package Generation ---

include(CMakePackageConfigHelpers)

# Generate the package configuration file
configure_package_config_file(
    PhysicsEngineConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/PhysicsEngineConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/PhysicsEngine
)

# Install the export set
install(EXPORT PhysicsEngineTargets
    FILE PhysicsEngineTargets.cmake
    NAMESPACE PhysicsEngine::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/PhysicsEngine
)

# Install the generated package configuration file
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/PhysicsEngineConfig.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/PhysicsEngine
)
