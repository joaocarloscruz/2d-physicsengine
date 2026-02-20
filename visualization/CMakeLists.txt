cmake_minimum_required(VERSION 3.10)
project(PhysicsVisualization)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set SFML_DIR if CMAKE_PREFIX_PATH is set
if(DEFINED ENV{CMAKE_PREFIX_PATH})
    string(REPLACE "\\" "/" SFML_PREFIX "$ENV{CMAKE_PREFIX_PATH}")
    set(SFML_DIR "${SFML_PREFIX}/lib/cmake/SFML")
    message(STATUS "Using SFML_DIR: ${SFML_DIR}")
endif()

# Use static SFML libraries
set(SFML_STATIC_LIBRARIES TRUE)

# Find SFML (supports both 2.x and 3.x)
find_package(SFML 3 COMPONENTS Graphics Window System QUIET)
if(SFML_FOUND)
    message(STATUS "Found SFML ${SFML_VERSION}")
else()
    message(STATUS "SFML 3 not found, trying version 2.5...")
    find_package(SFML 2.5 COMPONENTS graphics window system QUIET)
    if(SFML_FOUND)
        message(STATUS "Found SFML ${SFML_VERSION}")
    else()
        message(FATAL_ERROR "SFML not found. Please install SFML and set CMAKE_PREFIX_PATH to the SFML installation directory.")
    endif()
endif()

# Include the main engine's include directory
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)

# Add the visualization executable
add_executable(PhysicsVisualization
    main.cpp
    renderer.cpp
    renderer.h
)

# Link the engine library (assuming it's already built)
target_link_directories(PhysicsVisualization PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../build)

# Link SFML and the physics engine
if(SFML_VERSION_MAJOR EQUAL 3)
    target_link_libraries(PhysicsVisualization
        SFML::Graphics
        SFML::Window
        SFML::System
        Engine
    )
else()
    target_link_libraries(PhysicsVisualization
        sfml-graphics
        sfml-window
        sfml-system
        Engine
    )
endif()

# Copy the Engine DLL to the build directory after building (Windows)
if(WIN32)
    add_custom_command(TARGET PhysicsVisualization POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/../build/libEngine.dll"
            "$<TARGET_FILE_DIR:PhysicsVisualization>"
        COMMENT "Copying libEngine.dll to build directory"
    )
endif()

# Copy SFML DLLs to output directory on Windows (if needed)
if(WIN32)
    if(SFML_VERSION_MAJOR EQUAL 3)
        # Copy SFML DLLs for SFML 3.x
        foreach(lib IN ITEMS SFML::Graphics SFML::Window SFML::System)
            add_custom_command(TARGET PhysicsVisualization POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    $<TARGET_FILE:${lib}>
                    $<TARGET_FILE_DIR:PhysicsVisualization>)
        endforeach()
    else()
        # Copy SFML DLLs for SFML 2.x
        foreach(lib IN ITEMS sfml-graphics sfml-window sfml-system)
            add_custom_command(TARGET PhysicsVisualization POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    $<TARGET_FILE:${lib}>
                    $<TARGET_FILE_DIR:PhysicsVisualization>)
        endforeach()
    endif()
endif()
