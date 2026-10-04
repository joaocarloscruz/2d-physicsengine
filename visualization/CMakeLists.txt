cmake_minimum_required(VERSION 3.16)
project(PhysicsVisualization LANGUAGES CXX)
set(SFML_STATIC_LIBRARIES TRUE)
find_package(SFML 3 CONFIG REQUIRED COMPONENTS Graphics Window System)
find_package(PhysicsEngine 0.2 CONFIG REQUIRED)
add_executable(PhysicsVisualization main.cpp renderer.cpp renderer.h)
target_link_libraries(PhysicsVisualization PRIVATE
    SFML::Graphics SFML::Window SFML::System PhysicsEngine::Engine)
if(WIN32)
    get_target_property(ENGINE_TYPE PhysicsEngine::Engine TYPE)
    if(ENGINE_TYPE STREQUAL "SHARED_LIBRARY")
        add_custom_command(TARGET PhysicsVisualization POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:PhysicsEngine::Engine> $<TARGET_FILE_DIR:PhysicsVisualization>)
    endif()
endif()
