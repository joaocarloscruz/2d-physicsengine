execute_process(COMMAND "${PROGRAM}" RESULT_VARIABLE result OUTPUT_VARIABLE report ERROR_VARIABLE error)
if(NOT result EQUAL 0)
    message(FATAL_ERROR "Contact-load diagnostic failed: ${error}")
endif()
# JSON parsing was added in 3.19. Keep the project's 3.16 execution support;
# current CI and release toolchains additionally validate the report schema.
if(NOT CMAKE_VERSION VERSION_LESS 3.19)
    string(JSON classification GET "${report}" classification)
    string(JSON count LENGTH "${report}" rows)
    string(JSON changed GET "${report}" productionDefaultsChanged)
    if(NOT classification STREQUAL "observation" OR NOT count EQUAL 32 OR changed)
        message(FATAL_ERROR "Unexpected contact-load observation schema")
    endif()
endif()
