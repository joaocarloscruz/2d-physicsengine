foreach(name first second)
    execute_process(COMMAND "${PROGRAM}" "${OUTPUT_DIR}/replay-${name}"
        RESULT_VARIABLE result)
    if(NOT result EQUAL 0)
        message(FATAL_ERROR "Replay ${name} failed: ${result}")
    endif()
endforeach()
foreach(extension csv json)
    execute_process(COMMAND "${CMAKE_COMMAND}" -E compare_files
        "${OUTPUT_DIR}/replay-first.${extension}" "${OUTPUT_DIR}/replay-second.${extension}"
        RESULT_VARIABLE result)
    if(NOT result EQUAL 0)
        message(FATAL_ERROR "Replay ${extension} outputs differ")
    endif()
endforeach()
