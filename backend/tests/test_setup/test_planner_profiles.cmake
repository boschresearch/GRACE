# Copyright (c) 2026 Robert Bosch GmbH
# SPDX-License-Identifier: AGPL-3.0-only

cmake_minimum_required(VERSION 3.22)

if(CHECK_PROFILE)
    include("${GRACE_SOURCE_DIR}/cmake/GraceOptions.cmake")
    if(RECONFIGURE_WITHOUT_USC)
        set(GRACE_ENABLE_USC_PLANNERS OFF)
        include("${GRACE_SOURCE_DIR}/cmake/GraceOptions.cmake")
    endif()
    set(actual)
    foreach(planner IN LISTS GRACE_PLANNERS)
        if(GRACE_BUILD_PLANNER_${planner})
            list(APPEND actual "${planner}")
        endif()
    endforeach()
    list(SORT actual)
    list(SORT EXPECTED)
    if(NOT "${actual}" STREQUAL "${EXPECTED}")
        message(FATAL_ERROR
            "${GRACE_PLANNER_PROFILE}: expected '${EXPECTED}', got '${actual}'")
    endif()
    return()
endif()

function(check_profile profile usc expected)
    set(opt_in_arguments)
    if(NOT usc STREQUAL "DEFAULT")
        list(APPEND opt_in_arguments "-DGRACE_ENABLE_USC_PLANNERS=${usc}")
    endif()
    execute_process(
        COMMAND "${CMAKE_COMMAND}"
            "-DGRACE_SOURCE_DIR=${GRACE_SOURCE_DIR}"
            -DCHECK_PROFILE=ON
            "-DGRACE_PLANNER_PROFILE=${profile}"
            ${opt_in_arguments}
            "-DEXPECTED=${expected}"
            ${ARGN}
            -P "${CMAKE_CURRENT_FUNCTION_LIST_FILE}"
        RESULT_VARIABLE result
        OUTPUT_VARIABLE output
        ERROR_VARIABLE error
    )
    if(NOT result EQUAL 0)
        message(FATAL_ERROR "${profile} (USC=${usc}): ${output}\n${error}")
    endif()
endfunction()

set(usc_planners EECBS CBSH2RTC MAPFLNS2)
set(standard_full LACAM0 MAPFGPTDDG SCRIMP CCBS PSIPPCTC DBCBS SST)
check_profile(FULL DEFAULT "${standard_full}")
check_profile(NONE OFF "")
check_profile(GRID OFF LACAM0)
check_profile(ROADMAP OFF "CCBS;PSIPPCTC")
check_profile(CONTINUOUS OFF "DBCBS;SST")
check_profile(HOSTED OFF "LACAM0;CCBS;PSIPPCTC")
check_profile(FULL OFF "${standard_full}")
check_profile(CUSTOM OFF LACAM0 -DGRACE_BUILD_PLANNER_LACAM0=ON)
check_profile(NONE ON "")
check_profile(GRID ON "${usc_planners};LACAM0")
check_profile(HOSTED ON "${usc_planners};LACAM0;CCBS;PSIPPCTC")
check_profile(FULL ON "${usc_planners};${standard_full}")
check_profile(FULL ON "${standard_full}" -DRECONFIGURE_WITHOUT_USC=ON)

foreach(planner IN LISTS usc_planners)
    check_profile(CUSTOM ON "${planner}" "-DGRACE_BUILD_PLANNER_${planner}=ON")
    execute_process(
        COMMAND "${CMAKE_COMMAND}"
            "-DGRACE_SOURCE_DIR=${GRACE_SOURCE_DIR}"
            -DCHECK_PROFILE=ON
            -DGRACE_PLANNER_PROFILE=CUSTOM
            -DGRACE_ENABLE_USC_PLANNERS=OFF
            "-DGRACE_BUILD_PLANNER_${planner}=ON"
            -P "${CMAKE_CURRENT_LIST_FILE}"
        RESULT_VARIABLE result
        OUTPUT_VARIABLE output
        ERROR_VARIABLE error
    )
    if(result EQUAL 0 OR NOT error MATCHES "requires explicit opt-in")
        message(FATAL_ERROR "CUSTOM ${planner} bypassed opt-in: ${output}\n${error}")
    endif()
endforeach()

file(READ "${GRACE_SOURCE_DIR}/CMakePresets.json" presets)
string(JSON count LENGTH "${presets}" configurePresets)
math(EXPR last "${count} - 1")
set(checked_presets)
foreach(index RANGE "${last}")
    string(JSON name GET "${presets}" configurePresets "${index}" name)
    if(name STREQUAL "base" OR name STREQUAL "usc-grid")
        list(APPEND checked_presets "${name}")
        string(JSON enabled GET "${presets}" configurePresets "${index}"
            cacheVariables GRACE_ENABLE_USC_PLANNERS)
        if(name STREQUAL "base" AND enabled)
            message(FATAL_ERROR "Standard presets must reset USC opt-in to OFF")
        elseif(name STREQUAL "usc-grid" AND NOT enabled)
            message(FATAL_ERROR "usc-grid must explicitly enable USC planners")
        endif()
    endif()
endforeach()
if(NOT "base" IN_LIST checked_presets OR NOT "usc-grid" IN_LIST checked_presets)
    message(FATAL_ERROR "Missing standard or opt-in configure preset")
endif()
