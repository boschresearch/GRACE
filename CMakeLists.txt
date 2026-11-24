# Copyright (c) 2026 Robert Bosch GmbH
# SPDX-License-Identifier: AGPL-3.0-only

cmake_minimum_required(VERSION 3.22)

if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif()
if(POLICY CMP0167)
    cmake_policy(SET CMP0167 OLD)
endif()

project(
    grace
    DESCRIPTION
        "Unified 2D multi-robot path-planning simulator and benchmark"
    LANGUAGES C CXX
)


list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(GraceOptions)
include(GraceWarnings)

if(GRACE_BUILD_FRONTEND AND NOT GRACE_BUILD_BACKEND)
    message(FATAL_ERROR "GRACE_BUILD_FRONTEND requires GRACE_BUILD_BACKEND")
endif()

include(Dependencies)
include(PlannerHelpers)

if(GRACE_BUILD_BACKEND AND GRACE_BUILD_TESTS)
    enable_testing()
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/CTestCustom.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/CTestCustom.cmake"
        COPYONLY
    )
endif()

if(GRACE_BUILD_BACKEND)
    add_subdirectory(backend)
endif()

if(GRACE_BUILD_FRONTEND)
    add_subdirectory(frontend)
endif()