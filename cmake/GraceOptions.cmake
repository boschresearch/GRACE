# Copyright (c) 2026 Robert Bosch GmbH
# SPDX-License-Identifier: AGPL-3.0-only

option(GRACE_BUILD_BACKEND "Build the planning backend" ON)
option(GRACE_BUILD_FRONTEND "Build the graphical frontend" ON)
option(GRACE_BUILD_TESTS "Build backend tests" ON)
option(
    GRACE_BUILD_PLANNER_INTEGRATION_TESTS
    "Build planner tests that execute bundled or external planner integrations"
    OFF
)
option(GRACE_BUILD_BENCHMARKS "Build backend benchmarks" OFF)
option(
    GRACE_ENABLE_USC_PLANNERS
    "Opt in to building USC-restricted planners; review 3rd-party-licenses.txt first"
    OFF
)

set(
    GRACE_PLANNER_PROFILE
    "NONE"
    CACHE STRING
    "Bundled planner profile: NONE, GRID, ROADMAP, CONTINUOUS, HOSTED, FULL, or CUSTOM"
)
set_property(
    CACHE GRACE_PLANNER_PROFILE
    PROPERTY STRINGS NONE GRID ROADMAP CONTINUOUS HOSTED FULL CUSTOM
)
string(TOUPPER "${GRACE_PLANNER_PROFILE}" GRACE_PLANNER_PROFILE)
set(
    GRACE_PLANNER_PROFILES
    NONE GRID ROADMAP CONTINUOUS HOSTED FULL CUSTOM
)
if(NOT GRACE_PLANNER_PROFILE IN_LIST GRACE_PLANNER_PROFILES)
    message(
        FATAL_ERROR
        "Unknown GRACE_PLANNER_PROFILE='${GRACE_PLANNER_PROFILE}'. "
        "Expected one of: ${GRACE_PLANNER_PROFILES}"
    )
endif()

set(
    GRACE_PLANNERS
    EECBS
    CBSH2RTC
    LACAM0
    MAPFLNS2
    MAPFGPTDDG
    SCRIMP
    CCBS
    PSIPPCTC
    DBCBS
    SST
)
set(GRACE_GRID_PLANNERS EECBS CBSH2RTC LACAM0 MAPFLNS2 MAPFGPTDDG SCRIMP)
set(GRACE_ROADMAP_PLANNERS CCBS PSIPPCTC)
set(GRACE_CONTINUOUS_PLANNERS DBCBS SST)
set(GRACE_HOSTED_PLANNERS EECBS CBSH2RTC LACAM0 MAPFLNS2 CCBS PSIPPCTC)
set(GRACE_RESEARCH_ONLY_PLANNERS EECBS CBSH2RTC MAPFLNS2)

foreach(planner IN LISTS GRACE_PLANNERS)
    option(
        GRACE_BUILD_PLANNER_${planner}
        "Build the bundled ${planner} planner integration"
        OFF
    )
endforeach()

if(NOT GRACE_PLANNER_PROFILE STREQUAL "CUSTOM")
    foreach(planner IN LISTS GRACE_PLANNERS)
        set(enabled OFF)
        if(GRACE_PLANNER_PROFILE STREQUAL "FULL")
            set(enabled ON)
        elseif(GRACE_PLANNER_PROFILE STREQUAL "GRID" AND
               planner IN_LIST GRACE_GRID_PLANNERS AND
               NOT planner STREQUAL "MAPFGPTDDG" AND
               NOT planner STREQUAL "SCRIMP")
            set(enabled ON)
        elseif(GRACE_PLANNER_PROFILE STREQUAL "ROADMAP" AND
               planner IN_LIST GRACE_ROADMAP_PLANNERS)
            set(enabled ON)
        elseif(GRACE_PLANNER_PROFILE STREQUAL "CONTINUOUS" AND
               planner IN_LIST GRACE_CONTINUOUS_PLANNERS)
            set(enabled ON)
        elseif(GRACE_PLANNER_PROFILE STREQUAL "HOSTED" AND
               planner IN_LIST GRACE_HOSTED_PLANNERS)
            set(enabled ON)
        endif()
        if(planner IN_LIST GRACE_RESEARCH_ONLY_PLANNERS AND
           NOT GRACE_ENABLE_USC_PLANNERS)
            set(enabled OFF)
        endif()
        set(
            GRACE_BUILD_PLANNER_${planner}
            ${enabled}
            CACHE BOOL
            "Build the bundled ${planner} planner integration"
            FORCE
        )
    endforeach()
endif()

set(GRACE_RESEARCH_ONLY_PLANNERS_ENABLED)
foreach(planner IN LISTS GRACE_RESEARCH_ONLY_PLANNERS)
    if(GRACE_BUILD_PLANNER_${planner})
        if(NOT GRACE_ENABLE_USC_PLANNERS)
            message(
                FATAL_ERROR
                "GRACE_BUILD_PLANNER_${planner} requires explicit opt-in with "
                "-DGRACE_ENABLE_USC_PLANNERS=ON. Review 3rd-party-licenses.txt "
                "and the optional USC planner setup in README.md first."
            )
        endif()
        list(APPEND GRACE_RESEARCH_ONLY_PLANNERS_ENABLED ${planner})
    endif()
endforeach()
if(GRACE_RESEARCH_ONLY_PLANNERS_ENABLED)
    list(JOIN GRACE_RESEARCH_ONLY_PLANNERS_ENABLED ", " enabled_planners)
    message(
        WARNING
        "USC planner opt-in enables ${enabled_planners}. "
        "These third-party planners are licensed for educational, research, "
        "and non-profit use; commercial use requires separate permission "
        "from the University of Southern California. See "
        "3rd-party-licenses.txt. Opt-in does not grant additional rights "
        "or establish license compatibility for redistribution."
    )
endif()

set(GRACE_BUILD_ANY_PLANNER OFF)
set(GRACE_BUILD_ANY_GRID_PLANNER OFF)
set(GRACE_BUILD_ANY_ROADMAP_PLANNER OFF)
set(GRACE_BUILD_ANY_CONTINUOUS_PLANNER OFF)
foreach(planner IN LISTS GRACE_PLANNERS)
    if(GRACE_BUILD_PLANNER_${planner})
        set(GRACE_BUILD_ANY_PLANNER ON)
    endif()
endforeach()
foreach(planner IN LISTS GRACE_GRID_PLANNERS)
    if(GRACE_BUILD_PLANNER_${planner})
        set(GRACE_BUILD_ANY_GRID_PLANNER ON)
    endif()
endforeach()
foreach(planner IN LISTS GRACE_ROADMAP_PLANNERS)
    if(GRACE_BUILD_PLANNER_${planner})
        set(GRACE_BUILD_ANY_ROADMAP_PLANNER ON)
    endif()
endforeach()
foreach(planner IN LISTS GRACE_CONTINUOUS_PLANNERS)
    if(GRACE_BUILD_PLANNER_${planner})
        set(GRACE_BUILD_ANY_CONTINUOUS_PLANNER ON)
    endif()
endforeach()
option(
    GRACE_USE_SYSTEM_BOX2D
    "Use an installed Box2D 3 package instead of downloading the pinned revision"
    ON
)
option(
    GRACE_USE_SYSTEM_FMT
    "Use an installed fmt package instead of downloading the pinned release"
    OFF
)
option(
    GRACE_USE_SYSTEM_GTEST
    "Use an installed GoogleTest package instead of downloading the pinned release"
    OFF
)
option(
    GRACE_USE_SYSTEM_FRONTEND_DEPS
    "Use installed SFML 3 and ImGui-SFML 3 packages instead of pinned releases"
    OFF
)
set(
    GRACE_DEPENDENCY_DOWNLOAD_DIR
    ""
    CACHE PATH
    "Optional directory for reusable verified FetchContent archives"
)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(
    GRACE_OPENROBOTS_PREFIX
    "/opt/openrobots"
    CACHE PATH
    "OpenRobots installation prefix; leave empty to disable the additional search path"
)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND GRACE_OPENROBOTS_PREFIX)
    list(APPEND CMAKE_PREFIX_PATH "${GRACE_OPENROBOTS_PREFIX}")
endif()
