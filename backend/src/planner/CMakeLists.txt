# Copyright (c) 2026 Robert Bosch GmbH
# SPDX-License-Identifier: AGPL-3.0-only

add_library(grace_backend_planner_config INTERFACE)
add_library(grace::backend_planner_config ALIAS grace_backend_planner_config)
if(GRACE_BUILD_ANY_PLANNER)
    set(GRACE_HAS_BUNDLED_PLANNERS 1)
else()
    set(GRACE_HAS_BUNDLED_PLANNERS 0)
endif()
target_compile_definitions(
        grace_backend_planner_config
        INTERFACE
            BUILD_PATH="${CMAKE_CURRENT_BINARY_DIR}"
            GRACE_HAS_BUNDLED_PLANNERS=${GRACE_HAS_BUNDLED_PLANNERS}
)

set(
        DBCBS_BUILD_DRIVER
        ${GRACE_BUILD_PLANNER_DBCBS}
        CACHE BOOL
        "Build the db-CBS CLI"
        FORCE
)
set(
        DBCBS_BUILD_SST
        ${GRACE_BUILD_PLANNER_SST}
        CACHE BOOL
        "Build the SST core and CLI"
        FORCE
)

if(NOT GRACE_BUILD_ANY_PLANNER)
    add_library(grace_backend_planner_binaries INTERFACE)
    add_library(grace::backend_planner_binaries ALIAS grace_backend_planner_binaries)
    target_link_libraries(
            grace_backend_planner_binaries
            INTERFACE
                grace::backend_planner_config
                grace::backend_environment
                grace::backend_io
    )
else()
    function(grace_add_bundled_planner option source_dir binary_dir)
        if(NOT ${option})
            return()
        endif()
        if(NOT EXISTS "${source_dir}/CMakeLists.txt")
            string(REPLACE "GRACE_BUILD_PLANNER_" "" planner "${option}")
            if(planner IN_LIST GRACE_RESEARCH_ONLY_PLANNERS)
                message(
                    FATAL_ERROR
                    "Missing optional USC planner at ${source_dir}. "
                    "After reviewing its license, run from the repository root: "
                    "git submodule update --init --recursive --checkout -- ${source_dir}. "
                    "See README.md, or disable ${option}."
                )
            endif()
            message(
                FATAL_ERROR
                "Missing bundled planner at ${source_dir}. "
                "Initialize its submodule or disable ${option}."
            )
        endif()
        add_subdirectory("${source_dir}" "${binary_dir}")
    endfunction()

    grace_add_bundled_planner(
            GRACE_BUILD_PLANNER_CBSH2RTC
            "${GRACE_BACKEND_SOURCE_DIR}/extern/classical/CBSH2-RTC"
            extern/classical/CBSH2-RTC
    )
    grace_add_bundled_planner(
            GRACE_BUILD_PLANNER_EECBS
            "${GRACE_BACKEND_SOURCE_DIR}/extern/classical/EECBS"
            extern/classical/EECBS
    )
    grace_add_bundled_planner(
            GRACE_BUILD_PLANNER_LACAM0
            "${GRACE_BACKEND_SOURCE_DIR}/extern/classical/lacam0"
            extern/classical/lacam0
    )
    grace_add_bundled_planner(
            GRACE_BUILD_PLANNER_MAPFLNS2
            "${GRACE_BACKEND_SOURCE_DIR}/extern/classical/MAPF-LNS2"
            extern/classical/MAPF-LNS2
    )
    grace_add_bundled_planner(
            GRACE_BUILD_PLANNER_CCBS
            "${GRACE_BACKEND_SOURCE_DIR}/extern/roadmap/CCBS"
            extern/roadmap/CCBS
    )
    grace_add_bundled_planner(
            GRACE_BUILD_PLANNER_PSIPPCTC
            "${GRACE_BACKEND_SOURCE_DIR}/extern/roadmap/PSIPP-CTC"
            extern/roadmap/PSIPP-CTC
    )
    if(GRACE_BUILD_PLANNER_DBCBS OR GRACE_BUILD_PLANNER_SST)
        grace_add_bundled_planner(
                GRACE_BUILD_ANY_CONTINUOUS_PLANNER
                "${GRACE_BACKEND_SOURCE_DIR}/extern/continuous_space/db-CBS"
                extern/continuous_space/db-CBS
        )
    endif()

    add_library(grace_backend_planner_binaries STATIC)
    add_library(grace::backend_planner_binaries ALIAS grace_backend_planner_binaries)
    grace_configure_backend_library(grace_backend_planner_binaries)
    target_link_libraries(
            grace_backend_planner_binaries
            PUBLIC
                grace::backend_planner_config
                grace::backend_environment
                grace::backend_io
    )
endif()

register_planner(
        NAME            eecbs
        ENVIRONMENT     GRID
        MODE            BOTH
        CORE_TARGET     eecbs::core
        DRIVER_TARGET   eecbs
        BINARY_HEADER   "planner/classical/EECBS/EECBS_Binary.h"
        BINARY_CLASS    EECBS_Binary
        BINARY_CONFIG_TYPE EECBS_BinaryConfig
        SOLVER_HEADER   "inc/planner/classical/EECBS/EECBS_Source.h"
        SOLVER_CLASS    EECBS_Source
        CONFIG_TYPE     EECBS_SourceConfig
        BINARY_SOURCES  classical/EECBS/EECBS_Binary.cpp
        PLUGIN_SOURCES  classical/EECBS/EECBS_Source.cpp
        LINK_PRIVATE    grace::backend_io
)

register_planner(
        NAME            cbsh2rtc
        ENVIRONMENT     GRID
        MODE            BOTH
        CORE_TARGET     cbsh2rtc::core
        DRIVER_TARGET   cbs
        BINARY_HEADER   "planner/classical/CBSH2RTC/CBSH2RTC_Binary.h"
        BINARY_CLASS    CBSH2RTC_Binary
        BINARY_CONFIG_TYPE CBSH2RTC_BinaryConfig
        SOLVER_HEADER   "inc/planner/classical/CBSH2RTC/CBSH2RTC_Source.h"
        SOLVER_CLASS    CBSH2RTC_Source
        CONFIG_TYPE     CBSH2RTC_SourceConfig
        BINARY_SOURCES  classical/CBSH2RTC/CBSH2RTC_Binary.cpp
        PLUGIN_SOURCES  classical/CBSH2RTC/CBSH2RTC_Source.cpp
        LINK_PRIVATE    grace::backend_io
)

register_planner(
        NAME            lacam0
        ENVIRONMENT     GRID
        MODE            BOTH
        DEFAULT
        CORE_TARGET     lacam0::core
        DRIVER_TARGET   lacam0
        BINARY_HEADER   "planner/classical/LACAM0/LACAM0_Binary.h"
        BINARY_CLASS    LACAM0_Binary
        BINARY_CONFIG_TYPE LACAM0_BinaryConfig
        SOLVER_HEADER   "inc/planner/classical/LACAM0/LACAM0_Source.h"
        SOLVER_CLASS    LACAM0_Source
        CONFIG_TYPE     LACAM0_SourceConfig
        BINARY_SOURCES  classical/LACAM0/LACAM0_Binary.cpp
        PLUGIN_SOURCES  classical/LACAM0/LACAM0_Source.cpp
        LINK_PRIVATE    grace::backend_io
)

register_planner(
        NAME            mapflns2
        ENVIRONMENT     GRID
        MODE            BOTH
        CORE_TARGET     mapflns2::core
        DRIVER_TARGET   mapflns2
        BINARY_HEADER   "planner/classical/MAPFLNS2/MAPFLNS2_Binary.h"
        BINARY_CLASS    MAPFLNS2_Binary
        BINARY_CONFIG_TYPE MAPFLNS2_BinaryConfig
        SOLVER_HEADER   "inc/planner/classical/MAPFLNS2/MAPFLNS2_Source.h"
        SOLVER_CLASS    MAPFLNS2_Source
        CONFIG_TYPE     MAPFLNS2_SourceConfig
        BINARY_SOURCES  classical/MAPFLNS2/MAPFLNS2_Binary.cpp
        PLUGIN_SOURCES  classical/MAPFLNS2/MAPFLNS2_Source.cpp
        LINK_PRIVATE    grace::backend_io
)

add_subdirectory(classical/MAPFGPTDDG)
add_subdirectory(classical/SCRIMP)

register_planner(
        NAME            ccbs
        ENVIRONMENT     ROADMAP
        MODE            BOTH
        CORE_TARGET     ccbs::core
        DRIVER_TARGET   ccbs
        BINARY_HEADER   "planner/roadmap/CCBS/CCBS_Binary.h"
        BINARY_CLASS    CCBS_Binary
        BINARY_CONFIG_TYPE CCBS_BinaryConfig
        SOLVER_HEADER   "inc/planner/roadmap/CCBS/CCBS_Source.h"
        SOLVER_CLASS    CCBS_Source
        CONFIG_TYPE     CCBS_SourceConfig
        BINARY_SOURCES  roadmap/CCBS/CCBS_Binary.cpp
        PLUGIN_SOURCES  roadmap/CCBS/CCBS_Source.cpp
        LINK_PRIVATE    grace::backend_io
)

register_planner(
        NAME            psippctc
        ENVIRONMENT     ROADMAP
        MODE            BOTH
        DEFAULT
        CORE_TARGET     psippctc::core
        DRIVER_TARGET   psippctc
        BINARY_HEADER   "planner/roadmap/PSIPPCTC/PSIPPCTC_Binary.h"
        BINARY_CLASS    PSIPPCTC_Binary
        BINARY_CONFIG_TYPE PSIPPCTC_BinaryConfig
        SOLVER_HEADER   "inc/planner/roadmap/PSIPPCTC/PSIPPCTC_Source.h"
        SOLVER_CLASS    PSIPPCTC_Source
        CONFIG_TYPE     PSIPPCTC_SourceConfig
        BINARY_SOURCES  roadmap/PSIPPCTC/PSIPPCTC_Binary.cpp
        PLUGIN_SOURCES  roadmap/PSIPPCTC/PSIPPCTC_Source.cpp
        LINK_PRIVATE    grace::backend_io
)

register_planner(
        NAME            dbcbs
        ENVIRONMENT     CONTINUOUS
        MODE            BOTH
        DEFAULT
        CORE_TARGET     dbcbs::core
        DRIVER_TARGET   dbcbs
        BINARY_HEADER   "planner/continuous/DBCBS/DBCBS_Binary.h"
        BINARY_CLASS    DBCBS_Binary
        BINARY_CONFIG_TYPE DBCBS_BinaryConfig
        SOLVER_HEADER   "inc/planner/continuous/DBCBS/DBCBS_Source.h"
        SOLVER_CLASS    DBCBS_Source
        CONFIG_TYPE     DBCBS_SourceConfig
        BINARY_SOURCES  continuous/DBCBS/DBCBS_Binary.cpp  continuous/DBCBS/DBCBS_IO.cpp
        PLUGIN_SOURCES  continuous/DBCBS/DBCBS_Source.cpp continuous/DBCBS/DBCBS_IO.cpp
        LINK_PRIVATE    grace::backend_io
)

register_planner(
        NAME            sst
        ENVIRONMENT     CONTINUOUS
        MODE            BOTH
        CORE_TARGET     dbcbs::sst_core  # it lives inside dbcbs repo
        DRIVER_TARGET   dbcbs_sst
        BINARY_HEADER   "planner/continuous/SST/SST_Binary.h"
        BINARY_CLASS    SST_Binary
        BINARY_CONFIG_TYPE SST_BinaryConfig
        SOLVER_HEADER   "inc/planner/continuous/SST/SST_Source.h"
        SOLVER_CLASS    SST_Source
        CONFIG_TYPE     SST_SourceConfig
        BINARY_SOURCES  continuous/SST/SST_Binary.cpp
        PLUGIN_SOURCES  continuous/SST/SST_Source.cpp
        LINK_PRIVATE    grace::backend_io
)

set(
        GRACE_GENERATED_PLANNER_REGISTRY
        "${CMAKE_CURRENT_BINARY_DIR}/PlannerRegistry.cpp"
)
generate_planner_registry("${GRACE_GENERATED_PLANNER_REGISTRY}")
set(
        GRACE_GENERATED_PLANNER_REGISTRY
        "${GRACE_GENERATED_PLANNER_REGISTRY}"
        PARENT_SCOPE
)
