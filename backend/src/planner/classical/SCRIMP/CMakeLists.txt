# Copyright (c) 2026 Robert Bosch GmbH
# SPDX-License-Identifier: AGPL-3.0-only

if(NOT TARGET grace_backend_planner_config OR NOT TARGET grace_backend_planner_binaries)
    message(FATAL_ERROR "SCRIMP requires the backend planner aggregate targets")
endif()

if(GRACE_BUILD_PLANNER_SCRIMP)
    # Options
    set(SCRIMP_DOCKER_IMAGE "scrimp" CACHE STRING "Docker image for SCRIMP")
    set(SCRIMP_REPO_HOST_ROOT "${PROJECT_SOURCE_DIR}" CACHE PATH "Repository root to mount at /workspace")

# Wrapper
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/scrimp_docker.sh.in
        ${CMAKE_CURRENT_BINARY_DIR}/scrimp_docker.sh @ONLY)
    file(CHMOD ${CMAKE_CURRENT_BINARY_DIR}/scrimp_docker.sh
        PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
        GROUP_READ GROUP_EXECUTE
        WORLD_READ WORLD_EXECUTE)

# In-container driver path (we mount repo at /workspace)
    set(SCRIMP_DRIVER_IN_CONTAINER "/workspace/backend/src/planner/classical/SCRIMP/scrimp_driver.py")

# IMPORTED "executable" target so $<TARGET_FILE:...> resolves
    add_executable(scrimp_driver IMPORTED GLOBAL)
    set_target_properties(scrimp_driver PROPERTIES
        IMPORTED_LOCATION "${SCRIMP_DRIVER_IN_CONTAINER}"
)

# Per-planner python entry (do NOT reuse PYTHON_EXE to avoid conflicts)
    set(SCRIMP_PYTHON_EXE "${CMAKE_CURRENT_BINARY_DIR}/scrimp_docker.sh" CACHE STRING "" FORCE)

# Export macros for this planner
    target_compile_definitions(grace_backend_planner_config INTERFACE
        SCRIMP_PYTHON_EXE="${SCRIMP_PYTHON_EXE}"
    )
endif()

# Register planner (binary-only)
register_planner(
        NAME            scrimp
        ENVIRONMENT     GRID
        MODE            BINARY
        DRIVER_TARGET   scrimp_driver
        BINARY_HEADER   "planner/classical/SCRIMP/SCRIMP_Binary.h"
        BINARY_CLASS    SCRIMP_Binary
        BINARY_CONFIG_TYPE SCRIMP_BinaryConfig
        BINARY_SOURCES  SCRIMP_Binary.cpp
)

# Ensure wrapper exists before we try to use it
if(GRACE_BUILD_PLANNER_SCRIMP)
    add_dependencies(grace_backend_planner_binaries scrimp_driver)
endif()
