# Copyright (c) 2026 Robert Bosch GmbH
# SPDX-License-Identifier: AGPL-3.0-only

if(NOT TARGET grace_backend_planner_config OR NOT TARGET grace_backend_planner_binaries)
    message(FATAL_ERROR "MAPFGPTDDG requires the backend planner aggregate targets")
endif()

if(GRACE_BUILD_PLANNER_MAPFGPTDDG)
    # ---- Options ----
    set(MAPFGPTDDG_DOCKER_IMAGE "mapf-gpt" CACHE STRING "Docker image for MAPFGPTDDG")
    set(MAPFGPTDDG_WEIGHTS_DIR "" CACHE PATH "Host path to hf_weights (optional)")


# ---- Launcher/driver target (must look like an executable for $<TARGET_FILE:...>) ----

# mount this host path to /workspace
    set(MAPFGPTDDG_REPO_HOST_ROOT "${PROJECT_SOURCE_DIR}" CACHE PATH "Repository root to mount at /workspace")

# Generate docker wrapper
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/mapfgptddg_docker.sh.in
        ${CMAKE_CURRENT_BINARY_DIR}/mapfgptddg_docker.sh
        @ONLY
)
    file(CHMOD
        ${CMAKE_CURRENT_BINARY_DIR}/mapfgptddg_docker.sh
        PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
        GROUP_READ  GROUP_EXECUTE
        WORLD_READ  WORLD_EXECUTE
)

# use the driver inside the mounted repo
    set(MAPFGPTDDG_DRIVER_IN_CONTAINER
        "/workspace/backend/src/planner/classical/MAPFGPTDDG/mapfgptddg_driver.py")

# Create an IMPORTED executable target pointing at the wrapper
    add_executable(mapfgptddg_driver IMPORTED GLOBAL)
    set_target_properties(mapfgptddg_driver PROPERTIES
        IMPORTED_LOCATION "${MAPFGPTDDG_DRIVER_IN_CONTAINER}"
)

# The C++ adapter will exec PYTHON_EXE with first arg = $<TARGET_FILE:mapfgptddg_driver>
    set(MAPFGPTDDG_PYTHON_EXE "${CMAKE_CURRENT_BINARY_DIR}/mapfgptddg_docker.sh" CACHE STRING "" FORCE)

# export PYTHON_EXE macro (register_planner will export MAPFGPTDDG_DRIVER_PATH)
    target_compile_definitions(grace_backend_planner_config INTERFACE
        MAPFGPTDDG_PYTHON_EXE="${MAPFGPTDDG_PYTHON_EXE}"
    )
endif()

# Register binary-only adapter (sources path is relative to THIS dir)
register_planner(
        NAME            mapfgptddg
        ENVIRONMENT     GRID
        MODE            BINARY
        DRIVER_TARGET   mapfgptddg_driver
        BINARY_HEADER   "planner/classical/MAPFGPTDDG/MAPFGPTDDG_Binary.h"
        BINARY_CLASS    MAPFGPTDDG_Binary
        BINARY_CONFIG_TYPE MAPFGPTDDG_BinaryConfig
        BINARY_SOURCES  MAPFGPTDDG_Binary.cpp
)

# Optional: make aggregate binaries wait for the driver wrapper/copy to exist
if(GRACE_BUILD_PLANNER_MAPFGPTDDG)
    add_dependencies(grace_backend_planner_binaries mapfgptddg_driver)
endif()
