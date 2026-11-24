# Copyright (c) 2026 Robert Bosch GmbH
# SPDX-License-Identifier: AGPL-3.0-only

add_library(grace_project_warnings INTERFACE)
add_library(grace::warnings ALIAS grace_project_warnings)

if(MSVC)
    target_compile_options(grace_project_warnings INTERFACE /W4 /permissive-)
else()
    target_compile_options(
        grace_project_warnings
        INTERFACE
            -Wall
            -Wextra
            -Wpedantic
    )
endif()
