# Copyright (c) 2026 Robert Bosch GmbH
# SPDX-License-Identifier: AGPL-3.0-only

add_library(grace_backend_util STATIC
        ProcRunner.cpp
        Logger.cpp
        Metrics.cpp
        ResultsDir.cpp
        RunOutput.cpp
)
add_library(grace::backend_util ALIAS grace_backend_util)

grace_configure_backend_library(grace_backend_util)
target_link_libraries(
        grace_backend_util
        PUBLIC
            fmt::fmt
            box2d::box2d
)
