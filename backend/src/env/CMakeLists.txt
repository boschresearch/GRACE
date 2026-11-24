# Copyright (c) 2026 Robert Bosch GmbH
# SPDX-License-Identifier: AGPL-3.0-only

add_library(grace_backend_environment STATIC
        BaseEnvironment.cpp
        ContinuousEnvironment.cpp
        GridEnvironment.cpp
        RoadmapEnvironment.cpp
)
add_library(grace::backend_environment ALIAS grace_backend_environment)

grace_configure_backend_library(grace_backend_environment)
target_link_libraries(
        grace_backend_environment
        PUBLIC
            grace::backend_util
            box2d::box2d
)