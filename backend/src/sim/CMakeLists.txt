# Copyright (c) 2026 Robert Bosch GmbH
# SPDX-License-Identifier: AGPL-3.0-only

add_library(grace_backend_sim STATIC
        GoalCheck.cpp
        GridConflictAuditor.cpp
        PlanVerifier.cpp
        SpawnRobot.cpp
)
add_library(grace::backend_sim ALIAS grace_backend_sim)

grace_configure_backend_library(grace_backend_sim)
target_link_libraries(
        grace_backend_sim
        PUBLIC
            grace::backend_environment
            grace::backend_io
            grace::backend_util
            grace::backend_planner_binaries
            box2d::box2d
)
