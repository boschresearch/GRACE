# Copyright (c) 2026 Robert Bosch GmbH
# SPDX-License-Identifier: AGPL-3.0-only

add_library(grace_backend_robot STATIC
        Controller.cpp
        RobotSystem.cpp
        ActionSpace.cpp
)
add_library(grace::backend_robot ALIAS grace_backend_robot)

grace_configure_backend_library(grace_backend_robot)
target_link_libraries(
        grace_backend_robot
        PUBLIC
            grace::backend_util
            box2d::box2d
)
