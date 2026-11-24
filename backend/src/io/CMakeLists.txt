# Copyright (c) 2026 Robert Bosch GmbH
# SPDX-License-Identifier: AGPL-3.0-only

add_library(grace_backend_io STATIC
        ContinuousYaml.cpp
        GridLORR.cpp
        GridMovingAI.cpp
        GridStartGoal.cpp
        GridXML.cpp
        RoadmapXML.cpp
)
add_library(grace::backend_io ALIAS grace_backend_io)

grace_configure_backend_library(grace_backend_io)
target_link_libraries(
        grace_backend_io
        PUBLIC
            grace::backend_environment
            grace::backend_util
        PRIVATE
            grace::pugixml
            grace::yaml-cpp
)
