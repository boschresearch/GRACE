# Copyright (c) 2026 Robert Bosch GmbH
# SPDX-License-Identifier: AGPL-3.0-only

add_library(grace_backend_app STATIC
        SimulationApp.cpp
)
add_library(grace::backend_app ALIAS grace_backend_app)

grace_configure_backend_library(grace_backend_app)
target_link_libraries(
        grace_backend_app
        PUBLIC
            grace::backend
        PRIVATE
            Boost::program_options
            grace::backend_sim_io
)
