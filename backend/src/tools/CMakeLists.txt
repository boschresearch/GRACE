# Copyright (c) 2026 Robert Bosch GmbH
# SPDX-License-Identifier: AGPL-3.0-only

add_library(grace_backend_map_convert STATIC
        MapConvertCore.cpp
        MapConvertGrid.cpp
        MapConvertRoadmap.cpp
        MapConvertScenario.cpp
)
add_library(grace::backend_map_convert ALIAS grace_backend_map_convert)

grace_configure_backend_library(grace_backend_map_convert)
target_link_libraries(grace_backend_map_convert
        PUBLIC
            grace::backend_io
            grace::backend_environment
            grace::backend_util
        PRIVATE
            grace::ompl
)

add_library(grace_backend_map_convert_cli STATIC
        MapConvertCli.cpp
)
add_library(grace::backend_map_convert_cli ALIAS grace_backend_map_convert_cli)

grace_configure_backend_library(grace_backend_map_convert_cli)
target_link_libraries(grace_backend_map_convert_cli
        PUBLIC
            grace::backend_map_convert
        PRIVATE
            Boost::program_options
)
