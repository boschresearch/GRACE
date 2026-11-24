# Copyright (c) 2026 Robert Bosch GmbH
# SPDX-License-Identifier: AGPL-3.0-only

add_subdirectory(util)
add_subdirectory(env)
add_subdirectory(robot)
add_subdirectory(io)
add_subdirectory(planner)
add_subdirectory(sim)
add_subdirectory(tools)

add_library(grace_backend_plugin STATIC
        planner/plugin/PlannerFactory.cpp
        planner/plugin/PluginLoader.cpp
        "${GRACE_GENERATED_PLANNER_REGISTRY}"
)
add_library(grace::backend_plugin ALIAS grace_backend_plugin)
grace_configure_backend_library(grace_backend_plugin)
target_link_libraries(
        grace_backend_plugin
        PUBLIC
            grace::backend_planner_binaries
        PRIVATE
            ${CMAKE_DL_LIBS}
)
finalize_planners(grace_backend_plugin)

add_library(grace_backend STATIC
        Simulation.cpp
)
add_library(grace::backend ALIAS grace_backend)

grace_configure_backend_library(grace_backend)
target_link_libraries(grace_backend
        PUBLIC
            grace::backend_sim
            grace::backend_map_convert
            grace::backend_io
            grace::backend_robot
            grace::backend_planner_binaries
            grace::backend_plugin
)

if(APPLE)
    target_link_libraries(grace_backend PUBLIC
            Boost::program_options
            Boost::filesystem
            Boost::serialization
            Boost::process
    )
else()
    target_link_libraries(grace_backend PUBLIC
            Boost::system
            Boost::program_options
            Boost::filesystem
            Boost::serialization
    )
endif()

finalize_planners(grace_backend)

add_library(grace_backend_sim_io STATIC
        io/SimContinuousYamlOps.cpp
        io/SimRoadmapOps.cpp
)
add_library(grace::backend_sim_io ALIAS grace_backend_sim_io)

grace_configure_backend_library(grace_backend_sim_io)
target_link_libraries(
        grace_backend_sim_io
        PUBLIC
            grace::backend
        PRIVATE
            grace::yaml-cpp
)

add_subdirectory(app)
