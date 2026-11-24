# Copyright (c) 2026 Robert Bosch GmbH
# SPDX-License-Identifier: AGPL-3.0-only

set(GRACE_FRONTEND_SOURCES
        input/Input.cpp

        platform/PlatformCore.cpp
        platform/PlatformEditors.cpp
        platform/PlatformOverlays.cpp
        platform/PlatformPlanning.cpp

        render/DebugConfig.cpp
        render/CrossComparisonRenderer.cpp
        render/DebugDrawQueue.cpp
        render/RendererSfml.cpp

        sample/Sample.cpp
        sample/SampleContext.cpp
        sample/SampleRegistry.cpp
        sample/SampleTasks.cpp

        ui/panels/AgentsPanel.cpp
        ui/panels/Draw.cpp
        ui/panels/EnvironmentPanel.cpp
        ui/panels/MetricsCompareUI.cpp
        ui/panels/MetricsUI.cpp
        ui/panels/PlannerPanel.cpp
        ui/panels/SimulationPanel.cpp
        ui/panels/UiWidgets.cpp
        ui/widgets/FilePicker.cpp

        ui/UiOverlays.cpp
        ui/UiStyle.cpp
        ui/UiTools.cpp
        ui/ViewController.cpp

        main.cpp
        Sample_MAPF_Continuous.cpp
        Sample_MAPF_CrossCompare.cpp
        Sample_MAPF_Grid.cpp
        Sample_MAPF_Roadmap.cpp
        extern/enkiTS/src/TaskScheduler.cpp
)

add_library(grace_frontend_workflows STATIC
        platform/CrossComparisonWorkflow.cpp
        platform/PlannerCatalog.cpp
        platform/PlanningWorkflow.cpp
        platform/ResetWorkflow.cpp
)
add_library(grace::frontend_workflows ALIAS grace_frontend_workflows)
target_compile_features(grace_frontend_workflows PUBLIC cxx_std_20)
target_include_directories(
        grace_frontend_workflows
        PUBLIC
            "${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(
        grace_frontend_workflows
        PUBLIC
            grace::backend
        PRIVATE
            grace::warnings
)

add_executable(grace_frontend ${GRACE_FRONTEND_SOURCES})
add_executable(grace::frontend ALIAS grace_frontend)
target_compile_features(grace_frontend PRIVATE cxx_std_20)

set(GRACE_JSMN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/extern/jsmn")
add_library(grace_jsmn INTERFACE)
add_library(grace::jsmn ALIAS grace_jsmn)
target_include_directories(grace_jsmn INTERFACE "${GRACE_JSMN_DIR}")

set(GRACE_SHARED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/extern/shared")
set(GRACE_SHARED_SOURCES
        "${GRACE_SHARED_DIR}/benchmarks.c"
        "${GRACE_SHARED_DIR}/benchmarks.h"
        "${GRACE_SHARED_DIR}/random.c"
        "${GRACE_SHARED_DIR}/random.h"
)
add_library(grace_shared STATIC ${GRACE_SHARED_SOURCES})
add_library(grace::shared ALIAS grace_shared)
target_include_directories(grace_shared PUBLIC "${GRACE_SHARED_DIR}")

target_include_directories(grace_frontend
        PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/extern/enkiTS/src"
)

target_compile_definitions(
        grace_frontend
        PRIVATE
        GRACE_REPO_ROOT="${PROJECT_SOURCE_DIR}"
)

target_link_libraries(grace_frontend
        PRIVATE
        grace::jsmn
        grace::shared
        grace::warnings
        fmt::fmt
        ImGui-SFML::ImGui-SFML
        box2d::box2d
        grace::backend
        grace::backend_sim_io
        grace::frontend_workflows
)

if(GRACE_BUILD_TESTS)
    add_subdirectory(tests)
endif()
