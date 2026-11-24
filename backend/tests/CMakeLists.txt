# Copyright (c) 2026 Robert Bosch GmbH
# SPDX-License-Identifier: AGPL-3.0-only

include(GoogleTest)

add_test(
    NAME grace_test_planner_profiles
    COMMAND "${CMAKE_COMMAND}"
        "-DGRACE_SOURCE_DIR=${CMAKE_SOURCE_DIR}"
        -P "${CMAKE_CURRENT_SOURCE_DIR}/test_setup/test_planner_profiles.cmake"
)
set_tests_properties(grace_test_planner_profiles PROPERTIES LABELS "setup")

find_package(Git QUIET)
if(Git_FOUND)
    add_test(
        NAME grace_test_optional_submodules
        COMMAND "${CMAKE_COMMAND}"
            "-DGRACE_SOURCE_DIR=${CMAKE_SOURCE_DIR}"
            "-DTEST_BINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}"
            "-DGIT_EXECUTABLE=${GIT_EXECUTABLE}"
            -P "${CMAKE_CURRENT_SOURCE_DIR}/test_setup/test_optional_submodules.cmake"
    )
    set_tests_properties(
        grace_test_optional_submodules
        PROPERTIES LABELS "setup" TIMEOUT 60
    )
endif()

set(GRACE_TEST_FIXTURE_DIR "${CMAKE_CURRENT_BINARY_DIR}/fixtures")
file(MAKE_DIRECTORY "${GRACE_TEST_FIXTURE_DIR}")

function(grace_copy_test_fixture source relative_destination)
    get_filename_component(destination_dir
        "${GRACE_TEST_FIXTURE_DIR}/${relative_destination}" DIRECTORY)
    file(MAKE_DIRECTORY "${destination_dir}")
    configure_file(
        "${GRACE_BACKEND_SOURCE_DIR}/${source}"
        "${GRACE_TEST_FIXTURE_DIR}/${relative_destination}"
        COPYONLY
    )
endfunction()

grace_copy_test_fixture(
    data/maps/yaml/default/problem.yaml
    continuous/problem.yaml
)
function(grace_add_backend_test target source)
    set(options ENABLE_EXPORTS HERMETIC_ENVIRONMENT)
    set(one_value_args LABEL)
    set(multi_value_args LIBRARIES)
    cmake_parse_arguments(
        PARSE_ARGV 2
        TEST
        "${options}"
        "${one_value_args}"
        "${multi_value_args}"
    )

    if(NOT TEST_LABEL OR NOT TEST_LIBRARIES)
        message(FATAL_ERROR
            "${target} requires LABEL and at least one entry in LIBRARIES")
    endif()

    add_executable(${target} ${source})
    if(TEST_HERMETIC_ENVIRONMENT)
        target_sources(${target} PRIVATE EnvironmentTestMain.cpp)
    endif()
    target_compile_features(${target} PRIVATE cxx_std_20)
    target_include_directories(${target} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(
        ${target}
        PRIVATE
            GTest::gtest_main
            grace::warnings
            ${TEST_LIBRARIES}
    )
    if(TEST_ENABLE_EXPORTS)
        set_target_properties(${target} PROPERTIES ENABLE_EXPORTS ON)
    endif()
    gtest_discover_tests(
        ${target}
        TEST_PREFIX "${target}."
        DISCOVERY_MODE POST_BUILD
        PROPERTIES LABELS "${TEST_LABEL}"
    )
endfunction()

grace_add_backend_test(
    grace_test_env_grid
    test_envs/test_env_grid.cpp
    LABEL environment
    LIBRARIES grace::backend_io
    HERMETIC_ENVIRONMENT
)
grace_add_backend_test(
    grace_test_env_roadmap
    test_envs/test_env_roadmap.cpp
    LABEL environment
    LIBRARIES grace::backend_environment
    HERMETIC_ENVIRONMENT
)
grace_add_backend_test(
    grace_test_env_continuous
    test_envs/test_env_continuous.cpp
    LABEL environment
    LIBRARIES grace::backend_environment grace::yaml-cpp
    HERMETIC_ENVIRONMENT
)
target_compile_definitions(
    grace_test_env_continuous
    PRIVATE
        CONT_PROBLEM_PATH="${GRACE_TEST_FIXTURE_DIR}/continuous/problem.yaml"
)

function(grace_define_enabled_planners target)
    foreach(planner IN LISTS ARGN)
        if(GRACE_BUILD_PLANNER_${planner})
            target_compile_definitions(
                ${target}
                PRIVATE
                    GRACE_BUILD_PLANNER_${planner}=1
            )
        endif()
    endforeach()
endfunction()

if(GRACE_BUILD_PLANNER_INTEGRATION_TESTS)
    if(GRACE_BUILD_ANY_GRID_PLANNER)
        grace_copy_test_fixture(
            data/maps/movingAI/default/random-32-32-20.map
            grid/random-32-32-20.map
        )
        grace_copy_test_fixture(
            data/maps/movingAI/default/random-32-32-20-random-1.scen
            grid/random-32-32-20-random-1.scen
        )
        grace_add_backend_test(
            grace_test_planners_grid
            test_planners/test_planners_grid.cpp
            LABEL planner-grid
            LIBRARIES grace::backend
            ENABLE_EXPORTS
            HERMETIC_ENVIRONMENT
        )
        grace_define_enabled_planners(
            grace_test_planners_grid
            ${GRACE_GRID_PLANNERS}
        )
        target_compile_definitions(
            grace_test_planners_grid
            PRIVATE
                GRID_MAP_PATH="${GRACE_TEST_FIXTURE_DIR}/grid/random-32-32-20.map"
                GRID_SCEN_PATH="${GRACE_TEST_FIXTURE_DIR}/grid/random-32-32-20-random-1.scen"
        )
    endif()

    if(GRACE_BUILD_ANY_ROADMAP_PLANNER)
        grace_copy_test_fixture(
            data/maps/xml/default/roadmap/roadmap.xml
            roadmap/roadmap.xml
        )
        grace_copy_test_fixture(
            data/maps/xml/default/roadmap/roadmap_task.xml
            roadmap/roadmap_task.xml
        )
        grace_add_backend_test(
            grace_test_planners_roadmap
            test_planners/test_planners_roadmap.cpp
            LABEL planner-roadmap
            LIBRARIES
                grace::backend_sim_io
                grace::pugixml
            ENABLE_EXPORTS
            HERMETIC_ENVIRONMENT
        )
        grace_define_enabled_planners(
            grace_test_planners_roadmap
            ${GRACE_ROADMAP_PLANNERS}
        )
        target_compile_definitions(
            grace_test_planners_roadmap
            PRIVATE
                ROADMAP_MAP_PATH="${GRACE_TEST_FIXTURE_DIR}/roadmap/roadmap.xml"
                ROADMAP_SCEN_PATH="${GRACE_TEST_FIXTURE_DIR}/roadmap/roadmap_task.xml"
        )
    endif()

    if(GRACE_BUILD_ANY_CONTINUOUS_PLANNER)
        grace_add_backend_test(
            grace_test_planners_continuous
            test_planners/test_planners_continuous.cpp
            LABEL planner-continuous
            LIBRARIES
                grace::backend_sim_io
                grace::yaml-cpp
            ENABLE_EXPORTS
            HERMETIC_ENVIRONMENT
        )
        grace_define_enabled_planners(
            grace_test_planners_continuous
            ${GRACE_CONTINUOUS_PLANNERS}
        )
        target_compile_definitions(
            grace_test_planners_continuous
            PRIVATE
                CONT_PROBLEM_PATH="${GRACE_TEST_FIXTURE_DIR}/continuous/problem.yaml"
        )
    endif()
endif()

grace_add_backend_test(
    grace_test_metrics
    test_utils/test_metrics.cpp
    LABEL utility
    LIBRARIES grace::backend_util
)
grace_add_backend_test(
    grace_test_run_output
    test_utils/test_run_output.cpp
    LABEL utility
    LIBRARIES grace::backend_util
)
grace_add_backend_test(
    grace_test_benchmark_config
    test_benchmarking/test_benchmark_config.cpp
    LABEL benchmarking
    LIBRARIES grace::backend_benchmark_config
)
grace_add_backend_test(
    grace_test_io_roadmap_xml
    test_io/test_io_roadmap_xml.cpp
    LABEL io
    LIBRARIES grace::backend_io
    HERMETIC_ENVIRONMENT
)
grace_add_backend_test(
    grace_test_io_grid
    test_io/test_io_grid.cpp
    LABEL io
    LIBRARIES grace::backend_io
    HERMETIC_ENVIRONMENT
)
grace_add_backend_test(
    grace_test_map_convert
    test_tools/test_map_convert_con2grid.cpp
    LABEL tools
    LIBRARIES grace::backend_map_convert_cli
    HERMETIC_ENVIRONMENT
)

grace_add_backend_test(
    grace_test_simulation
    test_sim/test_simulation.cpp
    LABEL simulation
    LIBRARIES grace::backend
    HERMETIC_ENVIRONMENT
)

grace_add_backend_test(
    grace_test_sim_continuous_yaml_ops
    test_sim/test_sim_continuous_yaml_ops.cpp
    LABEL simulation
    LIBRARIES grace::backend_sim_io
    HERMETIC_ENVIRONMENT
)

grace_add_backend_test(
    grace_test_simulation_app
    test_app/test_simulation_app.cpp
    LABEL app
    LIBRARIES grace::backend_app
)

function(grace_add_test_plugin
         target output_name planner_id advertised_environment
         implementation_environment)
    add_library(${target} SHARED fixtures/FakePlannerPlugin.cpp)
    target_compile_features(${target} PRIVATE cxx_std_20)
    target_include_directories(
        ${target}
        PRIVATE
            "${GRACE_BACKEND_SOURCE_DIR}/inc"
    )
    target_compile_definitions(
        ${target}
        PRIVATE
            TEST_PLUGIN_ID=\"${planner_id}\"
            TEST_PLUGIN_ENVIRONMENT=${advertised_environment}
            TEST_PLANNER_ENVIRONMENT=EnvironmentType::${implementation_environment}
            ${ARGN}
    )
    target_link_libraries(
        ${target}
        PRIVATE
            box2d::box2d
            fmt::fmt-header-only
    )
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND
       CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        target_compile_options(${target} PRIVATE -fno-gnu-unique)
    endif()
    set_target_properties(
        ${target}
        PROPERTIES
            LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/plugins"
            RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/plugins"
            OUTPUT_NAME "${output_name}"
            WINDOWS_EXPORT_ALL_SYMBOLS ON
    )
endfunction()

grace_add_test_plugin(
    grace_test_plugin
    planner_eecbs_src
    eecbs-src
    GRACE_PLUGIN_ENVIRONMENT_GRID
    GridEnvironment
)
grace_add_test_plugin(
    grace_test_plugin_bad_id
    planner_lacam0_src
    wrong-planner-id
    GRACE_PLUGIN_ENVIRONMENT_GRID
    GridEnvironment
)
grace_add_test_plugin(
    grace_test_plugin_bad_environment
    planner_mapflns2_src
    mapflns2-src
    GRACE_PLUGIN_ENVIRONMENT_CONTINUOUS
    GridEnvironment
)
grace_add_test_plugin(
    grace_test_plugin_bad_implementation
    planner_cbsh2rtc_src
    cbsh2rtc-src
    GRACE_PLUGIN_ENVIRONMENT_GRID
    ContinuousEnvironment
)
grace_add_test_plugin(
    grace_test_plugin_legacy_abi
    planner_ccbs_src
    ccbs-src
    GRACE_PLUGIN_ENVIRONMENT_ROADMAP
    RoadmapEnvironment
    TEST_PLUGIN_ABI=2
)
grace_add_test_plugin(
    grace_test_plugin_invalid_environment
    planner_psippctc_src
    psippctc-src
    99
    RoadmapEnvironment
)
grace_add_test_plugin(
    grace_test_plugin_empty_id
    planner_dbcbs_src
    ""
    GRACE_PLUGIN_ENVIRONMENT_CONTINUOUS
    ContinuousEnvironment
)
grace_add_test_plugin(
    grace_test_plugin_null_planner
    planner_sst_src
    sst-src
    GRACE_PLUGIN_ENVIRONMENT_CONTINUOUS
    ContinuousEnvironment
    TEST_PLUGIN_CREATE_NULL
)
grace_add_test_plugin(
    grace_test_plugin_missing_metadata
    planner_missing_metadata
    unused
    GRACE_PLUGIN_ENVIRONMENT_GRID
    GridEnvironment
    TEST_PLUGIN_OMIT_ID
)
grace_add_test_plugin(
    grace_test_plugin_throwing_metadata
    planner_throwing_metadata
    unused
    GRACE_PLUGIN_ENVIRONMENT_GRID
    GridEnvironment
    TEST_PLUGIN_THROW_ID
)
grace_add_test_plugin(
    grace_test_plugin_throwing_create
    planner_throwing_create
    throwing-create
    GRACE_PLUGIN_ENVIRONMENT_GRID
    GridEnvironment
    TEST_PLUGIN_THROW_CREATE
)
grace_add_test_plugin(
    grace_test_plugin_throwing_destroy
    planner_throwing_destroy
    throwing-destroy
    GRACE_PLUGIN_ENVIRONMENT_GRID
    GridEnvironment
    TEST_PLUGIN_THROW_DESTROY
)
grace_add_test_plugin(
    grace_test_plugin_throwing_operations
    planner_throwing_operations
    throwing-operations
    GRACE_PLUGIN_ENVIRONMENT_GRID
    GridEnvironment
    TEST_PLUGIN_THROW_PLAN
    TEST_PLUGIN_THROW_CONFIGURE
)

grace_add_backend_test(
    grace_test_planner_factory
    test_planners/test_planner_factory.cpp
    LABEL plugin
    LIBRARIES grace::backend_plugin
)
target_compile_definitions(
    grace_test_planner_factory
    PRIVATE
        TEST_PLUGIN_DIR="$<TARGET_FILE_DIR:grace_test_plugin>"
)
add_dependencies(
    grace_test_planner_factory
    grace_test_plugin
    grace_test_plugin_bad_id
    grace_test_plugin_bad_environment
    grace_test_plugin_bad_implementation
    grace_test_plugin_legacy_abi
    grace_test_plugin_invalid_environment
    grace_test_plugin_empty_id
    grace_test_plugin_null_planner
    grace_test_plugin_missing_metadata
    grace_test_plugin_throwing_metadata
    grace_test_plugin_throwing_create
    grace_test_plugin_throwing_destroy
    grace_test_plugin_throwing_operations
)

find_program(GRACE_BASH_EXECUTABLE bash)
find_program(GRACE_ECHO_EXECUTABLE echo)
if(GRACE_BASH_EXECUTABLE AND GRACE_ECHO_EXECUTABLE)
    add_test(
        NAME grace_test_conversion_pipeline
        COMMAND
            "${GRACE_BASH_EXECUTABLE}"
            "${GRACE_BACKEND_SOURCE_DIR}/run_pipeline.sh"
            "${GRACE_ECHO_EXECUTABLE}"
    )
    set_tests_properties(
        grace_test_conversion_pipeline
        PROPERTIES
            LABELS "app"
            ENVIRONMENT
                "GRACE_PIPELINE_OUTDIR=${CMAKE_CURRENT_BINARY_DIR}/pipeline-smoke"
    )
endif()
