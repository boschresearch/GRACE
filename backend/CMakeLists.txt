# Copyright (c) 2026 Robert Bosch GmbH
# SPDX-License-Identifier: AGPL-3.0-only

set(GRACE_BACKEND_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

function(grace_configure_backend_library target)
    target_include_directories(
        ${target}
        PUBLIC
            "$<BUILD_INTERFACE:${GRACE_BACKEND_SOURCE_DIR}/inc>"
        PRIVATE
            "${GRACE_BACKEND_SOURCE_DIR}"
            "${GRACE_BACKEND_SOURCE_DIR}/src"
    )
    target_compile_features(${target} PUBLIC cxx_std_20)
    target_link_libraries(${target} PRIVATE grace::warnings)
    set_hidden_visibility(${target})
endfunction()

add_subdirectory(src)

function(grace_add_backend_executable target source)
    add_executable(${target} ${source})
    target_compile_features(${target} PRIVATE cxx_std_20)
    target_link_libraries(${target} PRIVATE grace::warnings)
    set_target_properties(${target} PROPERTIES ENABLE_EXPORTS ON)
endfunction()

grace_add_backend_executable(grace_backend_run main.cpp)
target_link_libraries(
    grace_backend_run
    PRIVATE
        grace::backend
        grace::backend_app
)
target_compile_definitions(
    grace_backend_run
    PRIVATE
        GRACE_BACKEND_DATA_DIR="${GRACE_BACKEND_SOURCE_DIR}/data"
)
set_target_properties(
    grace_backend_run
    PROPERTIES
        OUTPUT_NAME grace-backend
)

grace_add_backend_executable(grace_backend_convert main_mapConvert.cpp)
target_link_libraries(
    grace_backend_convert
    PRIVATE
        grace::backend_map_convert_cli
)
target_compile_definitions(
    grace_backend_convert
    PRIVATE
        GRACE_BACKEND_DATA_DIR="${GRACE_BACKEND_SOURCE_DIR}/data"
)

if(GRACE_BUILD_BENCHMARKS OR GRACE_BUILD_TESTS)
    add_library(
        grace_backend_benchmark_config STATIC
        src/benchmarking/BenchmarkConfig.cpp
    )
    add_library(
        grace::backend_benchmark_config ALIAS
        grace_backend_benchmark_config
    )
    grace_configure_backend_library(grace_backend_benchmark_config)
    target_link_libraries(
        grace_backend_benchmark_config
        PRIVATE
            Boost::program_options
            fmt::fmt-header-only
            grace::backend_io
            grace::pugixml
            nlohmann_json::nlohmann_json
    )
endif()

if(GRACE_BUILD_BENCHMARKS)
    find_package(Threads REQUIRED)
    add_library(
        grace_backend_benchmark_lib STATIC
        src/benchmarking/BenchmarkRun.cpp
    )
    add_library(
        grace::backend_benchmark ALIAS
        grace_backend_benchmark_lib
    )
    grace_configure_backend_library(grace_backend_benchmark_lib)
    target_link_libraries(
        grace_backend_benchmark_lib
        PUBLIC
            grace::backend_benchmark_config
        PRIVATE
            grace::backend
            grace::backend_sim_io
            Threads::Threads
    )

    grace_add_backend_executable(
        grace_backend_benchmark
        benchmarking/benchmarking.cpp
    )
    target_link_libraries(
        grace_backend_benchmark
        PRIVATE
            grace::backend_benchmark
    )
endif()

if(GRACE_BUILD_TESTS)
    add_subdirectory(tests)
endif()