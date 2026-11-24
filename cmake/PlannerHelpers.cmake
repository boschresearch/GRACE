# Copyright (c) 2026 Robert Bosch GmbH
# SPDX-License-Identifier: AGPL-3.0-only

# (helper macros)

## (1) util function to control the visibility
function(set_hidden_visibility tgt)
    set_target_properties(${tgt} PROPERTIES
            CXX_VISIBILITY_PRESET hidden
            VISIBILITY_INLINES_HIDDEN ON
            POSITION_INDEPENDENT_CODE ON
    )
    if (UNIX AND NOT APPLE)
        target_link_options(${tgt} PRIVATE -Wl,--exclude-libs,ALL)
    endif()
endfunction()

## (2) One-liner to define planner source plugins
function(add_planner_plugin name)
    if(NOT GRACE_BACKEND_SOURCE_DIR)
        message(FATAL_ERROR "add_planner_plugin() requires GRACE_BACKEND_SOURCE_DIR")
    endif()

    # ARGN = srcs...; last arg(s) can be LINK_PRIVATE <libs...>
    set(options)
    set(oneValueArgs)
    set(multiValueArgs LINK_PRIVATE)
    cmake_parse_arguments(APP "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    add_library(${name} SHARED ${APP_UNPARSED_ARGUMENTS})
    target_include_directories(${name} PRIVATE
            ${GRACE_BACKEND_SOURCE_DIR}/src
            ${GRACE_BACKEND_SOURCE_DIR}
    )
    if (APP_LINK_PRIVATE)
        target_link_libraries(${name} PRIVATE ${APP_LINK_PRIVATE} fmt::fmt-header-only)
    else()
        target_link_libraries(${name} PRIVATE fmt::fmt-header-only)
    endif()
    target_link_libraries(${name} PRIVATE grace::warnings)
    target_compile_features(${name} PRIVATE cxx_std_20)
    set_hidden_visibility(${name})
endfunction()

# (3) Register build targets and runtime metadata for one planner.
#
# Required metadata:
#   NAME, ENVIRONMENT (GRID | ROADMAP | CONTINUOUS), MODE
# Binary variants additionally require:
#   DRIVER_TARGET, BINARY_HEADER, BINARY_CLASS, BINARY_CONFIG_TYPE
# Source variants additionally require:
#   CORE_TARGET, SOLVER_HEADER, SOLVER_CLASS, CONFIG_TYPE
# DEFAULT marks the source variant, or the binary variant for MODE=BINARY.
function(_grace_append_planner_descriptor runtime_id descriptor)
    get_property(
            _registered_ids
            GLOBAL
            PROPERTY GRACE_PLANNER_REGISTRY_IDS
    )
    if(runtime_id IN_LIST _registered_ids)
        message(FATAL_ERROR
                "Planner runtime ID '${runtime_id}' is registered more than once")
    endif()
    set_property(
            GLOBAL
            APPEND
            PROPERTY GRACE_PLANNER_REGISTRY_IDS
            "${runtime_id}"
    )
    set_property(
            GLOBAL
            APPEND_STRING
            PROPERTY GRACE_PLANNER_REGISTRY_ENTRIES
            "${descriptor}\n"
    )
endfunction()

function(_grace_set_default_planner environment runtime_id)
    get_property(
            _existing_default
            GLOBAL
            PROPERTY "GRACE_PLANNER_DEFAULT_${environment}"
    )
    if(_existing_default)
        message(FATAL_ERROR
                "${environment} already has default planner ${_existing_default}; "
                "cannot also register ${runtime_id}")
    endif()
    set_property(
            GLOBAL
            PROPERTY "GRACE_PLANNER_DEFAULT_${environment}"
            "${runtime_id}"
    )
endfunction()

function(register_planner)
    set(options DEFAULT)
    set(oneValueArgs
            NAME
            ENVIRONMENT
            MODE
            CORE_TARGET
            DRIVER_TARGET
            BINARY_HEADER
            BINARY_CLASS
            BINARY_CONFIG_TYPE
            PLUGIN_NAME
            SOLVER_HEADER
            SOLVER_CLASS
            CONFIG_TYPE
            CONFIG_INIT
    )
    set(multiValueArgs BINARY_SOURCES PLUGIN_SOURCES LINK_PRIVATE)
    cmake_parse_arguments(RP "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if(RP_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR
                "register_planner(${RP_NAME}): unknown arguments: ${RP_UNPARSED_ARGUMENTS}")
    endif()
    if(NOT RP_NAME OR NOT RP_ENVIRONMENT)
        message(FATAL_ERROR
                "register_planner(): NAME and ENVIRONMENT are required")
    endif()

    if(NOT RP_MODE)
        set(RP_MODE BOTH)
    endif()
    string(TOUPPER "${RP_MODE}" RP_MODE)
    if(NOT (RP_MODE STREQUAL "BINARY" OR
            RP_MODE STREQUAL "SOURCE" OR
            RP_MODE STREQUAL "BOTH"))
        message(FATAL_ERROR "register_planner(): MODE must be BINARY, SOURCE, or BOTH")
    endif()

    string(TOUPPER "${RP_ENVIRONMENT}" RP_ENVIRONMENT)
    if(RP_ENVIRONMENT STREQUAL "GRID")
        set(_environment_type GridEnvironment)
        set(_plugin_environment GRACE_PLUGIN_ENVIRONMENT_GRID)
    elseif(RP_ENVIRONMENT STREQUAL "ROADMAP")
        set(_environment_type RoadmapEnvironment)
        set(_plugin_environment GRACE_PLUGIN_ENVIRONMENT_ROADMAP)
    elseif(RP_ENVIRONMENT STREQUAL "CONTINUOUS")
        set(_environment_type ContinuousEnvironment)
        set(_plugin_environment GRACE_PLUGIN_ENVIRONMENT_CONTINUOUS)
    else()
        message(FATAL_ERROR
                "register_planner(${RP_NAME}): ENVIRONMENT must be GRID, ROADMAP, or CONTINUOUS")
    endif()

    if(NOT TARGET grace_backend_planner_config)
        message(FATAL_ERROR "register_planner(): grace_backend_planner_config must exist")
    endif()
    if(NOT TARGET grace_backend_planner_binaries)
        message(FATAL_ERROR "register_planner(): grace_backend_planner_binaries must exist")
    endif()

    string(TOUPPER "${RP_NAME}" _UP)
    set(_build_option "GRACE_BUILD_PLANNER_${_UP}")
    if(NOT DEFINED ${_build_option})
        message(FATAL_ERROR
                "register_planner(${RP_NAME}): ${_build_option} is not defined")
    endif()
    if(${_build_option})
        set(_is_bundled true)
        set_property(
                GLOBAL APPEND PROPERTY GRACE_ENABLED_PLANNERS "${RP_NAME}")
        set_property(
                GLOBAL PROPERTY "GRACE_HAS_${RP_ENVIRONMENT}_PLANNERS" TRUE)
        target_compile_definitions(
                grace_backend_planner_config
                INTERFACE
                    ${_build_option}=1
        )
    else()
        set(_is_bundled false)
    endif()
    if(NOT RP_PLUGIN_NAME)
        set(RP_PLUGIN_NAME "planner_${RP_NAME}_src")
    endif()

    if(RP_MODE STREQUAL "BINARY" OR RP_MODE STREQUAL "BOTH")
        if(NOT RP_BINARY_HEADER OR
           NOT RP_BINARY_CLASS OR
           NOT RP_BINARY_CONFIG_TYPE)
            message(FATAL_ERROR
                    "register_planner(${RP_NAME}): BINARY_HEADER, BINARY_CLASS, "
                    "and BINARY_CONFIG_TYPE are required for MODE=${RP_MODE}")
        endif()

        set(_binary_default false)
        if(RP_DEFAULT AND RP_MODE STREQUAL "BINARY")
            set(_binary_default true)
            _grace_set_default_planner(
                    "${RP_ENVIRONMENT}"
                    "${RP_NAME}-bin"
            )
        endif()
        set(_built_in_factory nullptr)
        if(${_build_option})
            set_property(GLOBAL APPEND_STRING
                    PROPERTY GRACE_PLANNER_REGISTRY_INCLUDES
                    "#include \"${RP_BINARY_HEADER}\"\n")
            set(
                    _built_in_factory
                    "&createBuiltInPlanner<${RP_BINARY_CLASS}, ${RP_BINARY_CONFIG_TYPE}>"
            )
        endif()
        _grace_append_planner_descriptor(
                "${RP_NAME}-bin"
                "    {\"${RP_NAME}-bin\", EnvironmentType::${_environment_type}, PlannerImplementation::Binary, {}, ${_binary_default}, ${_is_bundled}, ${_built_in_factory}},"
        )

        if(${_build_option})
            if(NOT RP_DRIVER_TARGET)
                message(FATAL_ERROR
                        "register_planner(${RP_NAME}): DRIVER_TARGET is required for MODE=${RP_MODE}")
            endif()
            if(RP_BINARY_SOURCES)
                target_sources(
                        grace_backend_planner_binaries
                        PRIVATE
                            ${RP_BINARY_SOURCES}
                )
            endif()
            set(_driver_executable "$<TARGET_FILE:${RP_DRIVER_TARGET}>")
            target_compile_definitions(
                    grace_backend_planner_config
                    INTERFACE
                        ${_UP}_DRIVER_PATH="${_driver_executable}"
                        ${_UP}_HAS_BINARY=1
            )
            add_dependencies(
                    grace_backend_planner_config
                    ${RP_DRIVER_TARGET}
            )
        endif()
    endif()

    if(RP_MODE STREQUAL "SOURCE" OR RP_MODE STREQUAL "BOTH")
        if(NOT RP_SOLVER_HEADER OR NOT RP_SOLVER_CLASS OR NOT RP_CONFIG_TYPE)
            message(FATAL_ERROR
                    "register_planner(${RP_NAME}): SOLVER_HEADER, SOLVER_CLASS, "
                    "and CONFIG_TYPE are required for MODE=${RP_MODE}")
        endif()

        set(_source_default false)
        if(RP_DEFAULT)
            set(_source_default true)
            _grace_set_default_planner(
                    "${RP_ENVIRONMENT}"
                    "${RP_NAME}-src"
            )
        endif()
        _grace_append_planner_descriptor(
                "${RP_NAME}-src"
                "    {\"${RP_NAME}-src\", EnvironmentType::${_environment_type}, PlannerImplementation::Plugin, \"${RP_PLUGIN_NAME}\", ${_source_default}, ${_is_bundled}, nullptr},"
        )

        if(${_build_option})
            if(NOT RP_CORE_TARGET)
                message(FATAL_ERROR
                        "register_planner(${RP_NAME}): CORE_TARGET is required for MODE=${RP_MODE}")
            endif()
            set(_plugin_sources
                    ${RP_PLUGIN_SOURCES}
                    ${GRACE_BACKEND_SOURCE_DIR}/src/planner/plugin/plugin_entry.cpp
            )
            set(_plugin_private_dependencies
                    ${RP_CORE_TARGET}
                    grace::backend_environment
            )
            if(RP_LINK_PRIVATE)
                list(APPEND
                        _plugin_private_dependencies
                        ${RP_LINK_PRIVATE}
                )
            endif()

            add_planner_plugin(
                    ${RP_PLUGIN_NAME}
                    ${_plugin_sources}
                    LINK_PRIVATE
                        ${_plugin_private_dependencies}
            )

            target_compile_definitions(
                    ${RP_PLUGIN_NAME}
                    PRIVATE
                        PLUGIN_PLANNER_HEADER=\"${RP_SOLVER_HEADER}\"
                        PLUGIN_PLANNER_CLASS=${RP_SOLVER_CLASS}
                        PLUGIN_CONFIG_TYPE=${RP_CONFIG_TYPE}
                        PLUGIN_ID_STR=\"${RP_NAME}-src\"
                        PLUGIN_ENVIRONMENT=${_plugin_environment}
            )
            if(RP_CONFIG_INIT)
                target_compile_definitions(
                        ${RP_PLUGIN_NAME}
                        PRIVATE
                            PLUGIN_CONFIG_INIT=${RP_CONFIG_INIT}
                )
            endif()
            target_compile_definitions(
                    grace_backend_planner_config
                    INTERFACE
                        ${_UP}_HAS_SOURCE=1
            )
            set_property(
                    GLOBAL
                    APPEND
                    PROPERTY GRACE_REGISTERED_PLUGINS
                    ${RP_PLUGIN_NAME}
            )
        endif()
    endif()
endfunction()

function(generate_planner_registry output_file)
    foreach(_environment GRID ROADMAP CONTINUOUS)
        get_property(
                _default
                GLOBAL
                PROPERTY "GRACE_PLANNER_DEFAULT_${_environment}"
        )
        if(NOT _default)
            message(FATAL_ERROR
                    "No default planner registered for ${_environment}")
        endif()
    endforeach()

    get_property(
            GRACE_PLANNER_BINARY_INCLUDES
            GLOBAL
            PROPERTY GRACE_PLANNER_REGISTRY_INCLUDES
    )
    get_property(
            GRACE_PLANNER_DESCRIPTOR_ENTRIES
            GLOBAL
            PROPERTY GRACE_PLANNER_REGISTRY_ENTRIES
    )
    get_property(
            _planner_ids
            GLOBAL
            PROPERTY GRACE_PLANNER_REGISTRY_IDS
    )
    list(LENGTH _planner_ids GRACE_PLANNER_DESCRIPTOR_COUNT)
    configure_file(
            "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/PlannerRegistry.cpp.in"
            "${output_file}"
            @ONLY
    )
endfunction()

function(finalize_planners owner_target)
    get_property(_plugins GLOBAL PROPERTY GRACE_REGISTERED_PLUGINS)
    if (_plugins)
        add_dependencies(${owner_target} ${_plugins})
    endif()
endfunction()
