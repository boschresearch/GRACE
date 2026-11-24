# Copyright (c) 2026 Robert Bosch GmbH
# SPDX-License-Identifier: AGPL-3.0-only

include(FetchContent)

set(GRACE_FETCHCONTENT_DOWNLOAD_ARGS)
if(GRACE_DEPENDENCY_DOWNLOAD_DIR)
    list(
        APPEND
        GRACE_FETCHCONTENT_DOWNLOAD_ARGS
        DOWNLOAD_DIR
        "${GRACE_DEPENDENCY_DOWNLOAD_DIR}"
    )
endif()

if(GRACE_BUILD_BACKEND OR GRACE_BUILD_FRONTEND)
    if(GRACE_USE_SYSTEM_BOX2D)
        find_package(box2d 3 CONFIG REQUIRED)
    else()
        FetchContent_Declare(
            box2d
            URL
                "https://github.com/erincatto/box2d/archive/56edae79f2949d86142b03450d5d60f63bcf5a6f.tar.gz"
            URL_HASH
                "SHA256=fa412f9d70d34a8a94618db71406817da23cd77ada45ce5d748a02db6363073d"
            ${GRACE_FETCHCONTENT_DOWNLOAD_ARGS}
        )
        FetchContent_MakeAvailable(box2d)
    endif()

    if(GRACE_USE_SYSTEM_FMT)
        find_package(fmt CONFIG REQUIRED)
    else()
        FetchContent_Declare(
            fmt
            URL
                "https://github.com/fmtlib/fmt/archive/1be298e1bd68957e4cd352e1f676f00e07dcfb57.tar.gz"
            URL_HASH
                "SHA256=d041f836d9b2a3c1c0fe3cce81ffe6a4a257028faa2f4af0dd20d7e58b493766"
            ${GRACE_FETCHCONTENT_DOWNLOAD_ARGS}
        )
        FetchContent_MakeAvailable(fmt)
    endif()
endif()

if(GRACE_BUILD_BACKEND)
    if(APPLE)
        set(GRACE_BOOST_COMPONENTS program_options filesystem serialization process)
    else()
        set(GRACE_BOOST_COMPONENTS program_options filesystem serialization system)
    endif()

    find_package(Boost REQUIRED COMPONENTS ${GRACE_BOOST_COMPONENTS})
    find_package(ompl REQUIRED)
    find_package(pugixml REQUIRED)
    find_package(yaml-cpp REQUIRED)
    if(GRACE_BUILD_TESTS OR GRACE_BUILD_BENCHMARKS)
        find_package(nlohmann_json CONFIG REQUIRED)
    endif()
    find_package(Python3 COMPONENTS Interpreter OPTIONAL_COMPONENTS Development)

    add_library(grace_ompl INTERFACE)
    add_library(grace::ompl ALIAS grace_ompl)
    if(TARGET ompl::ompl)
        target_link_libraries(grace_ompl INTERFACE ompl::ompl)
    elseif(TARGET ompl)
        target_link_libraries(grace_ompl INTERFACE ompl)
    else()
        target_include_directories(grace_ompl INTERFACE ${OMPL_INCLUDE_DIRS})
        target_link_libraries(grace_ompl INTERFACE ${OMPL_LIBRARIES})
    endif()

    add_library(grace_pugixml INTERFACE)
    add_library(grace::pugixml ALIAS grace_pugixml)
    if(TARGET pugixml::pugixml)
        target_link_libraries(grace_pugixml INTERFACE pugixml::pugixml)
    else()
        target_link_libraries(grace_pugixml INTERFACE pugixml)
    endif()

    add_library(grace_yaml_cpp INTERFACE)
    add_library(grace::yaml-cpp ALIAS grace_yaml_cpp)
    if(TARGET yaml-cpp::yaml-cpp)
        target_link_libraries(grace_yaml_cpp INTERFACE yaml-cpp::yaml-cpp)
    else()
        target_link_libraries(grace_yaml_cpp INTERFACE yaml-cpp)
    endif()

    if(GRACE_BUILD_ANY_CONTINUOUS_PLANNER)
        find_package(crocoddyl REQUIRED)
    endif()

    if(GRACE_BUILD_TESTS)
        if(GRACE_USE_SYSTEM_GTEST)
            find_package(GTest REQUIRED)
        else()
            FetchContent_Declare(
                googletest
                URL
                    "https://github.com/google/googletest/archive/52eb8108c5bdec04579160ae17225d66034bd723.tar.gz"
                URL_HASH
                    "SHA256=745c55415660044610f7fcd3af7a6420d5de16a7dbb9ebfe2e131275676232be"
                ${GRACE_FETCHCONTENT_DOWNLOAD_ARGS}
            )
            set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
            FetchContent_MakeAvailable(googletest)
        endif()
    endif()
endif()

if(GRACE_BUILD_FRONTEND)
    if(GRACE_USE_SYSTEM_FRONTEND_DEPS)
        find_package(SFML 3 CONFIG REQUIRED COMPONENTS Graphics Window System)
        find_package(ImGui-SFML CONFIG REQUIRED)
    else()
        set(SFML_BUILD_AUDIO OFF CACHE BOOL "" FORCE)
        set(SFML_BUILD_NETWORK OFF CACHE BOOL "" FORCE)
        set(IMGUI_SFML_FIND_SFML OFF CACHE BOOL "" FORCE)
        set(IMGUI_SFML_IMGUI_DEMO ON CACHE BOOL "" FORCE)

        if(APPLE)
            find_package(Freetype REQUIRED)
            set(SFML_USE_SYSTEM_DEPS ON CACHE BOOL "" FORCE)
            set(IMGUI_SFML_USE_FREETYPE ON CACHE BOOL "" FORCE)
            set(IMGUI_SFML_FREETYPE_TARGET Freetype::Freetype CACHE STRING "" FORCE)
        endif()

        FetchContent_Declare(
            SFML
            URL
                "https://github.com/SFML/SFML/archive/7f1162dfea4969bc17417563ac55d93b72e84c1e.tar.gz"
            URL_HASH
                "SHA256=503d90b6576a3343f7344ee5eff8642d06a487db3a83b5f36e92359f4cc896a7"
            ${GRACE_FETCHCONTENT_DOWNLOAD_ARGS}
        )
        FetchContent_MakeAvailable(SFML)

        FetchContent_Declare(
            imgui
            URL
                "https://github.com/ocornut/imgui/archive/dbb5eeaadffb6a3ba6a60de1290312e5802dba5a.tar.gz"
            URL_HASH
                "SHA256=5df3e8d67efffaccb8a93b2db53f1fe903984931f43dd79761bd65839be7d8e2"
            ${GRACE_FETCHCONTENT_DOWNLOAD_ARGS}
        )
        FetchContent_MakeAvailable(imgui)

        set(IMGUI_DIR "${imgui_SOURCE_DIR}")
        FetchContent_Declare(
            imgui-sfml
            URL
                "https://github.com/SFML/imgui-sfml/archive/5e8f8fb857c8f3cff1096bf3e4402c64e3bcf6af.tar.gz"
            URL_HASH
                "SHA256=19d82a41db9376f7598b47f1fd0fc4b26dd9e5f09beefb40f50cbc91056ec383"
            ${GRACE_FETCHCONTENT_DOWNLOAD_ARGS}
        )
        FetchContent_MakeAvailable(imgui-sfml)
    endif()
endif()
