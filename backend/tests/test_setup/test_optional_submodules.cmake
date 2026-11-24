# Copyright (c) 2026 Robert Bosch GmbH
# SPDX-License-Identifier: AGPL-3.0-only

cmake_minimum_required(VERSION 3.22)
find_package(Git REQUIRED)

if(NOT EXISTS "${GRACE_SOURCE_DIR}/.gitmodules" OR NOT TEST_BINARY_DIR)
    message(FATAL_ERROR "GRACE_SOURCE_DIR and TEST_BINARY_DIR must be provided")
endif()

string(RANDOM LENGTH 12 ALPHABET 0123456789abcdef suffix)
set(work "${TEST_BINARY_DIR}/optional-submodules-${suffix}")
file(MAKE_DIRECTORY "${work}")
file(WRITE "${work}/gitconfig" "")
set(ENV{GIT_CONFIG_GLOBAL} "${work}/gitconfig")
set(ENV{GIT_CONFIG_NOSYSTEM} 1)
set(ENV{GIT_TERMINAL_PROMPT} 0)
set(git "${GIT_EXECUTABLE}"
    -c protocol.file.allow=always
    -c "core.hooksPath=${work}/no-hooks"
    -c user.name=GRACE-setup-test
    -c user.email=grace-setup-test@example.invalid
    -c commit.gpgSign=false
    -c init.defaultBranch=main
)

function(run_git)
    execute_process(
        COMMAND ${git} ${ARGN}
        RESULT_VARIABLE result
        OUTPUT_VARIABLE output
        ERROR_VARIABLE error
    )
    if(NOT result EQUAL 0)
        message(FATAL_ERROR "Git setup fixture failed: ${ARGN}\n${output}\n${error}")
    endif()
    set(git_output "${output}" PARENT_SCOPE)
endfunction()

set(commit_message
    "Setup test fixture\n\nCo-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>")
set(origin "${work}/planner-source")
run_git(init --quiet "${origin}")
file(WRITE "${origin}/fixture.txt" "Local planner fixture; no third-party source.\n")
run_git(-C "${origin}" add fixture.txt)
run_git(-C "${origin}" commit --quiet -m "${commit_message}")
run_git(-C "${origin}" rev-parse HEAD)
string(STRIP "${git_output}" revision)

set(restricted
    backend/extern/classical/CBSH2-RTC
    backend/extern/classical/EECBS
    backend/extern/classical/MAPF-LNS2
)
set(standard backend/extern/classical/lacam0)
set(superproject "${work}/superproject")
run_git(init --quiet "${superproject}")
foreach(path IN ITEMS ${restricted} "${standard}")
    run_git(-C "${superproject}" config --file .gitmodules
        "submodule.${path}.path" "${path}")
    run_git(-C "${superproject}" config --file .gitmodules
        "submodule.${path}.url" "${origin}")
    if(path IN_LIST restricted)
        run_git(config --file "${GRACE_SOURCE_DIR}/.gitmodules"
            --get "submodule.${path}.update")
        string(STRIP "${git_output}" policy)
        if(NOT policy STREQUAL "none")
            message(FATAL_ERROR "${path} must be excluded from recursive setup")
        endif()
        run_git(-C "${superproject}" config --file .gitmodules
            "submodule.${path}.update" "${policy}")
    endif()
    run_git(-C "${superproject}" update-index --add --cacheinfo
        "160000,${revision},${path}")
endforeach()
run_git(-C "${superproject}" add .gitmodules)
run_git(-C "${superproject}" commit --quiet -m "${commit_message}")

set(checkout "${work}/checkout")
run_git(clone --quiet --recurse-submodules "${superproject}" "${checkout}")
run_git(-C "${checkout}" submodule update --init --recursive)
if(NOT EXISTS "${checkout}/${standard}/fixture.txt")
    message(FATAL_ERROR "Standard recursive setup did not obtain LaCAM0")
endif()
foreach(path IN LISTS restricted)
    if(EXISTS "${checkout}/${path}/fixture.txt" OR
       EXISTS "${checkout}/.git/modules/${path}")
        message(FATAL_ERROR "Standard recursive setup fetched ${path}")
    endif()
endforeach()

run_git(-C "${checkout}" submodule update --init --recursive --checkout -- ${restricted})
foreach(path IN LISTS restricted)
    if(NOT EXISTS "${checkout}/${path}/fixture.txt")
        message(FATAL_ERROR "Explicit checkout did not obtain ${path}")
    endif()
    run_git(-C "${checkout}" config --get "submodule.${path}.update")
    string(STRIP "${git_output}" policy)
    if(NOT policy STREQUAL "none")
        message(FATAL_ERROR "Explicit checkout changed the default policy for ${path}")
    endif()
    run_git(-C "${checkout}/${path}" rev-parse HEAD)
    string(STRIP "${git_output}" actual)
    if(NOT actual STREQUAL revision)
        message(FATAL_ERROR "${path} was not checked out at its pinned revision")
    endif()
endforeach()
file(REMOVE_RECURSE "${work}")
